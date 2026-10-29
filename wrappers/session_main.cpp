#include "simple_rbt/event_log.hpp"
#include "simple_rbt/ipc.hpp"
#include "simple_rbt/robots.hpp"
#include "simple_rbt/ros_bridge.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>
using namespace simple_rbt;
namespace {
std::atomic<bool> stop{false};
void interrupted(int) { stop = true; }
void validate_distro() {
  const char *distro = std::getenv("ROS_DISTRO");
  const char *version = std::getenv("ROS_VERSION");
  if (!distro || distro != std::string(SIMPLE_RBT_ROS_DISTRO) ||
      !version || version != std::string(SIMPLE_RBT_ROS_VERSION))
    throw std::runtime_error(
        "This node requires ROS" SIMPLE_RBT_ROS_VERSION " " SIMPLE_RBT_ROS_DISTRO
        "; source /opt/ros/" SIMPLE_RBT_ROS_DISTRO
        "/setup.bash or use simple-rbt-ros" SIMPLE_RBT_ROS_VERSION ":" SIMPLE_RBT_ROS_DISTRO
        ". Actual ROS_DISTRO=" + std::string(distro ? distro : "<unset>") +
        ", ROS_VERSION=" + std::string(version ? version : "<unset>"));
}
RosOptions options(const ipc::Json &j) {
  RosOptions o;
  o.robot_name = j.at("robot_name").get<std::string>();
  o.command_timeout_s = j.value("command_timeout_s", .5);
  o.publish_pointcloud2 = j.value("publish_pointcloud2", true);
  o.publish_odom_tf = j.value("publish_odom_tf", true);
  o.odom_topic = j.value("odom_topic", "/odom");
  o.perspective_fov_y_deg = j.value("perspective_fov_y_deg", 60.);
  o.odom_frame = j.value("odom_frame", "odom");
  o.base_frame = j.value("base_frame", "base_link");
  o.lidar_frame = j.value("lidar_frame", "livox_frame");
  o.imu_frame = j.value("imu_frame", "imu_frame");
  o.camera_frame = j.value("camera_frame", "camera_optical_frame");
  o.camera_depth_frame = j.value("camera_depth_frame", "camera_depth_optical_frame");
  o.camera_imu_frame = j.value("camera_imu_frame", "camera_imu_optical_frame");
  o.status_topic = j.value("status_topic", "/full_run/status");
  o.node_name = "simple_rbt_session";
  return o;
}
} // namespace
int main(int argc, char **argv) {
#ifdef SIMPLE_RBT_ROS1
    const std::string role = "ros1";
#else
    const std::string role = "ros2";
#endif
  std::unique_ptr<diagnostics::Log> events;
  try {
    std::filesystem::path session;
    double duration = 0.;
    bool runtime_info = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--session-dir" && i + 1 < argc)
        session = argv[++i];
      else if (arg == "--duration" && i + 1 < argc)
        duration = std::stod(argv[++i]);
      else if (arg == "--runtime-info")
        runtime_info = true;
      else if (arg == "--help") {
        std::cout << "simple-rbt-ros-session --session-dir PATH [--duration "
                     "SECONDS]\nsimple-rbt-ros-session --runtime-info\n";
        return 0;
      } else
        throw std::invalid_argument("unknown or incomplete option: " + arg);
    }
    events=std::make_unique<diagnostics::Log>(role,"",std::getenv("SIMPLE_RBT_LOG_STDOUT")!=nullptr);
    validate_distro();
    if (runtime_info) {
      std::cout << ipc::Json{{"ros_distro", SIMPLE_RBT_ROS_DISTRO},
                             {"ros_version", SIMPLE_RBT_ROS_VERSION},
                             {"ros_client_version",
                              SIMPLE_RBT_ROS_CLIENT_VERSION},
                             {"transport", "unix_ipc"},
                             {"configurable_odometry", true},
                             {"camera_devices", true},
                             {"joint_control", true},
                             {"flight_control", true},
                             {"external_camera", true},
                             {"diagnostics", true},
                             {"runtime_language", "C++17"}}
                       .dump()
                << '\n';
      return 0;
    }
    if (session.empty())
      throw std::invalid_argument("--session-dir is required");
    std::signal(SIGINT, interrupted);
    std::signal(SIGTERM, interrupted);
    ipc::Client client;
    const auto startup = std::chrono::steady_clock::now();
    while (!stop &&
           !client.connect(session, role,
                           {"clock", "imu", "camera_imu", "lidar", "odom",
                            "camera", "panorama", "external_camera", "joints",
                            "status", "metadata"})) {
      if (std::chrono::steady_clock::now() - startup > std::chrono::seconds(20))
        throw std::runtime_error("core IPC session unavailable");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::unique_ptr<RosBridge> bridge;
    const RobotSpec *spec = nullptr;
    ipc::Json configuration;
    unsigned last_mask=15;
    auto modules=[&](const ipc::Json &settings) {
      unsigned mask=settings.value("module_mask",15U);if(mask==last_mask)return;
      std::vector<std::string> names;unsigned bit=1;for(auto name:{"sensor","ipc","ros","control"}) {if(mask&bit)names.push_back(name);bit*=2;}events->set_modules(names);last_mask=mask;
    };
    auto sent = std::chrono::steady_clock::now();
    std::uint64_t count = 0;
    while (!stop && client.connected()) {
      for (const auto &packet : client.poll()) {
        if (packet.kind == ipc::Kind::Hello) {
          const auto hello = ipc::decode_json(packet);
          modules(hello.value("logging",ipc::Json::object()));
          events->set_session(hello.value("log_session",events->session()));
          if(events->tracing()!=hello.value("trace",false)) events->set_trace(hello.value("trace",false));
          events->event("ipc.connected",{{"role",role}});
          const ipc::Json next = {{"options", hello.at("options")},
                                  {"extrinsics", hello.at("extrinsics")}};
          if (configuration != next) {
            bridge.reset();
            const auto settings = options(next.at("options"));
            bridge = std::make_unique<RosBridge>(
                settings,
                ipc::decode_extrinsics(next.at("extrinsics")));
            spec = find_robot_spec(settings.robot_name);
            configuration = next;
            std::cerr << role << " node connected to independent core for "
                      << settings.robot_name << std::endl;
          }
          continue;
        }
        if (!bridge)
          continue;
        if(packet.kind==ipc::Kind::Metadata) {
          auto data=ipc::decode_json(packet);
          modules(data.value("logging",ipc::Json::object()));
          bool trace=data.value("logging",ipc::Json::object()).value("trace",false);
          if(trace!=events->tracing()) events->set_trace(trace);
          continue;
        }
        const std::string stream =
            packet.kind == ipc::Kind::Lidar            ? "lidar"
            : packet.kind == ipc::Kind::Imu            ? "imu"
            : packet.kind == ipc::Kind::CameraImu      ? "camera_imu"
            : packet.kind == ipc::Kind::Camera         ? "camera"
            : packet.kind == ipc::Kind::Panorama       ? "panorama"
            : packet.kind == ipc::Kind::ExternalCamera ? "external_camera"
            : packet.kind == ipc::Kind::JointState     ? "joints"
            : packet.kind == ipc::Kind::Odometry       ? "odom"
                                                       : "clock";
        const auto key=diagnostics::sample_key(stream,packet.stamp_ns);
        events->trace("ipc.received",{{"sample",key},{"bytes",packet.payload.size()}},packet.stamp_ns);
        switch (packet.kind) {
        case ipc::Kind::JointState:
          bridge->publish_joints(packet.stamp_ns, ipc::decode_joints(packet));
          break;
        case ipc::Kind::ExternalCamera:
          bridge->publish_external_camera(packet.stamp_ns,
                                          ipc::decode_camera(packet));
          break;
        case ipc::Kind::Clock:
          bridge->publish_clock(packet.stamp_ns);
          break;
        case ipc::Kind::Imu:
          bridge->publish_imu(ipc::decode_imu(packet));
          break;
        case ipc::Kind::CameraImu:
          bridge->publish_camera_imu(ipc::decode_imu(packet));
          break;
        case ipc::Kind::Lidar:
          bridge->publish_scan(ipc::decode_scan(packet));
          break;
        case ipc::Kind::Odometry: {
          auto o = ipc::decode_odometry(packet);
          bridge->publish_odometry(packet.stamp_ns, o.pose, o.linear,
                                   o.angular);
          break;
        }
        case ipc::Kind::Camera:
          bridge->publish_camera(packet.stamp_ns, ipc::decode_camera(packet));
          break;
        case ipc::Kind::Panorama:
          bridge->publish_camera(packet.stamp_ns, ipc::decode_camera(packet),
                                 true);
          break;
        case ipc::Kind::Status:
          bridge->publish_status(ipc::decode_json(packet).dump());
          break;
        default:
          break;
        }
        events->trace("ros.publish_called",{{"sample",key},{"stream",stream}},packet.stamp_ns);
        ++count;
      }
      if (bridge)
        bridge->spin_once();
      const auto now = std::chrono::steady_clock::now();
      if (bridge && now - sent >= std::chrono::milliseconds(20)) {
        if (spec && spec->joint_control)
          client.send_packet(ipc::encode_joint_command(
              bridge->joint_command().value_or(JointCommand{{}, {}, true})));
        else if(spec && spec->flight_control)
          client.send_packet(ipc::encode_flight_command(bridge->flight_command()));
        else
          client.send_command(bridge->command());
        sent = now;
      }
      if (duration > 0. &&
          now - startup >= std::chrono::duration<double>(duration))
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    events->event("ipc.disconnected",{{"packets",count},{"requested_stop",bool(stop)}});
    if(bridge && spec && spec->flight_control)
      client.send_packet(ipc::encode_flight_command({}));
    else
      client.send_command({});
    client.close();
    if (bridge)
      bridge->close();
    std::cerr << role << " session stopped; published " << count
              << " packets\n";
    return 0;
  } catch (const std::exception &e) {
    if(events) events->event("process.failed",{{"message",e.what()}},0,"error");
    std::cerr << "ROS session: " << e.what() << '\n';
    return 1;
  }
}
