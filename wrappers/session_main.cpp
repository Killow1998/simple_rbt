#include "simple_rbt/ipc.hpp"
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
  o.perspective_fov_y_deg = j.value("perspective_fov_y_deg", 60.);
  o.odom_frame = j.value("odom_frame", "odom");
  o.base_frame = j.value("base_frame", "base_link");
  o.lidar_frame = j.value("lidar_frame", "livox_frame");
  o.imu_frame = j.value("imu_frame", "imu_frame");
  o.camera_frame = j.value("camera_frame", "camera_optical_frame");
  o.status_topic = j.value("status_topic", "/full_run/status");
  o.node_name = "simple_rbt_session";
  return o;
}
} // namespace
int main(int argc, char **argv) {
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
    validate_distro();
    if (runtime_info) {
      std::cout << ipc::Json{{"ros_distro", SIMPLE_RBT_ROS_DISTRO},
                            {"ros_version", SIMPLE_RBT_ROS_VERSION},
                            {"ros_client_version", SIMPLE_RBT_ROS_CLIENT_VERSION},
                            {"transport", "unix_ipc"},
                            {"runtime_language", "C++17"}}
                       .dump() << '\n';
      return 0;
    }
    if (session.empty())
      throw std::invalid_argument("--session-dir is required");
    std::signal(SIGINT, interrupted);
    std::signal(SIGTERM, interrupted);
#ifdef SIMPLE_RBT_ROS1
    const std::string role = "ros1";
#else
    const std::string role = "ros2";
#endif
    ipc::Client client;
    const auto startup = std::chrono::steady_clock::now();
    while (!stop && !client.connect(session, role,
                                    {"clock", "imu", "lidar", "odom", "camera",
                                     "panorama", "status"})) {
      if (std::chrono::steady_clock::now() - startup > std::chrono::seconds(20))
        throw std::runtime_error("core IPC session unavailable");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::unique_ptr<RosBridge> bridge;
    ipc::Json configuration;
    auto sent = std::chrono::steady_clock::now();
    std::uint64_t count = 0;
    while (!stop && client.connected()) {
      for (const auto &packet : client.poll()) {
        if (packet.kind == ipc::Kind::Hello) {
          const auto hello = ipc::decode_json(packet);
          const ipc::Json next = {{"options", hello.at("options")},
                                  {"extrinsics", hello.at("extrinsics")}};
          if (configuration != next) {
            bridge.reset();
            bridge = std::make_unique<RosBridge>(
                options(next.at("options")),
                ipc::decode_extrinsics(next.at("extrinsics")));
            configuration = next;
            std::cout << role << " node connected to independent core for "
                      << bridge->options().robot_name << std::endl;
          }
          continue;
        }
        if (!bridge)
          continue;
        switch (packet.kind) {
        case ipc::Kind::Clock:
          bridge->publish_clock(packet.stamp_ns);
          break;
        case ipc::Kind::Imu:
          bridge->publish_imu(ipc::decode_imu(packet));
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
        ++count;
      }
      if (bridge)
        bridge->spin_once();
      const auto now = std::chrono::steady_clock::now();
      if (bridge && now - sent >= std::chrono::milliseconds(20)) {
        client.send_command(bridge->command());
        sent = now;
      }
      if (duration > 0. &&
          now - startup >= std::chrono::duration<double>(duration))
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    client.send_command({});
    client.close();
    if (bridge)
      bridge->close();
    std::cout << role << " session stopped; published " << count
              << " packets\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "ROS session: " << e.what() << '\n';
    return 1;
  }
}
