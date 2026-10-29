#include "simple_rbt/event_log.hpp"
#include "simple_rbt/session.hpp"
#include "session_observer.hpp"
#include "sensor_devices.hpp"
#include <csignal>
#include <regex>
namespace simple_rbt::session_detail {
Pose parse_pose(const Json &json) {
  Pose p;
  p.position = json.at("position").get<Vec3>();
  p.quaternion = json.at("quaternion").get<Quat>();
  return p;
}
Json pose_json(const Pose &p) {
  return {{"position", p.position}, {"quaternion", p.quaternion}};
}
} // namespace simple_rbt::session_detail
namespace simple_rbt {
namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
} // namespace
Json run_session(const Json &config, const std::filesystem::path &directory,
                 const std::filesystem::path &project_root) {
  if (directory.empty())
    throw std::invalid_argument("--session-dir is required");
  Json summary=config;summary.erase("scene");
  if(config.contains("scene")) summary["scene_summary"]={{"id",config["scene"].value("id",std::string())},{"objects",config["scene"].value("objects",Json::array()).size()}};
  diagnostics::log().event("session.configuration",summary);
  RunOptions args;
  args.simulation.project_root = project_root;
  args.simulation.scene = config.value("scene", Json());
  args.simulation.scene_texture = config.value("scene_texture", std::string());
  args.simulation.scene_spawn = config.value("scene_spawn", std::string());
  args.simulation.robot = config.value("robot", std::string("ranger_mini"));
  args.simulation.controller = config.value("controller", std::string("auto"));
  args.simulation.control_mode =
      config.value("control_mode", std::string("tracked"));
  args.simulation.terrain = config.value("terrain", std::string("flat"));
  args.simulation.terrain_level =
      config.value("terrain_level", std::string("standard"));
  args.simulation.difficulty = config.value("difficulty", 1.);
  if (config.contains("mounts"))
    for (const auto &it : config.at("mounts").items()) {
      if (it.key() == "mid360")
        args.simulation.mid360_mount = session_detail::parse_pose(it.value());
      else if (it.key() == "camera")
        args.simulation.camera_mount = session_detail::parse_pose(it.value());
      else if (it.key() == "external_camera")
        args.simulation.external_camera_mount =
            session_detail::parse_pose(it.value());
      else
        throw std::invalid_argument("Unknown sensor mount");
    }
  const Json sensors = config.value("sensors", Json::object());
  args.mid360_enabled = sensors.value("mid360", true);
  args.camera_enabled = sensors.value("camera", true);
  args.depth_enabled = sensors.value("depth", true);
  args.panorama_enabled = sensors.value("panorama", false);
  args.simulation.external_camera_enabled =
      sensors.value("external_camera", false);
  args.sensors =
      (args.camera_enabled || args.depth_enabled || args.panorama_enabled ||
       args.simulation.external_camera_enabled)
          ? "full"
      : args.mid360_enabled ? "lidar"
                            : "physics";
  args.simulation.lidar_model = parse_lidar_model(config.value("lidar_model", std::string("mid360")));
  args.simulation.camera_model=config.value("camera_model",std::string("d435"));
  if(config.contains("camera_extrinsics")) {
    const auto &x=config.at("camera_extrinsics");
    for(const auto &it:x.items()) {
      auto p=session_detail::parse_pose(it.value());
      if(it.key()=="color") args.simulation.camera_color_offset=p;
      else if(it.key()=="depth") args.simulation.camera_depth_offset=p;
      else if(it.key()=="imu") args.simulation.camera_imu_offset=p;
      else throw std::invalid_argument("Unknown camera calibration frame");
    }
  }
  args.backend = config.value("backend", std::string("auto"));
  args.camera_hz = config.value("camera_hz", 10.);
  args.rays = config.value("rays", std::size_t(0));
  args.settle = config.value("settle_s", 1.);
  args.realtime = true;
  args.indefinite = !config.contains("duration_s");
  args.duration = config.value("duration_s", 10.);
  args.summary_only = true;
  if (config.contains("output"))
    args.output = config.at("output").get<std::string>();
  TransportOptions options;
  options.robot_name = args.simulation.robot;
  const auto ros = config.value("ros", Json::object());
  options.publish_odom_tf = ros.value("publish_odom_tf", true);
  options.odom_topic = ros.value("odom_topic", std::string("/odom"));
  // Keep portable absolute topic names accepted by both supported adapters.
  if (!std::regex_match(options.odom_topic,
                       std::regex("/([A-Za-z_][A-Za-z0-9_]*)(/[A-Za-z_][A-Za-z0-9_]*)*")))
    throw std::invalid_argument("ros.odom_topic must be an absolute topic, e.g. /ground_truth/odom");
  session_detail::SessionObserver observer(args, config.value("source", std::string("gui")),
                                          config.value("start_immediately", false), interrupted);
  auto old_int = std::signal(SIGINT, interrupt),
       old_term = std::signal(SIGTERM, interrupt);
  interrupted = 0;
  struct Restore {
    decltype(old_int) a, b;
    ~Restore() {
      std::signal(SIGINT, a);
      std::signal(SIGTERM, b);
    }
  } restore{old_int, old_term};
  return run_simulation(args, &observer, [&](const Extrinsics &x) {
    return std::make_unique<ipc::SessionServer>(directory, options, x);
  });
}
} // namespace simple_rbt
