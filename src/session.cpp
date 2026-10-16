#include "simple_rbt/session.hpp"
#include "session_observer.hpp"
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
Extrinsics transport_extrinsics(Simulation &sim, bool camera) {
  Extrinsics x;
  x.lidar = sim.extrinsics("lidar");
  x.imu = sim.extrinsics("mid360_imu");
  x.camera = sim.extrinsics("camera_mount");
  x.has_camera = camera;
  double mount[9], optical[9];
  mju_quat2Mat(mount, x.camera.quaternion.data());
  for (int row = 0; row < 3; ++row) {
    optical[row * 3] = -mount[row * 3 + 1];
    optical[row * 3 + 1] = -mount[row * 3 + 2];
    optical[row * 3 + 2] = mount[row * 3];
  }
  mju_mat2Quat(x.camera.quaternion.data(), optical);
  return x;
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
  RunOptions args;
  args.simulation.project_root = project_root;
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
      else
        throw std::invalid_argument("Unknown sensor mount");
    }
  const Json sensors = config.value("sensors", Json::object());
  args.mid360_enabled = sensors.value("mid360", true);
  args.camera_enabled = sensors.value("camera", true);
  args.depth_enabled = sensors.value("depth", true);
  args.panorama_enabled = sensors.value("panorama", false);
  args.sensors =
      (args.camera_enabled || args.depth_enabled || args.panorama_enabled)
          ? "full"
      : args.mid360_enabled ? "lidar"
                            : "physics";
  args.backend = config.value("backend", std::string("auto"));
  args.camera_hz = config.value("camera_hz", 10.);
  args.rays = config.value("rays", std::size_t(20000));
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
  session_detail::SessionObserver observer(config, interrupted);
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
