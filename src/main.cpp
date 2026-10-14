#include "simple_rbt/full_run.hpp"
#include "simple_rbt/runtime.hpp"
#if defined(SIMPLE_RBT_ROS1) || defined(SIMPLE_RBT_ROS2)
#include "simple_rbt/ros_bridge.hpp"
#endif
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

using namespace simple_rbt;
namespace {
void help(const std::filesystem::path &project_root) {
  std::cout << R"(simple-rbt-native — C++17 / MuJoCo / native CUDA / optional ROS wrapper
Usage: ./run.sh [options]
)";
  std::cout << "  --robot ";
  bool separator = false;
  for (const auto &spec : robot_specs()) {
    if (separator)
      std::cout << '|';
    std::cout << spec.name;
    separator = true;
  }
  Json catalog;
  std::ifstream(project_root / "assets/catalog.json") >> catalog;
  std::set<std::string> controllers;
  for (const auto &entry : catalog.at("models").items())
    controllers.insert(entry.key().substr(entry.key().find('-') + 1));
  std::cout << "\n  --controller auto";
  for (const auto &controller : controllers)
    std::cout << '|' << controller;
  std::cout << R"(
  --control-mode native|tracked     tracked uses ideal body velocity/heading feedback
  --terrain flat|ramp|stairs|rough|hills   pure wheel robots allow flat/ramp/hills
  --terrain-level standard|moderate|hard|extreme
  --difficulty NUMBER --seed UNSIGNED --friction NUMBER
  --cmd VX VY WZ --settle SECONDS --duration SECONDS
  --sensors physics|lidar|full --cameras --camera-hz NUMBER
  --backend cpu|cuda|warp           warp is a compatibility alias for native CUDA
  --lidar-motion rolling|snapshot --rays NUMBER (1..20000, diagnostic subset)
  --ros1 | --ros2                    selects compiled wrapper; /<robot>/cmd_vel; timeout 0.5s
  --realtime --viewer --summary-only --output NEW_DIRECTORY
  --go2-torque-mode configured|upstream --go2-startup crouched|upstream_getup
  --project-root DIRECTORY
No Python interpreter, Warp or LLVM is used by this executable.
)";
}
double number(const std::string &s) {
  std::size_t n = 0;
  double v = std::stod(s, &n);
  if (n != s.size() || !std::isfinite(v))
    throw std::invalid_argument("Expected a finite number");
  return v;
}
std::uint64_t integer(const std::string &s) {
  if (s.empty() || s[0] == '-')
    throw std::invalid_argument("Expected nonnegative integer");
  std::size_t n = 0;
  auto v = std::stoull(s, &n);
  if (n != s.size())
    throw std::invalid_argument("Expected nonnegative integer");
  return v;
}
} // namespace
int main(int argc, char **argv) {
  try {
    RunOptions args;
    args.simulation.project_root = SIMPLE_RBT_PROJECT_ROOT;
    std::filesystem::path config_path;
    bool explicit_cameras = false;
    std::string ros_backend;
    auto value = [&](int &i) {
      if (++i >= argc)
        throw std::invalid_argument("Missing option value");
      return std::string(argv[i]);
    };
    for (int i = 1; i < argc; i++) {
      std::string option = argv[i];
      if (option == "--help" || option == "-h") {
        help(args.simulation.project_root);
        return 0;
      } else if (option == "--project-root")
        args.simulation.project_root = value(i);
      else if (option == "--worker-config")
        config_path = value(i);
      else if (option == "--robot")
        args.simulation.robot = value(i);
      else if (option == "--controller")
        args.simulation.controller = value(i);
      else if (option == "--terrain")
        args.simulation.terrain = value(i);
      else if (option == "--terrain-level")
        args.simulation.terrain_level = value(i);
      else if (option == "--control-mode")
        args.simulation.control_mode = value(i);
      else if (option == "--go2-torque-mode")
        args.simulation.go2_torque_mode = value(i);
      else if (option == "--go2-startup")
        args.simulation.go2_startup = value(i);
      else if (option == "--difficulty")
        args.simulation.difficulty = number(value(i));
      else if (option == "--friction")
        args.simulation.friction = number(value(i));
      else if (option == "--seed")
        args.simulation.seed = integer(value(i));
      else if (option == "--backend")
        args.backend = value(i);
      else if (option == "--sensors")
        args.sensors = value(i);
      else if (option == "--lidar-motion")
        args.lidar_motion = value(i);
      else if (option == "--duration")
        args.duration = number(value(i));
      else if (option == "--settle")
        args.settle = number(value(i));
      else if (option == "--camera-hz")
        args.camera_hz = number(value(i));
      else if (option == "--rays")
        args.rays = integer(value(i));
      else if (option == "--output")
        args.output = value(i);
      else if (option == "--cmd")
        for (double &x : args.simulation_command)
          x = number(value(i));
      else if (option == "--ros1" || option == "--ros2") {
        const auto selected = option.substr(2);
        if (!ros_backend.empty() && ros_backend != selected)
          throw std::invalid_argument("Select one ROS wrapper");
        ros_backend = selected;
      }
      else if (option == "--realtime")
        args.realtime = true;
      else if (option == "--viewer")
        args.viewer = true;
      else if (option == "--summary-only")
        args.summary_only = true;
      else if (option == "--cameras")
        explicit_cameras = true;
      else
        throw std::invalid_argument("Unknown option: " + option);
    }
    if (explicit_cameras)
      args.sensors = "full";
    std::unique_ptr<RunObserver> observer;
    if (!config_path.empty()) {
      std::ifstream file(config_path);
      Json c;
      file >> c;
      args.simulation.robot = c.at("robot");
      args.simulation.controller = c.at("controller");
      args.simulation.control_mode = c.at("control_mode");
      args.simulation.terrain = c.at("terrain");
      args.simulation.terrain_level = c.at("terrain_level");
      args.simulation.seed = c.at("seed");
      if (!c.at("friction").is_null())
        args.simulation.friction = c.at("friction");
      args.backend = c.at("backend");
      args.camera_hz = c.at("camera_hz");
      args.settle = c.at("settle_s");
      args.output = c.at("output").get<std::string>();
      ros_backend = "ros2";
      args.realtime = true;
      args.sensors = "full";
      args.summary_only = true;
      args.output_precreated = true;
      Json catalog;
      std::ifstream f(args.simulation.project_root /
                      "assets/catalog.json");
      f >> catalog;
      double finish =
          catalog["terrain_presets"]
                 [args.simulation.terrain + "-" + args.simulation.terrain_level]
                 ["metadata"]["finish_x_m"];
      args.duration =
          args.settle +
          std::max(8., finish + 2.) / c.at("speed_m_s").get<double>() + 2.;
      observer = std::make_unique<FullRunObserver>(config_path);
    }
    if (!supported_command(args.simulation.robot, args.simulation_command))
      throw std::invalid_argument("Unsupported/nonfinite planar command");
    TransportFactory factory;
    if (!ros_backend.empty()) {
#if defined(SIMPLE_RBT_ROS1) || defined(SIMPLE_RBT_ROS2)
      RosOptions options;
      options.robot_name = args.simulation.robot;
#ifdef SIMPLE_RBT_ROS1
      if (ros_backend != "ros1")
#else
      if (ros_backend != "ros2")
#endif
        throw std::invalid_argument("Requested ROS wrapper is not in this build");
      factory = [options](const Extrinsics& extrinsics) {
        return std::make_unique<RosBridge>(options, extrinsics);
      };
#else
      throw std::invalid_argument("This build has no ROS wrapper");
#endif
    }
    run_simulation(args, observer.get(), factory);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "simple-rbt-native: " << e.what() << '\n';
    return 2;
  }
}
