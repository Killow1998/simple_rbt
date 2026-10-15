#include "simple_rbt/paths.hpp"
#include "simple_rbt/runtime.hpp"
#include "simple_rbt/session.hpp"
#include "simple_rbt/runtime_info.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

using namespace simple_rbt;
namespace {
void help(const std::filesystem::path &project_root) {
  std::cout
      << R"(simple-rbt-sim — C++17 / MuJoCo / native CUDA / independent session transport
Usage: simple-rbt-sim [options]
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
  --backend auto|cpu|cuda|warp           warp is a compatibility alias for native CUDA
  --lidar-motion rolling|snapshot --rays NUMBER (1..20000, diagnostic subset)
  --session-config JSON --session-dir DIRECTORY   desktop/ROS process session
  --realtime --viewer --summary-only --output NEW_DIRECTORY
  --go2-torque-mode configured|upstream --go2-startup crouched|upstream_getup
  --project-root DIRECTORY
  --list-robots                     JSON registry; does not initialize simulation
  --runtime-info                    Loaded SDK, CPU/CUDA build and asset status as JSON
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
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) != "--list-robots")
        continue;
      Json registry = Json::array();
      for (const auto &spec : robot_specs())
        registry.push_back({{"name", spec.name},
                            {"default_controller", spec.default_controller},
                            {"continuous_terrain", spec.continuous_terrain},
                            {"allows_lateral", spec.allows_lateral},
                            {"allows_in_place_turn", spec.allows_in_place_turn},
                            {"command_limits", spec.command_limits},
                            {"min_turning_radius", spec.min_turning_radius}});
      std::cout << registry.dump() << '\n';
      return 0;
    }
    RunOptions args;
    for (int i = 1; i + 1 < argc; ++i)
      if (std::string(argv[i]) == "--project-root")
        args.simulation.project_root = argv[++i];
    bool information = false;
    for (int i = 1; i < argc; ++i)
      information = information || std::string(argv[i]) == "--runtime-info";
    if (information) {
      if (args.simulation.project_root.empty()) {
        try { args.simulation.project_root = default_project_root(); }
        catch (const std::exception &) {}
      }
      std::cout << runtime_info(args.simulation.project_root).dump() << '\n';
      return 0;
    }
    if (args.simulation.project_root.empty())
      args.simulation.project_root = default_project_root();
    std::filesystem::path session_config, session_dir;
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
      else if (option == "--session-config")
        session_config = value(i);
      else if (option == "--session-dir")
        session_dir = value(i);
      else if (option == "--worker-config")
        throw std::invalid_argument(
            "The same-process C++ ROS validator was superseded; use "
            "tools/test_ros_session.py for independent ROS E2E checks");
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
      } else if (option == "--realtime")
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
    if (!session_config.empty()) {
      Json c;
      std::ifstream in(session_config);
      if (!in)
        throw std::invalid_argument("Cannot open session config");
      in >> c;
      run_session(c, session_dir, args.simulation.project_root);
      return 0;
    }
    if (!supported_command(args.simulation.robot, args.simulation_command))
      throw std::invalid_argument("Unsupported/nonfinite planar command");
    TransportFactory factory;
    if (!ros_backend.empty())
      throw std::invalid_argument(
          "ROS adapters run independently; use --session-config and "
          "simple-rbt-ros1/ros2 --session-dir");
    run_simulation(args, nullptr, factory);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "simple-rbt-sim: " << e.what() << '\n';
    return 2;
  }
}
