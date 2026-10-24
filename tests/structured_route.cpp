#include "simple_rbt/core.hpp"
#include "simple_rbt/lidar.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/resource.h>
using namespace simple_rbt;
int main(int argc, char **argv) {
  try {
    if (argc > 2)
      return 2;
    std::filesystem::path root = SIMPLE_RBT_PROJECT_ROOT, out;
    if (argc == 2) {
      out = argv[1];
      if (!std::filesystem::create_directory(out))
        return 2;
    }
    Json scene;
    std::ifstream(root / "schemas/examples/structured-building.scene.json") >>
        scene;
    for (auto &o : scene["objects"])
      if (o["id"] == "spawn-ground")
        o["pose"]["position_m"] = {-2.9, 0, 0};
    SimulationOptions options;
    options.project_root = root;
    options.robot = "go2w";
    options.controller = "rl_sar";
    options.scene = scene;
    Simulation sim(options);
    if (!out.empty()) {
      std::ofstream(out / "scene.json") << scene.dump(2);
      std::ofstream(out / "compiled-model.xml") << sim.xml();
    }
    std::ofstream trajectory;
    if (!out.empty())
      trajectory.open(out / "trajectory.csv");
    trajectory << "stamp_ns,phase,x,y,z,tilt_deg\n";
    CsvPattern pattern((root / "assets/lidar/mid360-real-centr.csv").string());
    RollingLidar lidar(sim.model(), LidarBackend::Cpu, pattern);
    PoseHistory history;
    history.add(0, sim.pose("lidar"));
    struct Goal {
      double x, y, yaw, z;
    };
    std::vector<Goal> goals{{1.45, 0, 0, 1.5},
                            {1.45, 1.4, 0, 1.5},
                            {.8, 1.4, 3.141592653589793, 1.5},
                            {-2.9, 1.4, 3.141592653589793, 3},
                            {.8, 1.4, 0, 1.5},
                            {1.45, 1.4, 0, 1.5},
                            {1.45, 0, 0, 1.5},
                            {-2.9, 0, 3.141592653589793, 0}};
    int phase = 0;
    std::size_t scans = 0;
    double maximumTilt = 0;
    auto began = std::chrono::steady_clock::now();
    std::string failure;
    while (sim.stamp_ns() < 140000000000ULL) {
      auto p = sim.pose();
      double yaw = std::atan2(2 * (p.quaternion[0] * p.quaternion[3] +
                                   p.quaternion[1] * p.quaternion[2]),
                              1 - 2 * (p.quaternion[2] * p.quaternion[2] +
                                       p.quaternion[3] * p.quaternion[3]));
      auto goal = goals[phase];
      double dx = goal.x - p.position[0], dy = goal.y - p.position[1],
             angle = std::remainder(goal.yaw - yaw, 2 * std::acos(-1.));
      Vec3 cmd{};
      if (sim.stamp_ns() > 3000000000ULL) {
        cmd = {std::clamp(std::cos(yaw) * dx + std::sin(yaw) * dy, -.28, .28),
               std::clamp(-std::sin(yaw) * dx + std::cos(yaw) * dy, -.18, .18),
               std::clamp(1.5 * angle, -.6, .6)};
        if (std::abs(angle) > .3)
          cmd[0] = cmd[1] = 0;
      }
      sim.set_command(cmd);
      sim.step();
      auto stamp = sim.stamp_ns();
      history.add(stamp, sim.pose("lidar"));
      p = sim.pose();
      int site = mj_name2id(sim.model(), mjOBJ_SITE, "base_origin");
      double tilt =
          std::acos(std::clamp(sim.data()->site_xmat[9 * site + 8], -1., 1.)) *
          180 / std::acos(-1.);
      maximumTilt = std::max(maximumTilt, tilt);
      if (trajectory.is_open() && stamp % 20000000 == 0)
        trajectory << stamp << ',' << phase << ',' << p.position[0] << ','
                   << p.position[1] << ',' << p.position[2] << ',' << tilt
                   << '\n';
      if (stamp % 100000000 == 0) {
        LidarScan scan;
        lidar.scan(sim.data(), stamp - 100000000, 20000, history, scan);
        ++scans;
      }
      if (tilt > 60 || p.position[2] < -.1) {
        failure = "fallen";
        break;
      }
      if (std::hypot(dx, dy) < .15 && std::abs(angle) < .15) {
        if (std::abs(p.position[2] - goal.z - .38) > .3) {
          failure = "waypoint reached at incorrect storey";
          break;
        }
        ++phase;
        if (phase == int(goals.size()))
          break;
      }
    }
    if (phase < int(goals.size()) && failure.empty())
      failure = "route timeout";
    rusage resources{};
    getrusage(RUSAGE_SELF, &resources);
    double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    Json report{{"passed", failure.empty()},
                {"completed_waypoints", phase},
                {"failure", failure},
                {"simulation_seconds", sim.stamp_ns() * 1e-9},
                {"wall_seconds", wall},
                {"max_tilt_deg", maximumTilt},
                {"peak_rss_mib", resources.ru_maxrss / 1024.},
                {"scans", scans},
                {"scope", "Synthetic v2 U-stairwell; tracked Go2W, CPU MID360; "
                          "no ROS or camera in this route probe"}};
    if (!out.empty())
      std::ofstream(out / "report.json") << report.dump(2);
    std::cout << report.dump(2) << std::endl;
    return failure.empty() ? 0 : 1;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
