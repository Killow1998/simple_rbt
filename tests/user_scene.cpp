#include "simple_rbt/core.hpp"
#include "simple_rbt/lidar.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace simple_rbt;
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
Json scene() {
  auto box = [](std::string id, std::string floor, Vec3 pos, Vec3 size) {
    return Json{{"id", id},
                {"name", id},
                {"floor", floor},
                {"kind", "floor"},
                {"position", pos},
                {"size", size},
                {"quaternion", {1, 0, 0, 0}}};
  };
  return {{"version", 1},
          {"id", "building"},
          {"name", "Two storeys"},
          {"floors",
           {{{"id", "ground"}, {"name", "Ground"}, {"height", 0}},
            {{"id", "upper"}, {"name", "Upper"}, {"height", 3}}}},
          {"objects",
           {box("slab0", "ground", {0, 0, -.1}, {20, 20, .2}),
            box("slab1", "upper", {0, 0, -.1}, {8, 8, .2})}},
          {"spawn",
           {{"floor", "upper"}, {"position", {0, 0, 0}}, {"yaw_deg", 90}}}};
}
void check_connections() {
  SimulationOptions options;
  options.project_root = SIMPLE_RBT_PROJECT_ROOT;
  options.robot = "turtlebot3_waffle_pi";
  options.scene = scene();
  // Keep floor slabs away from the test flight so downward rays hit the steps.
  options.scene["objects"][1]["position"] = {30, 0, -.1};
  Json connection = {{"id", "flight"},
                     {"name", "Flight"},
                     {"floor", "ground"},
                     {"kind", "stairs"},
                     {"position", {0, 0, 0}},
                     {"quaternion", {1, 0, 0, 0}},
                     {"size", {4.2, 1.2, .2}},
                     {"connection", {{"to_floor", "upper"}, {"steps", 15}}}};
  options.scene["objects"].push_back(connection);
  options.scene["spawn"]["position"] = {30, 0, 0};
  mjtByte groups[6]{0, 1, 0, 0, 0, 0};
  auto height = [&](Simulation &sim, double x, double y) {
    mjtNum origin[3]{x, y, 6}, direction[3]{0, 0, -1};
    int geom = -1;
    double distance = mj_ray(sim.model(), sim.data(), origin, direction, groups,
                             1, -1, &geom, nullptr);
    require(distance >= 0, "Expected physical surface");
    return 6 - distance;
  };
  Simulation stairs(options);
  for (int i = 0; i < 15; ++i)
    require(std::abs(height(stairs, (i + .5) * .28, 0) - (i + 1) * .2) < 1e-7,
            "Step rise or tread depth differs from parameters");
  require(std::abs(height(stairs, 4.06, .59) - 3) < 1e-7 &&
              std::abs(height(stairs, 4.06, .61)) < 1e-7,
          "Stair width incorrect");
  options.scene["objects"][2]["kind"] = "ramp";
  options.scene["objects"][2]["size"] = {12, 2, .2};
  Simulation ramp(options);
  for (double x : {.01, 3., 6., 11.99})
    require(std::abs(height(ramp, x, 0) - x * .25) < 1e-7,
            "Ramp surface misses floor elevations or incline");
  options.scene["floors"][1]["height"] = 4;
  Simulation raised(options);
  require(std::abs(height(raised, 6, 0) - 2) < 1e-7,
          "Ramp failed to follow floor height");
  options.scene["objects"][2]["connection"]["to_floor"] = "ground";
  bool rejected = false;
  try {
    Simulation invalid(options);
  } catch (const std::exception &) {
    rejected = true;
  }
  require(rejected, "Non-ascending connection accepted");
  std::cout << "Parametric stairs, width, ramp incline, endpoints and "
               "floor-height changes passed\n";
}
void check_openings_and_landings() {
  SimulationOptions options;
  options.project_root = SIMPLE_RBT_PROJECT_ROOT;
  options.robot = "turtlebot3_waffle_pi";
  options.scene = scene();
  options.scene["spawn"]["floor"] = "ground";
  options.scene["objects"][1]["opening"] = {{"center", {0, 0}},
                                            {"size", {2, 4}}};
  auto height = [](Simulation &sim, double x, double y) {
    mjtNum origin[3]{x, y, 6}, direction[3]{0, 0, -1};
    mjtByte groups[6]{0, 1, 0, 0, 0, 0};
    int geom = -1;
    double d = mj_ray(sim.model(), sim.data(), origin, direction, groups, 1, -1,
                      &geom, nullptr);
    require(d >= 0, "Missing physical surface");
    return 6 - d;
  };
  Simulation opening(options);
  require(std::abs(height(opening, 0, 0)) < 1e-7 &&
              std::abs(height(opening, 2, 0) - 3) < 1e-7,
          "Opening still has a collider or lost its rim");
  options.scene["objects"][1]["quaternion"] = {std::sqrt(.5), 0, 0,
                                               std::sqrt(.5)};
  Simulation rotated(options);
  require(std::abs(height(rotated, 1.5, 0)) < 1e-7 &&
              std::abs(height(rotated, 0, 1.5) - 3) < 1e-7,
          "Opening did not rotate with its slab");
  options.scene["objects"][1].erase("opening");
  options.scene["objects"][1]["position"] = {30, 0, -.1};
  options.scene["objects"].push_back({{"id", "turn"},
                                      {"name", "Turn"},
                                      {"floor", "ground"},
                                      {"kind", "stairs"},
                                      {"position", {0, 0, 0}},
                                      {"quaternion", {1, 0, 0, 0}},
                                      {"size", {4.2, 1.2, .2}},
                                      {"connection",
                                       {{"to_floor", "upper"},
                                        {"steps", 15},
                                        {"turn_deg", 90},
                                        {"landing_m", 1.2}}}});
  for (int turn : {90, -90, 180}) {
    options.scene["objects"][2]["connection"]["turn_deg"] = turn;
    Simulation sim(options);
    require(std::abs(height(sim, 2.56, 0) - 1.4) < 1e-7,
            "Landing elevation incorrect");
    double x = turn == 180 ? -.14 : 2.56, y = turn == 180  ? 1.2
                                              : turn == 90 ? 2.7
                                                           : -2.7;
    require(std::abs(height(sim, x, y) - 3) < 1e-7,
            "Turned stairs miss the upper elevation");
  }
  options.scene["objects"][2]["kind"] = "ramp";
  options.scene["objects"][2]["size"] = {12, 2, .2};
  options.scene["objects"][2]["connection"]["turn_deg"] = 90;
  options.scene["objects"][2]["connection"]["landing_m"] = 2;
  Simulation ramp(options);
  require(std::abs(height(ramp, 3, 0) - .75) < 1e-7 &&
              std::abs(height(ramp, 7, 0) - 1.5) < 1e-7 &&
              std::abs(height(ramp, 7, 4) - 2.25) < 1e-7,
          "Turning ramp/landing is discontinuous");
  std::cout << "Slab openings, rotated cuts, three stair turns and turning "
               "ramp passed\n";
}
int main() {
  try {
    check_connections();
    check_openings_and_landings();
    for (auto robot : {"turtlebot3_waffle_pi", "go2w"}) {
      SimulationOptions o;
      o.project_root = SIMPLE_RBT_PROJECT_ROOT;
      o.robot = robot;
      o.scene = scene();
      Simulation sim(o);
      require(sim.pose().position[2] > 3, "Upper-floor spawn was ignored");
      int geom = -1;
      mjtNum origin[3]{2, 0, 4}, dir[3]{0, 0, -1};
      mjtByte groups[6]{0, 1, 0, 0, 0, 0};
      double d = mj_ray(sim.model(), sim.data(), origin, dir, groups, 1, -1,
                        &geom, nullptr);
      require(std::abs(d - 1) < 1e-8,
              "Physical upper slab missing or at wrong height");
      origin[2] = 1;
      dir[2] = 1;
      d = mj_ray(sim.model(), sim.data(), origin, dir, groups, 1, -1, &geom,
                 nullptr);
      require(std::abs(d - 1.8) < 1e-8,
              "Lower floor cannot see the underside of upper floor");
      auto start = sim.pose().position;
      for (int i = 0; i < 1500; i++) {
        sim.set_command(i < 500 ? Vec3{} : Vec3{.2, 0, 0});
        sim.step();
        require(sim.pose().position[2] > 2.9, "Robot fell through upper slab");
      }
      require(std::hypot(sim.pose().position[0] - start[0],
                         sim.pose().position[1] - start[1]) > .1,
              "Commands did not move the robot on upper floor");
      CsvPattern pattern(std::string(SIMPLE_RBT_PROJECT_ROOT) +
                         "/assets/lidar/mid360-real-centr.csv");
      RollingLidar lidar(sim.model(), LidarBackend::Cpu, pattern);
      PoseHistory history;
      Pose pose;
      pose.position = {2, 0, 1};
      history.add(0, pose);
      history.add(100000000, pose);
      LidarScan scan;
      lidar.scan(sim.data(), 0, 20000, history, scan);
      int ceiling = 0, ground = 0;
      for (const auto &p : scan.xyz) {
        if (std::abs(p[2] - 1.8) < 1e-4)
          ceiling++;
        if (std::abs(p[2] + 1) < 1e-4)
          ground++;
      }
      require(ceiling > 50 && ground > 50,
              "MID360 does not observe both floor and ceiling");
      auto bad = o;
      bad.scene["objects"][0]["size"][0] = -1;
      bool rejected = false;
      try {
        Simulation invalid(bad);
      } catch (const std::exception &) {
        rejected = true;
      }
      require(rejected, "Invalid scene accepted");
      std::cout << robot
                << ": upper-floor motion, floor/ceiling collision, MID360 and "
                   "invalid scene rejection passed\n";
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
