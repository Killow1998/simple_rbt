#include "../src/lidar_scene.hpp"
#include "simple_rbt/core.hpp"
#include "simple_rbt/lidar.hpp"
#include "simple_rbt/scene_geometry.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
using namespace simple_rbt;
namespace {
void check(bool ok, const std::string &message) {
  if (!ok)
    throw std::runtime_error(message);
}
double ray(Simulation &sim, Vec3 origin, Vec3 direction) {
  mjtByte groups[6]{0, 1, 0, 0, 0, 0};
  int hit = -1;
  return mj_ray(sim.model(), sim.data(), origin.data(), direction.data(),
                groups, 1, -1, &hit, nullptr);
}
void check_ray_index(Simulation &sim) {
  std::vector<int> ids;
  for (int g = 0; g < sim.model()->ngeom; ++g)
    if (sim.model()->geom_group[g] == 1)
      ids.push_back(g);
  detail::StaticRayIndex index(sim.model(), sim.data(), ids);
  std::mt19937 random(1701);
  std::uniform_real_distribution<double> u(-1, 1);
  for (int i = 0; i < 20000; ++i) {
    Vec3 origin{20 + 25 * u(random), 5 * u(random), 3 + 5 * u(random)},
        direction{u(random), u(random), u(random)};
    mju_normalize3(direction.data());
    double expected = ray(sim, origin, direction),
           actual = index.ray(origin.data(), direction.data(), 100);
    if (expected >= 100)
      expected = -1;
    check(std::abs(expected - actual) < 1e-10,
          "accelerated ray differs from MuJoCo");
  }
}
void rejects(Json scene, const std::string &why) {
  bool rejected = false;
  try {
    resolve_structured_scene(scene);
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected, "accepted invalid " + why);
}
Json primitive(std::string id, Json geometry, Vec3 position) {
  return {
      {"id", id},
      {"name", id},
      {"kind", "primitive"},
      {"parent", "world"},
      {"pose", {{"position_m", position}, {"quaternion_wxyz", {1, 0, 0, 0}}}},
      {"geometry", geometry},
      {"evidence", {{"basis", "manual"}}}};
}
} // namespace
int main() {
  try {
    Json scene;
    std::ifstream(std::string(SIMPLE_RBT_PROJECT_ROOT) +
                  "/schemas/examples/structured-building.scene.json") >>
        scene;
    auto compiled = resolve_structured_scene(scene);
    check(compiled.at("geometry_count") == 28,
          "unexpected convex decomposition");
    SimulationOptions options;
    options.project_root = SIMPLE_RBT_PROJECT_ROOT;
    options.robot = "turtlebot3_waffle_pi";
    options.scene = scene;
    Simulation sim(options);
    check_ray_index(sim);
    check(std::abs(ray(sim, {0, 0, 4}, {0, 0, -1}) - 2.8) < 1e-6,
          "hole closed or stair tread wrong");
    check(std::abs(ray(sim, {3, -2, 4}, {0, 0, -1}) - 1) < 1e-6,
          "upper slab not at parent elevation");
    check(std::abs(ray(sim, {3, -2, 2}, {0, 0, -1}) - 2) < 1e-6,
          "lower slab elevation incorrect");
    for (int i = 0; i < 10; ++i) {
      double x = -2 + (i + .5) * .28, z = (i + 1) * .15;
      check(std::abs(ray(sim, {x, 0, 2.8}, {0, 0, -1}) - (2.8 - z)) < 1e-6,
            "tread dimensions incorrect");
    }
    // The underside is above this horizontal ray. Filled legacy step boxes
    // would hit, while a correctly suspended flight leaves the space open.
    check(ray(sim, {-.1, -.7, .2}, {0, 1, 0}) > 1.4,
          "stairs fill the underside void");
    const double x = -.6, pitch = .15 / .28,
                 underside = pitch * (x + 2) - .16 * std::hypot(1., pitch);
    check(std::abs(ray(sim, {x, 0, .01}, {0, 0, 1}) - (underside - .01)) < 1e-6,
          "stair underside thickness incorrect");
    check(std::abs(sim.pose().position[0] - 3) < 1e-8 &&
              std::abs(sim.pose().position[1] + 2) < 1e-8,
          "spawn transform incorrect");
    int contactSteps = 0;
    for (int i = 0; i < 1000; ++i) {
      sim.step();
      contactSteps += sim.data()->ncon > 0;
    }
    check(contactSteps > 500 && sim.pose().position[2] > -.01 &&
              sim.pose().position[2] < .5,
          "robot did not settle on floor: contacts=" +
              std::to_string(contactSteps) +
              " z=" + std::to_string(sim.pose().position[2]));
    CsvPattern pattern(std::string(SIMPLE_RBT_PROJECT_ROOT) +
                       "/assets/lidar/mid360-real-centr.csv");
    RollingLidar lidar(sim.model(), LidarBackend::Cpu, pattern);
    PoseHistory history;
    history.add(sim.stamp_ns(), sim.pose("lidar"));
    const auto start = sim.stamp_ns();
    while (sim.stamp_ns() - start < 100000000) {
      sim.step();
      history.add(sim.stamp_ns(), sim.pose("lidar"));
    }
    LidarScan scan;
    lidar.scan(sim.data(), start, 20000, history, scan);
    check(scan.rays_cast == 20000 && scan.xyz.size() > 5000,
          "structured rolling LiDAR failed");
    Json bad = scene;
    bad["objects"][0]["parent"] = bad["objects"][0]["id"];
    rejects(bad, "parent cycle");
    bad = scene;
    bad["objects"][0]["pose"]["quaternion_wxyz"] = {2, 0, 0, 0};
    rejects(bad, "quaternion");
    bad = scene;
    bad["geometry_templates"].begin().value()["steps"] = 4097;
    rejects(bad, "expanded geometry budget");
    bad = scene;
    bad["objects"].push_back(
        primitive("bad",
                  {{"type", "planar_region"},
                   {"outer_xy_m", {{0, 0}, {3, 0}, {3, 3}, {0, 3}}},
                   {"holes_xy_m", {{{2, 2}, {2, 4}, {4, 4}, {4, 2}}}},
                   {"thickness_m", .2}},
                  {20, 0, 0}));
    rejects(bad, "crossing hole");
    Json shapes = scene;
    shapes["objects"] = Json::array();
    shapes["geometry_templates"] = Json::object();
    std::vector<Json> geometries{
        {{"type", "box"}, {"length_m", 2}, {"width_m", 2}, {"height_m", 2}},
        {{"type", "cylinder"}, {"radius_m", 1}, {"height_m", 2}},
        {{"type", "sphere"}, {"radius_m", 1}},
        {{"type", "ellipsoid"}, {"radii_m", {1, 1, 1}}},
        {{"type", "frustum"},
         {"bottom_radius_m", 1},
         {"top_radius_m", .5},
         {"height_m", 2}},
        {{"type", "capsule"}, {"radius_m", .5}, {"cylinder_length_m", 1}},
        {{"type", "ramp"},
         {"width_m", 2},
         {"run_m", 2},
         {"rise_m", 1},
         {"slab_thickness_m", .2}}};
    for (std::size_t i = 0; i < geometries.size(); ++i)
      shapes["objects"].push_back(primitive(
          "shape_" + std::to_string(i), geometries[i], {10. + i * 4, 0, 0}));
    options.scene = shapes;
    Simulation all(options);
    check_ray_index(all);
    for (int i = 0; i < 6; ++i)
      check(std::abs(ray(all, {10. + i * 4, 0, 3}, {0, 0, -1}) - 2) < 1e-6,
            "curved primitive ray mismatch");
    check(std::abs(ray(all, {35, 0, 3}, {0, 0, -1}) - 2.5) < 1e-6,
          "ramp top incorrect");
    check(ray(all, {33.99, 0, 3}, {0, 0, -1}) < 0,
          "ramp extends outside horizontal bounds");
    RollingLidar curved(all.model(), LidarBackend::Cpu, pattern);
    std::cout << Json{{"passed", true},
                      {"convex_parts", compiled.at("geometry_count")},
                      {"rays", scan.rays_cast},
                      {"returns", scan.xyz.size()},
                      {"contact_steps", contactSteps}}
                     .dump()
              << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
