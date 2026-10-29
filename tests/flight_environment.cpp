#include "simple_rbt/core.hpp"
#include <cmath>
#include <iostream>
using namespace simple_rbt;
namespace {
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
void advance(Simulation &s, const FlightCommand &c, double seconds) {
  for (int i = 0; i < seconds / s.model()->opt.timestep; ++i) {
    s.set_flight(c);
    s.step();
  }
}
Json room(double height, double ceiling) {
  auto box = [](const char *id, Vec3 position, Vec3 size) {
    return Json{
        {"id", id},
        {"name", id},
        {"kind", "primitive"},
        {"parent", "world"},
        {"pose", {{"position_m", position}, {"quaternion_wxyz", {1, 0, 0, 0}}}},
        {"geometry",
         {{"type", "box"},
          {"length_m", size[0]},
          {"width_m", size[1]},
          {"height_m", size[2]}}},
        {"evidence", {{"basis", "manual"}}}};
  };
  Json objects =
      Json::array({box("floor", {0, 0, height - .1}, {8, 8, .2}),
                   box("ceiling", {0, 0, height + ceiling + .1}, {8, 8, .2}),
                   box("wall", {1.5, 0, height + 1.5}, {.2, 8, 3})});
  objects.push_back(
      {{"id", "spawn"},
       {"name", "spawn"},
       {"kind", "group"},
       {"semantic", "spawn"},
       {"parent", "world"},
       {"pose",
        {{"position_m", {0, 0, height}}, {"quaternion_wxyz", {1, 0, 0, 0}}}}});
  Json scene{{"format", "sprobo.scene"},
             {"version", 2},
             {"id", "flight-room"},
             {"name", "Flight room"},
             {"units", {{"length", "m"}, {"angle", "rad"}}},
             {"coordinate_system", "right_handed_z_up"},
             {"objects", objects}};
  return scene;
}
void check_export(const Simulation &sim) {
  const auto xml = sim.xml();
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "export.xml", xml.data(), xml.size());
  char error[1024]{};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> restored(
      mj_loadXML("export.xml", &vfs, error, sizeof(error)), mj_deleteModel);
  mj_deleteVFS(&vfs);
  check(bool(restored), error);
  std::unique_ptr<mjData, decltype(&mj_deleteData)> original(
      mj_makeData(sim.model()), mj_deleteData),
      replay(mj_makeData(restored.get()), mj_deleteData);
  mj_copyData(original.get(), sim.model(), sim.data());
  mj_copyData(replay.get(), restored.get(), sim.data());
  for (int step = 0; step < 200; ++step) {
    mj_step(sim.model(), original.get());
    mj_step(restored.get(), replay.get());
  }
  double difference = 0;
  for (int i = 0; i < sim.model()->nq; ++i)
    difference =
        std::max(difference, std::abs(original->qpos[i] - replay->qpos[i]));
  check(difference < 1e-8,
        "exported payload model does not reproduce physical motion");
}
} // namespace
int main() {
  try {
    SimulationOptions o;
    o.project_root = SIMPLE_RBT_PROJECT_ROOT;
    o.robot = "skydio_x2";
    o.scene_spawn = "spawn";
    FlightCommand takeoff;
    takeoff.action = "takeoff";
    o.scene = room(3., 3.);
    Simulation upper(o);
    std::cerr << "upper initial " << upper.pose().position[2] << "\n";
    advance(upper, {}, 1);
    advance(upper, takeoff, 8);
    std::cerr << "upper after " << upper.pose().position[2] << " "
              << upper.flight_status() << "\n";
    check(std::abs(upper.pose().position[2] - 4.) < .08,
          "takeoff is not relative to upper floor");
    o.scene = room(0., .8);
    Simulation low(o);
    advance(low, {}, 1);
    advance(low, takeoff, 2);
    check(low.flight_status().at("state") == "takeoff_blocked",
          "low ceiling allowed takeoff");
    check(low.pose().position[2] < .04, "blocked vehicle left floor");
    o.scene = room(0., 3.);
    o.lidar_payload_enabled = false;
    o.camera_payload_enabled = false;
    Simulation dry(o);
    const double dry_mass = dry.flight_status().at("mass_kg");
    o.lidar_payload_enabled = true;
    o.camera_payload_enabled = true;
    o.lidar_model = LidarModel::Avia;
    Simulation loaded(o);
    auto mount = loaded.extrinsics("lidar");
    mount.position = {.1, .08, .25};
    loaded.set_mount("mid360", mount);
    check(loaded.flight_status().at("mass_kg").get<double>() > dry_mass + .5,
          "payload mass omitted");
    advance(loaded, {}, 1);
    advance(loaded, takeoff, 10);
    advance(loaded, {}, 10);
    check(std::abs(loaded.pose().position[2] - 1.) < .08,
          "offset payload cannot hover");
    check_export(loaded);
    FlightCommand move;
    move.action = "velocity";
    move.velocity = {.8, 0, 0};
    bool rotor_contact = false;
    double max_x = 0;
    const int wall = mj_name2id(loaded.model(), mjOBJ_GEOM, "scene_object_2");
    check(wall >= 0, "wall collision geometry missing");
    for (int i = 0; i < 1200; ++i) {
      loaded.set_flight(move);
      loaded.step();
      max_x = std::max(max_x, loaded.pose().position[0]);
      for (int j = 0; j < loaded.data()->ncon; ++j) {
        const auto &c = loaded.data()->contact[j];
        if (c.geom[0] == wall || c.geom[1] == wall)
          rotor_contact = true;
      }
    }
    check(rotor_contact, "physical wall contact missing");
    check(max_x < 1.45, "quadrotor passed through wall");
    std::cout << Json{{"passed", true},
                      {"upper_hover_z", upper.pose().position[2]},
                      {"ceiling_state", low.flight_status().at("state")},
                      {"dry_mass_kg", dry_mass},
                      {"offset_payload_mass_kg",
                       loaded.flight_status().at("mass_kg")},
                      {"wall_max_x_m", max_x}}
              << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
