#include "simple_rbt/cameras.hpp"
#include "simple_rbt/core.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
using namespace simple_rbt;
namespace {
void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
void close(double a, double b, double tolerance = 1e-7) {
  if (std::abs(a - b) >= tolerance)
    throw std::runtime_error("device transform mismatch: " + std::to_string(a) +
                             " vs " + std::to_string(b));
}
void offset(Simulation &sim, const char *site, const Pose &mount,
            const Vec3 &expected) {
  const auto actual = sim.extrinsics(site);
  Vec3 rotated;
  mju_rotVecQuat(rotated.data(), expected.data(), mount.quaternion.data());
  for (int i = 0; i < 3; ++i)
    close(actual.position[i], mount.position[i] + rotated[i]);
}
void geometry(Simulation &sim) {
  Json manifest;
  std::ifstream(std::filesystem::path(SIMPLE_RBT_PROJECT_ROOT) /
                "assets/visual/sensors/provenance.json") >>
      manifest;
  if (std::string(sim.lidar_device().name) == "avia") {
    Json avia;
    std::ifstream(std::filesystem::path(SIMPLE_RBT_PROJECT_ROOT) /
                  "assets/visual/sensors/avia-provenance.json") >> avia;
    for (const auto &part : avia.at("parts")) manifest["parts"].push_back(part);
  }
  for (const auto &part : manifest.at("parts")) {
    const std::string name = part.at("part");
    if (name.rfind("d435", 0) != 0 && name.rfind(sim.lidar_device().name, 0) != 0) continue;
    for (std::size_t i = 0; i < part.at("files").size(); ++i) {
      const std::string name = "visual_device_" +
                               part.at("part").get<std::string>() + "_" +
                               std::to_string(i);
      const auto *m = sim.model();
      const auto *d = sim.data();
      const int g = mj_name2id(m, mjOBJ_GEOM, name.c_str());
      require(g >= 0, "missing official device geometry");
      const int mesh = m->geom_dataid[g];
      require(g >= 0 && m->geom_contype[g] == 0 && m->geom_conaffinity[g] == 0,
              "visual device affects contacts");
      const auto mount = sim.pose(
          name.find("d435") != std::string::npos ? "camera_mount" : "lidar");
      Quat inverse;
      mju_negQuat(inverse.data(), mount.quaternion.data());
      Vec3 lo{1e10, 1e10, 1e10}, hi{-1e10, -1e10, -1e10};
      for (int v = m->mesh_vertadr[mesh];
           v < m->mesh_vertadr[mesh] + m->mesh_vertnum[mesh]; ++v) {
        Vec3 vertex{}, world{}, local{};
        for (int a = 0; a < 3; ++a)
          vertex[a] = m->mesh_vert[3 * v + a];
        mju_mulMatVec3(world.data(), d->geom_xmat + 9 * g, vertex.data());
        for (int a = 0; a < 3; ++a)
          world[a] += d->geom_xpos[3 * g + a] - mount.position[a];
        mju_rotVecQuat(local.data(), world.data(), inverse.data());
        for (int a = 0; a < 3; ++a) {
          lo[a] = std::min(lo[a], local[a]);
          hi[a] = std::max(hi[a], local[a]);
        }
      }
      const auto bounds = part.at("files")[i].at("bounds_m");
      for (int a = 0; a < 3; ++a) {
        close(lo[a], bounds[0][a], 2e-6);
        close(hi[a], bounds[1][a], 2e-6);
      }
    }
  }
}
} // namespace
int main(int argc, char **argv) {
  try {
    SimulationOptions o;
    o.project_root = SIMPLE_RBT_PROJECT_ROOT;
    o.robot = "ranger_mini";
    o.camera_model = "d435i";
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--avia") o.lidar_model = LidarModel::Avia;
    const Vec3 lidar_offset = o.lidar_model == LidarModel::Avia
        ? Vec3{-.04165, -.02326, .02840} : Vec3{.011, .02329, -.04412};
    Simulation sim(o);
    const auto body = sim.extrinsics("imu");
    const std::vector<double> masses(
        sim.model()->body_mass, sim.model()->body_mass + sim.model()->nbody);
    Pose mount{{.24, -.07, .36},
               {.8660254037844386, .28867513459481287, .28867513459481287,
                .28867513459481287}};
    for (const auto &p : {Pose{{.24, 0, .36}}, mount}) {
      sim.set_mount("camera", p);
      sim.set_mount("mid360", p);
      offset(sim, "camera_color", p, {0, .015, 0});
      offset(sim, "camera_depth", p, {});
      offset(sim, "camera_imu", p, {-.01174, -.00552, .0051});
      offset(sim, "mid360_imu", p, lidar_offset);
      geometry(sim);
      require(sim.extrinsics("imu").position == body.position &&
                  sim.extrinsics("imu").quaternion == body.quaternion,
              "device mount moved body IMU");
      require(std::equal(masses.begin(), masses.end(), sim.model()->body_mass),
              "device changed body mass");
    }
    // A saved scene must reproduce the current device geometry and sample
    // sites.
    const auto path =
        std::filesystem::temp_directory_path() /
        ("simple-rbt-device-roundtrip-" + std::to_string(getpid()) + ".xml");
    std::ofstream(path) << sim.xml();
    char error[1024];
    auto *copy = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    std::filesystem::remove(path);
    require(copy != nullptr, error);
    for (const char *name :
         {"camera_color", "camera_depth", "camera_imu", "mid360_imu"}) {
      const int a = mj_name2id(sim.model(), mjOBJ_SITE, name),
                b = mj_name2id(copy, mjOBJ_SITE, name);
      for (int axis = 0; axis < 3; ++axis)
        close(sim.model()->site_pos[3 * a + axis],
              copy->site_pos[3 * b + axis]);
      for (int axis = 0; axis < 4; ++axis)
        close(sim.model()->site_quat[4 * a + axis],
              copy->site_quat[4 * b + axis]);
    }
    for (int g = 0; g < sim.model()->ngeom; ++g) {
      const char *name = mj_id2name(sim.model(), mjOBJ_GEOM, g);
      if (!name || std::string(name).rfind("visual_device_", 0) != 0)
        continue;
      const int other = mj_name2id(copy, mjOBJ_GEOM, name);
      for (int a = 0; a < 3; ++a)
        close(sim.model()->geom_pos[3 * g + a], copy->geom_pos[3 * other + a]);
      for (int a = 0; a < 4; ++a)
        close(sim.model()->geom_quat[4 * g + a],
              copy->geom_quat[4 * other + a]);
    }
    mj_deleteModel(copy);
    // At rest, independent IMUs agree after rotation into world coordinates.
    for (int i = 0; i < 1500; ++i)
      sim.step();
    const auto camera = sim.camera_imu(), lidar = sim.imu();
    Vec3 ca{}, la{};
    mju_rotVecQuat(ca.data(), camera.specific_force.data(),
                   sim.pose("camera_imu").quaternion.data());
    mju_rotVecQuat(la.data(), lidar.specific_force.data(),
                   sim.pose("mid360_imu").quaternion.data());
    for (int i = 0; i < 3; ++i) {
      close(ca[i], i == 2 ? 9.80665 : 0., .03);
      close(la[i], ca[i], .03);
    }
    require(camera.stamp_ns == sim.stamp_ns(),
            "camera IMU timestamp differs from physics");
    require(camera.specific_force != lidar.specific_force,
            "camera IMU reused lidar axes");
    o.camera_model = "d435";
    Simulation plain(o);
    require(!plain.has_camera_imu(), "D435 unexpectedly contains an IMU");
    bool rejected = false;
    try {
      plain.camera_imu();
    } catch (const std::exception &) {
      rejected = true;
    }
    require(rejected, "D435 exposed camera IMU");
    o.camera_model = "d435i";
    o.camera_imu_offset.position = {-.012, -.006, .005};
    Simulation calibrated(o);
    offset(calibrated, "camera_imu", calibrated.extrinsics("camera_mount"),
           o.camera_imu_offset.position);
    if (argc > 1 && std::string(argv[1]) == "--render") {
      CameraConfig config;
      config.render_panorama = false;
      CameraRig rig(sim.model(), config);
      CameraPair split, color, depth;
      rig.render_into(sim.data(), sim.camera_pose(),
                      sim.camera_pose("camera_depth"), split);
      rig.render_into(sim.data(), sim.camera_pose(), color);
      rig.render_into(sim.data(), sim.camera_pose("camera_depth"), depth);
      require(split.perspective.rgb == color.perspective.rgb,
              "RGB sample origin changed");
      require(split.perspective.depth == depth.perspective.depth,
              "Depth does not use its own optical origin");
      require(color.perspective.depth != depth.perspective.depth,
              "Separate camera origins have no observable effect");
      std::cout << "Renderer: " << rig.renderer_name() << '\n';
    }
    std::cout << "PASS: official geometry, rigid mounts, independent IMU, "
                 "calibration and scene roundtrip\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
