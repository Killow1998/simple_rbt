#include "sensor_devices.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
namespace simple_rbt {
namespace {
std::string vector(const double *v, int count) {
  std::ostringstream s;
  s << std::setprecision(17);
  for (int i = 0; i < count; ++i) {
    if (i)
      s << ' ';
    s << v[i];
  }
  return s.str();
}
Pose compose(const Pose &a, const Pose &b) {
  Pose p;
  Vec3 shift;
  mju_rotVecQuat(shift.data(), b.position.data(), a.quaternion.data());
  for (int i = 0; i < 3; ++i)
    p.position[i] = a.position[i] + shift[i];
  mju_mulQuat(p.quaternion.data(), a.quaternion.data(), b.quaternion.data());
  return p;
}
Pose optical(Pose p) {
  const Quat rotation{.5, -.5, .5,
                      -.5}; // optical +X right, +Y down, +Z forward
  Quat q;
  mju_mulQuat(q.data(), p.quaternion.data(), rotation.data());
  p.quaternion = q;
  return p;
}
void site(mjModel *m, const char *name, const Pose &p) {
  const int id = mj_name2id(m, mjOBJ_SITE, name);
  if (id < 0)
    throw std::runtime_error(std::string("Missing device site: ") + name);
  m->site_sameframe[id] = mjSAMEFRAME_NONE;
  std::copy(p.position.begin(), p.position.end(), m->site_pos + 3 * id);
  std::copy(p.quaternion.begin(), p.quaternion.end(), m->site_quat + 4 * id);
}
} // namespace
void add_sensor_devices(tinyxml2::XMLDocument &doc,
                        const SimulationOptions &o) {
  if (o.camera_model != "d435" && o.camera_model != "d435i")
    throw std::invalid_argument("camera_model must be d435 or d435i");
  auto *root = doc.RootElement(), *assets = root->FirstChildElement("asset");
  if (!assets) {
    assets = doc.NewElement("asset");
    root->InsertEndChild(assets);
  }
  Json manifest;
  std::ifstream file(o.project_root / "assets/visual/sensors/provenance.json");
  if (!file || !(file >> manifest))
    throw std::runtime_error("Sensor visual manifest missing");
  auto visit = [&](auto &&self, tinyxml2::XMLElement *parent) -> void {
    for (auto *e = parent->FirstChildElement(); e;
         e = e->NextSiblingElement()) {
      const char *name = e->Attribute("name");
      if (std::string(e->Name()) == "site" && name &&
          (std::string(name) == "lidar" ||
           std::string(name) == "camera_mount")) {
        const bool camera = std::string(name) == "camera_mount";
        e->SetAttribute(
            "rgba", "0 0 0 0"); // The device mesh replaces the editing marker.
        for (const auto &part : manifest.at("parts")) {
          const std::string part_name = part.at("part");
          if (camera != (part_name.rfind("d435", 0) == 0))
            continue;
          for (std::size_t i = 0; i < part.at("files").size(); ++i) {
            const auto &entry = part.at("files")[i];
            const std::string mesh_name =
                "device_" + part_name + "_" + std::to_string(i);
            auto *mesh = doc.NewElement("mesh");
            mesh->SetAttribute("name", mesh_name.c_str());
            const auto path =
                o.project_root / entry.at("file").get<std::string>();
            if (sha256_file(path) != entry.at("sha256").get<std::string>())
              throw std::runtime_error("Sensor mesh checksum mismatch");
            mesh->SetAttribute("file", path.c_str());
            assets->InsertEndChild(mesh);
            auto *g = doc.NewElement("geom");
            g->SetAttribute("name", ("visual_" + mesh_name).c_str());
            g->SetAttribute("type", "mesh");
            g->SetAttribute("mesh", mesh_name.c_str());
            const auto rgba = entry.at("rgba").get<std::array<double, 4>>();
            g->SetAttribute("rgba", vector(rgba.data(), 4).c_str());
            g->SetAttribute("group", "2");
            g->SetAttribute("contype", 0);
            g->SetAttribute("conaffinity", 0);
            g->SetAttribute("mass", 0);
            g->SetAttribute("density", 0);
            parent->InsertEndChild(g);
          }
        }
        if (camera) {
          for (const auto *child :
               {"camera_color", "camera_depth", "camera_imu"}) {
            auto *s = doc.NewElement("site");
            s->SetAttribute("name", child);
            s->SetAttribute("size", ".001");
            s->SetAttribute("rgba", "0 0 0 0");
            parent->InsertEndChild(s);
          }
        }
      }
      self(self, e);
    }
  };
  visit(visit, root->FirstChildElement("worldbody"));
  if (o.camera_model == "d435i") {
    auto *sensors = root->FirstChildElement("sensor");
    if (!sensors) {
      sensors = doc.NewElement("sensor");
      root->InsertEndChild(sensors);
    }
    for (const auto *type : {"gyro", "accelerometer"}) {
      auto *s = doc.NewElement(type);
      s->SetAttribute("name", (std::string("camera_") + type).c_str());
      s->SetAttribute("site", "camera_imu");
      sensors->InsertEndChild(s);
    }
  }
}
void move_sensor_device(mjModel *m, const std::string &device,
                        const Pose &mount, const SimulationOptions &o) {
  const std::string prefix =
      device == "camera" ? "visual_device_d435" : "visual_device_mid360";
  for (int id = 0; id < m->ngeom; ++id) {
    const char *name = mj_id2name(m, mjOBJ_GEOM, id);
    if (name && std::string(name).rfind(prefix, 0) == 0) {
      m->geom_sameframe[id] = mjSAMEFRAME_NONE;
      Pose mesh;
      const int mesh_id = m->geom_dataid[id];
      std::copy_n(m->mesh_pos + 3 * mesh_id, 3, mesh.position.begin());
      std::copy_n(m->mesh_quat + 4 * mesh_id, 4, mesh.quaternion.begin());
      const auto placed =
          compose(mount, mesh); // Preserve MuJoCo mesh centering.
      std::copy(placed.position.begin(), placed.position.end(),
                m->geom_pos + id * 3);
      std::copy(placed.quaternion.begin(), placed.quaternion.end(),
                m->geom_quat + id * 4);
    }
  }
  if (device == "camera") {
    site(m, "camera_color", compose(mount, o.camera_color_offset));
    site(m, "camera_depth", compose(mount, o.camera_depth_offset));
    site(m, "camera_imu", optical(compose(mount, o.camera_imu_offset)));
  }
}
Extrinsics sensor_extrinsics(Simulation &sim, bool camera) {
  Extrinsics x;
  x.lidar = sim.extrinsics("lidar");
  x.imu = sim.extrinsics("mid360_imu");
  x.camera = optical(sim.extrinsics("camera_color"));
  x.camera_depth = optical(sim.extrinsics("camera_depth"));
  x.camera_imu = sim.extrinsics("camera_imu");
  x.has_camera = camera || sim.has_camera_imu();
  x.separate_camera_frames = true;
  x.has_camera_imu = sim.has_camera_imu();
  return x;
}
} // namespace simple_rbt
