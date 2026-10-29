#include "sensor_devices.hpp"
#include <algorithm>
#include <cmath>
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
  if (o.external_camera_enabled) {
    auto *mount = doc.NewElement("site");
    mount->SetAttribute("name", "external_camera_mount");
    mount->SetAttribute("pos", "1.2 -1.2 1.1");
    const double pitch = 20. * std::acos(-1.) / 360.,
                 yaw = 135. * std::acos(-1.) / 360.;
    const double q[]{
        std::cos(pitch) * std::cos(yaw), -std::sin(pitch) * std::sin(yaw),
        std::sin(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw)};
    mount->SetAttribute("quat", vector(q, 4).c_str());
    mount->SetAttribute("size", ".001");
    root->FirstChildElement("worldbody")->InsertEndChild(mount);
  }
  if (!assets) {
    assets = doc.NewElement("asset");
    root->InsertEndChild(assets);
  }
  Json manifest;
  std::ifstream file(o.project_root / "assets/visual/sensors/provenance.json");
  if (!file || !(file >> manifest))
    throw std::runtime_error("Sensor visual manifest missing");
  if (o.lidar_model == LidarModel::Avia) {
    Json avia;
    std::ifstream source(o.project_root / "assets/visual/sensors/avia-provenance.json");
    if (!source || !(source >> avia))
      throw std::runtime_error("Avia visual manifest missing");
    for (const auto &part : avia.at("parts")) manifest["parts"].push_back(part);
  }
  auto visit = [&](auto &&self, tinyxml2::XMLElement *parent) -> void {
    for (auto *e = parent->FirstChildElement(); e;
         e = e->NextSiblingElement()) {
      const char *name = e->Attribute("name");
      if (std::string(e->Name()) == "site" && name &&
          (std::string(name) == "lidar" ||
           std::string(name) == "camera_mount" ||
           std::string(name) == "external_camera_mount")) {
        const bool external = std::string(name) == "external_camera_mount";
        const bool camera = std::string(name) != "lidar";
        e->SetAttribute(
            "rgba", "0 0 0 0"); // The device mesh replaces the editing marker.
        if(robot_spec(o.robot).flight_control && !external) {
          auto *payload=doc.NewElement("geom");
          payload->SetAttribute("name",camera?"payload_camera":"payload_lidar");
          payload->SetAttribute("type","box");
          payload->SetAttribute("size",camera?".045 .015 .015":
              o.lidar_model==LidarModel::Avia?".045 .045 .035":".033 .033 .03");
          payload->SetAttribute("mass",0);payload->SetAttribute("group",3);
          payload->SetAttribute("rgba","0 0 0 0");parent->InsertEndChild(payload);
        }
        for (const auto &part : manifest.at("parts")) {
          const std::string part_name = part.at("part");
          const std::string device = camera ? "d435" : lidar_spec(o.lidar_model).name;
          if (part_name.rfind(device, 0) != 0) continue;
          for (std::size_t i = 0; i < part.at("files").size(); ++i) {
            const auto &entry = part.at("files")[i];
            const std::string mesh_name =
                std::string(external ? "external_device_" : "device_") +
                part_name + "_" + std::to_string(i);
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
            s->SetAttribute(
                "name",
                (std::string(external ? "external_" : "") + child).c_str());
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
  if(robot_spec(o.robot).flight_control && device!="external_camera") {
    const int g=mj_name2id(m,mjOBJ_GEOM,device=="mid360"?"payload_lidar":"payload_camera");
    if(g>=0) {
      m->geom_sameframe[g]=mjSAMEFRAME_NONE;
      auto p=mount;
      if(device=="mid360")p=compose(mount,Pose{{0,0,-.03}});
      std::copy(p.position.begin(),p.position.end(),m->geom_pos+3*g);
      std::copy(p.quaternion.begin(),p.quaternion.end(),m->geom_quat+4*g);
    }
  }
  const std::string prefix = device == "external_camera"
                                 ? "visual_external_device_d435"
                             : device == "camera" ? "visual_device_d435"
                                                  : std::string("visual_device_") + lidar_spec(o.lidar_model).name;
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
  } else if (device == "external_camera") {
    site(m, "external_camera_color", compose(mount, o.camera_color_offset));
    site(m, "external_camera_depth", compose(mount, o.camera_depth_offset));
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
  if (!sim.joint_controls().empty())
    x.camera_parent_frame = "arm_tool";
  if (mj_name2id(sim.model(), mjOBJ_SITE, "external_camera_mount") >= 0) {
    x.has_external_camera = true;
    x.external_camera = optical(sim.pose("external_camera_color"));
    x.external_depth = optical(sim.pose("external_camera_depth"));
  }
  return x;
}
} // namespace simple_rbt
