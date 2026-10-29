#include "model_export.hpp"
#include <iomanip>
#include <sstream>
#include <tinyxml2.h>
namespace simple_rbt {
std::string export_model(const std::string &scene_xml, const mjModel *model,
                         bool flight) {
  tinyxml2::XMLDocument doc;
  doc.Parse(scene_xml.c_str());
  auto numbers = [](const double *v, int count) {
    std::ostringstream out;
    out << std::setprecision(17);
    for (int i = 0; i < count; ++i) {
      if (i)
        out << ' ';
      out << v[i];
    }
    return out.str();
  };
  auto visit = [&](auto &&self, tinyxml2::XMLElement *e) -> void {
    if (flight && std::string(e->Name()) == "body" &&
        e->FirstChildElement("freejoint")) {
      const int body = mj_name2id(model, mjOBJ_BODY, e->Attribute("name"));
      auto *inertial = e->FirstChildElement("inertial");
      if (!inertial) {
        inertial = doc.NewElement("inertial");
        e->InsertFirstChild(inertial);
      }
      inertial->DeleteAttribute("fullinertia");
      inertial->SetAttribute("mass",
                             numbers(model->body_mass + body, 1).c_str());
      inertial->SetAttribute("pos",
                             numbers(model->body_ipos + 3 * body, 3).c_str());
      inertial->SetAttribute("quat",
                             numbers(model->body_iquat + 4 * body, 4).c_str());
      inertial->SetAttribute(
          "diaginertia", numbers(model->body_inertia + 3 * body, 3).c_str());
    }
    if (std::string(e->Name()) == "site") {
      const char *name = e->Attribute("name");
      if (name &&
          (std::string(name) == "lidar" || std::string(name) == "mid360_imu" ||
           (std::string(name).rfind("camera_", 0) == 0 ||
            std::string(name).rfind("external_camera_", 0) == 0))) {
        int i = mj_name2id(model, mjOBJ_SITE, name);
        e->SetAttribute("pos", numbers(model->site_pos + 3 * i, 3).c_str());
        e->SetAttribute("quat", numbers(model->site_quat + 4 * i, 4).c_str());
      }
    }
    if (std::string(e->Name()) == "geom") {
      const char *name = e->Attribute("name");
      if (name && std::string(name).rfind("payload_", 0) == 0) {
        const int g = mj_name2id(model, mjOBJ_GEOM, name);
        e->SetAttribute("pos", numbers(model->geom_pos + 3 * g, 3).c_str());
        e->SetAttribute("quat", numbers(model->geom_quat + 4 * g, 4).c_str());
        e->SetAttribute("contype", model->geom_contype[g]);
        e->SetAttribute("conaffinity", model->geom_conaffinity[g]);
      }
      if (name &&
          (std::string(name).rfind("visual_device_", 0) == 0 ||
           std::string(name).rfind("visual_external_device_", 0) == 0)) {
        const auto *site =
            (std::string(name).rfind("visual_external_device_", 0) == 0
                 ? "external_camera_mount"
             : std::string(name).rfind("visual_device_d435", 0) == 0
                 ? "camera_mount"
                 : "lidar");
        const int site_id = mj_name2id(model, mjOBJ_SITE, site);
        e->SetAttribute("pos",
                        numbers(model->site_pos + 3 * site_id, 3).c_str());
        e->SetAttribute("quat",
                        numbers(model->site_quat + 4 * site_id, 4).c_str());
        const int id = mj_name2id(model, mjOBJ_GEOM, name);
        double rgba[4];
        for (int i = 0; i < 4; ++i)
          rgba[i] = model->geom_rgba[4 * id + i];
        e->SetAttribute("rgba", numbers(rgba, 4).c_str());
      }
    }
    for (auto *child = e->FirstChildElement(); child;
         child = child->NextSiblingElement())
      self(self, child);
  };
  visit(visit, doc.RootElement());
  tinyxml2::XMLPrinter out;
  doc.Print(&out);
  return out.CStr();
}
} // namespace simple_rbt
