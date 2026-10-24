#include "scene_geometry.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>
namespace simple_rbt::structured {
namespace {
std::string numbers(const Json &values) {
  std::ostringstream out;
  out << std::setprecision(17);
  for (const auto &v : values) {
    if (v.is_array())
      for (const auto &x : v)
        out << x.get<double>() << ' ';
    else
      out << v.get<double>() << ' ';
  }
  return out.str();
}
} // namespace
void append(tinyxml2::XMLDocument &doc, const Json &scene) {
  const auto compiled = resolve_structured_scene(scene);
  auto *root = doc.RootElement(), *world = root->FirstChildElement("worldbody"),
       *assets = root->FirstChildElement("asset");
  if (!assets) {
    assets = doc.NewElement("asset");
    root->InsertEndChild(assets);
  }
  int index = 0;
  for (const auto &part : compiled.at("parts")) {
    const auto name = "scene_object_" + std::to_string(index++);
    auto *geom = doc.NewElement("geom");
    geom->SetAttribute("name", name.c_str());
    const std::string type = part.at("type");
    geom->SetAttribute("type", type.c_str());
    geom->SetAttribute("pos", numbers(part.at("position")).c_str());
    geom->SetAttribute("quat", numbers(part.at("quaternion")).c_str());
    if (type == "mesh") {
      auto *mesh = doc.NewElement("mesh");
      mesh->SetAttribute("name", name.c_str());
      mesh->SetAttribute("vertex", numbers(part.at("vertices")).c_str());
      mesh->SetAttribute("face", numbers(part.at("faces")).c_str());
      mesh->SetAttribute("smoothnormal", "false");
      assets->InsertEndChild(mesh);
      geom->SetAttribute("mesh", name.c_str());
    } else
      geom->SetAttribute("size", numbers(part.at("size")).c_str());
    const std::string semantic = part.at("semantic");
    geom->SetAttribute("rgba", semantic == "wall"    ? ".63 .67 .72 1"
                               : semantic == "floor" ? ".4 .48 .57 1"
                                                     : ".74 .52 .34 1");
    geom->SetAttribute("group", "1");
    geom->SetAttribute("friction", "1 .01 .001");
    world->InsertEndChild(geom);
  }
}
Json spawn_document(const Json &scene, const std::string &selected) {
  const auto compiled = resolve_structured_scene(scene);
  std::string marker = selected;
  if (marker.empty())
    for (const auto &o : scene.at("objects"))
      if (o.value("semantic", std::string()) == "spawn") {
        require(marker.empty(),
                "multiple spawn markers; select scene_spawn in session");
        marker = o.at("id");
      }
  Vec3 position{};
  Quat q{1, 0, 0, 0};
  if (!marker.empty()) {
    require(compiled.at("nodes").contains(marker),
            "unknown spawn marker " + marker);
    bool valid = false;
    for (const auto &o : scene.at("objects"))
      if (o.at("id") == marker)
        valid = o.at("kind") == "group" &&
                o.value("semantic", std::string()) == "spawn";
    require(valid, "scene_spawn must select a geometry-free spawn group");
    position = compiled.at("nodes").at(marker).at("position").get<Vec3>();
    q = compiled.at("nodes").at(marker).at("quaternion").get<Quat>();
    require(std::abs(q[1]) + std::abs(q[2]) < 1e-6,
            "robot spawn must be upright");
  }
  return {{"floors", Json::array({{{"id", "world"}, {"height", 0}}})},
          {"spawn",
           {{"floor", "world"},
            {"position", position},
            {"yaw_deg", 2 * std::atan2(q[3], q[0]) * 180 / std::acos(-1.)}}}};
}
} // namespace simple_rbt::structured
