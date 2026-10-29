#include "simple_rbt/runtime_info.hpp"
#include "simple_rbt/core.hpp"
#include <fstream>
#include <tinyxml2.h>

namespace simple_rbt {
namespace {
namespace fs = std::filesystem;
void require_file(const fs::path &path) {
  if (!fs::is_regular_file(path))
    throw std::runtime_error("Required asset is missing: " + path.string());
}
void check_digest(const fs::path &path, const Json &expected) {
  require_file(path);
  if (sha256_file(path) != expected.get<std::string>())
    throw std::runtime_error("Asset digest mismatch: " + path.string());
}
void check_assets(const fs::path &root) {
  const auto catalog_path = root / "assets/catalog.json";
  require_file(catalog_path);
  Json catalog;
  std::ifstream(catalog_path) >> catalog;
  for (const auto &entry : catalog.at("models").items()) {
    const auto path = root / "assets" / entry.value().at("file").get<std::string>();
    check_digest(path, entry.value().at("sha256"));
    tinyxml2::XMLDocument xml;
    if (xml.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("Invalid model XML: " + path.string());
    const auto *compiler = xml.RootElement()->FirstChildElement("compiler");
    const char *directory = compiler ? compiler->Attribute("meshdir") : nullptr;
    auto references = [&](auto &&self, const tinyxml2::XMLElement *element) -> void {
      if (const char *file = element->Attribute("file")) {
        std::string relative = file;
        const std::string prefix = "@PROJECT_ROOT@/";
        if (relative.rfind(prefix, 0) != 0 && directory)
          relative = std::string(directory) + "/" + relative;
        if (relative.rfind(prefix, 0) == 0)
          relative.erase(0, prefix.size());
        require_file(root / relative);
      }
      for (auto *child = element->FirstChildElement(); child;
           child = child->NextSiblingElement())
        self(self, child);
    };
    references(references, xml.RootElement());
  }
  for (const auto &controller : catalog.at("controllers"))
    if (controller.contains("policy"))
      check_digest(root / controller.at("policy").get<std::string>(),
                   controller.at("policy_sha256"));
  require_file(root / "assets/lidar/mid360-real-centr.csv");
  require_file(root / "assets/lidar/avia.csv");
}
} // namespace
Json robot_registry() {
  Json robots = Json::array();
  for (const auto &spec : robot_specs())
    robots.push_back(
        {{"name", spec.name},
         {"default_controller", spec.default_controller},
         {"continuous_terrain", spec.continuous_terrain},
         {"allows_lateral", spec.allows_lateral},
         {"allows_in_place_turn", spec.allows_in_place_turn},
         {"allows_in_place_turn", spec.allows_in_place_turn},
         {"command_limits", spec.command_limits},
         {"joint_control", spec.joint_control},
         {"flight_control", spec.flight_control},
         {"min_turning_radius", spec.min_turning_radius}});
  return robots;
}
Json runtime_info(const std::filesystem::path &root) {
  Json info{{"product", "simple_rbt"}, {"framework", "simple_rbt"},
            {"launcher", "SpRobo"}, {"version", SIMPLE_RBT_VERSION},
            {"mujoco_version", mj_versionString()},
            {"mujoco_header_version", mjVERSION_HEADER},
            {"mujoco_runtime_version", mj_version()},
            {"cuda_compiled", bool(SIMPLE_RBT_CUDA_COMPILED)},
            {"controller_backend", "cpu"}, {"assets_valid", false},
            {"lidar_models", {"mid360", "avia"}},
            {"robots", robot_registry()}};
  try {
    if (mj_version() != mjVERSION_HEADER)
      throw std::runtime_error("MuJoCo library and header versions differ");
    check_assets(root);
    info["assets_valid"] = true;
  } catch (const std::exception &error) {
    info["assets_error"] = error.what();
  }
  return info;
}
} // namespace simple_rbt
