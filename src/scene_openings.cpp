#include "user_scene.hpp"
#include <cmath>
namespace simple_rbt {
Json opening_boxes(const Json &object) {
  auto require = [](bool ok) {
    if (!ok)
      throw std::invalid_argument("Invalid slab opening");
  };
  require(object.at("kind") == "floor" && !object.contains("connection"));
  const auto &opening = object.at("opening");
  require(opening.at("center").is_array() && opening.at("center").size() == 2 &&
          opening.at("size").is_array() && opening.at("size").size() == 2);
  auto center = opening.at("center").get<std::array<double, 2>>();
  auto hole = opening.at("size").get<std::array<double, 2>>();
  auto size = object.at("size").get<Vec3>();
  auto position = object.at("position").get<Vec3>();
  auto quaternion = object.at("quaternion").get<Quat>();
  for (int i = 0; i < 2; ++i)
    require(std::isfinite(center[i]) && std::isfinite(hole[i]) &&
            hole[i] >= .02 &&
            std::abs(center[i]) + hole[i] / 2 <= size[i] / 2 + 1e-8);
  double left = center[0] - hole[0] / 2, right = center[0] + hole[0] / 2,
         bottom = center[1] - hole[1] / 2, top = center[1] + hole[1] / 2;
  Json result = Json::array();
  for (auto rect :
       {std::array<double, 4>{-size[0] / 2, left, -size[1] / 2, size[1] / 2},
        {right, size[0] / 2, -size[1] / 2, size[1] / 2},
        {left, right, -size[1] / 2, bottom},
        {left, right, top, size[1] / 2}}) {
    if (rect[1] - rect[0] < 1e-8 || rect[3] - rect[2] < 1e-8)
      continue;
    Vec3 local{(rect[0] + rect[1]) / 2, (rect[2] + rect[3]) / 2, 0}, world;
    mju_rotVecQuat(world.data(), local.data(), quaternion.data());
    for (int i = 0; i < 3; ++i)
      world[i] += position[i];
    Json part = object;
    part.erase("opening");
    part["position"] = world;
    part["size"] = {rect[1] - rect[0], rect[3] - rect[2], size[2]};
    result.push_back(std::move(part));
  }
  return result;
}
} // namespace simple_rbt
