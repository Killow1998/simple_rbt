#include "user_scene.hpp"
#include <cmath>
#include <map>

namespace simple_rbt {
Json scene_boxes(const Json &scene) {
  std::map<std::string, double> floors;
  for (const auto &floor : scene.at("floors"))
    floors.emplace(floor.at("id"), floor.at("height"));
  Json result = Json::array();
  auto require = [](bool ok, const char *message) {
    if (!ok)
      throw std::invalid_argument(message);
  };
  for (const auto &object : scene.at("objects")) {
    const std::string kind = object.at("kind");
    if (!object.contains("connection")) {
      require(kind != "stairs", "Stairs require a floor connection");
      result.push_back(object);
      continue;
    }
    const auto &connection = object.at("connection");
    const std::string top = connection.at("to_floor");
    require(floors.count(top), "Unknown upper floor");
    double rise = floors.at(top) - floors.at(object.at("floor"));
    auto position = object.at("position").get<Vec3>();
    auto size = object.at("size").get<Vec3>();
    auto quaternion = object.at("quaternion").get<Quat>();
    require(
        (kind == "ramp" || kind == "stairs") && rise >= .02 &&
            std::abs(position[2]) < 1e-8 &&
            std::abs(quaternion[1]) + std::abs(quaternion[2]) < 1e-8,
        "Connections require ascending floors, a ground anchor and yaw only");
    auto box = [&](Vec3 local, Vec3 dimensions, Quat rotation) {
      Vec3 rotated;
      Quat orientation;
      mju_rotVecQuat(rotated.data(), local.data(), quaternion.data());
      mju_mulQuat(orientation.data(), quaternion.data(), rotation.data());
      for (int i = 0; i < 3; ++i)
        rotated[i] += position[i];
      Json value = object;
      value.erase("connection");
      value["position"] = rotated;
      value["quaternion"] = orientation;
      value["size"] = dimensions;
      result.push_back(std::move(value));
    };
    if (kind == "stairs") {
      require(connection.at("steps").is_number_integer(),
              "Step count must be an integer");
      int steps = connection.at("steps");
      require(steps >= 1 && steps <= 128, "Stairs require 1..128 steps");
      double tread = size[0] / steps, height = rise / steps;
      require(tread >= .02 && height >= .02,
              "Step dimensions must be at least 0.02 m");
      for (int i = 1; i <= steps; ++i)
        box({(i - .5) * tread, 0, i * height * .5},
            {tread, size[1], i * height}, {1, 0, 0, 0});
    } else {
      double angle = std::atan2(rise, size[0]);
      require(angle <= 80 * 3.14159265358979323846 / 180,
              "Ramp incline exceeds 80 degrees");
      // The upper surface, rather than the slab centre, joins the floor levels.
      box({size[0] * .5, 0, rise * .5 - size[2] / (2 * std::cos(angle))},
          {std::hypot(size[0], rise) + size[2] * std::tan(angle), size[1],
           size[2]},
          {std::cos(angle * .5), 0, -std::sin(angle * .5), 0});
    }
    require(result.size() <= 2048, "Scene exceeds 2048 physical boxes");
  }
  require(result.size() <= 2048, "Scene exceeds 2048 physical boxes");
  return result;
}
} // namespace simple_rbt
