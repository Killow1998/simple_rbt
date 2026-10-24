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
    if (object.contains("opening")) {
      for (auto &part : opening_boxes(object))
        result.push_back(std::move(part));
      continue;
    }
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
    int steps = 0;
    if (kind == "stairs") {
      require(connection.at("steps").is_number_integer(),
              "Step count must be an integer");
      steps = connection.at("steps");
      require(steps >= 1 && steps <= 128, "Stairs require 1..128 steps");
      require(size[0] / steps >= .02 && rise / steps >= .02,
              "Step dimensions must be at least 0.02 m");
    } else
      require(std::atan2(rise, size[0]) <= 80 * 3.14159265358979323846 / 180,
              "Ramp incline exceeds 80 degrees");
    require(!connection.contains("turn_deg") ||
                connection.at("turn_deg").is_number_integer(),
            "Turn must be an integer angle");
    int turn = connection.value("turn_deg", 0);
    double landing = connection.value("landing_m", 0.);
    require(turn == 0 || turn == 90 || turn == -90 || turn == 180,
            "Unsupported connection turn");
    if (turn)
      require(std::isfinite(landing) && landing >= size[1] && landing <= 200 &&
                  (kind != "stairs" || steps >= 2),
              "Invalid landing depth or step count");
    auto flight = [&](Vec3 start, double yaw, double run, double height,
                      int count) {
      double half = yaw * 3.14159265358979323846 / 360;
      Quat heading{std::cos(half), 0, 0, std::sin(half)};
      auto part = [&](Vec3 local, Vec3 dimensions, Quat rotation) {
        Vec3 at;
        Quat orientation;
        mju_rotVecQuat(at.data(), local.data(), heading.data());
        for (int i = 0; i < 3; ++i)
          at[i] += start[i];
        mju_mulQuat(orientation.data(), heading.data(), rotation.data());
        box(at, dimensions, orientation);
      };
      if (kind == "stairs") {
        double tread = run / count, h = height / count;
        for (int i = 1; i <= count; ++i)
          part({(i - .5) * tread, 0, i * h / 2}, {tread, size[1], i * h},
               {1, 0, 0, 0});
      } else {
        double angle = std::atan2(height, run);
        part({run / 2, 0, height / 2 - size[2] / (2 * std::cos(angle))},
             {std::hypot(run, height) + size[2] * std::tan(angle), size[1],
              size[2]},
             {std::cos(angle / 2), 0, -std::sin(angle / 2), 0});
      }
    };
    if (!turn)
      flight({0, 0, 0}, 0, size[0], rise, steps);
    else {
      int first = steps / 2;
      double fraction = kind == "stairs" ? double(first) / steps : .5;
      double run = size[0] * fraction, height = rise * fraction,
             width = size[1];
      flight({0, 0, 0}, 0, run, height, first);
      Vec3 center{run + landing / 2, 0, height - size[2] / 2},
          dimensions{landing, width, size[2]};
      Vec3 start{run + landing / 2, turn == -90 ? -width / 2 : width / 2,
                 height};
      if (turn == 180) {
        center[1] = width / 2;
        dimensions[1] = 2 * width;
        start = {run, width, height};
      }
      box(center, dimensions, {1, 0, 0, 0});
      flight(start, turn, size[0] - run, rise - height, steps - first);
    }
    require(result.size() <= max_scene_boxes, "Scene exceeds 2048 physical boxes");
  }
  require(result.size() <= max_scene_boxes, "Scene exceeds 2048 physical boxes");
  return result;
}
} // namespace simple_rbt
