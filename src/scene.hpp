#pragma once
#include "simple_rbt/core.hpp"
namespace simple_rbt {
struct ScenePlacement {
  Vec3 position;
  double floor_height;
  double yaw_degrees;
};
Json load_catalog(const std::filesystem::path &project_root);
std::string build_scene(const SimulationOptions &, const Json &catalog,
                        const std::string &controller, Json &profile,
                        std::optional<ScenePlacement> &spawn);
} // namespace simple_rbt
