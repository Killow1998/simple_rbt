#pragma once
#include "simple_rbt/core.hpp"
namespace simple_rbt {
Json load_catalog(const std::filesystem::path &project_root);
std::string build_scene(const SimulationOptions &, const Json &catalog,
                        const std::string &controller, Json &profile);
} // namespace simple_rbt
