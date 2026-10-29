#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>

namespace simple_rbt {
nlohmann::json robot_registry();
// Reads the loaded SDK and asset closure without simulation or GPU creation.
nlohmann::json runtime_info(const std::filesystem::path &root);
} // namespace simple_rbt
