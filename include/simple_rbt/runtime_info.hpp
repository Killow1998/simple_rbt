#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>

namespace simple_rbt {
// Reads the loaded SDK and asset closure without simulation or GPU creation.
nlohmann::json runtime_info(const std::filesystem::path &root);
} // namespace simple_rbt
