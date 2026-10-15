#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>
namespace simple_rbt {
nlohmann::json run_session(const nlohmann::json &,
                           const std::filesystem::path &directory,
                           const std::filesystem::path &project_root);
}
