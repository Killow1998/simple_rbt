#pragma once
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
namespace simple_rbt {
inline std::filesystem::path default_project_root() {
  namespace fs = std::filesystem;
  auto valid = [](const fs::path &p) {
    return fs::is_regular_file(p / "assets/catalog.json");
  };
  if (const char *p = std::getenv("SIMPLE_RBT_ASSETS")) {
    fs::path root = fs::path(p).parent_path();
    if (!valid(root))
      throw std::runtime_error(
          "SIMPLE_RBT_ASSETS does not contain catalog.json");
    return root;
  }
  if (const char *p = std::getenv("SIMPLE_RBT_DATA_ROOT")) {
    if (!valid(p))
      throw std::runtime_error(
          "SIMPLE_RBT_DATA_ROOT does not contain assets/catalog.json");
    return p;
  }
  std::error_code ec;
  auto exe = fs::read_symlink("/proc/self/exe", ec).parent_path();
  if (!ec) {
    auto installed = (exe / "../share/simple-rbt").lexically_normal();
    if (valid(installed))
      return installed;
    for (int i = 0; i < 5 && !exe.empty(); ++i, exe = exe.parent_path())
      if (valid(exe))
        return exe;
  }
  if (valid("/usr/share/simple-rbt"))
    return "/usr/share/simple-rbt";
  throw std::runtime_error(
      "Cannot locate simple_rbt assets; set SIMPLE_RBT_ASSETS");
}
inline std::filesystem::path user_output_root() {
  if (const char *p = std::getenv("XDG_STATE_HOME"))
    return std::filesystem::path(p) / "simple-rbt/output";
  if (const char *p = std::getenv("HOME"))
    return std::filesystem::path(p) / ".local/state/simple-rbt/output";
  throw std::runtime_error(
      "Set XDG_STATE_HOME for writable simulation results");
}
} // namespace simple_rbt
