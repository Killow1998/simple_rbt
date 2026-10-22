#pragma once
#include <atomic>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>
namespace simple_rbt::diagnostics {
using Json = nlohmann::json;
constexpr std::size_t storage_limit =
    100000000; // 100 MB, shared by all writers.
struct StoreBusy : std::runtime_error {
  using std::runtime_error::runtime_error;
};
constexpr std::size_t segment_limit = 5000000;
// The store appends batches to bounded segments; flock serializes global
// retention across native and Go writers. Only srbt-<digits>-<digits>.jsonl
// files are owned. Empty directories and unrelated files are never removed.
void append_batch(const std::filesystem::path &, const std::string &,
                  std::size_t limit = storage_limit);
std::filesystem::path default_directory();
class Log {
public:
  Log(std::string component, std::string session = "",
      bool stdout_sink = false);
  ~Log();
  Log(const Log &) = delete;
  void event(const std::string &name, Json fields = Json::object(),
             std::uint64_t sim_ns = 0, const std::string &level = "info");
  void trace(const std::string &name, Json fields, std::uint64_t sim_ns = 0);
  void set_trace(bool);
  void set_session(std::string);
  void set_modules(const std::vector<std::string> &);
  bool tracing() const;
  Json health() const;
  const std::string &session() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
// Process-local sink. Runtime/core use it without depending on ROS or GUI.
Log &log();
std::string sample_key(const std::string &stream, std::uint64_t stamp);
} // namespace simple_rbt::diagnostics
