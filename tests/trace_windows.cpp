#include "simple_rbt/event_log.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
using namespace simple_rbt::diagnostics;
namespace fs = std::filesystem;
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
std::vector<Json> read(const fs::path &directory) {
  std::vector<Json> rows;
  for (const auto &file : fs::directory_iterator(directory)) {
    if (file.path().extension() != ".jsonl")
      continue;
    std::ifstream input(file.path());
    for (std::string line; std::getline(input, line);)
      rows.push_back(Json::parse(line));
  }
  return rows;
}
int main(int argc, char **argv) {
  if (argc == 3 && std::string(argv[1]) == "--shutdown-probe") {
    setenv("SIMPLE_RBT_LOG_DIR", argv[2], 1);
    auto &events = simple_rbt::diagnostics::log();
    events.event("before_timer");
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    events.event("final_shutdown_probe");
    return 0; // Exercise real process-static destruction, not a local Log.
  }
  const auto root = fs::temp_directory_path() /
                    ("srbt-trace-test-" + std::to_string(getpid()));
  try {
    fs::create_directories(root);
    setenv("SIMPLE_RBT_LOG_DIR", root.c_str(), 1);
    unsetenv("SIMPLE_RBT_TRACE_RAW_HIGH_RATE");
    {
      Log events("core", "first");
      events.set_trace(true);
      events.set_modules({"sensor", "ipc", "ros"});
      for (const auto *stream : {"clock", "imu", "camera_imu"})
        for (const auto *stage : {"sensor.generated", "ipc.offered",
                                  "ipc.received", "ros.publish_called"})
          for (std::uint64_t n = 1; n <= 1000; ++n)
            events.trace(stage,
                         {{"sample", sample_key(stream, n)}, {"bytes", 8}}, n);
      events.trace("sensor.generated", {{"sample", "lidar:1000"}}, 1000);
      events.event("sensor.failed", {{"reason", "test fault"}}, 1000, "error");
      std::this_thread::sleep_for(std::chrono::milliseconds(1100));
      // Across a flushed window, duplicate/backwards timestamps remain visible.
      events.trace("sensor.generated", {{"sample", "imu:1000"}}, 1000);
      events.trace("sensor.generated", {{"sample", "imu:999"}}, 999);
      events.set_session("second");
      events.trace("sensor.generated", {{"sample", "imu:1"}}, 1);
      events.set_modules({"sensor"});
      events.trace("ipc.offered", {{"sample", "imu:2"}}, 2);
    }
    std::map<std::pair<std::string, std::string>, std::uint64_t> counts;
    int anomalies = 0, lidar = 0, failures = 0, second = 0;
    auto rows = read(root);
    for (const auto &r : rows) {
      auto event = r["event"].get<std::string>();
      auto f = r["fields"];
      if (event == "stream.summary") {
        if (r["session"] == "first") {
          counts[{f["stage"], f["stream"]}] += f["count"].get<std::uint64_t>();
          if (f["count"] == 1000)
            require(f["first_sim_ns"] == 1 && f["last_sim_ns"] == 1000 &&
                        f["payload_bytes"] == 8000,
                    "summary loses timestamps/bytes");
        } else {
          ++second;
          require(f["stage"] == "sensor.generated" && f["count"] == 1 &&
                      f["duplicates"] == 0 && f["backwards"] == 0,
                  "session isolation or module filter failed");
        }
      } else if (event == "stream.timestamp_anomaly") {
        ++anomalies;
        require(r["level"] == "warning" && r["session"] == "first",
                "timestamp anomaly hidden or crosses session");
      } else if (event == "sensor.generated") {
        require(f["sample"] == "lidar:1000", "high-rate sample not coalesced");
        ++lidar;
      } else if (event == "sensor.failed")
        ++failures;
    }
    require(counts.size() == 12, "pipeline stage missing");
    for (const auto &[key, count] : counts)
      require(count == (key == std::make_pair(std::string("sensor.generated"),
                                              std::string("imu"))
                            ? 1002
                            : 1000),
              "aggregation lost events");
    require(anomalies == 2 && lidar == 1 && failures == 1 && second == 1,
            "anomaly, lidar, error or final window missing");
    require(rows.size() < 50, "high-rate record count unbounded");
    setenv("SIMPLE_RBT_TRACE_RAW_HIGH_RATE", "1", 1);
    {
      Log events("core", "raw");
      events.set_trace(true);
      events.set_modules({"sensor"});
      for (int n = 0; n < 20; ++n)
        events.trace("sensor.generated", {{"sample", sample_key("imu", n)}}, n);
    }
    int raw = 0;
    for (const auto &r : read(root))
      if (r["session"] == "raw" && r["level"] == "trace") {
        require(r["event"] == "sensor.generated", "raw override aggregated");
        ++raw;
      }
    require(raw == 20, "raw override lost samples");
    const auto probe = root / "shutdown";
    const auto executable = fs::read_symlink("/proc/self/exe").string();
    const auto child = fork();
    require(child >= 0, "cannot start shutdown probe");
    if (child == 0) {
      execl(executable.c_str(), executable.c_str(), "--shutdown-probe",
            probe.c_str(), nullptr);
      _exit(127);
    }
    int status = 0;
    require(waitpid(child, &status, 0) == child && WIFEXITED(status) &&
                WEXITSTATUS(status) == 0,
            "shutdown probe failed");
    bool final = false, stopped = false;
    for (const auto &r : read(probe)) {
      final |= r["event"] == "final_shutdown_probe";
      stopped |= r["event"] == "process.stopped";
    }
    require(final && stopped, "process-static logger lost its final batch");
    fs::remove_all(root);
    std::cout << "PASS: window counts, boundaries, anomalies, isolation, raw "
                 "override\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
