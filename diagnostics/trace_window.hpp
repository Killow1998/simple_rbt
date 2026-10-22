#pragma once
#include "simple_rbt/event_log.hpp"
#include <map>
#include <utility>

namespace simple_rbt::diagnostics {
// Only the fixed clock/IMU pipeline is aggregated. Memory is independent of
// event count; stage counts remain separate so receipt is never inferred.
class TraceWindow {
  struct Counts {
    std::uint64_t count = 0, first = 0, last = 0, bytes = 0;
    std::uint64_t duplicates = 0, backwards = 0;
    std::uint64_t previous = 0;
    bool seen = false;
  };
  std::map<std::pair<std::string, std::string>, Counts> streams_;

public:
  template <class Emit>
  bool add(const std::string &event, const Json &fields, std::uint64_t stamp,
           Emit emit) {
    if (event != "sensor.generated" && event != "ipc.offered" &&
        event != "ipc.received" && event != "ros.publish_called")
      return false;
    auto sample = fields.find("sample");
    if (sample == fields.end() || !sample->is_string())
      return false;
    const auto &key = sample->get_ref<const std::string &>();
    const auto stream = key.substr(0, key.find(':'));
    if (stream != "clock" && stream != "imu" && stream != "camera_imu")
      return false;
    auto &c = streams_[{event, stream}];
    if (c.seen && stamp <= c.previous) {
      if (stamp == c.previous)
        ++c.duplicates;
      else
        ++c.backwards;
      emit("stream.timestamp_anomaly",
           {{"stage", event},
            {"sample", key},
            {"previous_sim_ns", c.previous},
            {"reason", stamp == c.previous ? "duplicate" : "backwards"}},
           stamp, "warning");
    }
    if (!c.count)
      c.first = stamp;
    ++c.count;
    c.last = c.previous = stamp;
    c.seen = true;
    c.bytes += fields.value("bytes", std::uint64_t{0});
    return true;
  }

  template <class Emit> void flush(Emit emit) {
    for (auto &[key, c] : streams_) {
      if (!c.count)
        continue;
      emit("stream.summary",
           {{"stage", key.first},
            {"stream", key.second},
            {"count", c.count},
            {"first_sim_ns", c.first},
            {"last_sim_ns", c.last},
            {"payload_bytes", c.bytes},
            {"duplicates", c.duplicates},
            {"backwards", c.backwards}},
           c.last, "trace");
      // Keep the previous timestamp across windows to detect boundary
      // anomalies.
      c.count = c.bytes = c.duplicates = c.backwards = 0;
    }
  }
  void clear() { streams_.clear(); }
};
} // namespace simple_rbt::diagnostics
