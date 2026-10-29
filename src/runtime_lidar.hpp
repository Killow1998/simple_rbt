#pragma once
#include "simple_rbt/lidar.hpp"
#include "simple_rbt/runtime.hpp"
#include "simple_rbt/transport.hpp"
namespace simple_rbt {
class ArtifactWriter;
class RuntimeLidar {
public:
  RuntimeLidar(Simulation &, const RunOptions &, Transport *);
  void record_pose(const Simulation &);
  void scan(Simulation &, Transport *, RunObserver *);
  Json report() const;
  Json scan_model() const;
  void save(ArtifactWriter &) const;
  std::uint64_t frames() const { return lidar_frames; }
  double last_ms() const {
    return lidar_times.empty() ? 0 : lidar_times.back() * 1000;
  }

private:
  const RunOptions &args;
  std::string active_backend;
  std::unique_ptr<CsvPattern> pattern;
  std::unique_ptr<RollingLidar> lidar;
  PoseHistory history;
  LidarScan last_scan;
  Pose last_snapshot_pose;
  std::vector<double> lidar_times;
  std::uint64_t lidar_frames = 0;
  std::size_t min_points = 24000, max_points = 0;
  bool scan_slow = false;
};
} // namespace simple_rbt
