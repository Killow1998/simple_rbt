#pragma once
#include "simple_rbt/types.hpp"
#include "simple_rbt/lidar_model.hpp"
#include <mujoco/mujoco.h>
#include <deque>
#include <memory>
#include <string>

namespace simple_rbt {
enum class LidarBackend { Cpu, Cuda };

struct PatternRay {
  std::array<float, 3> direction{};
};

// Ordered full-pattern replay; the selected model owns timing and line count.
class CsvPattern {
 public:
  explicit CsvPattern(const std::string& csv_path, LidarModel model = LidarModel::Mid360);
  const LidarSpec &spec() const { return lidar_spec(model_); }
  std::uint64_t offset_ns(std::size_t sample) const {
    return (sample * 1000000000ULL + spec().point_rate / 2) / spec().point_rate;
  }
  void reset() noexcept { cursor_ = 0; emitted_ = 0; }
  std::size_t cursor() const noexcept { return cursor_; }
  std::size_t size() const noexcept { return rays_.size(); }
  std::uint64_t emitted() const noexcept { return emitted_; }
  const PatternRay& ray(std::size_t index) const { return rays_.at(index); }
  void advance(std::size_t count);

 private:
  LidarModel model_;
  std::vector<PatternRay> rays_;
  std::size_t cursor_ = 0;
  std::uint64_t emitted_ = 0;
};

struct PoseKnot { std::uint64_t stamp_ns; Pose pose; };
class PoseHistory {
 public:
  explicit PoseHistory(std::uint64_t retention_ns = 250000000);
  void add(std::uint64_t stamp_ns, const Pose& pose);
  Pose interpolate(std::uint64_t stamp_ns) const;
  std::array<std::uint64_t, 2> bounds() const;
  std::size_t size() const noexcept { return knots_.size(); }
  std::uint64_t maximum_interval(std::uint64_t start_ns, std::uint64_t end_ns) const;
 private:
  std::uint64_t retention_ns_;
  std::deque<PoseKnot> knots_;
};

// Instantaneous local XYZ at each point's own acquisition time, not deskewed.
// One static environment, complete robot-root exclusion, no live model mutation.
// CPU = independent mj_ray on one private model/data with visibility masks.
// CUDA = native analytic plane/box kernel, no MuJoCo model/data copy retained.
// Other environment primitive kinds and non-robot dynamic bodies fail explicitly.
class RollingLidar {
 public:
  RollingLidar(const mjModel* model, LidarBackend backend, CsvPattern& pattern,
               double min_range_m = .1, double max_range_m = 40.,
               const std::string& site_name = "lidar");
  ~RollingLidar();
  RollingLidar(const RollingLidar&) = delete;
  RollingLidar& operator=(const RollingLidar&) = delete;
  void scan(const mjData* live_data, std::uint64_t start_ns, std::size_t n_rays,
            const PoseHistory& history, LidarScan& output);
  std::size_t environment_geom_count() const noexcept;
  std::size_t ray_capacity() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace simple_rbt
