#pragma once
#include "simple_rbt/runtime.hpp"
#include <filesystem>
#include <memory>

namespace simple_rbt {
// Formal course and independent payload evidence for validate_full_run.py.
// All records come from the runtime's actual observation/publication hooks.
class FullRunObserver : public RunObserver {
 public:
  explicit FullRunObserver(const std::filesystem::path& config_path);
  ~FullRunObserver() override;
  void before_loop(Simulation&, RosBridge*) override;
  void on_step(Simulation&, double monotonic_s) override;
  void on_imu(const ImuSample&, const std::array<std::uint64_t, 2>& source_stamps) override;
  void on_scan(const LidarScan&, const std::array<std::uint64_t, 2>& history_bounds,
               std::uint64_t maximum_interval_ns) override;
  void on_camera(std::uint64_t stamp_ns, const CameraPair&) override;
  void on_odometry(std::uint64_t stamp_ns, const Pose&, const Vec3&, const Vec3&) override;
  bool should_stop(Simulation&) override;
  void finish(Json& summary) override;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace simple_rbt
