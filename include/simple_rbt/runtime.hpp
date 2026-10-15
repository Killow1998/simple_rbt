#pragma once
#include "simple_rbt/core.hpp"
#include "simple_rbt/transport.hpp"
#include <filesystem>
#include <functional>

namespace simple_rbt {
struct RunOptions {
  SimulationOptions simulation;
  std::string backend = "cpu", sensors = "lidar", lidar_motion = "rolling";
  double duration = 10., settle = 1., camera_hz = 1.;
  std::size_t rays = 20000;
  Vec3 simulation_command{};
  bool realtime = false, viewer = false, summary_only = false, indefinite = false;
  bool mid360_enabled = true, camera_enabled = true, depth_enabled = true, panorama_enabled = true;
  bool output_precreated =
      false; // Only the validator's newly allocated folder.
  std::filesystem::path output;
};
class RunObserver {
public:
  virtual ~RunObserver() = default;
  // Called after sensor warmup, before the measured loop/clock zero. May wait
  // for transport discovery; sampling/physics have not advanced.
  virtual void configure(Simulation &, Transport *) {}
  virtual bool before_step(Simulation &, Transport *) { return true; }
  virtual void before_loop(Simulation &, Transport *) {}
  // After applying Simulation.command() and completing mj_step+mj_forward.
  // command() is that step's actual bounded command; monotonic_s is Linux's
  // steady clock in the same epoch as Python time.monotonic().
  virtual void on_step(Simulation &, double) {}
  virtual void on_imu(const ImuSample &, const std::array<std::uint64_t, 2> &) {
  }
  // history covers the scheduled points; maximum interval is between native
  // physics pose knots used for interpolation in this 100ms acquisition.
  virtual void on_scan(const LidarScan &, const std::array<std::uint64_t, 2> &,
                       std::uint64_t) {}
  virtual void on_camera(std::uint64_t, const CameraPair &) {}
  virtual void on_odometry(std::uint64_t, const Pose &, const Vec3 &,
                           const Vec3 &) {}
  // Checked after all this tick's publications, so the final clock is emitted.
  virtual bool should_stop(Simulation &) { return false; }
  // summary supplies resources_loop/run, wall_seconds, real_time_factor,
  // simulation_end_ns, final_applied_command_m_s_rad_s, controller/metadata.
  // Called before transport drain/shutdown. Observer owns its evidence file.
  virtual void finish(Json &) {}
};
using TransportFactory = std::function<std::unique_ptr<Transport>(const Extrinsics&)>;
Json run_simulation(const RunOptions &, RunObserver *observer = nullptr,
                    const TransportFactory& transport_factory = {});
} // namespace simple_rbt
