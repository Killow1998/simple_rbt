#include "runtime_lidar.hpp"
#include "artifacts.hpp"
#include "resources.hpp"
#include "simple_rbt/event_log.hpp"
#include <algorithm>
#include <cmath>
namespace simple_rbt {
RuntimeLidar::RuntimeLidar(Simulation &sim, const RunOptions &options,
                           Transport *bridge)
    : args(options), active_backend(args.backend) {
  if (args.sensors != "physics" && args.mid360_enabled) {
    const auto &device = lidar_spec(args.simulation.lidar_model);
    pattern = std::make_unique<CsvPattern>(
        (args.simulation.project_root / "assets/lidar" / device.csv).string(),
        args.simulation.lidar_model);
    if (args.backend == "auto") {
      try {
        lidar = std::make_unique<RollingLidar>(sim.model(), LidarBackend::Cuda,
                                               *pattern, device.minimum_range, device.maximum_range);
        active_backend = "cuda";
      } catch (const std::exception &e) {
        active_backend = "cpu";
        lidar = std::make_unique<RollingLidar>(sim.model(), LidarBackend::Cpu,
                                               *pattern, device.minimum_range, device.maximum_range);
        if (bridge)
          bridge->publish_status(
              Json{{"lidar_backend", "cpu"}, {"reason", e.what()}}.dump());
      }
    } else
      lidar = std::make_unique<RollingLidar>(
          sim.model(),
          args.backend == "cpu" ? LidarBackend::Cpu : LidarBackend::Cuda,
          *pattern, device.minimum_range, device.maximum_range);
    auto pose = sim.pose("lidar");
    history.add(0, pose);
    PoseHistory warm;
    warm.add(0, pose);
    warm.add(100000000, pose);
    lidar->scan(sim.data(), 0, args.rays, warm, last_scan);
    pattern->reset();
  }
}
void RuntimeLidar::record_pose(const Simulation &sim) {
  if (lidar && args.lidar_motion == "rolling")
    history.add(sim.stamp_ns(), sim.pose("lidar"));
}
void RuntimeLidar::scan(Simulation &sim, Transport *bridge,
                        RunObserver *observer) {
  const auto stamp = sim.stamp_ns();
  if (lidar) {
    double began = monotonic_seconds();
    std::array<std::uint64_t, 2> bounds{};
    std::uint64_t interval = 0;
    if (args.lidar_motion == "rolling") {
      lidar->scan(sim.data(), stamp - 100000000, args.rays, history, last_scan);
      bounds = history.bounds();
      interval = history.maximum_interval(
          last_scan.stamp_ns, last_scan.stamp_ns + pattern->offset_ns(args.rays - 1));
    } else {
      PoseHistory scratch;
      auto pose = sim.pose("lidar");
      last_snapshot_pose = pose;
      scratch.add(stamp, pose);
      scratch.add(stamp + 100000000, pose);
      lidar->scan(sim.data(), stamp, args.rays, scratch, last_scan);
      std::fill(last_scan.offsets_ns.begin(), last_scan.offsets_ns.end(), 0);
      bounds = scratch.bounds();
    }
    const auto frame_points = pattern->spec().point_rate / 10;
    if (args.rays < frame_points)
      pattern->advance(frame_points - args.rays);
    last_snapshot_pose = sim.pose("lidar");
    lidar_times.push_back(monotonic_seconds() - began);
    diagnostics::log().trace(
        "sensor.generated",
        {{"sample", diagnostics::sample_key("lidar", last_scan.stamp_ns)},
         {"points", last_scan.xyz.size()},
         {"duration_ms", lidar_times.back() * 1000}},
        last_scan.stamp_ns);
    bool slow = lidar_times.back() > .1;
    if (slow != scan_slow) {
      scan_slow = slow;
      diagnostics::log().event("sensor.deadline",
                               {{"sensor", "lidar"},
                                {"overrun", slow},
                                {"duration_ms", lidar_times.back() * 1000}},
                               stamp, slow ? "warning" : "info");
    }
    lidar_frames++;
    min_points = std::min(min_points, last_scan.xyz.size());
    max_points = std::max(max_points, last_scan.xyz.size());
    if (bridge)
      bridge->publish_scan(last_scan);
    if (observer)
      observer->on_scan(last_scan, bounds, interval);
  }
}
Json RuntimeLidar::scan_model() const {
  Json model = nullptr;
  if (lidar)
    model = {{"acquisition_model", args.lidar_motion == "rolling"
                                       ? "rolling_static_environment"
                                       : "snapshot_end"},
             {"point_time_model", args.lidar_motion == "rolling"
                                      ? "uniform_from_csv_sample_order"
                                      : "all_returns_at_snapshot_stamp"},
             {"device", pattern->spec().name},
             {"sample_rate_hz", pattern->spec().point_rate},
             {"line_count", pattern->spec().lines},
             {"minimum_range_m", pattern->spec().minimum_range},
             {"maximum_range_m", pattern->spec().maximum_range},
             {"robot_self_occlusion", false},
             {"dynamic_environment_supported", false},
             {"backend_implementation",
              active_backend == "cpu" ? "native_mujoco_ray_bvh" : "native_cuda_aot"},
             {"pose_interpolation", args.lidar_motion == "rolling"
                                        ? "linear_translation_short_arc_slerp"
                                        : "none"},
             {"environment_geom_count", lidar->environment_geom_count()},
             {"ray_buffer_capacity", lidar->ray_capacity()},
             {"pattern_timing_source",
              "synthetic CSV sample order; not recorded hardware timestamps"}};
  return model;
}
Json RuntimeLidar::report() const {
  Json result = {
      {"backend", lidar ? Json(active_backend) : Json(nullptr)},
      {"backend_implementation",
       lidar ? (active_backend == "cpu" ? "native_mujoco_ray_bvh" : "native_cuda_aot")
             : "none"},
      {"rays_per_frame", args.rays},
      {"lidar_frames", lidar_frames},
      {"min_valid_points", lidar_frames ? min_points : 0},
      {"max_valid_points", max_points},
      {"lidar_median_ms", lidar_times.empty()
                              ? Json(nullptr)
                              : Json(percentile(lidar_times, .5) * 1000)},
      {"lidar_p95_ms", lidar_times.empty()
                           ? Json(nullptr)
                           : Json(percentile(lidar_times, .95) * 1000)},
      {"scan_model", scan_model()}};
  return result;
}
void RuntimeLidar::save(ArtifactWriter &artifacts) const {
  if (lidar && lidar_frames)
    artifacts.lidar(last_scan, last_snapshot_pose, history,
                    args.lidar_motion == "rolling", scan_model());
}
} // namespace simple_rbt
