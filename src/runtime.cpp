#include "simple_rbt/runtime.hpp"
#include "artifacts.hpp"
#include "resources.hpp"
#include "simple_rbt/cameras.hpp"
#include "simple_rbt/lidar.hpp"
#include "simple_rbt/ros_bridge.hpp"
#include "simple_rbt/viewer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <thread>
#include <unistd.h>

namespace simple_rbt {
Json run_simulation(const RunOptions &args, RunObserver *observer) {
  ResourceMeter run_meter;
  if (!std::isfinite(args.duration) || args.duration <= 0 ||
      !std::isfinite(args.settle) || args.settle < 0 ||
      !std::isfinite(args.camera_hz) || args.camera_hz <= 0 ||
      args.camera_hz > 30 || args.rays < 1 || args.rays > 20000)
    throw std::invalid_argument(
        "Invalid duration, settle, camera rate or ray count");
  if (args.sensors != "physics" && args.sensors != "lidar" &&
      args.sensors != "full")
    throw std::invalid_argument("sensors must be physics/lidar/full");
  if (args.backend != "cpu" && args.backend != "cuda" && args.backend != "warp")
    throw std::invalid_argument("backend must be cpu/cuda (warp is a "
                                "compatibility alias for native CUDA)");
  if (args.lidar_motion != "rolling" && args.lidar_motion != "snapshot")
    throw std::invalid_argument("lidar-motion must be rolling/snapshot");
  Simulation sim(args.simulation);
  const auto &m = *sim.model();
  const int base_site = mj_name2id(&m, mjOBJ_SITE, "base_origin");
  ArtifactWriter artifacts(args.simulation.project_root, args.output,
                           args.summary_only, args.output_precreated);
  artifacts.model(sim.xml());
  std::unique_ptr<CsvPattern> pattern;
  std::unique_ptr<RollingLidar> lidar;
  PoseHistory history;
  LidarScan last_scan;
  Pose last_snapshot_pose;
  if (args.sensors != "physics") {
    pattern = std::make_unique<CsvPattern>(
        (args.simulation.project_root /
         "assets/lidar/mid360-real-centr.csv")
            .string());
    lidar = std::make_unique<RollingLidar>(
        sim.model(),
        args.backend == "cpu" ? LidarBackend::Cpu : LidarBackend::Cuda,
        *pattern);
    auto pose = sim.pose("lidar");
    history.add(0, pose);
    PoseHistory warm;
    warm.add(0, pose);
    warm.add(100000000, pose);
    lidar->scan(sim.data(), 0, args.rays, warm, last_scan);
    pattern->reset();
  }
  std::unique_ptr<CameraRig> cameras;
  CameraPair frames;
  if (args.sensors == "full") {
    cameras = std::make_unique<CameraRig>(sim.model());
    cameras->render_into(sim.data(), sim.camera_pose(), frames);
  }
  std::unique_ptr<RosBridge> bridge;
  if (args.ros2) {
    Extrinsics extrinsics;
    extrinsics.lidar = sim.extrinsics("lidar");
    extrinsics.imu = sim.extrinsics("mid360_imu");
    extrinsics.camera = sim.extrinsics("camera_mount");
    // The mounted camera frame remains part of the robot even when images
    // are disabled, matching the existing ROS interface.
    extrinsics.has_camera = true;
    double mount[9], optical[9];
    mju_quat2Mat(mount, extrinsics.camera.quaternion.data());
    for (int row = 0; row < 3; row++) {
      optical[3 * row] = -mount[3 * row + 1];
      optical[3 * row + 1] = -mount[3 * row + 2];
      optical[3 * row + 2] = mount[3 * row];
    }
    mju_mat2Quat(extrinsics.camera.quaternion.data(), optical);
    RosOptions options;
    options.robot_name = args.simulation.robot;
    bridge = std::make_unique<RosBridge>(options, extrinsics);
  }
  std::unique_ptr<Viewer> viewer;
  if (args.viewer)
    viewer = std::make_unique<Viewer>(sim.model(), sim.data());
  artifacts.open_streams();
  if (observer)
    observer->before_loop(sim, bridge.get());
  const std::uint64_t camera_period = std::llround(1e9 / args.camera_hz);
  std::uint64_t next_camera = camera_period, next_imu = 0;
  auto emit_imu = [&](const ImuSample &sample,
                      std::array<std::uint64_t, 2> brackets) {
    if (bridge)
      bridge->publish_imu(sample);
    artifacts.imu(sample);
    if (observer)
      observer->on_imu(sample, brackets);
  };
  ImuSample previous = sim.imu();
  std::uint64_t imu_samples = 1, interpolated = 0, camera_frames = 0,
                lidar_frames = 0;
  std::vector<double> lidar_times, camera_times;
  std::size_t min_points = 20000, max_points = 0;
  Pose initial = sim.pose();
  double min_height = initial.position[2], max_tilt = 0;
  std::optional<double> policy_origin;
  ResourceMeter loop_meter;
  double start = monotonic_seconds();
  if (bridge)
    bridge->publish_clock(0);
  emit_imu(previous, {0, 0});
  next_imu = 5000000;
  std::size_t steps = std::ceil(args.duration / m.opt.timestep);
  const Vec3 local_command = args.simulation_command;
  for (std::size_t step = 0; step < steps; step++) {
    if (viewer && !viewer->running())
      break;
    if (bridge)
      bridge->spin_once();
    if (sim.startup_complete() && !policy_origin)
      policy_origin = sim.data()->time;
    bool ready =
        policy_origin && sim.data()->time >= *policy_origin + args.settle;
    Vec3 requested = bridge ? bridge->command() : local_command;
    sim.set_command(ready ? requested : Vec3{});
    sim.step();
    auto stamp = sim.stamp_ns();
    if (lidar && args.lidar_motion == "rolling")
      history.add(stamp, sim.pose("lidar"));
    auto base = sim.pose();
    min_height = std::min(min_height, base.position[2]);
    double upright = sim.data()->site_xmat[9 * base_site + 8];
    max_tilt =
        std::max(max_tilt, std::acos(std::min(1., std::max(-1., upright))) *
                               180 / 3.14159265358979323846);
    if (observer)
      observer->on_step(sim, monotonic_seconds());
    if (bridge)
      bridge->publish_clock(stamp);
    ImuSample current = sim.imu();
    while (next_imu <= stamp) {
      ImuSample sample;
      sample.stamp_ns = next_imu;
      std::array<std::uint64_t, 2> source;
      if (next_imu == stamp) {
        sample = current;
        source = {stamp, stamp};
      } else {
        double f = double(next_imu - previous.stamp_ns) /
                   double(stamp - previous.stamp_ns);
        for (int i = 0; i < 3; i++) {
          sample.angular_velocity[i] =
              previous.angular_velocity[i] +
              f * (current.angular_velocity[i] - previous.angular_velocity[i]);
          sample.specific_force[i] =
              previous.specific_force[i] +
              f * (current.specific_force[i] - previous.specific_force[i]);
        }
        source = {previous.stamp_ns, stamp};
        interpolated++;
      }
      emit_imu(sample, source);
      imu_samples++;
      next_imu += 5000000;
    }
    previous = current;
    if (stamp % 100000000 == 0) {
      if (lidar) {
        double began = monotonic_seconds();
        std::array<std::uint64_t, 2> bounds{};
        std::uint64_t interval = 0;
        if (args.lidar_motion == "rolling") {
          lidar->scan(sim.data(), stamp - 100000000, args.rays, history,
                      last_scan);
          bounds = history.bounds();
          interval = history.maximum_interval(
              last_scan.stamp_ns, last_scan.stamp_ns + (args.rays - 1) * 5000);
        } else {
          PoseHistory scratch;
          auto pose = sim.pose("lidar");
          last_snapshot_pose = pose;
          scratch.add(stamp, pose);
          scratch.add(stamp + 100000000, pose);
          lidar->scan(sim.data(), stamp, args.rays, scratch, last_scan);
          std::fill(last_scan.offsets_ns.begin(), last_scan.offsets_ns.end(),
                    0);
          bounds = scratch.bounds();
        }
        if (args.rays < 20000)
          pattern->advance(20000 - args.rays);
        last_snapshot_pose = sim.pose("lidar");
        lidar_times.push_back(monotonic_seconds() - began);
        lidar_frames++;
        min_points = std::min(min_points, last_scan.xyz.size());
        max_points = std::max(max_points, last_scan.xyz.size());
        if (bridge)
          bridge->publish_scan(last_scan);
        if (observer)
          observer->on_scan(last_scan, bounds, interval);
      }
      auto lin = sim.linear_velocity(), ang = sim.angular_velocity();
      if (bridge)
        bridge->publish_odometry(stamp, base, lin, ang);
      if (observer)
        observer->on_odometry(stamp, base, lin, ang);
      artifacts.trajectory(stamp, base, lin, ang);
    }
    if (cameras && stamp >= next_camera) {
      double began = monotonic_seconds();
      cameras->render_into(sim.data(), sim.camera_pose(), frames);
      if (bridge) {
        bridge->publish_camera(stamp, frames.perspective);
        bridge->publish_camera(stamp, frames.panorama, true);
      }
      if (observer)
        observer->on_camera(stamp, frames);
      camera_times.push_back(monotonic_seconds() - began);
      camera_frames++;
      next_camera += camera_period;
    }
    if (viewer && step % 5 == 0)
      viewer->sync();
    if (args.realtime || args.viewer || args.ros2) {
      double remaining = stamp * 1e-9 - (monotonic_seconds() - start);
      if (remaining > 0)
        std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
    }
    if (observer && observer->should_stop(sim))
      break;
  }
  double wall = monotonic_seconds() - start;
  Json usage = loop_meter.report();
  Pose final = sim.pose();
  Vec3 displacement{};
  for (int i = 0; i < 3; i++)
    displacement[i] = final.position[i] - initial.position[i];
  Json scan_model = nullptr;
  if (lidar)
    scan_model = {
        {"acquisition_model", args.lidar_motion == "rolling"
                                  ? "rolling_static_environment"
                                  : "snapshot_end"},
        {"point_time_model", args.lidar_motion == "rolling"
                                 ? "uniform_from_csv_sample_order"
                                 : "all_returns_at_snapshot_stamp"},
        {"sample_rate_hz", 200000},
        {"robot_self_occlusion", false},
        {"dynamic_environment_supported", false},
        {"backend_implementation",
         args.backend == "cpu" ? "native_mj_ray" : "native_cuda_aot"},
        {"pose_interpolation", args.lidar_motion == "rolling"
                                   ? "linear_translation_short_arc_slerp"
                                   : "none"},
        {"environment_geom_count", lidar->environment_geom_count()},
        {"ray_buffer_capacity", lidar->ray_capacity()},
        {"pattern_timing_source",
         "synthetic CSV sample order; not recorded hardware timestamps"}};
  Json summary = {
      {"output", artifacts.output().string()},
      {"runtime_language", "C++17"},
      {"runtime_pid", getpid()},
      {"robot", args.simulation.robot},
      {"controller", sim.controller_name()},
      {"terrain", sim.terrain_metadata()},
      {"control_mode", args.simulation.control_mode},
      {"command_tracking", args.simulation.control_mode == "tracked"
                               ? sim.tracker_metadata()
                               : Json(nullptr)},
      {"deployment", sim.deployment_metadata()},
      {"backend", lidar ? Json(args.backend) : Json(nullptr)},
      {"backend_implementation",
       lidar ? (args.backend == "cpu" ? "native_mj_ray" : "native_cuda_aot")
             : "none"},
      {"simulation_seconds", sim.stamp_ns() * 1e-9},
      {"simulation_end_ns", sim.stamp_ns()},
      {"wall_seconds", wall},
      {"real_time_factor", sim.stamp_ns() * 1e-9 / wall},
      {"displacement_m", displacement},
      {"min_base_height_m", min_height},
      {"max_tilt_deg", max_tilt},
      {"final_applied_command_m_s_rad_s", sim.command()},
      {"ros_command_topic",
       bridge ? Json(bridge->command_topic()) : Json(nullptr)},
      {"ros_sensor_contract",
       bridge ? Json{{"robot_name", bridge->options().robot_name},
                     {"node_namespace", bridge->node_namespace()},
                     {"command_topic", bridge->command_topic()}}
              : Json(nullptr)},
      {"sensor_profile", args.sensors},
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
      {"scan_model", scan_model},
      {"imu_model",
       {{"rate_hz", 200},
        {"frame", "mid360_imu"},
        {"imu_position_in_lidar_m", {.011, .02329, -.04412}},
        {"imu_orientation_in_lidar_xyzw", {0., 0., 0., 1.}},
        {"sampling", "exact ticks or adjacent linear interpolation"},
        {"noise_bias", "none"}}},
      {"imu_samples", imu_samples},
      {"imu_interpolated_samples", interpolated},
      {"last_imu_stamp_ns", next_imu - 5000000},
      {"cameras", bool(cameras)},
      {"camera_hz", cameras ? args.camera_hz : 0},
      {"camera_frames_per_sensor", camera_frames},
      {"camera_pair_median_ms",
       camera_times.empty() ? Json(nullptr)
                            : Json(percentile(camera_times, .5) * 1000)},
      {"camera_pair_p95_ms", camera_times.empty()
                                 ? Json(nullptr)
                                 : Json(percentile(camera_times, .95) * 1000)},
      {"ros2", bool(bridge)},
      {"resources_loop", usage},
      {"resources_run", run_meter.report()},
      {"timing_scope",
       "physics, configured sensors, DDS and pacing; excludes "
       "model/device/renderer initialization and final artifact saving"},
      {"viewer", bool(viewer)}};
  summary["applied_command_limits"] = {
      {"vx_m_s", {-sim.command_limits()[0], sim.command_limits()[0]}},
      {"vy_m_s", {-sim.command_limits()[1], sim.command_limits()[1]}},
      {"wz_rad_s", {-sim.command_limits()[2], sim.command_limits()[2]}}};
  if (observer)
    observer->finish(summary);
  artifacts.close_streams();
  if (!args.summary_only) {
    if (cameras) {
      const auto camera_pose = sim.camera_pose();
      cameras->render_into(sim.data(), camera_pose, frames);
      artifacts.camera(sim.stamp_ns(), camera_pose, frames);
    }
    if (lidar && lidar_frames)
      artifacts.lidar(last_scan, last_snapshot_pose, history,
                      args.lidar_motion == "rolling", scan_model);
  }
  artifacts.summary(summary);
  if (bridge) {
    double deadline = monotonic_seconds() + .35;
    while (monotonic_seconds() < deadline)
      bridge->spin_once(.005);
    bridge->close();
  }
  std::cout << summary.dump(2) << '\n';
  return summary;
}
} // namespace simple_rbt
