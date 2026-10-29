#include "simple_rbt/runtime.hpp"
#include "artifacts.hpp"
#include "resources.hpp"
#include "runtime_cameras.hpp"
#include "runtime_imu.hpp"
#include "runtime_lidar.hpp"
#include "sensor_devices.hpp"
#include "simple_rbt/event_log.hpp"
#include "simple_rbt/transport.hpp"
#include "simple_rbt/viewer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <thread>
#include <unistd.h>

namespace simple_rbt {
Json run_simulation(const RunOptions &options, RunObserver *observer,
                    const TransportFactory &transport_factory) {
  RunOptions args = options;
  const auto &device = lidar_spec(args.simulation.lidar_model);
  if (args.rays == 0) args.rays = device.point_rate / 10;
  ResourceMeter run_meter;
  if (!std::isfinite(args.duration) || args.duration <= 0 ||
      !std::isfinite(args.settle) || args.settle < 0 ||
      !std::isfinite(args.camera_hz) || args.camera_hz <= 0 ||
      args.camera_hz > 30 || args.rays < 1 || args.rays > device.point_rate / 10)
    throw std::invalid_argument(
        "Invalid duration, settle, camera rate or ray count");
  if (args.sensors != "physics" && args.sensors != "lidar" &&
      args.sensors != "full")
    throw std::invalid_argument("sensors must be physics/lidar/full");
  if (args.backend != "cpu" && args.backend != "cuda" &&
      args.backend != "warp" && args.backend != "auto")
    throw std::invalid_argument("backend must be auto/cpu/cuda (warp is a "
                                "compatibility alias for native CUDA)");
  if (args.lidar_motion != "rolling" && args.lidar_motion != "snapshot")
    throw std::invalid_argument("lidar-motion must be rolling/snapshot");
  diagnostics::log().event("simulation.loading",
                           {{"robot", args.simulation.robot},
                            {"controller", args.simulation.controller},
                            {"backend", args.backend}});
  args.simulation.lidar_payload_enabled = args.mid360_enabled;
  args.simulation.camera_payload_enabled = args.camera_enabled || args.depth_enabled ||
      args.panorama_enabled || args.simulation.camera_model == "d435i";
  Simulation sim(args.simulation);
  diagnostics::log().event("simulation.loaded");
  for (int id = 0; id < sim.model()->ngeom; ++id) {
    const char *raw = mj_id2name(sim.model(), mjOBJ_GEOM, id);
    const std::string name = raw ? raw : "";
    const bool disabled =
        (name.rfind(std::string("visual_device_") + device.name, 0) == 0 && !args.mid360_enabled) ||
        (name.rfind("visual_device_d435", 0) == 0 &&
         !(args.camera_enabled || args.depth_enabled || args.panorama_enabled ||
           sim.has_camera_imu()));
    if (disabled)
      sim.model()->geom_rgba[4 * id + 3] = 0;
  }
  const auto &m = *sim.model();
  const int base_site = mj_name2id(&m, mjOBJ_SITE, "base_origin");
  ArtifactWriter artifacts(args.simulation.project_root, args.output,
                           args.summary_only, args.output_precreated);
  artifacts.model(sim.xml());
  std::unique_ptr<Transport> bridge;
  if (transport_factory) {
    bridge = transport_factory(sensor_extrinsics(sim, true));
    if (!bridge)
      throw std::runtime_error("Transport factory returned null");
  }
  if (observer) {
    observer->configure(sim, bridge.get());
    artifacts.model(sim.xml());
  }
  if (observer && observer->should_stop(sim)) {
    Json cancelled = {{"session_state", "closed"},
                      {"resources_run", run_meter.report()}};
    observer->finish(cancelled);
    artifacts.summary(cancelled);
    if (bridge)
      bridge->close();
    return cancelled;
  }
  RuntimeLidar lidar(sim, args, bridge.get());
  RuntimeCameras cameras(sim, args);
  std::unique_ptr<Viewer> viewer;
  if (args.viewer)
    viewer = std::make_unique<Viewer>(sim.model(), sim.data());
  artifacts.open_streams();
  if (observer)
    observer->before_loop(sim, bridge.get());
  RuntimeImus imus(sim);
  Pose initial = sim.pose();
  double min_height = initial.position[2], max_tilt = 0;
  std::optional<double> policy_origin;
  ResourceMeter loop_meter;
  double start = monotonic_seconds();
  if (bridge)
    bridge->publish_clock(0);
  std::size_t steps = std::ceil(args.duration / m.opt.timestep);
  const Vec3 local_command = args.simulation_command;
  const bool joint_control = robot_spec(args.simulation.robot).joint_control;
  const bool flight_control = robot_spec(args.simulation.robot).flight_control;
  Vec3 last_applied{};
  Json last_flight_state;
  double last_sensor_log = monotonic_seconds();
  for (std::size_t step = 0; args.indefinite || step < steps;) {
    cameras.publish_ready(bridge.get(), observer);
    if (observer && observer->should_stop(sim))
      break;
    double before = monotonic_seconds();
    if (observer && !observer->before_step(sim, bridge.get())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      start += monotonic_seconds() - before;
      continue;
    }

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
    if (flight_control)
      sim.set_flight(ready && bridge ? bridge->flight_command() : FlightCommand{});
    if (joint_control) {
      const auto joint_command =
          ready && bridge ? bridge->joint_command() : std::nullopt;
      if (joint_command)
        sim.set_joints(*joint_command);
      else
        sim.hold_joints();
    }
    if (sim.command() != last_applied) {
      last_applied = sim.command();
      diagnostics::log().event("command.applied", {{"velocity", last_applied}},
                               sim.stamp_ns());
    }
    sim.step();
    auto stamp = sim.stamp_ns();
    lidar.record_pose(sim);
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
    if (joint_control && bridge && stamp % 20000000 == 0)
      bridge->publish_joints(stamp, sim.joints());
    imus.advance(sim, args.mid360_enabled, bridge.get(), artifacts, observer);
    if (stamp % 100000000 == 0) {
      if (flight_control) {
        const auto status = sim.flight_status();
        const Json state{{"state", status.at("state")},
                         {"saturated", status.at("saturated")}};
        if (state != last_flight_state) {
          diagnostics::log().event("flight.state", status, stamp);
          last_flight_state = state;
        }
      }
      lidar.scan(sim, bridge.get(), observer);
      auto lin = sim.linear_velocity(), ang = sim.angular_velocity();
      if (bridge)
        bridge->publish_odometry(stamp, base, lin, ang);
      if (observer)
        observer->on_odometry(stamp, base, lin, ang);
      artifacts.trajectory(stamp, base, lin, ang);
    }
    cameras.advance(sim, bridge.get(), observer);
    if (monotonic_seconds() - last_sensor_log >= 5) {
      diagnostics::log().event("sensors.summary",
                               {{"imu_samples", imus.count()},
                                {"lidar_scans", lidar.frames()},
                                {"camera_frames", cameras.count()},
                                {"last_scan_ms", lidar.last_ms()}},
                               stamp);
      last_sensor_log = monotonic_seconds();
    }
    if (viewer && step % 5 == 0)
      viewer->sync();
    if (args.realtime || args.viewer || bridge) {
      double remaining = stamp * 1e-9 - (monotonic_seconds() - start);
      if (remaining > 0)
        std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
    }
    if (observer && observer->should_stop(sim))
      break;
    ++step;
  }
  cameras.finish(bridge.get(), observer);
  double wall = monotonic_seconds() - start;
  Json usage = loop_meter.report();
  Pose final = sim.pose();
  Vec3 displacement{};
  for (int i = 0; i < 3; i++)
    displacement[i] = final.position[i] - initial.position[i];
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
      {"ros2", bridge && std::string(bridge->backend()) == "ros2"},
      {"ros1", bridge && std::string(bridge->backend()) == "ros1"},
      {"resources_loop", usage},
      {"resources_run", run_meter.report()},
      {"timing_scope",
       "physics, configured sensors, DDS and pacing; excludes "
       "model/device/renderer initialization and final artifact saving"},
      {"viewer", bool(viewer)}};
  summary.update(lidar.report());
  summary.update(cameras.report());
  summary.update(imus.report());
  summary["applied_command_limits"] = {
      {"vx_m_s", {-sim.command_limits()[0], sim.command_limits()[0]}},
      {"vy_m_s", {-sim.command_limits()[1], sim.command_limits()[1]}},
      {"wz_rad_s", {-sim.command_limits()[2], sim.command_limits()[2]}}};
  if (observer)
    observer->finish(summary);
  artifacts.close_streams();
  if (!args.summary_only) {
    cameras.save(sim, artifacts);
    lidar.save(artifacts);
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
