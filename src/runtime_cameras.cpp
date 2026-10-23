#include "runtime_cameras.hpp"
#include "artifacts.hpp"
#include "resources.hpp"
#include "simple_rbt/event_log.hpp"
#include <algorithm>
#include <cmath>
namespace simple_rbt {
RuntimeCameras::RuntimeCameras(Simulation &sim, const RunOptions &options)
    : args(options), camera_period(std::llround(1e9 / args.camera_hz)),
      next_camera(camera_period) {
  if (args.sensors == "full" &&
      (args.camera_enabled || args.depth_enabled || args.panorama_enabled ||
       args.simulation.external_camera_enabled)) {
    CameraConfig config;
    config.render_depth = args.depth_enabled;
    config.render_panorama = args.panorama_enabled;
    config.render_perspective = args.camera_enabled || args.depth_enabled;
    config.render_rgb = args.camera_enabled;
    if (args.simulation.external_camera_enabled)
      config.external_camera =
          std::array<OpticalPose, 2>{sim.camera_pose("external_camera_color"),
                                     sim.camera_pose("external_camera_depth")};
    cameras = std::make_unique<CameraWorker>(sim.model(), sim.data(), config,
                                             sim.camera_pose(),
                                             sim.camera_pose("camera_depth"));
  }
}
void RuntimeCameras::advance(Simulation &sim, Transport *bridge,
                             RunObserver *observer) {
  const auto stamp = sim.stamp_ns();
  if (cameras && stamp >= next_camera) {
    finish(bridge, observer);
    cameras->submit(sim.data(), sim.camera_pose(),
                    sim.camera_pose("camera_depth"));
    pending_stamp = stamp;
    next_camera += camera_period;
  }
}
void RuntimeCameras::publish_ready(Transport *bridge, RunObserver *observer) {
  if (pending_stamp && cameras->ready())
    finish(bridge, observer);
}
void RuntimeCameras::finish(Transport *bridge, RunObserver *observer) {
  if (pending_stamp) {
    cameras->wait();
    const auto stamp = pending_stamp;
    const auto &frames = cameras->frames();
    double began = monotonic_seconds();
    if (args.camera_enabled || args.depth_enabled)
      diagnostics::log().trace(
          "sensor.generated",
          {{"sample", diagnostics::sample_key("camera", stamp)},
           {"duration_ms", cameras->seconds() * 1000}},
          stamp);
    if (args.panorama_enabled)
      diagnostics::log().trace(
          "sensor.generated",
          {{"sample", diagnostics::sample_key("panorama", stamp)}}, stamp);
    if (args.simulation.external_camera_enabled)
      diagnostics::log().trace(
          "sensor.generated",
          {{"sample", diagnostics::sample_key("external_camera", stamp)}},
          stamp);
    if (bridge) {
      if (args.simulation.external_camera_enabled)
        bridge->publish_external_camera(stamp, frames.external);
      if (args.camera_enabled || args.depth_enabled)
        bridge->publish_camera(stamp, frames.perspective);
      if (args.panorama_enabled)
        bridge->publish_camera(stamp, frames.panorama, true);
    }
    if (observer)
      observer->on_camera(stamp, frames);
    camera_times.push_back(cameras->seconds() + monotonic_seconds() - began);
    camera_frames++;
    pending_stamp = 0;
  }
}
Json RuntimeCameras::report() const {
  return {{"cameras", bool(cameras)},
          {"camera_hz", cameras ? args.camera_hz : 0},
          {"camera_model", args.simulation.camera_model},
          {"external_camera_frames",
           args.simulation.external_camera_enabled ? camera_frames : 0},

          {"camera_frames_per_sensor", camera_frames},
          {"camera_pair_median_ms",
           camera_times.empty() ? Json(nullptr)
                                : Json(percentile(camera_times, .5) * 1000)},
          {"camera_pair_p95_ms",
           camera_times.empty() ? Json(nullptr)
                                : Json(percentile(camera_times, .95) * 1000)}};
}
void RuntimeCameras::save(Simulation &sim, ArtifactWriter &artifacts) {
  if (!cameras)
    return;
  const auto pose = sim.camera_pose();
  cameras->submit(sim.data(), pose, sim.camera_pose("camera_depth"));
  cameras->wait();
  std::optional<std::array<OpticalPose, 2>> external;
  if (args.simulation.external_camera_enabled)
    external =
        std::array<OpticalPose, 2>{sim.camera_pose("external_camera_color"),
                                   sim.camera_pose("external_camera_depth")};
  artifacts.camera(sim.stamp_ns(), pose, sim.camera_pose("camera_depth"),
                   cameras->frames(), external);
}
} // namespace simple_rbt
