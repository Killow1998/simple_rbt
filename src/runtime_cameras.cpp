#include "artifacts.hpp"
#include "resources.hpp"
#include "runtime_cameras.hpp"
#include "simple_rbt/event_log.hpp"
#include <algorithm>
#include <cmath>
namespace simple_rbt {
RuntimeCameras::RuntimeCameras(Simulation &sim, const RunOptions &options)
    : args(options), camera_period(std::llround(1e9 / args.camera_hz)),
      next_camera(camera_period) {
  if (args.sensors == "full" &&
      (args.camera_enabled || args.depth_enabled || args.panorama_enabled)) {
    CameraConfig config;
    config.render_depth = args.depth_enabled;
    config.render_panorama = args.panorama_enabled;
    config.render_perspective = args.camera_enabled || args.depth_enabled;
    config.render_rgb = args.camera_enabled;
    cameras = std::make_unique<CameraRig>(sim.model(), config);
    cameras->render_into(sim.data(), sim.camera_pose(),
                         sim.camera_pose("camera_depth"), frames);
  }
}
void RuntimeCameras::advance(Simulation &sim, Transport *bridge,
                             RunObserver *observer) {
  const auto stamp = sim.stamp_ns();
  if (cameras && stamp >= next_camera) {
    double began = monotonic_seconds();
    cameras->render_into(sim.data(), sim.camera_pose(),
                         sim.camera_pose("camera_depth"), frames);
    if (args.camera_enabled || args.depth_enabled)
      diagnostics::log().trace(
          "sensor.generated",
          {{"sample", diagnostics::sample_key("camera", stamp)},
           {"duration_ms", (monotonic_seconds() - began) * 1000}},
          stamp);
    if (args.panorama_enabled)
      diagnostics::log().trace(
          "sensor.generated",
          {{"sample", diagnostics::sample_key("panorama", stamp)}}, stamp);
    if (bridge) {
      if (args.camera_enabled || args.depth_enabled)
        bridge->publish_camera(stamp, frames.perspective);
      if (args.panorama_enabled)
        bridge->publish_camera(stamp, frames.panorama, true);
    }
    if (observer)
      observer->on_camera(stamp, frames);
    camera_times.push_back(monotonic_seconds() - began);
    camera_frames++;
    next_camera += camera_period;
  }
}
Json RuntimeCameras::report() const {
  return {{"cameras", bool(cameras)},
          {"camera_hz", cameras ? args.camera_hz : 0},
          {"camera_model", args.simulation.camera_model},

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
  cameras->render_into(sim.data(), pose, sim.camera_pose("camera_depth"),
                       frames);
  artifacts.camera(sim.stamp_ns(), pose, sim.camera_pose("camera_depth"),
                   frames);
}
} // namespace simple_rbt
