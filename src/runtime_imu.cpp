#include "runtime_imu.hpp"
#include "artifacts.hpp"
#include "simple_rbt/event_log.hpp"
namespace simple_rbt {
RuntimeImus::RuntimeImus(const Simulation &sim)
    : imu_sampler(sim.imu(), sim.pose("mid360_imu").quaternion) {
  if (sim.has_camera_imu())
    camera_imu_sampler.emplace(sim.camera_imu(),
                               sim.pose("camera_imu").quaternion);
}
void RuntimeImus::advance(const Simulation &sim, bool mid360_enabled,
                          Transport *bridge, ArtifactWriter &artifacts,
                          RunObserver *observer) {
  imu_sampler.advance(
      sim.imu(), sim.pose("mid360_imu").quaternion,
      [&](const ImuSample &sample, std::array<std::uint64_t, 2> source) {
        if (sample.stamp_ns != sim.stamp_ns())
          ++interpolated;
        if (diagnostics::log().tracing())
          diagnostics::log().trace(
              "sensor.generated",
              {{"sample", diagnostics::sample_key("imu", sample.stamp_ns)}},
              sample.stamp_ns);
        if (bridge && mid360_enabled)
          bridge->publish_imu(sample);
        artifacts.imu(sample);
        if (observer)
          observer->on_imu(sample, source);
        ++imu_samples;
        last_stamp = sample.stamp_ns;
      });
  if (camera_imu_sampler)
    camera_imu_sampler->advance(
        sim.camera_imu(), sim.pose("camera_imu").quaternion,
        [&](const ImuSample &sample, auto) {
          if (diagnostics::log().tracing())
            diagnostics::log().trace(
                "sensor.generated",
                {{"sample",
                  diagnostics::sample_key("camera_imu", sample.stamp_ns)}},
                sample.stamp_ns);
          if (bridge)
            bridge->publish_camera_imu(sample);
        });
}
Json RuntimeImus::report() const {
  return {
      {"imu_model",
       {{"rate_hz", 200},
        {"frame", "mid360_imu"},
        {"imu_position_in_lidar_m", {.011, .02329, -.04412}},
        {"imu_orientation_in_lidar_xyzw", {0., 0., 0., 1.}},
        {"sampling", "200 Hz interval-averaged specific force and angular "
                     "rate; centre timestamp and sensor frame"},
        {"acceleration_source",
         "native site velocity increments, including rigid lever arm"},
        {"resampling", "world-frame impulse accumulation across physics ticks"},
        {"first_stamp_ns", 2500000},
        {"availability_delay_ns", 2500000},
        {"noise_bias", "none"}}},
      {"imu_samples", imu_samples},
      {"imu_interpolated_samples", interpolated},
      {"last_imu_stamp_ns", last_stamp},
      {"camera_imu_samples", camera_imu_sampler ? imu_samples : 0}};
}
} // namespace simple_rbt
