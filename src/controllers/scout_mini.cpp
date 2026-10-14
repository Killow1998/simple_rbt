#include "../controller_internal.hpp"
namespace simple_rbt {
namespace {
using namespace control_detail;
class ScoutMiniController final : public Controller, private WheelBinding {
  mjModel *m;
  mjData *d;
  int gyro_address;
  double radius, track, rate_limit, yaw_kp, yaw_ki, filter_time, integral_limit;
  double yaw_integral = 0, filtered_yaw = 0;

public:
  explicit ScoutMiniController(const ControllerContext &c)
      : WheelBinding(c.model), m(c.model), d(c.data) {
    const auto &geometry = c.config.at("implemented_geometry");
    const auto &control = c.config.at("control");
    const auto &feedback = control.at("yaw_feedback");
    radius = geometry.at("wheel_radius_m").get<double>();
    track = geometry.at("track_m").get<double>();
    rate_limit = control.at("wheel_velocity_limit_rad_s").get<double>();
    yaw_kp = feedback.at("kp").get<double>();
    yaw_ki = feedback.at("ki_per_s").get<double>();
    filter_time = feedback.at("measurement_filter_time_s").get<double>();
    integral_limit = feedback.at("integral_limit_rad_s").get<double>();
    gyro_address = m->sensor_adr[id(m, mjOBJ_SENSOR, "gyro")];
  }
  void update(const Vec3 &command) override {
    const double vx = command[0], wz = command[2],
                 measured = d->sensordata[gyro_address + 2];
    filtered_yaw +=
        -std::expm1(-m->opt.timestep / filter_time) * (measured - filtered_yaw);
    double error = wz - filtered_yaw;
    double candidate =
        std::abs(vx) < 1e-12 && std::abs(wz) < 1e-12
            ? 0
            : clamp(yaw_integral + yaw_ki * error * m->opt.timestep,
                    -integral_limit, integral_limit);
    double effective = wz + yaw_kp * error + candidate;
    const std::array<double, 4> targets{(vx - track * effective / 2) / radius,
                                        (vx + track * effective / 2) / radius,
                                        (vx - track * effective / 2) / radius,
                                        (vx + track * effective / 2) / radius};
    double max = 0;
    for (double value : targets)
      max = std::max(max, std::abs(value));
    double scale = max == 0 ? 1 : std::min(1., rate_limit / max);
    if (scale == 1 || candidate * error <= 0)
      yaw_integral = candidate;
    for (int i = 0; i < 4; ++i)
      d->ctrl[actuators[i]] = targets[i] * scale;
  }
};
} // namespace
std::unique_ptr<Controller>
make_scout_mini_controller(const ControllerContext &c) {
  return std::make_unique<ScoutMiniController>(c);
}
} // namespace simple_rbt
