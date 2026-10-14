#include "../controller_internal.hpp"
namespace simple_rbt {
namespace {
using namespace control_detail;
class FrMiniController final : public Controller, private WheelBinding {
  mjData *d;
  double radius, track, wheelbase, rate_limit, curvature_limit;

public:
  explicit FrMiniController(const ControllerContext &c)
      : WheelBinding(c.model, true, true), d(c.data) {
    const auto &g = c.config.at("implemented_geometry");
    const auto &spec = c.config.at("manufacturer_specifications");
    radius = g.at("tire_local_semiaxes_m").at(0).get<double>();
    track = spec.at("wheel_track_m").get<double>();
    wheelbase = g.at("front_axle_x_in_base_m").get<double>() -
                g.at("rear_axle_x_in_base_m").get<double>();
    rate_limit =
        c.config.at("control").at("wheel_rate_limit_rad_s").get<double>();
    const double turning_radius =
        robot_spec(c.options.robot).min_turning_radius;
    if (turning_radius <= 0)
      throw std::runtime_error(
          "Ackermann robot requires a positive turning radius");
    curvature_limit = 1 / turning_radius;
  }
  void update(const Vec3 &policy_command) override {
    double vx = policy_command[0], wz = policy_command[2], maxrate = 0;

    double k = std::abs(vx) > 1e-9
                   ? clamp(wz / vx, -curvature_limit, curvature_limit)
                   : 0.;
    double yaw = vx * k;
    double left = vx - yaw * track / 2, right = vx + yaw * track / 2;
    maxrate = rate_limit;
    double max = std::max(std::abs(left / radius), std::abs(right / radius));
    double scale = max == 0 ? 1 : std::min(1., maxrate / max);
    d->ctrl[actuators[0]] = left / radius * scale;
    d->ctrl[actuators[1]] = right / radius * scale;
    d->ctrl[steering_a[0]] = std::atan(wheelbase * k / (1 - track * k / 2));
    d->ctrl[steering_a[1]] = std::atan(wheelbase * k / (1 + track * k / 2));
    return;
  }
};
} // namespace
std::unique_ptr<Controller>
make_fr_mini_controller(const ControllerContext &c) {
  return std::make_unique<FrMiniController>(c);
}
} // namespace simple_rbt
