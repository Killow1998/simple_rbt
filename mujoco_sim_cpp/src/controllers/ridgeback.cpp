#include "../controller_internal.hpp"
namespace simple_rbt {
namespace {
using namespace control_detail;
class RidgebackController final : public Controller, private WheelBinding {
  mjData *d;
  double radius, turn_lever, rate_limit;

public:
  explicit RidgebackController(const ControllerContext &c)
      : WheelBinding(c.model), d(c.data) {
    const auto &g = c.config.at("implemented_geometry");
    radius = g.at("wheel_radius_m").get<double>();
    turn_lever = g.at("wheelbase_m").get<double>() / 2 +
                 g.at("track_m").get<double>() / 2;
    rate_limit =
        c.config.at("control").at("hub_speed_limit_rad_s").get<double>();
  }
  void update(const Vec3 &policy_command) override {
    double vx = policy_command[0], vy = policy_command[1],
           wz = policy_command[2];
    std::array<double, 4> targets{};
    double maxrate = 0;

    double yaw = turn_lever * wz;
    targets = {(vx - vy - yaw) / radius, (vx + vy + yaw) / radius,
               (vx + vy - yaw) / radius, (vx - vy + yaw) / radius};
    maxrate = rate_limit;

    double max = 0;
    for (double v : targets)
      max = std::max(max, std::abs(v));
    double scale = max == 0 ? 1 : std::min(1., maxrate / max);
    for (int i = 0; i < 4; i++)
      d->ctrl[actuators[i]] = targets[i] * scale;
  }
};
} // namespace
std::unique_ptr<Controller>
make_ridgeback_controller(const ControllerContext &c) {
  return std::make_unique<RidgebackController>(c);
}
} // namespace simple_rbt
