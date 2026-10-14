#include "../controller_internal.hpp"
#include <limits>
namespace simple_rbt {
namespace {
using namespace control_detail;
class RangerMiniController final : public Controller, private WheelBinding {
  mjData *d;
  double radius, wheelbase, track, rate_limit, steering_gain,
      steering_rate_limit, alignment_release;
  std::array<double, 2> steering_range;
  bool braking = false;
  std::array<double, 4> steering_targets{};

public:
  explicit RangerMiniController(const ControllerContext &c)
      : WheelBinding(c.model, false, false, true), d(c.data) {
    const auto &g = c.config.at("implemented_geometry");
    const auto &control = c.config.at("control");
    radius = g.at("wheel_radius_m").get<double>();
    wheelbase = g.at("wheelbase_m").get<double>();
    track = g.at("track_m").get<double>();
    rate_limit = control.at("wheel_velocity_limit_rad_s").get<double>();
    steering_gain = control.at("steering_position_gain_per_s").get<double>();
    steering_rate_limit =
        control.at("steering_rate_target_limit_rad_s").get<double>();
    alignment_release = control.at("alignment_release_rad").get<double>();
    steering_range =
        control.at("steering_range_rad").get<std::array<double, 2>>();
  }
  void update(const Vec3 &policy_command) override {
    double vx = policy_command[0], vy = policy_command[1],
           wz = policy_command[2];
    std::array<double, 4> targets{};
    double maxrate = 0;

    bool brake = std::max({std::abs(vx), std::abs(vy), std::abs(wz)}) < 1e-12,
         aligned = true;
    for (int i = 0; i < 4; i++) {
      double current = d->qpos[steering_q[i]],
             x = (i < 2 ? 1 : -1) * wheelbase / 2,
             y = (i % 2 == 0 ? 1 : -1) * track / 2, ux = vx - wz * y,
             uy = vy + wz * x, speed = std::hypot(ux, uy);
      bool active = false;
      if (brake) {
        if (!braking)
          steering_targets[i] = current;
      } else if (speed < 1e-10)
        steering_targets[i] = current;
      else {
        active = true;
        double a = std::atan2(uy, ux),
               best = std::numeric_limits<double>::infinity(), sign = 1;
        for (int k : {0, -1, 1, -2, 2}) {
          double candidate = a + k * pi;
          if (candidate < steering_range[0] || candidate > steering_range[1])
            continue;
          double delta = std::abs(candidate - current);
          if (delta < best) {
            best = delta;
            steering_targets[i] = candidate;
            sign = k % 2 ? -1 : 1;
          }
        }
        targets[i] = sign * speed / radius;
      }
      double error = steering_targets[i] - current;
      d->ctrl[steering_a[i]] = clamp(steering_gain * error,
                                     -steering_rate_limit, steering_rate_limit);
      if (active && std::abs(error) > alignment_release)
        aligned = false;
    }
    braking = brake;
    maxrate = rate_limit;
    if (!aligned)
      targets.fill(0);

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
make_ranger_mini_controller(const ControllerContext &c) {
  return std::make_unique<RangerMiniController>(c);
}
} // namespace simple_rbt
