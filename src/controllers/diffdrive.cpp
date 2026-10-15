#include "../controller_internal.hpp"
namespace simple_rbt {
namespace {
using namespace control_detail;
class DiffDriveController final : public Controller {
  mjData *d;
  double radius, half_track;
  int left = 0, right = 1;

public:
  explicit DiffDriveController(const ControllerContext &c) : d(c.data) {
    const auto &geometry = c.config.at("implemented_geometry");
    radius = geometry.at("wheel_radius_m").get<double>();
    half_track = geometry.at("track_m").get<double>() / 2;
    if (!std::isfinite(radius) || !std::isfinite(half_track) || radius <= 0. || half_track <= 0.)
      throw std::invalid_argument("Differential drive wheel radius and track must be positive");
    if (c.config.contains("wheel_actuators")) {
      const auto names = c.config.at("wheel_actuators").get<std::vector<std::string>>();
      if (names.size() != 2)
        throw std::invalid_argument("Differential drive requires two named wheel actuators");
      left = id(c.model, mjOBJ_ACTUATOR, names[0]);
      right = id(c.model, mjOBJ_ACTUATOR, names[1]);
      if (left == right)
        throw std::invalid_argument("Differential drive wheel actuators must differ");
    }
  }
  void update(const Vec3 &policy_command) override {
    double vx = policy_command[0], wz = policy_command[2];

    d->ctrl[left] = (vx - half_track * wz) / radius;
    d->ctrl[right] = (vx + half_track * wz) / radius;
    return;
  }
};
} // namespace
std::unique_ptr<Controller>
make_diffdrive_controller(const ControllerContext &c) {
  return std::make_unique<DiffDriveController>(c);
}
} // namespace simple_rbt
