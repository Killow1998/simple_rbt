#include "../controller_internal.hpp"
namespace simple_rbt {
namespace {
using namespace control_detail;
class DiffDriveController final : public Controller {
  mjData *d;
  double radius, half_track;

public:
  explicit DiffDriveController(const ControllerContext &c) : d(c.data) {
    const auto &geometry = c.config.at("implemented_geometry");
    radius = geometry.at("wheel_radius_m").get<double>();
    half_track = geometry.at("track_m").get<double>() / 2;
  }
  void update(const Vec3 &policy_command) override {
    double vx = policy_command[0], wz = policy_command[2];

    d->ctrl[0] = (vx - half_track * wz) / radius;
    d->ctrl[1] = (vx + half_track * wz) / radius;
    return;
  }
};
} // namespace
std::unique_ptr<Controller>
make_diffdrive_controller(const ControllerContext &c) {
  return std::make_unique<DiffDriveController>(c);
}
} // namespace simple_rbt
