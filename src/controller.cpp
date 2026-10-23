#include "controller.hpp"
#include <stdexcept>
namespace simple_rbt {
std::unique_ptr<Controller> make_arm_controller(const ControllerContext &);
std::unique_ptr<Controller> make_go2_controller(const ControllerContext &);
std::unique_ptr<Controller> make_go2w_controller(const ControllerContext &);
std::unique_ptr<Controller>
make_diffdrive_controller(const ControllerContext &);
std::unique_ptr<Controller>
make_scout_mini_controller(const ControllerContext &);
std::unique_ptr<Controller> make_fr_mini_controller(const ControllerContext &);
std::unique_ptr<Controller>
make_ridgeback_controller(const ControllerContext &);
std::unique_ptr<Controller>
make_ranger_mini_controller(const ControllerContext &);
std::unique_ptr<Controller> make_controller(const ControllerContext &c) {
  using Factory = std::unique_ptr<Controller> (*)(const ControllerContext &);
  struct Entry {
    const char *name;
    Factory make;
  };
  static constexpr Entry factories[]{
      {"joint_position", make_arm_controller},
      {"robot_lab", make_go2_controller},
      {"himloco", make_go2_controller},
      {"legacy", make_go2_controller},
      {"rl_sar", make_go2w_controller},
      {"none", make_diffdrive_controller},
      {"diff_drive", make_diffdrive_controller},
      {"skid_steer", make_scout_mini_controller},
      {"ackermann", make_fr_mini_controller},
      {"mecanum", make_ridgeback_controller},
      {"swerve", make_ranger_mini_controller},
  };
  for (const auto &entry : factories)
    if (c.name == entry.name)
      return entry.make(c);
  throw std::invalid_argument("Unsupported controller: " + c.name);
}
} // namespace simple_rbt
