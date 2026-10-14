// Exercise the real command -> tracker -> swerve -> contact/physics path.
#include "simple_rbt/core.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
  try {
    simple_rbt::SimulationOptions options;
    options.project_root = SIMPLE_RBT_PROJECT_ROOT;
    options.robot = "ranger_mini";
    options.control_mode = "tracked";
    simple_rbt::Simulation sim(options);
    const double dt = sim.model()->opt.timestep;
    const int base_site = mj_name2id(sim.model(), mjOBJ_SITE, "base_origin");
    auto phase = [&](const simple_rbt::Vec3 &command, double duration) {
      sim.set_command(command);
      simple_rbt::Vec3 mean{};
      const int ticks = std::lround(duration / dt);
      int samples = 0;
      for (int tick = 0; tick < ticks; ++tick) {
        sim.step();
        const auto linear = sim.linear_velocity();
        const auto angular = sim.angular_velocity();
        if (tick >= ticks * .6) {
          mean[0] += linear[0];
          mean[1] += linear[1];
          mean[2] += angular[2];
          ++samples;
        }
        if (sim.data()->site_xmat[9 * base_site + 8] < .99)
          throw std::runtime_error("Robot lost upright support");
      }
      for (int axis = 0; axis < 3; ++axis) {
        mean[axis] /= samples;
        if (std::abs(mean[axis] - command[axis]) >
            std::max(.05, .15 * std::abs(command[axis])))
          throw std::runtime_error("Velocity phase failed: " +
                                   simple_rbt::Json({command, mean}).dump());
      }
      std::cout << simple_rbt::Json({{"command", command},
                                     {"measured", mean}}).dump() << '\n';
    };
    phase({}, 2.);
    phase({.3, 0., 0.}, 2.);
    phase({-.25, 0., 0.}, 2.);
    phase({}, 1.8);
    // Restart and test steering motions that a forward/reverse probe misses.
    phase({0., .3, 0.}, 6.);
    phase({}, 1.8);
    phase({0., 0., .4}, 6.);
    phase({}, 1.8);
    phase({.3, .2, .3}, 6.);
    phase({}, 1.8);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
