// Real wheel commands -> contact dynamics -> observed body motion and courses.
#include "simple_rbt/course.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace simple_rbt;
namespace {
SimulationOptions options(const std::string &mode, const std::string &terrain = "flat") {
  SimulationOptions result;
  result.project_root = SIMPLE_RBT_PROJECT_ROOT;
  result.robot = "turtlebot3_waffle_pi";
  result.control_mode = mode;
  result.terrain = terrain;
  return result;
}
Json motion(const std::string &mode) {
  Simulation sim(options(mode));
  const double dt = sim.model()->opt.timestep;
  const int base = mj_name2id(sim.model(), mjOBJ_SITE, "base_origin");
  Json phases = Json::array();
  auto phase = [&](const Vec3 &command, double seconds) {
    sim.set_command(command);
    const int ticks = std::lround(seconds / dt);
    Vec3 sum{};
    int samples = 0;
    double max_tilt = 0.;
    for (int step = 0; step < ticks; ++step) {
      sim.step();
      const double tilt = std::acos(std::clamp(sim.data()->site_xmat[9*base+8], -1., 1.));
      max_tilt = std::max(max_tilt, tilt);
      if (tilt > .2618)
        throw std::runtime_error(mode + ": Waffle Pi tilted beyond 15 degrees on flat ground");
      if (step >= ticks*2/3) {
        const Vec3 linear = sim.linear_velocity(), angular = sim.angular_velocity();
        sum[0] += linear[0]; sum[1] += linear[1]; sum[2] += angular[2];
        ++samples;
      }
    }
    for (int axis = 0; axis < 3; ++axis) {
      sum[axis] /= samples;
      const double tolerance = command == Vec3{} ? (axis == 2 ? .03 : .015)
          : std::max(axis == 2 ? .06 : .02, .1*std::abs(command[axis]));
      if (!std::isfinite(sum[axis]) || std::abs(sum[axis]-command[axis]) > tolerance)
        throw std::runtime_error(mode + ": Waffle Pi command/observed velocity mismatch " + Json({command,sum}).dump());
    }
    phases.push_back({{"command", command}, {"mean_body_velocity", sum}, {"max_tilt_rad", max_tilt}});
  };
  phase({}, 2.);
  phase({.18, 0., 0.}, 3.); phase({}, 1.);
  phase({-.15, 0., 0.}, 3.); phase({}, 1.);
  phase({0., 0., .6}, 3.); phase({}, 1.);
  phase({0., 0., -.6}, 3.); phase({}, 1.);
  phase({.12, 0., .4}, 3.); phase({}, 1.);
  return {{"control_mode", mode}, {"phases", phases}, {"passed", true}};
}
Json course(const std::string &mode, const std::string &terrain) {
  Simulation sim(options(mode, terrain));
  Json config{{"robot", "turtlebot3_waffle_pi"}, {"speed_m_s", .18},
              {"settle_s", 2.}, {"commanded_distance_budget_m", 8.}, {"friction", nullptr}};
  CourseMonitor monitor(sim, config);
  const double dt = sim.model()->opt.timestep;
  const int ticks = std::lround(48./dt);
  for (int step = 0; step < ticks; ++step) {
    sim.set_command(step*dt < 2. ? Vec3{} : Vec3{.18, 0., 0.});
    sim.step(); monitor.observe(sim);
    if (monitor.should_stop(sim)) break;
  }
  Json result = monitor.report();
  result["control_mode"] = mode;
  result["terrain"] = terrain;
  if (!result.at("success").get<bool>() || !result.at("speed_tracking").at("passed").get<bool>())
    throw std::runtime_error("Waffle Pi course failed: " + result.dump());
  return result;
}
} // namespace
int main(int argc, char **argv) {
  try {
    Json result{{"robot", "turtlebot3_waffle_pi"}, {"motion", Json::array()}, {"courses", Json::array()}};
    for (const std::string mode : {"native", "tracked"}) {
      result["motion"].push_back(motion(mode));
      for (const std::string terrain : {"flat", "ramp", "hills"})
        result["courses"].push_back(course(mode, terrain));
    }
    result["passed"] = true;
    if (argc == 2) std::ofstream(argv[1]) << result.dump(2) << '\n';
    std::cout << result.dump(2) << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
