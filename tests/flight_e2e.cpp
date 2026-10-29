#include "simple_rbt/core.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace simple_rbt;
namespace {
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
void run(Simulation &s, const FlightCommand &c, double seconds) {
  for (int i = 0; i < std::lround(seconds / s.model()->opt.timestep); ++i) {
    s.set_flight(c);
    s.step();
  }
}
double distance(const Vec3 &a, const Vec3 &b) {
  double n = 0;
  for (int i = 0; i < 3; ++i)
    n += (a[i] - b[i]) * (a[i] - b[i]);
  return std::sqrt(n);
}
} // namespace
int main() {
  try {
    SimulationOptions o;
    o.project_root = SIMPLE_RBT_PROJECT_ROOT;
    o.robot = "skydio_x2";
    o.camera_model = "d435i";
    Simulation s(o);
    run(s, {}, 1.);
    const auto ground = s.pose();
    FlightCommand takeoff;
    takeoff.action = "takeoff";
    run(s, takeoff, 8.);
    std::cout << "takeoff " << s.flight_status() << " pose "
              << s.pose().position[2] << std::endl;
    require(s.flight_status().at("state") == "hover",
            "takeoff did not reach hover");
    require(std::abs(s.pose().position[2] - 1.) < .08, "takeoff height");
    const auto initial = s.pose().position;
    double max_drift = 0.;
    for (int i = 0; i < 60; ++i) {
      run(s, {}, 1.);
      max_drift = std::max(max_drift, distance(s.pose().position, initial));
    }
    require(max_drift < .08, "60 second hover drift");
    FlightCommand command;
    command.action = "velocity";
    command.velocity = {.5, 0, 0};
    run(s, command, 4.);
    run(s, {}, 5.);
    require(s.pose().position[0] > 1.5,
            "horizontal velocity did not move the physical body");
    command.velocity = {0, 0, .3};
    run(s, command, 3.);
    run(s, {}, 5.);
    require(s.pose().position[2] > 1.6, "vertical velocity");
    command.velocity = {};
    command.yaw_rate = .5;
    run(s, command, 3.);
    run(s, {}, 3.);
    const auto q = s.pose().quaternion;
    const double yaw = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                  1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    require(std::abs(yaw - 1.5) < .15, "heading-rate tracking");
    command = {};
    command.action = "land";
    run(s, command, 12.);
    require(s.flight_status().at("state") == "grounded",
            "landing did not stop rotors");
    require(std::abs(s.pose().position[2] - ground.position[2]) < .04,
            "landing support");
    for (int i = 0; i < s.model()->nbody * 6; ++i)
      require(s.data()->xfrc_applied[i] == 0., "external wrench bypass");
    const auto landed = s.flight_status();
    run(s, takeoff, 6.);
    const auto height = s.pose().position[2];
    FlightCommand cut;
    cut.action = "disarm";
    run(s, cut, .005);
    auto rotors_stopped = [&]() {
      const auto status = s.flight_status();
      for (double thrust : status.at("rotor_thrust_n"))
        if (thrust != 0.)
          return false;
      for (int i = 0; i < s.model()->nu; ++i)
        if (s.data()->ctrl[i] != 0.)
          return false;
      return true;
    };
    require(rotors_stopped() && s.flight_status().at("state") == "disarmed",
            "motor cut state and telemetry");
    run(s, {}, .3);
    require(rotors_stopped() && s.pose().position[2] < height - .2,
            "hold rearmed cut motors");
    run(s, {}, 2.);
    run(s, takeoff, 6.);
    require(s.flight_status().at("state") == "hover",
            "explicit takeoff after motor cut");
    s.data()->qpos[3] = std::sqrt(.5);
    s.data()->qpos[4] = std::sqrt(.5);
    s.data()->qpos[5] = s.data()->qpos[6] = 0.;
    mj_forward(s.model(), s.data());
    run(s, {}, .005);
    require(s.flight_status().at("state") == "fault" && rotors_stopped(),
            "tilt fault did not cut actual and reported thrust together");
    std::cout << Json{{"passed", true},
                      {"hover_max_drift_m", max_drift},
                      {"yaw_rad", yaw},
                      {"motor_cut_and_tilt_fault", true},
                      {"final", landed}}
              << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
