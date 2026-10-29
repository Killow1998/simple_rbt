#include "../controller_internal.hpp"
#include <array>
namespace simple_rbt {
namespace {
using namespace control_detail;
Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
Vec3 column(const double *r, int c) { return {r[c], r[3 + c], r[6 + c]}; }
Vec3 unit(Vec3 v) {
  mju_normalize3(v.data());
  return v;
}
class Flight final : public Controller {
  mjModel *m;
  mjData *d;
  int body;
  std::array<int, 4> motors{}, sites{};
  FlightCommand input;
  std::string mode = "grounded", previous_action = "hold";
  Vec3 target{}, integral{}, desired_velocity{};
  double heading = 0., ground = 0., landed_for = 0.;
  bool horizontal_hold = true, vertical_hold = true, saturated = false;
  std::array<double, 4> thrust{};
  bool stopped() const {
    return mode == "grounded" || mode == "disarmed" ||
           mode == "takeoff_blocked" || mode == "fault";
  }
  void stop_rotors() {
    desired_velocity = {};
    thrust = {};
    saturated = false;
    for (int actuator : motors)
      d->ctrl[actuator] = 0.;
  }
  double ray(const Vec3 &p, double direction) const {
    const unsigned char groups[6]{0, 1, 0, 0, 0, 0};
    const double axis[3]{0, 0, direction};
    return mj_ray(m, d, p.data(), axis, groups, 1, body, nullptr, nullptr);
  }
  Vec3 position() const {
    const double *p = d->xpos + 3 * body;
    return {p[0], p[1], p[2]};
  }
  double yaw() const {
    const auto *r = d->xmat + 9 * body;
    return std::atan2(r[3], r[0]);
  }
  void hold() {
    target = position();
    heading = yaw();
    horizontal_hold = vertical_hold = true;
  }
  void transition(const std::string &next) {
    if (mode == next)
      return;
    mode = next;
    integral = {};
    landed_for = 0.;
  }
  void allocate(double total, const Vec3 &torque) {
    // Recompute around the loaded centre of mass after configuration edits.
    double a[4][5]{};
    for (int i = 0; i < 4; ++i) {
      const auto *p = m->site_pos + 3 * sites[i];
      a[0][i] = 1.;
      a[1][i] = p[1] - m->body_ipos[3 * body + 1];
      a[2][i] = -(p[0] - m->body_ipos[3 * body]);
      a[3][i] = m->actuator_gear[6 * motors[i] + 5];
    }
    a[0][4] = total;
    for (int i = 0; i < 3; ++i)
      a[i + 1][4] = torque[i];
    for (int col = 0; col < 4; ++col) {
      int pivot = col;
      for (int row = col + 1; row < 4; ++row)
        if (std::abs(a[row][col]) > std::abs(a[pivot][col]))
          pivot = row;
      for (int j = 0; j < 5; ++j)
        std::swap(a[col][j], a[pivot][j]);
      const double divisor = a[col][col];
      if (std::abs(divisor) < 1e-9)
        throw std::runtime_error("Singular quadrotor allocation");
      for (int j = col; j < 5; ++j)
        a[col][j] /= divisor;
      for (int row = 0; row < 4; ++row)
        if (row != col) {
          const double f = a[row][col];
          for (int j = col; j < 5; ++j)
            a[row][j] -= f * a[col][j];
        }
    }
    saturated = false;
    for (int i = 0; i < 4; ++i) {
      thrust[i] = clamp(a[i][4], 0., m->actuator_ctrlrange[2 * motors[i] + 1]);
      saturated |= std::abs(thrust[i] - a[i][4]) > 1e-8;
      d->ctrl[motors[i]] = thrust[i];
    }
  }

public:
  explicit Flight(const ControllerContext &c)
      : m(c.model), d(c.data), body(id(m, mjOBJ_BODY, "x2")) {
    for (int i = 0; i < 4; ++i) {
      const auto name = "thrust" + std::to_string(i + 1);
      motors[i] = id(m, mjOBJ_ACTUATOR, name);
      sites[i] = id(m, mjOBJ_SITE, name);
    }
    state.phase = "flight";
    hold();
  }
  void set_flight(const FlightCommand &c) override {
    if (!supported_flight_command(c))
      throw std::invalid_argument("Invalid flight command");
    input = c;
    for (int i = 0; i < 3; ++i)
      input.velocity[i] =
          clamp(c.velocity[i], i == 2 ? -.6 : -1., i == 2 ? .6 : 1.);
    input.yaw_rate = clamp(c.yaw_rate, -1., 1.);
  }
  Json flight_status() const override {
    return {{"state", mode},
            {"frame", "world_enu"},
            {"velocity", desired_velocity},
            {"yaw_rate", input.yaw_rate},
            {"target_position", target},
            {"rotor_thrust_n", thrust},
            {"saturated", saturated},
            {"mass_kg", m->body_subtreemass[body]},
            {"feedback", "simulation_state"},
            {"velocity_limits", {1., 1., .6}},
            {"yaw_rate_limit", 1.},
            {"max_tilt_deg", 25.}};
  }
  void update(const Vec3 &) override {
    const double dt = m->opt.timestep;
    const Vec3 p = position();
    const auto *r = d->xmat + 9 * body;
    if (input.action == "disarm") {
      transition("disarmed");
      hold();
    }
    if (input.action == "takeoff" && previous_action != "takeoff" &&
        (mode == "grounded" || mode == "disarmed")) {
      Vec3 origin = p;
      origin[2] += .05;
      const double distance = ray(origin, -1);
      if (distance < 0 || distance > .35 || r[8] < .95) {
        transition("takeoff_blocked");
      } else {
        ground = origin[2] - distance;
        hold();
        target[2] = ground + input.takeoff_height;
        bool clear = true;
        for (double x : {-.3, 0., .3})
          for (double y : {-.31, 0., .31}) {
            const Vec3 top{p[0] + x, p[1] + y, p[2] + .24};
            const double ceiling = ray(top, 1);
            clear &= ceiling < 0 || ceiling > target[2] - p[2] + .08;
          }
        transition(clear ? "takeoff" : "takeoff_blocked");
      }
    }
    if (input.action == "hold" && previous_action != "hold" && !stopped()) {
      hold();
      transition("hover");
    }
    if (input.action == "land" && previous_action != "land" && !stopped()) {
      hold();
      transition("landing");
    }
    previous_action = input.action;
    if (stopped()) {
      stop_rotors();
      return;
    }
    if (r[8] < std::cos(70. * pi / 180.)) {
      transition("fault");
      stop_rotors();
      return;
    }
    double motion[6];
    mj_objectVelocity(m, d, mjOBJ_BODY, body, motion, 0);
    const Vec3 velocity{motion[3], motion[4], motion[5]};
    if (input.action == "velocity") {
      const bool xy = input.velocity[0] == 0. && input.velocity[1] == 0.;
      const bool z = input.velocity[2] == 0.;
      if (xy && !horizontal_hold) {
        target[0] = p[0];
        target[1] = p[1];
      }
      if (z && !vertical_hold)
        target[2] = p[2];
      horizontal_hold = xy;
      vertical_hold = z;
      desired_velocity = input.velocity;
      if (xy)
        for (int i = 0; i < 2; ++i)
          desired_velocity[i] = clamp(1.8 * (target[i] - p[i]), -1., 1.);
      if (z)
        desired_velocity[2] = clamp(1.8 * (target[2] - p[2]), -.6, .6);
      heading = std::remainder(heading + input.yaw_rate * dt, 2 * pi);
      transition(xy && z && input.yaw_rate == 0. ? "hover" : "velocity");
    } else {
      for (int i = 0; i < 3; ++i)
        desired_velocity[i] = clamp(1.8 * (target[i] - p[i]),
                                    i == 2 ? -.6 : -1., i == 2 ? .6 : 1.);
      if (mode == "landing")
        desired_velocity[2] = -.3;
      if (mode == "takeoff" && std::abs(target[2] - p[2]) < .04 &&
          std::abs(velocity[2]) < .08)
        transition("hover");
    }
    if (mode == "landing") {
      Vec3 origin = p;
      origin[2] += .05;
      const double distance = ray(origin, -1);
      landed_for =
          (distance >= 0 && distance < .085 && std::abs(velocity[2]) < .2)
              ? landed_for + dt
              : 0.;
      if (landed_for > .25) {
        transition("grounded");
        stop_rotors();
        return;
      }
    }
    Vec3 force{};
    const double mass = m->body_subtreemass[body];
    for (int i = 0; i < 3; ++i) {
      const double error = desired_velocity[i] - velocity[i];
      if (!saturated)
        integral[i] = clamp(integral[i] + error * dt, -.6, .6);
      force[i] = (clamp((i == 2 ? 4. : 2.5) * error + .6 * integral[i],
                        i == 2 ? -3. : -4., i == 2 ? 3. : 4.) -
                  m->opt.gravity[i]) *
                 mass;
    }
    const double horizontal = std::hypot(force[0], force[1]),
                 limit = force[2] * std::tan(25 * pi / 180.);
    if (horizontal > limit)
      for (int i = 0; i < 2; ++i)
        force[i] *= limit / horizontal;
    const Vec3 z = unit(force),
               y = unit(cross(z, {std::cos(heading), std::sin(heading), 0})),
               x = cross(y, z);
    Vec3 error{};
    for (int i = 0; i < 3; ++i) {
      const auto e = cross(column(r, i), i == 0 ? x : i == 1 ? y : z);
      for (int j = 0; j < 3; ++j)
        error[j] += .5 * e[j];
    }
    Vec3 local_error{}, omega{}, rate{}, torque{};
    mju_mulMatTVec(local_error.data(), r, error.data(), 3, 3);
    mju_mulMatTVec(omega.data(), r, motion, 3, 3);
    for (int i = 0; i < 3; ++i)
      rate[i] = clamp(
          6. * local_error[i] +
              r[6 + i] * (input.action == "velocity" ? input.yaw_rate : 0.),
          -3., 3.);
    double rotation[9], tensor[9]{};
    mju_quat2Mat(rotation, m->body_iquat + 4 * body);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k)
          tensor[3 * i + j] += rotation[3 * i + k] *
                               m->body_inertia[3 * body + k] *
                               rotation[3 * j + k];
    Vec3 acceleration{}, momentum{};
    for (int i = 0; i < 3; ++i)
      acceleration[i] = 18. * (rate[i] - omega[i]);
    mju_mulMatVec(torque.data(), tensor, acceleration.data(), 3, 3);
    mju_mulMatVec(momentum.data(), tensor, omega.data(), 3, 3);
    const auto gyroscopic = cross(omega, momentum);
    for (int i = 0; i < 3; ++i)
      torque[i] += gyroscopic[i];
    allocate(std::max(0., mju_dot3(force.data(), column(r, 2).data())), torque);
    ++state.pd_calls;
  }
};
} // namespace
std::unique_ptr<Controller> make_flight_controller(const ControllerContext &c) {
  return std::make_unique<Flight>(c);
}
} // namespace simple_rbt
