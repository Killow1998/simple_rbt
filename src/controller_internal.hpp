#pragma once
#include "controller.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace simple_rbt::control_detail {
constexpr double pi = 3.141592653589793238462643383279502884;
inline double clamp(double x, double lo, double hi) {
  return std::min(std::max(x, lo), hi);
}
inline int id(const mjModel *m, int type, const std::string &n) {
  int i = mj_name2id(m, type, n.c_str());
  if (i < 0)
    throw std::runtime_error("Missing MuJoCo name " + n);
  return i;
}
struct JointBinding {
  std::vector<int> qpos, qvel, actuators;
  void add(const mjModel *m, const std::string &name) {
    int j = id(m, mjOBJ_JOINT, name);
    qpos.push_back(m->jnt_qposadr[j]);
    qvel.push_back(m->jnt_dofadr[j]);
    int a = -1;
    for (int i = 0; i < m->nu; ++i)
      if (m->actuator_trnid[2 * i] == j) {
        if (a != -1)
          throw std::runtime_error("Ambiguous actuator");
        a = i;
      }
    if (a < 0)
      throw std::runtime_error("Missing actuator for " + name);
    actuators.push_back(a);
  }
};
struct WheelBinding {
  std::vector<int> actuators, steering_q, steering_a;
  WheelBinding(const mjModel *m, bool rear_drive_only = false,
               bool front_steering = false, bool all_steering = false) {
    for (const std::string name :
         {"front_left", "front_right", "rear_left", "rear_right"}) {
      (void)id(m, mjOBJ_JOINT, name + "_wheel_joint");
      if (!rear_drive_only || name.rfind("rear", 0) == 0)
        actuators.push_back(id(m, mjOBJ_ACTUATOR, name + "_wheel"));
      if (all_steering || (front_steering && name.rfind("front", 0) == 0)) {
        int s = id(m, mjOBJ_JOINT, name + "_steering_joint");
        steering_q.push_back(m->jnt_qposadr[s]);
        steering_a.push_back(id(m, mjOBJ_ACTUATOR, name + "_steering"));
      }
    }
  }
};
} // namespace simple_rbt::control_detail
