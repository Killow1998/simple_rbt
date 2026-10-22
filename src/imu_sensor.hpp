#pragma once
#include "simple_rbt/types.hpp"
#include <algorithm>
#include <mujoco/mujoco.h>
#include <stdexcept>

namespace simple_rbt {
// A simulated accelerometer measures the specific impulse over a completed
// physics interval. Implicit integrators do not advance velocity by qacc * dt.
// Read site velocity (including the rigid lever arm), never base position.
class ImuSensor {
  int site_ = -1, gyro_ = -1, accel_ = -1;
  Vec3 velocity_{};
  Quat rotation_{};
  ImuSample sample_;

public:
  void configure(const mjModel *m, const char *site, const char *gyro,
                 const char *accel) {
    site_ = mj_name2id(m, mjOBJ_SITE, site);
    const int g = mj_name2id(m, mjOBJ_SENSOR, gyro);
    const int a = mj_name2id(m, mjOBJ_SENSOR, accel);
    if (site_ < 0 || g < 0 || a < 0)
      throw std::runtime_error("Missing IMU sensor");
    gyro_ = m->sensor_adr[g];
    accel_ = m->sensor_adr[a];
  }
  void reset(const mjModel *m, const mjData *d, std::uint64_t stamp) {
    mjtNum v[6];
    mj_objectVelocity(m, d, mjOBJ_SITE, site_, v, 0);
    std::copy_n(v + 3, 3, velocity_.begin());
    mju_mat2Quat(rotation_.data(), d->site_xmat + 9 * site_);
    sample_.stamp_ns = stamp;
    std::copy_n(d->sensordata + gyro_, 3, sample_.angular_velocity.begin());
    std::copy_n(d->sensordata + accel_, 3, sample_.specific_force.begin());
  }
  void advance(const mjModel *m, const mjData *d, std::uint64_t stamp) {
    if (stamp <= sample_.stamp_ns)
      throw std::runtime_error("IMU time must advance");
    const double dt = (stamp - sample_.stamp_ns) * 1e-9;
    mjtNum v[6], force[3];
    mj_objectVelocity(m, d, mjOBJ_SITE, site_, v, 0);
    for (int i = 0; i < 3; ++i) {
      force[i] = (v[i + 3] - velocity_[i]) / dt - m->opt.gravity[i];
      velocity_[i] = v[i + 3];
    }
    mju_mulMatTVec(sample_.specific_force.data(), d->site_xmat + 9 * site_,
                   force, 3, 3);
    Quat current{};
    mju_mat2Quat(current.data(), d->site_xmat + 9 * site_);
    mju_subQuat(sample_.angular_velocity.data(), current.data(),
                rotation_.data());
    for (double &v : sample_.angular_velocity)
      v /= dt;
    rotation_ = current;
    sample_.stamp_ns = stamp;
  }
  const ImuSample &sample() const { return sample_; }
};
} // namespace simple_rbt
