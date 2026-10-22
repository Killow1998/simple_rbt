#pragma once
#include "simple_rbt/types.hpp"
#include <algorithm>
#include <mujoco/mujoco.h>
#include <stdexcept>
namespace simple_rbt {
// One causal, interval-averaged IMU measurement every 5 ms, timestamped at
// its centre. Translation and rotation use the same support and sensor frame.
class ImuSampler {
  ImuSample previous_;
  Quat rotation_, window_start_, centre_;
  Vec3 impulse_{};
  std::uint64_t next_;
  static Quat interpolate(const Quat &a, const Quat &b, double fraction) {
    Quat q = a;
    Vec3 delta{};
    mju_subQuat(delta.data(), b.data(), a.data());
    mju_quatIntegrate(q.data(), delta.data(), fraction);
    return q;
  }

public:
  static constexpr std::uint64_t period_ns = 5000000;
  ImuSampler(const ImuSample &initial, const Quat &rotation)
      : previous_(initial), rotation_(rotation), window_start_(rotation),
        centre_(rotation), next_(initial.stamp_ns + period_ns) {}
  template <class Emit>
  void advance(const ImuSample &current, const Quat &rotation, Emit emit) {
    if (current.stamp_ns <= previous_.stamp_ns)
      throw std::runtime_error("IMU input timestamps must increase");
    Vec3 world_force{};
    mju_rotVecQuat(world_force.data(), current.specific_force.data(),
                   rotation.data());
    const auto pose_at = [&](std::uint64_t stamp) {
      return interpolate(rotation_, rotation,
                         double(stamp - previous_.stamp_ns) /
                             double(current.stamp_ns - previous_.stamp_ns));
    };
    auto cursor = previous_.stamp_ns;
    while (cursor < current.stamp_ns) {
      const auto end = std::min(current.stamp_ns, next_);
      const auto midpoint = next_ - period_ns / 2;
      if (cursor <= midpoint && midpoint <= end)
        centre_ = pose_at(midpoint);
      for (int i = 0; i < 3; ++i)
        impulse_[i] += world_force[i] * ((end - cursor) * 1e-9);
      cursor = end;
      if (end != next_)
        continue;
      const Quat end_rotation = pose_at(end);
      Quat inverse{};
      mju_negQuat(inverse.data(), centre_.data());
      Vec3 force{}, angle{}, world_angle{};
      for (int i = 0; i < 3; ++i)
        force[i] = impulse_[i] / (period_ns * 1e-9);
      mju_subQuat(angle.data(), end_rotation.data(), window_start_.data());
      mju_rotVecQuat(world_angle.data(), angle.data(), window_start_.data());
      for (double &v : world_angle)
        v /= period_ns * 1e-9;
      ImuSample output;
      output.stamp_ns = midpoint;
      mju_rotVecQuat(output.specific_force.data(), force.data(),
                     inverse.data());
      mju_rotVecQuat(output.angular_velocity.data(), world_angle.data(),
                     inverse.data());
      emit(output, std::array<std::uint64_t, 2>{end - period_ns, end});
      window_start_ = end_rotation;
      impulse_ = {};
      next_ += period_ns;
    }
    previous_ = current;
    rotation_ = rotation;
  }
};
} // namespace simple_rbt
