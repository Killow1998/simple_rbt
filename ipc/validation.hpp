#pragma once
#include "simple_rbt/types.hpp"
#include <cmath>
#include <stdexcept>
namespace simple_rbt::ipc {
inline void finite_vector(const Vec3 &v) {
  for (double x : v)
    if (!std::isfinite(x))
      throw std::invalid_argument("vector must contain three finite values");
}
inline void unit_pose(const Pose &pose) {
  finite_vector(pose.position);
  double norm2 = 0.;
  for (double x : pose.quaternion) {
    if (!std::isfinite(x))
      throw std::invalid_argument("quaternion must be finite");
    norm2 += x * x;
  }
  if (std::abs(std::sqrt(norm2) - 1.) > 1.e-5)
    throw std::invalid_argument("WXYZ quaternion must have unit norm");
}
} // namespace simple_rbt::ipc
