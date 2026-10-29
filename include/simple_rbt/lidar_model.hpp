#pragma once
#include "simple_rbt/types.hpp"
#include <cstdint>
#include <stdexcept>
#include <string>

namespace simple_rbt {
enum class LidarModel { Mid360, Avia };
struct LidarSpec {
  const char *name, *csv;
  std::uint32_t point_rate, lines;
  double minimum_range, maximum_range;
  Vec3 imu_offset;
};
inline const LidarSpec &lidar_spec(LidarModel model) {
  static const LidarSpec mid{"mid360", "mid360-real-centr.csv", 200000, 4,
                              .1, 40., {.011, .02329, -.04412}};
  static const LidarSpec avia{"avia", "avia.csv", 240000, 6,
                             1., 190., {-.04165, -.02326, .02840}};
  return model == LidarModel::Avia ? avia : mid;
}
inline LidarModel parse_lidar_model(const std::string &name) {
  if (name == "mid360") return LidarModel::Mid360;
  if (name == "avia") return LidarModel::Avia;
  throw std::invalid_argument("LiDAR model must be mid360 or avia");
}
} // namespace simple_rbt
