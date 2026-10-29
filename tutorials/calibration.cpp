#include "calibration.hpp"
#include "simple_rbt/lidar_model.hpp"
#include <cmath>
namespace simple_rbt::tutorial {
namespace {
Mat3 rotation(const Quat &q) {
  const auto w = q[0], x = q[1], y = q[2], z = q[3];
  return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
          2 * (x * z + w * y),     2 * (x * y + w * z),
          1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
          2 * (x * z - w * y),     2 * (y * z + w * x),
          1 - 2 * (x * x + y * y)};
}
// Return target_from_source. Both input poses are expressed in the base frame.
std::pair<Mat3, Vec3> relative(const Pose &target, const Pose &source) {
  const auto a = rotation(target.quaternion), b = rotation(source.quaternion);
  Mat3 r{};
  Vec3 t{};
  for (int i = 0; i < 3; ++i)
    for (int k = 0; k < 3; ++k) {
      t[i] += a[3 * k + i] * (source.position[k] - target.position[k]);
      for (int j = 0; j < 3; ++j)
        r[3 * i + j] += a[3 * k + i] * b[3 * k + j];
    }
  return {r, t};
}
} // namespace
Json frontend_parameters(Json base, const Extrinsics &e,
                         const std::string &algorithm,
                         const std::string &lidar_model) {
  const auto &device = lidar_spec(parse_lidar_model(lidar_model));
  const auto [r, t] = relative(e.imu, e.lidar);
  base["preprocess"]["scan_line"] = device.lines;
  base["preprocess"]["blind"] = std::max(device.minimum_range, .2);
  if (algorithm == "fast-lio") {
    base["mapping"]["extrinsic_T"] = t;
    base["mapping"]["extrinsic_R"] = r;
  } else if (algorithm == "fast-livo2") {
    if (!e.has_camera || !e.camera_parent_frame.empty())
      throw std::runtime_error(
          "FAST-LIVO2 tutorial requires a rigid base-mounted RGB camera");
    const auto [rcl, pcl] = relative(e.camera, e.lidar);
    base["extrin_calib"] = {
        {"extrinsic_R", r}, {"extrinsic_T", t}, {"Rcl", rcl}, {"Pcl", pcl}};
    // Qualified Avia near-wall retrieval setting; not an estimator change.
    base["vio"]["raycast_en"] = lidar_model == "avia";
  } else
    throw std::invalid_argument("Unknown tutorial algorithm");
  return base;
}
Json camera_parameters(int width, int height, double fov) {
  if (width < 1 || height < 1 || !std::isfinite(fov) || fov <= 0 || fov >= 180)
    throw std::invalid_argument("Invalid camera calibration");
  const double focal = height / (2 * std::tan(fov * std::acos(-1) / 360));
  return {
      {"cam_model", "Pinhole"}, {"cam_width", width}, {"cam_height", height},
      {"cam_fx", focal},        {"cam_fy", focal},    {"cam_cx", width / 2.},
      {"cam_cy", height / 2.},  {"cam_d0", 0.},       {"cam_d1", 0.},
      {"cam_d2", 0.},           {"cam_d3", 0.},       {"scale", 1.}};
}
} // namespace simple_rbt::tutorial
