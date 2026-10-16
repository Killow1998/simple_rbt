#include "artifacts.hpp"
#include "resources.hpp"
#include "simple_rbt/paths.hpp"
#include "simple_rbt/lidar.hpp"
#include <cmath>
#include <iomanip>
#include <png.h>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace simple_rbt {
namespace fs = std::filesystem;
using Json = nlohmann::json;
namespace {
void json_file(const fs::path &p, const Json &j) {
  std::ofstream f(p);
  f.exceptions(std::ios::failbit | std::ios::badbit);
  f << j.dump(2) << '\n';
  f.close();
}
template <typename T>
void npy(const fs::path &p, const std::vector<T> &v,
         std::vector<std::size_t> shape, const std::string &dtype) {
  std::ostringstream h;
  h << "{'descr': '" << dtype << "', 'fortran_order': False, 'shape': (";
  for (std::size_t n : shape)
    h << n << ", ";
  h << "), }";
  std::string header = h.str();
  std::size_t padding = 64 - ((10 + header.size() + 1) % 64);
  if (padding == 64)
    padding = 0;
  header += std::string(padding, ' ');
  header += '\n';
  std::ofstream f(p, std::ios::binary);
  f.exceptions(std::ios::failbit | std::ios::badbit);
  const char magic[] = {char(0x93), 'N', 'U', 'M', 'P', 'Y', 1, 0};
  f.write(magic, 8);
  std::uint16_t n = header.size();
  f.put(n & 255);
  f.put(n >> 8);
  f.write(header.data(), header.size());
  f.write(reinterpret_cast<const char *>(v.data()), v.size() * sizeof(T));
  f.close();
}
void save_camera(const fs::path &out, const std::string &name,
                 const CameraFrame &frame) {
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  image.width = frame.width;
  image.height = frame.height;
  image.format = PNG_FORMAT_RGB;
  if (!png_image_write_to_file(&image, (out / (name + "-rgb.png")).c_str(), 0,
                               frame.rgb.data(), 0, nullptr))
    throw std::runtime_error(image.message);
  npy(out / (name + "-depth-" + (frame.radial_depth ? "range" : "z") + ".npy"),
      frame.depth, {std::size_t(frame.height), std::size_t(frame.width)},
      "<f4");
}
} // namespace

ArtifactWriter::ArtifactWriter(const fs::path &project_root,
                               const fs::path &requested_output,
                               bool summary_only, bool precreated)
    : output_(requested_output), summary_only_(summary_only) {
  (void)project_root;
  if (output_.empty()) {
    std::ostringstream s;
    s << "native-" << getpid() << '-'
      << std::uint64_t(monotonic_seconds() * 1e6);
    output_ = user_output_root() / s.str();
  }
  if (precreated) {
    if (!fs::is_directory(output_) ||
        fs::exists(output_ / "compiled-model.xml") ||
        fs::exists(output_ / "summary.json"))
      throw std::invalid_argument(
          "Validator output is unavailable or already used");
  } else if (!fs::create_directories(output_))
    throw std::invalid_argument("Output directory already exists: " +
                                output_.string());
}
void ArtifactWriter::model(const std::string &text) {
  std::ofstream xml(output_ / "compiled-model.xml");
  xml.exceptions(std::ios::failbit | std::ios::badbit);
  xml << text;
  xml.close();
}
void ArtifactWriter::open_streams() {
  if (!summary_only_) {
    trajectory_.exceptions(std::ios::failbit | std::ios::badbit);
    imu_csv_.exceptions(std::ios::failbit | std::ios::badbit);
    trajectory_.open(output_ / "trajectory.csv");
    trajectory_ << std::setprecision(17)
                << "time_s,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n";
    imu_csv_.open(output_ / "mid360-imu-si.csv");
    imu_csv_ << std::setprecision(17)
             << "stamp_ns,wx_rad_s,wy_rad_s,wz_rad_s,specific_force_x_m_s2,"
                "specific_force_y_m_s2,specific_force_z_m_s2\n";
  }
}
void ArtifactWriter::imu(const ImuSample &sample) {
  if (!summary_only_) {
    imu_csv_ << sample.stamp_ns;
    for (double x : sample.angular_velocity)
      imu_csv_ << ',' << x;
    for (double x : sample.specific_force)
      imu_csv_ << ',' << x;
    imu_csv_ << '\n';
  }
}
void ArtifactWriter::trajectory(std::uint64_t stamp, const Pose &base,
                                const Vec3 &lin, const Vec3 &ang) {
  if (!summary_only_) {
    trajectory_ << stamp * 1e-9;
    for (double x : base.position)
      trajectory_ << ',' << x;
    for (int i : {1, 2, 3, 0})
      trajectory_ << ',' << base.quaternion[i];
    for (double x : lin)
      trajectory_ << ',' << x;
    for (double x : ang)
      trajectory_ << ',' << x;
    trajectory_ << '\n';
  }
}
void ArtifactWriter::close_streams() {
  if (!summary_only_) {
    trajectory_.close();
    imu_csv_.close();
  }
}
void ArtifactWriter::camera(std::uint64_t stamp, const OpticalPose &camera_pose,
                            const OpticalPose &depth_pose, const CameraPair &frames) {
  save_camera(output_, "perspective", frames.perspective);
  save_camera(output_, "panorama", frames.panorama);
  json_file(output_ / "camera-metadata.json",
            {{"stamp_ns", stamp},
             {"position_world_m", camera_pose.position},
             {"rotation_world_from_optical",
              {{camera_pose.rotation[0], camera_pose.rotation[1],
                camera_pose.rotation[2]},
               {camera_pose.rotation[3], camera_pose.rotation[4],
                camera_pose.rotation[5]},
               {camera_pose.rotation[6], camera_pose.rotation[7],
                camera_pose.rotation[8]}}},
             {"depth_unit", "metre"},
             {"no_surface_depth", "positive infinity"},
             {"optical_axes", "+X right, +Y down, +Z forward"},
             {"perspective",
              {{"width", 320},
               {"height", 240},
               {"depth_kind", "z"},
               {"depth_position_world_m", depth_pose.position},
               {"depth_rotation_world_from_optical", depth_pose.rotation},
               {"depth_registered_to_color", false},
               {"K",
                {{120. / std::tan(3.14159265358979323846 / 6), 0., 159.5},
                 {0., 120. / std::tan(3.14159265358979323846 / 6), 119.5},
                 {0., 0., 1.}}},
               {"vertical_fov_deg", 60}}},
             {"panorama",
              {{"width", 720},
               {"height", 360},
               {"depth_kind", "range"},
               {"projection", "equirectangular"},
               {"horizontal_fov_deg", 360},
               {"vertical_fov_deg", 180},
               {"cubemap_face_size", 256}}}});
}
void ArtifactWriter::lidar(const LidarScan &last_scan,
                           const Pose &last_snapshot_pose,
                           const PoseHistory &history, bool rolling,
                           const Json &scan_model) {
  std::vector<float> xyz;
  xyz.reserve(last_scan.xyz.size() * 3);
  for (auto &p : last_scan.xyz)
    xyz.insert(xyz.end(), p.begin(), p.end());
  npy(output_ / "lidar-xyz.npy", xyz, {last_scan.xyz.size(), 3}, "<f4");
  npy(output_ / "lidar-offsets-ns.npy", last_scan.offsets_ns,
      {last_scan.offsets_ns.size()}, "<u4");
  npy(output_ / "lidar-lines.npy", last_scan.lines, {last_scan.lines.size()},
      "|u1");
  npy(output_ / "lidar-ray-indices.npy", last_scan.ray_indices,
      {last_scan.ray_indices.size()}, "<u8");
  std::vector<double> capture_positions, capture_quaternions;
  capture_positions.reserve(last_scan.xyz.size() * 3);
  capture_quaternions.reserve(last_scan.xyz.size() * 4);
  for (auto offset : last_scan.offsets_ns) {
    const Pose pose = rolling ? history.interpolate(last_scan.stamp_ns + offset)
                              : last_snapshot_pose;
    capture_positions.insert(capture_positions.end(), pose.position.begin(),
                             pose.position.end());
    for (int axis : {1, 2, 3, 0})
      capture_quaternions.push_back(pose.quaternion[axis]);
  }
  npy(output_ / "lidar-capture-position-world.npy", capture_positions,
      {last_scan.xyz.size(), 3}, "<f8");
  npy(output_ / "lidar-capture-quaternion-world-xyzw.npy", capture_quaternions,
      {last_scan.xyz.size(), 4}, "<f8");
  json_file(
      output_ / "lidar-metadata.json",
      {{"stamp_ns", last_scan.stamp_ns},
       {"reference_stamp_ns",
        rolling ? last_scan.stamp_ns + 100000000 : last_scan.stamp_ns},
       {"position_world_m", last_snapshot_pose.position},
       {"quaternion_world_xyzw",
        {last_snapshot_pose.quaternion[1], last_snapshot_pose.quaternion[2],
         last_snapshot_pose.quaternion[3], last_snapshot_pose.quaternion[0]}},
       {"rays_cast", last_scan.rays_cast},
       {"point_num", last_scan.xyz.size()},
       {"scan_model", scan_model}});
}
void ArtifactWriter::summary(const Json &value) {
  json_file(output_ / "summary.json", value);
}
} // namespace simple_rbt
