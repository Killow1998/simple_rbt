#include "simple_rbt/ipc.hpp"
#include "validation.hpp"
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace simple_rbt::ipc {
namespace {
struct Writer {
  std::vector<std::uint8_t> data;
  explicit Writer(std::size_t size) { data.reserve(size); }
  template <class T> void value(T v) {
    static_assert(std::is_trivially_copyable_v<T>);
    std::uint8_t bits[sizeof(T)];
    std::memcpy(bits, &v, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(bits, bits + sizeof(T));
#endif
    data.insert(data.end(), bits, bits + sizeof(T));
  }
  template <class T, std::size_t N> void array(const std::array<T, N> &v) {
    for (T x : v)
      value(x);
  }
};
struct Reader {
  const std::vector<std::uint8_t> &data;
  std::size_t offset = 0;
  template <class T> T value() {
    std::uint8_t bits[sizeof(T)];
    std::memcpy(bits, data.data() + offset, sizeof(T));
    offset += sizeof(T);
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(bits, bits + sizeof(T));
#endif
    T result;
    std::memcpy(&result, bits, sizeof(T));
    return result;
  }
  template <class T, std::size_t N> void array(std::array<T, N> &v) {
    for (T &x : v)
      x = value<T>();
  }
};
void pose(Writer &w, const Pose &p) {
  w.array(p.position);
  w.array(p.quaternion);
}
void pose(Reader &r, Pose &p) {
  r.array(p.position);
  r.array(p.quaternion);
}
Json pose_json(const Pose &p) {
  return {{"position", p.position}, {"quaternion", p.quaternion}};
}
Pose from_pose(const Json &j) {
  return {j.at("position").get<Vec3>(), j.at("quaternion").get<Quat>()};
}
} // namespace
Packet json_packet(Kind k, const Json &j, std::uint64_t stamp) {
  auto text = j.dump();
  return {k, stamp, {text.begin(), text.end()}};
}
Json decode_json(const Packet &p) {
  return Json::parse(p.payload.begin(), p.payload.end());
}
Packet encode_imu(const ImuSample &s) {
  Writer w(6 * sizeof(double));
  w.array(s.angular_velocity);
  w.array(s.specific_force);
  return {Kind::Imu, s.stamp_ns, std::move(w.data)};
}
ImuSample decode_imu(const Packet &p) {
  if (p.payload.size() != 6 * sizeof(double))
    throw std::invalid_argument("Invalid IMU payload length");
  Reader r{p.payload};
  ImuSample s;
  s.stamp_ns = p.stamp_ns;
  r.array(s.angular_velocity);
  r.array(s.specific_force);
  finite_vector(s.angular_velocity);
  finite_vector(s.specific_force);
  return s;
}
Packet encode_scan(const LidarScan &s) {
  const auto n = s.xyz.size();
  Writer w(17 + n * (s.ray_indices.empty() ? 17 : 25));
  w.value<std::uint64_t>(s.rays_cast);
  w.value<std::uint64_t>(n);
  w.value<std::uint8_t>(!s.ray_indices.empty());
  for (std::size_t i = 0; i < n; ++i) {
    w.array(s.xyz[i]);
    w.value(s.offsets_ns[i]);
    w.value(s.lines[i]);
    if (!s.ray_indices.empty())
      w.value(s.ray_indices[i]);
  }
  return {Kind::Lidar, s.stamp_ns, std::move(w.data)};
}
LidarScan decode_scan(const Packet &p) {
  if (p.payload.size() < 17) throw std::invalid_argument("Truncated scan header");
  Reader r{p.payload};
  LidarScan s;
  s.stamp_ns = p.stamp_ns;
  s.rays_cast = r.value<std::uint64_t>();
  const auto n = r.value<std::uint64_t>();
  const auto indices = r.value<std::uint8_t>();
  if (indices > 1 || n > (max_payload_bytes - 17) / (indices ? 25 : 17) ||
      p.payload.size() != 17 + n * (indices ? 25 : 17))
    throw std::invalid_argument("invalid IPC scan length");
  s.xyz.resize(n);
  s.offsets_ns.resize(n);
  s.lines.resize(n);
  if (indices)
    s.ray_indices.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    r.array(s.xyz[i]);
    for (float value : s.xyz[i])
      if (!std::isfinite(value)) throw std::invalid_argument("scan xyz must be finite");
    s.offsets_ns[i] = r.value<std::uint32_t>();
    s.lines[i] = r.value<std::uint8_t>();
    if (indices)
      s.ray_indices[i] = r.value<std::uint64_t>();
  }
  return s;
}
Packet encode_odometry(std::uint64_t ns, const Pose &p, const Vec3 &linear,
                       const Vec3 &angular) {
  Writer w(13 * sizeof(double));
  pose(w, p);
  w.array(linear);
  w.array(angular);
  return {Kind::Odometry, ns, std::move(w.data)};
}
Odometry decode_odometry(const Packet &p) {
  if (p.payload.size() != 13 * sizeof(double))
    throw std::invalid_argument("Invalid odometry payload length");
  Reader r{p.payload};
  Odometry o;
  pose(r, o.pose);
  r.array(o.linear);
  r.array(o.angular);
  unit_pose(o.pose);
  finite_vector(o.linear);
  finite_vector(o.angular);
  return o;
}
Packet encode_camera(std::uint64_t ns, const CameraFrame &f, Kind kind) {
  Writer w(25 + f.rgb.size() + f.depth.size() * sizeof(float));
  w.value<std::uint32_t>(f.width);
  w.value<std::uint32_t>(f.height);
  w.value<std::uint8_t>(f.radial_depth);
  w.value<std::uint64_t>(f.rgb.size());
  w.value<std::uint64_t>(f.depth.size());
  w.data.insert(w.data.end(), f.rgb.begin(), f.rgb.end());
  for (float x : f.depth)
    w.value(x);
  return {kind, ns, std::move(w.data)};
}
CameraFrame decode_camera(const Packet &p) {
  if (p.payload.size() < 25) throw std::invalid_argument("Truncated camera header");
  Reader r{p.payload};
  CameraFrame f;
  f.width = r.value<std::uint32_t>();
  f.height = r.value<std::uint32_t>();
  const auto radial = r.value<std::uint8_t>();
  const auto rgb = r.value<std::uint64_t>(), depth = r.value<std::uint64_t>();
  const auto pixels = static_cast<std::uint64_t>(f.width) * f.height;
  if (f.width <= 0 || f.height <= 0 || radial != (p.kind == Kind::Panorama) ||
      pixels > (max_payload_bytes - 25) / 7 ||
      (rgb != pixels * 3 && rgb != 0) || (depth != pixels && depth != 0) ||
      (rgb == 0 && (p.kind == Kind::View || depth == 0)) ||
      p.payload.size() != 25 + rgb + depth * 4)
    throw std::invalid_argument("invalid IPC camera channels or length");
  f.radial_depth = radial;
  f.rgb.assign(p.payload.begin() + r.offset,
               p.payload.begin() + r.offset + rgb);
  r.offset += rgb;
  f.depth.resize(depth);
  for (float &x : f.depth)
    x = r.value<float>();
  return f;
}
Json extrinsics_json(const Extrinsics &e) {
  return {{"camera_parent_frame", e.camera_parent_frame},
          {"has_external_camera", e.has_external_camera},
          {"external_camera", pose_json(e.external_camera)},
          {"external_depth", pose_json(e.external_depth)},
          {"lidar", pose_json(e.lidar)},
          {"imu", pose_json(e.imu)},
          {"camera", pose_json(e.camera)},
          {"has_camera", e.has_camera},
          {"camera_depth", pose_json(e.camera_depth)},
          {"camera_imu", pose_json(e.camera_imu)},
          {"separate_camera_frames", e.separate_camera_frames},
          {"has_camera_imu", e.has_camera_imu}};
}
Extrinsics decode_extrinsics(const Json &j) {
  Extrinsics e;
  e.lidar = from_pose(j.at("lidar"));
  e.imu = from_pose(j.at("imu"));
  e.camera = from_pose(j.at("camera"));
  e.has_camera = j.at("has_camera").get<bool>();
  e.separate_camera_frames=j.value("separate_camera_frames",false);
  e.has_camera_imu=j.value("has_camera_imu",false);
  if(e.separate_camera_frames) e.camera_depth=from_pose(j.at("camera_depth"));
  if(e.has_camera_imu) e.camera_imu=from_pose(j.at("camera_imu"));
  e.camera_parent_frame = j.value("camera_parent_frame", std::string());
  e.has_external_camera = j.value("has_external_camera", false);
  if (e.has_external_camera) {
    e.external_camera = from_pose(j.at("external_camera"));
    e.external_depth = from_pose(j.at("external_depth"));
  }
  unit_pose(e.lidar); unit_pose(e.imu);
  if (e.has_camera) unit_pose(e.camera);
  if (e.has_camera && e.separate_camera_frames) unit_pose(e.camera_depth);
  if (e.has_camera_imu) unit_pose(e.camera_imu);
  if (e.has_external_camera) { unit_pose(e.external_camera); unit_pose(e.external_depth); }
  return e;
}
} // namespace simple_rbt::ipc
