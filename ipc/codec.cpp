#include "simple_rbt/ipc.hpp"
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace simple_rbt::ipc {
namespace {
struct Writer {
  std::vector<std::uint8_t> data;
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
    if (offset + sizeof(T) > data.size())
      throw std::invalid_argument("truncated IPC payload");
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
  void done() const {
    if (offset != data.size())
      throw std::invalid_argument("trailing IPC payload");
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
  Writer w;
  w.array(s.angular_velocity);
  w.array(s.specific_force);
  return {Kind::Imu, s.stamp_ns, std::move(w.data)};
}
ImuSample decode_imu(const Packet &p) {
  Reader r{p.payload};
  ImuSample s;
  s.stamp_ns = p.stamp_ns;
  r.array(s.angular_velocity);
  r.array(s.specific_force);
  r.done();
  return s;
}
Packet encode_scan(const LidarScan &s) {
  const auto n = s.xyz.size();
  if (s.offsets_ns.size() != n || s.lines.size() != n ||
      (!s.ray_indices.empty() && s.ray_indices.size() != n))
    throw std::invalid_argument("IPC scan columns differ");
  if (n > (max_payload_bytes - 17) / 25)
    throw std::invalid_argument("IPC scan too large");
  Writer w;
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
    s.offsets_ns[i] = r.value<std::uint32_t>();
    s.lines[i] = r.value<std::uint8_t>();
    if (indices)
      s.ray_indices[i] = r.value<std::uint64_t>();
  }
  r.done();
  return s;
}
Packet encode_odometry(std::uint64_t ns, const Pose &p, const Vec3 &linear,
                       const Vec3 &angular) {
  Writer w;
  pose(w, p);
  w.array(linear);
  w.array(angular);
  return {Kind::Odometry, ns, std::move(w.data)};
}
Odometry decode_odometry(const Packet &p) {
  Reader r{p.payload};
  Odometry o;
  pose(r, o.pose);
  r.array(o.linear);
  r.array(o.angular);
  r.done();
  return o;
}
Packet encode_camera(std::uint64_t ns, const CameraFrame &f, Kind kind) {
  const auto pixels = static_cast<std::size_t>(f.width) * f.height;
  if (f.width <= 0 || f.height <= 0 || pixels > (max_payload_bytes - 25) / 7 ||
      (f.rgb.size() != pixels * 3 && !f.rgb.empty()) ||
      (f.depth.size() != pixels && !f.depth.empty()) ||
      (f.rgb.empty() && (kind == Kind::View || f.depth.empty())))
    throw std::invalid_argument("invalid IPC camera dimensions or channels");
  Writer w;
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
  Reader r{p.payload};
  CameraFrame f;
  f.width = r.value<std::uint32_t>();
  f.height = r.value<std::uint32_t>();
  const auto radial = r.value<std::uint8_t>();
  const auto rgb = r.value<std::uint64_t>(), depth = r.value<std::uint64_t>();
  const auto pixels = static_cast<std::uint64_t>(f.width) * f.height;
  if (f.width <= 0 || f.height <= 0 || radial > 1 ||
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
  r.done();
  return f;
}
Json extrinsics_json(const Extrinsics &e) {
  return {{"lidar", pose_json(e.lidar)},
          {"imu", pose_json(e.imu)},
          {"camera", pose_json(e.camera)},
          {"has_camera", e.has_camera}};
}
Extrinsics decode_extrinsics(const Json &j) {
  return {from_pose(j.at("lidar")), from_pose(j.at("imu")),
          from_pose(j.at("camera")), j.at("has_camera").get<bool>()};
}
} // namespace simple_rbt::ipc
