#include "flight_payload.hpp"
#include <algorithm>
namespace simple_rbt {
namespace {
Mat3 rotated(const Vec3 &diagonal, const Quat &q) {
  double r[9];
  mju_quat2Mat(r, q.data());
  Mat3 out{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
        out[3 * i + j] += r[3 * i + k] * diagonal[k] * r[3 * j + k];
  return out;
}
void parallel_axis(Mat3 &tensor, double mass, const Vec3 &p) {
  const double square = mju_dot3(p.data(), p.data());
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      tensor[3 * i + j] += mass * ((i == j ? square : 0.) - p[i] * p[j]);
}
} // namespace
FlightPayload::FlightPayload(const mjModel *m)
    : body_(mj_name2id(m, mjOBJ_BODY, "x2")), dry_mass_(m->body_mass[body_]) {
  std::copy_n(m->body_ipos + 3 * body_, 3, dry_com_.begin());
  Vec3 diagonal;
  Quat q;
  std::copy_n(m->body_inertia + 3 * body_, 3, diagonal.begin());
  std::copy_n(m->body_iquat + 4 * body_, 4, q.begin());
  dry_inertia_ = rotated(diagonal, q);
}
void FlightPayload::update(mjModel *m,
                           const SimulationOptions &options) {
  double mass = dry_mass_;
  Vec3 moment = dry_com_;
  for (auto &v : moment)
    v *= mass;
  Mat3 tensor = dry_inertia_;
  parallel_axis(tensor, dry_mass_, dry_com_);
  for (const auto *name : {"payload_lidar", "payload_camera"}) {
    const int g = mj_name2id(m, mjOBJ_GEOM, name);
    if (g < 0)
      continue;
    const bool lidar = std::string(name) == "payload_lidar";
    const bool enabled =
        lidar ? options.lidar_payload_enabled : options.camera_payload_enabled;
    // Nominal simulation payloads, not a vendor-specific mass calibration.
    const double weight =
        enabled ? (lidar ? (options.lidar_model == LidarModel::Avia ? .5 : .27)
                         : .075)
                : 0.;
    m->geom_contype[g] = enabled ? 1 : 0;
    m->geom_conaffinity[g] = enabled ? 1 : 0;
    Vec3 p, size;
    Quat q;
    std::copy_n(m->geom_pos + 3 * g, 3, p.begin());
    std::copy_n(m->geom_size + 3 * g, 3, size.begin());
    std::copy_n(m->geom_quat + 4 * g, 4, q.begin());
    Vec3 diagonal{weight * (size[1] * size[1] + size[2] * size[2]) / 3.,
                  weight * (size[0] * size[0] + size[2] * size[2]) / 3.,
                  weight * (size[0] * size[0] + size[1] * size[1]) / 3.};
    const auto local = rotated(diagonal, q);
    for (int i = 0; i < 9; ++i)
      tensor[i] += local[i];
    parallel_axis(tensor, weight, p);
    mass += weight;
    for (int i = 0; i < 3; ++i)
      moment[i] += weight * p[i];
  }
  Vec3 com;
  for (int i = 0; i < 3; ++i)
    com[i] = moment[i] / mass;
  parallel_axis(tensor, -mass, com);
  double vectors[9], q[4], eigen[3];
  mju_eig3(eigen, vectors, q, tensor.data());
  m->body_mass[body_] = mass;
  std::copy(com.begin(), com.end(), m->body_ipos + 3 * body_);
  std::copy_n(eigen, 3, m->body_inertia + 3 * body_);
  std::copy_n(q, 4, m->body_iquat + 4 * body_);
  // mj_setConst uses qpos0 scratch state. Keep the configured spawn/current
  // pose intact when a payload changes the model's constant inertia fields.
  std::unique_ptr<mjData, decltype(&mj_deleteData)> scratch(mj_makeData(m),
                                                        mj_deleteData);
  if (!scratch)
    throw std::runtime_error("Cannot allocate payload configuration scratch");
  mj_setConst(m, scratch.get());
}
} // namespace simple_rbt
