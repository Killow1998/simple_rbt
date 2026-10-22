#include "simple_rbt/lidar.hpp"
#include "lidar_cuda.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <future>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace simple_rbt {
namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
void require(bool value, const char* message) { if (!value) throw std::invalid_argument(message); }
bool finite_pose(const Pose& pose) {
  for (double v : pose.position) if (!std::isfinite(v)) return false;
  for (double v : pose.quaternion) if (!std::isfinite(v)) return false;
  return true;
}
Vec3 rotate(const Quat& q, const std::array<float, 3>& input) {
  const double x = input[0], y = input[1], z = input[2];
  const Vec3 uv{2. * (q[2]*z - q[3]*y), 2. * (q[3]*x - q[1]*z), 2. * (q[1]*y - q[2]*x)};
  Vec3 result{x + q[0]*uv[0] + q[2]*uv[2] - q[3]*uv[1],
              y + q[0]*uv[1] + q[3]*uv[0] - q[1]*uv[2],
              z + q[0]*uv[2] + q[1]*uv[1] - q[2]*uv[0]};
  const double norm = std::sqrt(result[0]*result[0] + result[1]*result[1] + result[2]*result[2]);
  for (double& v : result) v /= norm;
  return result;
}
struct ModelDelete { void operator()(mjModel* p) const { mj_deleteModel(p); } };
struct DataDelete { void operator()(mjData* p) const { mj_deleteData(p); } };
using ModelPtr = std::unique_ptr<mjModel, ModelDelete>;
using DataPtr = std::unique_ptr<mjData, DataDelete>;
}

CsvPattern::CsvPattern(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot open LiDAR CSV: " + path);
  std::string line;
  require(bool(std::getline(file, line)), "LiDAR CSV is empty");
  std::size_t line_number = 1;
  std::int64_t previous_sample_id = 0;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    double id, azimuth, zenith;
    std::string extra;
    if (!(row >> id >> azimuth >> zenith) || (row >> extra))
      throw std::invalid_argument("Expected three numeric CSV columns at line " + std::to_string(line_number));
    require(std::isfinite(id) && std::isfinite(azimuth) && std::isfinite(zenith), "CSV contains nonfinite values");
    require(id == std::floor(id) && id >= -9007199254740991. && id <= 9007199254740991.,
            "CSV sample IDs must be exactly representable integers");
    require(zenith >= 0. && zenith <= 180., "CSV zenith must lie in [0,180] degrees");
    const auto sample_id = static_cast<std::int64_t>(id);
    require(rays_.empty() || sample_id == previous_sample_id + 1, "CSV sample IDs must be consecutive");
    PatternRay ray;
    previous_sample_id = sample_id;
    // Python oracle: deg2rad(float64), cast float32, float32 trig and products.
    const float theta = static_cast<float>(azimuth * (pi / 180.));
    const float phi = static_cast<float>((90. - zenith) * (pi / 180.));
    const float c = std::cos(phi);
    ray.direction = {c * std::cos(theta), c * std::sin(theta), std::sin(phi)};
    rays_.push_back(ray);
  }
  require(!rays_.empty(), "LiDAR CSV has no samples");
  rays_.shrink_to_fit();
}
void CsvPattern::advance(std::size_t count) {
  require(count <= std::numeric_limits<std::uint64_t>::max() - emitted_, "LiDAR emitted counter overflow");
  cursor_ = (cursor_ + count % rays_.size()) % rays_.size();
  emitted_ += count;
}

PoseHistory::PoseHistory(std::uint64_t retention_ns) : retention_ns_(retention_ns) {
  require(retention_ns >= 250000000, "Pose history must retain at least 250ms");
}
void PoseHistory::add(std::uint64_t stamp, const Pose& input) {
  require(finite_pose(input), "Pose must be finite");
  require(knots_.empty() || stamp > knots_.back().stamp_ns, "Pose timestamps must increase strictly");
  Pose pose = input;
  double norm_squared = 0.;
  for (double v : pose.quaternion) norm_squared += v*v;
  const double norm = std::sqrt(norm_squared);
  require(std::isfinite(norm) && norm >= 1e-12, "Quaternion must have a finite nonzero norm");
  for (double& v : pose.quaternion) v /= norm;
  knots_.push_back({stamp, pose});
  const std::uint64_t boundary = stamp > retention_ns_ ? stamp-retention_ns_ : 0;
  // Preserve the knot immediately BEFORE the retention boundary, also if the
  // boundary itself is a knot. This mirrors Python's bisect_left(...)-1.
  while (knots_.size() > 1 && knots_[1].stamp_ns < boundary) knots_.pop_front();
}
std::array<std::uint64_t, 2> PoseHistory::bounds() const {
  require(!knots_.empty(), "Pose history is empty");
  return {knots_.front().stamp_ns, knots_.back().stamp_ns};
}
Pose PoseHistory::interpolate(std::uint64_t stamp) const {
  const auto limits = bounds();
  require(stamp >= limits[0] && stamp <= limits[1], "Pose history extrapolation is forbidden");
  auto right = std::upper_bound(knots_.begin(), knots_.end(), stamp,
                               [](auto t, const PoseKnot& p) { return t < p.stamp_ns; });
  const auto& left = *std::prev(right);
  if (left.stamp_ns == stamp || right == knots_.end()) return left.pose;
  const double u = static_cast<double>(stamp-left.stamp_ns) / static_cast<double>(right->stamp_ns-left.stamp_ns);
  Pose output;
  for (unsigned i=0; i<3; ++i) output.position[i] = left.pose.position[i] + u*(right->pose.position[i]-left.pose.position[i]);
  Quat q = right->pose.quaternion;
  double dot = 0.;
  for (unsigned i=0; i<4; ++i) dot += left.pose.quaternion[i]*q[i];
  if (dot < 0.) { for (double& v : q) v = -v; dot = -dot; }
  dot = std::clamp(dot, 0., 1.);
  if (dot > .9995) {
    for (unsigned i=0; i<4; ++i) output.quaternion[i] = left.pose.quaternion[i] + u*(q[i]-left.pose.quaternion[i]);
  } else {
    const double angle = std::acos(dot), denominator = std::sin(angle);
    for (unsigned i=0; i<4; ++i)
      output.quaternion[i] = (std::sin((1.-u)*angle)*left.pose.quaternion[i] + std::sin(u*angle)*q[i]) / denominator;
  }
  double norm = 0.;
  for (double v : output.quaternion) norm += v*v;
  norm = std::sqrt(norm);
  for (double& v : output.quaternion) v /= norm;
  return output;
}
std::uint64_t PoseHistory::maximum_interval(std::uint64_t start, std::uint64_t end) const {
  require(start <= end, "Reversed pose interval");
  const auto limits = bounds();
  require(start >= limits[0] && end <= limits[1], "Pose history extrapolation is forbidden");
  std::uint64_t result = 0;
  for (std::size_t i=1; i<knots_.size(); ++i) {
    if (knots_[i].stamp_ns > start && knots_[i-1].stamp_ns < end)
      result = std::max(result, knots_[i].stamp_ns-knots_[i-1].stamp_ns);
  }
  return result;
}

struct RollingLidar::Impl {
  const mjModel* live_model;
  LidarBackend mode;
  CsvPattern& pattern;
  double min_range, max_range;
  std::vector<int> ids;
  std::vector<detail::RayGeometry> geometries;
  ModelPtr cpu_model;
  DataPtr cpu_data;
  detail::CudaRayState* cuda = nullptr;
  std::vector<double> origins, directions, ranges;
  mjtByte groups[mjNGROUP]{1,0,0,0,0,0};
  Impl(const mjModel* model, LidarBackend backend, CsvPattern& p, double minimum, double maximum)
    : live_model(model), mode(backend), pattern(p), min_range(minimum), max_range(maximum) {}
  ~Impl() { detail::free_cuda_rays(cuda); }
};

RollingLidar::RollingLidar(const mjModel* model, LidarBackend mode, CsvPattern& pattern,
                         double minimum, double maximum, const std::string& site)
  : impl_(std::make_unique<Impl>(model, mode, pattern, minimum, maximum)) {
  require(model != nullptr, "LiDAR requires a model");
  require(mode == LidarBackend::Cpu || mode == LidarBackend::Cuda, "Unknown LiDAR backend");
  require(std::isfinite(minimum) && std::isfinite(maximum) && 0. <= minimum && minimum < maximum,
          "Require finite 0 <= min_range < max_range");
  const int site_id = mj_name2id(model, mjOBJ_SITE, site.c_str());
  require(site_id >= 0, "LiDAR site is missing");
  const int root = model->body_rootid[model->site_bodyid[site_id]];
  std::vector<bool> robot(model->nbody, false);
  for (int b=1; b<model->nbody; ++b) {
    robot[b] = root != 0 && model->body_rootid[b] == root;
    if (!robot[b] && (model->body_jntnum[b] > 0 || model->body_mocapid[b] >= 0))
      throw std::invalid_argument("Rolling static environment rejects non-robot dynamic bodies");
  }
  DataPtr scratch;
  const mjData* poses;
  if (mode == LidarBackend::Cpu) {
    impl_->cpu_model.reset(mj_copyModel(nullptr, model));
    if (!impl_->cpu_model) throw std::runtime_error("Cannot copy CPU ray model");
    for (int g=0; g<model->ngeom; ++g) impl_->cpu_model->geom_group[g] = robot[model->geom_bodyid[g]] ? 1 : 0;
    impl_->cpu_data.reset(mj_makeData(impl_->cpu_model.get()));
    if (!impl_->cpu_data) throw std::runtime_error("Cannot allocate CPU ray data");
    mj_forward(impl_->cpu_model.get(), impl_->cpu_data.get());
    poses = impl_->cpu_data.get();
  } else {
    scratch.reset(mj_makeData(model));
    if (!scratch) throw std::runtime_error("Cannot allocate ray-scene initialization data");
    mj_kinematics(model, scratch.get());
    poses = scratch.get();
  }
  for (int g=0; g<model->ngeom; ++g) {
    if (robot[model->geom_bodyid[g]]) continue;
    require(model->geom_type[g] == mjGEOM_PLANE || model->geom_type[g] == mjGEOM_BOX,
            "Native static LiDAR supports every current terrain (plane/box); unsupported environment geom rejected");
    // mj_ray excludes fully transparent geometry; retain the same visibility.
    const int material = model->geom_matid[g];
    const float alpha = material >= 0 ? model->mat_rgba[4*material+3] : model->geom_rgba[4*g+3];
    detail::RayGeometry geom;
    geom.type = alpha == 0.f ? -1 : model->geom_type[g];
    std::copy_n(poses->geom_xpos+3*g, 3, geom.position);
    std::copy_n(poses->geom_xmat+9*g, 9, geom.rotation);
    std::copy_n(model->geom_size+3*g, 3, geom.size);
    impl_->ids.push_back(g);
    impl_->geometries.push_back(geom);
  }
  if (mode == LidarBackend::Cuda) impl_->cuda = detail::make_cuda_rays(impl_->geometries);
  impl_->origins.reserve(60000); impl_->directions.reserve(60000); impl_->ranges.reserve(20000);
}
RollingLidar::~RollingLidar() = default;
std::size_t RollingLidar::environment_geom_count() const noexcept { return impl_->ids.size(); }
std::size_t RollingLidar::ray_capacity() const noexcept { return impl_->ranges.capacity(); }

void RollingLidar::scan(const mjData* data, std::uint64_t start, std::size_t count,
                       const PoseHistory& history, LidarScan& output) {
  require(data != nullptr, "LiDAR scan requires current live data for the static-scene check");
  require(count > 0, "Ray count must be positive");
  require(count-1 <= std::numeric_limits<std::uint32_t>::max()/CsvPattern::point_period_ns,
          "Scan offsets exceed uint32 nanoseconds");
  const std::uint64_t last_offset = (count-1)*CsvPattern::point_period_ns;
  require(start <= std::numeric_limits<std::uint64_t>::max()-last_offset, "Scan timestamp overflow");
  history.interpolate(start); history.interpolate(start+last_offset);
  for (std::size_t i=0; i<impl_->ids.size(); ++i) {
    const int g = impl_->ids[i];
    const auto& fixed = impl_->geometries[i];
    for (int k=0; k<3; ++k)
      require(std::isfinite(data->geom_xpos[3*g+k]) && std::abs(data->geom_xpos[3*g+k]-fixed.position[k]) <= 1e-10,
              "Static environment position changed after LiDAR construction");
    for (int k=0; k<9; ++k)
      require(std::isfinite(data->geom_xmat[9*g+k]) && std::abs(data->geom_xmat[9*g+k]-fixed.rotation[k]) <= 1e-10,
              "Static environment rotation changed after LiDAR construction");
  }
  auto& origins = impl_->origins; auto& directions = impl_->directions; auto& ranges = impl_->ranges;
  origins.resize(3*count); directions.resize(3*count); ranges.resize(count);
  const auto begin = impl_->pattern.cursor(), pattern_size = impl_->pattern.size();
  for (std::size_t i=0; i<count; ++i) {
    const auto pose = history.interpolate(start+i*CsvPattern::point_period_ns);
    const auto direction = rotate(pose.quaternion, impl_->pattern.ray((begin+i)%pattern_size).direction);
    std::copy(pose.position.begin(), pose.position.end(), origins.begin()+3*i);
    std::copy(direction.begin(), direction.end(), directions.begin()+3*i);
  }
  if (impl_->mode == LidarBackend::Cpu) {
    // Independent native ray queries share immutable scene data. Join before
    // assembling returns, preserving sample order and failure propagation.
    const std::size_t workers = count >= 4096
        ? std::min(4u, std::max(1u, std::thread::hardware_concurrency())) : 1;
    auto trace = [&](std::size_t first, std::size_t last) {
      for (std::size_t i = first; i < last; ++i)
        ranges[i] = mj_ray(impl_->cpu_model.get(), impl_->cpu_data.get(),
                           origins.data() + 3*i, directions.data() + 3*i,
                           impl_->groups, true, -1, nullptr, nullptr);
    };
    std::vector<std::future<void>> jobs;
    for (std::size_t worker = 1; worker < workers; ++worker)
      jobs.push_back(std::async(std::launch::async, trace,
                               count*worker/workers, count*(worker+1)/workers));
    trace(0, count/workers);
    for (auto& job : jobs) job.get();
  } else {
    detail::trace_cuda_rays(impl_->cuda, origins.data(), directions.data(), count, impl_->max_range, ranges.data());
  }
  output.xyz.clear(); output.offsets_ns.clear(); output.lines.clear(); output.ray_indices.clear();
  output.xyz.reserve(count); output.offsets_ns.reserve(count); output.lines.reserve(count); output.ray_indices.reserve(count);
  for (std::size_t i=0; i<count; ++i) {
    if (!std::isfinite(ranges[i]) || ranges[i] < impl_->min_range || ranges[i] >= impl_->max_range) continue;
    const auto index = (begin+i)%pattern_size;
    const auto& local = impl_->pattern.ray(index).direction;
    output.xyz.push_back({static_cast<float>(local[0]*ranges[i]), static_cast<float>(local[1]*ranges[i]), static_cast<float>(local[2]*ranges[i])});
    output.offsets_ns.push_back(static_cast<std::uint32_t>(i*CsvPattern::point_period_ns));
    output.lines.push_back(static_cast<std::uint8_t>(index%CsvPattern::line_count));
    output.ray_indices.push_back(index);
  }
  // Commit pattern/output clock only after successful interpolation and tracing.
  impl_->pattern.advance(count);
  output.stamp_ns = start;
  output.rays_cast = count;
}
}  // namespace simple_rbt
