#include "simple_rbt/full_run.hpp"
#include "simple_rbt/paths.hpp"
#include "simple_rbt/course.hpp"
#include "simple_rbt/robots.hpp"
#include "simple_rbt/transport.hpp"
#include <openssl/evp.h>
#include <unistd.h>
#include <array>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace simple_rbt {
namespace {
namespace fs = std::filesystem;
constexpr double kG0 = 9.80665;
Json read_json(const fs::path& path) {
  std::ifstream input(path); if (!input) throw std::runtime_error("Cannot read " + path.string());
  Json value; input >> value; return value;
}
void write_json(const fs::path& path, const Json& value) {
  const fs::path temporary(path.string() + ".tmp");
  std::ofstream output(temporary); output << value.dump(2) << '\n'; output.close();
  if (!output) throw std::runtime_error("Cannot write " + temporary.string());
  fs::rename(temporary, path);
}
struct Hash {
  EVP_MD_CTX* context = EVP_MD_CTX_new();
  Hash() { if (!context || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) throw std::runtime_error("SHA256 init failed"); }
  ~Hash() { EVP_MD_CTX_free(context); }
  Hash(const Hash&) = delete;
  void update(const void* data, std::size_t count) {
    if (count && EVP_DigestUpdate(context, data, count) != 1) throw std::runtime_error("SHA256 update failed");
  }
  std::string hex() const {
    EVP_MD_CTX* copy = EVP_MD_CTX_new();
    unsigned char bytes[EVP_MAX_MD_SIZE]; unsigned length = 0;
    if (!copy || EVP_MD_CTX_copy_ex(copy, context) != 1 || EVP_DigestFinal_ex(copy, bytes, &length) != 1) {
      EVP_MD_CTX_free(copy); throw std::runtime_error("SHA256 final failed");
    }
    EVP_MD_CTX_free(copy);
    std::ostringstream result; result << std::hex << std::setfill('0');
    for (unsigned index = 0; index < length; ++index) result << std::setw(2) << static_cast<unsigned>(bytes[index]);
    return result.str();
  }
};
template<class T> void pack(std::uint8_t* output, T value) {
  std::array<std::uint8_t, sizeof(T)> bytes; std::memcpy(bytes.data(), &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(bytes.begin(), bytes.end());
#endif
  std::memcpy(output, bytes.data(), sizeof(T));
}
template<class T> void hash_scalar(Hash& hash, T value) {
  std::array<std::uint8_t, sizeof(T)> bytes; pack(bytes.data(), value); hash.update(bytes.data(), bytes.size());
}
std::string digest_text(const std::string& value) { Hash hash; hash.update(value.data(), value.size()); return hash.hex(); }
std::array<double, 4> xyzw(const Quat& quaternion) { return {quaternion[1], quaternion[2], quaternion[3], quaternion[0]}; }
double wire_float32(double value) {
  // Match actual Livox binary32 storage even under Release vectorization.
  volatile float stored = static_cast<float>(value);
  return stored;
}

}  // namespace

struct FullRunObserver::Impl {
  Json config, sources;
  Json command_transitions = Json::array(), imu_samples = Json::array(), scans = Json::array();
  Json cameras = Json::object(), odometry = Json::array(), emitted = Json::object();
  fs::path output, project_root;
  std::string config_sha;
  Simulation* sim = nullptr;
  Transport* bridge = nullptr;
  std::unique_ptr<CourseMonitor> course;
  Vec3 previous_command{};
  bool have_command = false;
  Hash imu_hash, imu_si_hash;

  explicit Impl(const fs::path& path) : config(read_json(path)), output(config.at("output").get<std::string>()) {
    project_root = default_project_root();
    sources = read_json(output / "sources.json");
    config_sha = digest_text(config.dump(-1, ' ', true));
  }
  void count(const std::string& name) { emitted[name] = emitted.value(name, std::uint64_t{0}) + 1; }
  Json transport() const {
    const auto& options = bridge->options();
    return {{"robot_name", options.robot_name}, {"node_namespace", bridge->node_namespace()}, {"command_topic", bridge->command_topic()},
      {"command_timeout_wall_s", options.command_timeout_s},
      {"command_allows_lateral", robot_spec(options.robot_name).allows_lateral},
      {"command_allows_in_place_turn", robot_spec(options.robot_name).allows_in_place_turn},
      {"invalid_command_policy", "clear all velocity components immediately; valid subsequent commands resume control"},
      {"driver_contract", "livox_ros_driver2 6b9356cadf77084619ba406e6a0eb41163b08039"},
      {"imu_input", "MID360 internal IMU gyro rad/s and specific force m/s^2; sampling owned by caller"}, {"imu_rate_hz", 200},
      {"imu_raw", {{"topic", "/livox/imu"}, {"frame_id", options.lidar_frame}, {"angular_velocity_unit", "rad/s"},
        {"linear_acceleration_unit", "g"}, {"g0_m_s2", kG0}, {"numeric_precision", "float32 UDP values promoted to ROS float64"},
        {"orientation", "unavailable; fixed identity placeholder and zero covariances match ROS2 driver defaults, not an estimate"},
        {"frame_convention", "driver uses LiDAR housing axes/name; measurement origin is internal IMU, see imu_frame TF"}}},
      {"imu_si", {{"topic", "/livox/imu_si"}, {"frame_id", options.imu_frame}, {"angular_velocity_unit", "rad/s"},
        {"linear_acceleration_unit", "m/s^2"}, {"orientation", "unavailable; zero quaternion, orientation_covariance[0]=-1"},
        {"source", "same sampled physical SI values before UDP float32 conversion"}}},
      {"pointcloud2", {{"point_step", 26}, {"timestamp", "float64 absolute simulation nanoseconds at byte18"},
        {"tag", "0: MID360 normal high-confidence synthetic return"}}}};
  }
};

FullRunObserver::FullRunObserver(const fs::path& config_path) : impl_(std::make_unique<Impl>(config_path)) {}
FullRunObserver::~FullRunObserver() = default;
void FullRunObserver::before_loop(Simulation& sim, Transport* bridge) {
  auto& p = *impl_; p.sim = &sim; p.bridge = bridge;
  if (!bridge) throw std::invalid_argument("Full-run validation requires ROS transport");
  p.course = std::make_unique<CourseMonitor>(sim, p.config);
  char error[2048]{};
  if (!mj_saveLastXML((p.output / "compiled-model.xml").c_str(), sim.model(), error, sizeof(error)))
    throw std::runtime_error(std::string("Cannot save compiled model: ") + error);
  const std::array<const char*, 14> topics{"/clock", "/livox/lidar", "/livox/points", "/livox/imu", "/livox/imu_si", "/odom", "/tf", "/tf_static",
      "/camera/image_raw", "/camera/depth/image_raw", "/camera/panorama/image_raw", "/camera/panorama/depth/image_raw", "/camera/camera_info", "/full_run/status"};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (true) {
    bool ready = bridge->command_publisher_count() >= 1;
    for (const auto* topic : topics) ready = ready && bridge->subscriber_count(topic) >= 1;
    if (ready) break;
    if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("DDS discovery did not complete for every required topic");
    bridge->spin_once(.01);
  }
  bridge->publish_status(Json{{"run_id", p.config.at("run_id")}, {"pid", ::getpid()},
      {"command_topic", bridge->command_topic()}, {"config_sha256", p.config_sha}, {"finish_x_m", p.course->finish_x()},
      {"physics_timestep_s", p.course->timestep()}, {"ready", true}}.dump());
}
void FullRunObserver::on_step(Simulation& sim, double monotonic_s) {
  auto& p = *impl_;
  const Vec3 command = sim.command();
  if (!p.have_command || command != p.previous_command) {
    // Hook runs immediately after the step using this command. Time is the
    // actual observed steady clock, never inferred from simulation timestamps.
    p.command_transitions.push_back({{"stamp_ns", sim.stamp_ns()}, {"monotonic_s", monotonic_s}, {"command", command}});
    p.previous_command = command; p.have_command = true;
  }
  p.course->observe(sim);
  p.count("clock");
}
void FullRunObserver::on_imu(const ImuSample& sample, const std::array<std::uint64_t, 2>& source_stamps) {
  auto& p = *impl_;
  hash_scalar(p.imu_hash, sample.stamp_ns); hash_scalar(p.imu_si_hash, sample.stamp_ns);
  for (double value : sample.angular_velocity) hash_scalar(p.imu_hash, wire_float32(value));
  for (double value : sample.specific_force) hash_scalar(p.imu_hash, wire_float32(value / kG0));
  for (double value : {0., 0., 0., 1.}) hash_scalar(p.imu_hash, value);
  for (double value : sample.angular_velocity) hash_scalar(p.imu_si_hash, value);
  for (double value : sample.specific_force) hash_scalar(p.imu_si_hash, value);
  for (int index = 0; index < 4; ++index) hash_scalar(p.imu_si_hash, 0.);
  p.count("imu"); p.count("imu_si");
  p.imu_samples.push_back({{"stamp_ns", sample.stamp_ns}, {"source_stamps_ns", source_stamps},
      {"sampling_method", source_stamps[0] == source_stamps[1] ? "exact" : "linear_interpolation"}});
}
void FullRunObserver::on_scan(const LidarScan& scan, const std::array<std::uint64_t, 2>& history_bounds, std::uint64_t maximum_interval_ns) {
  auto& p = *impl_; Hash scan_hash, points_hash;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  static_assert(sizeof(std::array<float, 3>) == 3 * sizeof(float), "Packed XYZ required");
  scan_hash.update(scan.xyz.data(), scan.xyz.size() * 3 * sizeof(float));
  scan_hash.update(scan.offsets_ns.data(), scan.offsets_ns.size() * sizeof(std::uint32_t));
#else
  for (const auto& xyz : scan.xyz) for (float value : xyz) hash_scalar(scan_hash, value);
  for (std::uint32_t value : scan.offsets_ns) hash_scalar(scan_hash, value);
#endif
  scan_hash.update(scan.lines.data(), scan.lines.size());
  std::vector<std::uint8_t> packed(scan.xyz.size() * 26);
  for (std::size_t index = 0; index < scan.xyz.size(); ++index) {
    auto* target = packed.data() + 26 * index;
    for (int axis = 0; axis < 3; ++axis) pack(target + 4 * axis, scan.xyz[index][axis]);
    pack(target + 12, 100.f); target[16] = 0; target[17] = scan.lines[index];
    pack(target + 18, static_cast<double>(scan.stamp_ns + scan.offsets_ns[index]));
  }
  points_hash.update(packed.data(), packed.size());
  p.scans.push_back({{"stamp_ns", scan.stamp_ns}, {"point_count", scan.xyz.size()},
      {"scan_sha256", scan_hash.hex()}, {"pointcloud2_sha256", points_hash.hex()},
      {"metadata", {{"acquisition_model", "rolling_static_environment"}, {"history_coverage_ns", history_bounds},
        {"first_scheduled_point_ns", scan.stamp_ns}, {"last_scheduled_point_ns", scan.stamp_ns + (scan.rays_cast ? scan.rays_cast - 1 : 0) * 5000},
        {"maximum_pose_interval_ns", maximum_interval_ns}, {"pose_interpolation", "linear_translation_short_arc_slerp"},
        {"point_time_model", "uniform_from_csv_sample_order"}, {"sample_rate_hz", 200000}, {"robot_self_occlusion", false}}}});
  p.count("lidar"); p.count("pointcloud2");
}
void FullRunObserver::on_camera(std::uint64_t stamp, const CameraPair& pair) {
  auto& p = *impl_;
  auto record = [&](const std::string& prefix, const CameraFrame& frame) {
    Hash rgb, depth; rgb.update(frame.rgb.data(), frame.rgb.size());
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    depth.update(frame.depth.data(), frame.depth.size() * sizeof(float));
#else
    for (float value : frame.depth) hash_scalar(depth, value);
#endif
    const std::string rgb_key = prefix + "rgb", depth_key = prefix + "depth";
    if (!p.cameras.contains(rgb_key)) p.cameras[rgb_key] = Json::array();
    if (!p.cameras.contains(depth_key)) p.cameras[depth_key] = Json::array();
    p.cameras[rgb_key].push_back({{"stamp_ns", stamp}, {"sha256", rgb.hex()}});
    p.cameras[depth_key].push_back({{"stamp_ns", stamp}, {"sha256", depth.hex()}});
    p.count(rgb_key); p.count(depth_key);
  };
  record("", pair.perspective); record("panorama_", pair.panorama); p.count("camera_info");
}
void FullRunObserver::on_odometry(std::uint64_t stamp, const Pose& pose, const Vec3& linear, const Vec3& angular) {
  auto& p = *impl_;
  p.odometry.push_back({{"stamp_ns", stamp}, {"position", pose.position}, {"quaternion", xyzw(pose.quaternion)},
      {"linear", linear}, {"angular", angular}});
  p.count("odom"); p.count("tf");
}
bool FullRunObserver::should_stop(Simulation& sim) {
  return impl_->course->should_stop(sim);
}
void FullRunObserver::finish(Json& summary) {
  auto& p = *impl_;
  if (!p.sim || !p.bridge) throw std::runtime_error("FullRunObserver finish without before_loop");
  Json current_sources = Json::object();
  for (auto item = p.sources.begin(); item != p.sources.end(); ++item) current_sources[item.key()] = sha256_file(p.project_root / item.key());
  // Runtime provides measured intervals ending before DDS drain; never estimate
  // performance from observer setup or append synthetic sensor counts.
  for (const auto* key : {"wall_seconds", "real_time_factor", "resources_loop", "resources_run"})
    if (!summary.contains(key)) throw std::runtime_error(std::string("Runtime summary missing ") + key);
  if (summary.contains("error") && !summary["error"].is_null()) {
    p.course->mark_error();
  }
  summary.update({{"run_id", p.config.at("run_id")}, {"pid", ::getpid()}, {"config_sha256", p.config_sha},
      {"runtime", "cpp"}, {"command_topic", p.bridge->command_topic()}, {"config", p.config}, {"sources", current_sources},
      {"terrain", p.course->profile()}, {"controller", p.sim->controller_name()}, {"physics_timestep_s", p.course->timestep()},
      {"deployment", p.sim->deployment_metadata()}, {"control_mode", p.config.at("control_mode")},
      {"control_feedback_scope", "Outer body velocity/heading tracker; controller feedback is recorded in deployment"},
      {"control_feedback_source", p.config.at("control_mode") == "tracked" && robot_spec(p.config.at("robot")).command_tracking ? Json("ideal simulated body velocity and heading") : Json(nullptr)},
      {"formal_course", p.course->report()}, {"friction", p.course->friction_report()},
      {"simulation_end_ns", p.sim->stamp_ns()}, {"emitted", p.emitted}, {"imu_sha256", p.imu_hash.hex()},
      {"imu_si_sha256", p.imu_si_hash.hex()}, {"imu_samples", p.imu_samples}, {"scans", p.scans},
      {"cameras", p.cameras}, {"odometry", p.odometry}, {"command_transitions", p.command_transitions},
      {"final_command", p.sim->command()}, {"ros_transport", p.transport()}});
  if (!summary.contains("error")) summary["error"] = nullptr;
  summary["lidar_model"] = {{"acquisition_model", "rolling_static_environment"}, {"sample_rate_hz", 200000},
      {"point_time_model", "uniform_from_csv_sample_order"}, {"pose_interpolation", "linear_translation_short_arc_slerp"},
      {"robot_self_occlusion", false}, {"range_limits_m", {.1, 40.}}};
  summary["imu_sampler"] = {{"rate_hz", 200}, {"period_ns", 5000000}, {"samples_emitted", p.imu_samples.size()},
      {"sampling_approximation", "exact physical values on shared timestamps; linear interpolation between adjacent physics samples otherwise"}};
  write_json(p.output / "worker.json", summary);
}
}  // namespace simple_rbt
