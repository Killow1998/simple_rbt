#include "simple_rbt/ros_bridge.hpp"
#include "simple_rbt/robots.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <regex>
#include <set>
#include <stdexcept>
#include <utility>

namespace simple_rbt {
namespace {
using SteadyClock = std::chrono::steady_clock;
constexpr double kG0 = 9.80665;
constexpr std::uint64_t kBillion = 1000000000ULL;

void finite(const Vec3& v) {
  if (!std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); }))
    throw std::invalid_argument("vector must contain three finite values");
}
void valid_pose(const Pose& pose) {
  finite(pose.position);
  double norm2 = 0.;
  for (double x : pose.quaternion) {
    if (!std::isfinite(x)) throw std::invalid_argument("quaternion must be finite");
    norm2 += x * x;
  }
  if (std::abs(std::sqrt(norm2) - 1.) > 1.e-5)
    throw std::invalid_argument("WXYZ quaternion must have unit norm");
}
builtin_interfaces::msg::Time stamp(std::uint64_t ns) {
  if (ns / kBillion > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()))
    throw std::invalid_argument("simulation timestamp is outside ROS Time range");
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(ns / kBillion);
  result.nanosec = static_cast<std::uint32_t>(ns % kBillion);
  return result;
}
std_msgs::msg::Header header(std::uint64_t ns, const std::string& frame) {
  std_msgs::msg::Header result;
  result.stamp = stamp(ns);
  result.frame_id = frame;
  return result;
}
template<class Vector> void xyz(Vector& out, const Vec3& in) {
  out.x = in[0]; out.y = in[1]; out.z = in[2];
}
void quaternion(geometry_msgs::msg::Quaternion& out, const Quat& wxyz) {
  out.w = wxyz[0]; out.x = wxyz[1]; out.y = wxyz[2]; out.z = wxyz[3];
}
geometry_msgs::msg::TransformStamped transform(std::uint64_t ns, const std::string& parent,
                                              const std::string& child, const Pose& pose) {
  geometry_msgs::msg::TransformStamped result;
  result.header = header(ns, parent); result.child_frame_id = child;
  xyz(result.transform.translation, pose.position);
  quaternion(result.transform.rotation, pose.quaternion);
  return result;
}
template<class T> void little_endian(std::uint8_t* target, T value) {
  std::array<std::uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(bytes.begin(), bytes.end());
#endif
  std::memcpy(target, bytes.data(), sizeof(T));
}
std::size_t domain(const RosOptions& options) {
  if (options.domain_id) return *options.domain_id;
  const char* setting = std::getenv("ROS_DOMAIN_ID");
  if (!setting) return 83;
  const std::string value(setting);
  if (value.empty() || !std::all_of(value.begin(), value.end(), [](char c) { return c >= '0' && c <= '9'; }))
    throw std::invalid_argument("ROS_DOMAIN_ID must be a nonnegative integer");
  return std::stoull(value);
}
}  // namespace

std::string robot_command_topic(const std::string& robot_name) {
  static const std::regex name("[A-Za-z_][A-Za-z0-9_]*");
  if (!std::regex_match(robot_name, name))
    throw std::invalid_argument("robot_name must be one ROS name and cannot begin with a digit");
  return "/" + robot_name + "/cmd_vel";
}
bool robot_command_supported(const std::string& robot_name, const Vec3& command) {
  return supported_command(robot_name, command);
}

struct RosBridge::Impl {
  RosOptions options;
  std::string command_topic, node_namespace;
  mutable std::mutex publication_mutex, command_mutex;
  Vec3 latest_command{};
  std::optional<SteadyClock::time_point> received_at;
  bool closed = false;
  bool has_camera = false;
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commands;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu, imu_si;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 2> rgb, depth;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf;

  Impl(RosOptions given, const Extrinsics& extrinsics) : options(std::move(given)),
      command_topic(robot_command_topic(options.robot_name)), node_namespace("/" + options.robot_name),
      has_camera(extrinsics.has_camera) {
    if (!std::isfinite(options.command_timeout_s) || options.command_timeout_s <= 0.)
      throw std::invalid_argument("command timeout must be positive and finite");
    if (!std::isfinite(options.perspective_fov_y_deg) || options.perspective_fov_y_deg <= 0. ||
        options.perspective_fov_y_deg >= 180.)
      throw std::invalid_argument("perspective field of view must lie between 0 and 180 degrees");
    valid_pose(extrinsics.lidar); valid_pose(extrinsics.imu);
    if (has_camera) valid_pose(extrinsics.camera);
    std::vector<std::string> frames{options.odom_frame, options.base_frame, options.lidar_frame, options.imu_frame};
    if (has_camera) frames.push_back(options.camera_frame);
    for (const auto& frame : frames)
      if (frame.empty() || frame.front() == '/') throw std::invalid_argument("TF frames must be nonempty without leading slash");
    if (std::set<std::string>(frames.begin(), frames.end()).size() != frames.size())
      throw std::invalid_argument("TF frame names must be distinct");
    // Respect deliberate user settings. Default discovery stays on this host.
    if (!std::getenv("ROS_LOCALHOST_ONLY")) ::setenv("ROS_LOCALHOST_ONLY", "1", 0);
    context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init;
    init.set_domain_id(domain(options));
    context->init(0, nullptr, init);
    try {
      rclcpp::NodeOptions node_options;
      node_options.context(context).use_global_arguments(false);
      node = std::make_shared<rclcpp::Node>(options.node_name, node_namespace, node_options);
      rclcpp::ExecutorOptions executor_options;
      executor_options.context = context;
      executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
      executor->add_node(node);
      clock = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1).best_effort());
      lidar = node->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", 10);
      if (options.publish_pointcloud2) points = node->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/points", 10);
      imu = node->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", 10);
      imu_si = node->create_publisher<sensor_msgs::msg::Imu>("/livox/imu_si", 10);
      odom = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
      if (has_camera) {
        for (int index = 0; index < 2; ++index) {
          const std::string prefix = index == 0 ? "/camera" : "/camera/panorama";
          rgb[index] = node->create_publisher<sensor_msgs::msg::Image>(prefix + "/image_raw", 2);
          depth[index] = node->create_publisher<sensor_msgs::msg::Image>(prefix + "/depth/image_raw", 2);
        }
        camera_info = node->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 2);
      }
      commands = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
          [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
            const Vec3 command{message->linear.x, message->linear.y, message->angular.z};
            std::lock_guard<std::mutex> lock(command_mutex);
            if (robot_command_supported(options.robot_name, command)) {
              latest_command = command; received_at = SteadyClock::now();
            } else {
              latest_command = {}; received_at.reset();
            }
          });
      status = node->create_publisher<std_msgs::msg::String>(options.status_topic, rclcpp::QoS(1).transient_local());
      tf = std::make_unique<tf2_ros::TransformBroadcaster>(node);
      static_tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
      std::vector<geometry_msgs::msg::TransformStamped> transforms{
          transform(0, options.base_frame, options.lidar_frame, extrinsics.lidar),
          transform(0, options.base_frame, options.imu_frame, extrinsics.imu)};
      if (has_camera) transforms.push_back(transform(0, options.base_frame, options.camera_frame, extrinsics.camera));
      static_tf->sendTransform(transforms);
    } catch (...) {
      context->shutdown("ROS bridge construction failed");
      throw;
    }
  }
  void require_open() const {
    if (closed || !context->is_valid()) throw std::runtime_error("ROS bridge is closed");
  }
};

RosBridge::RosBridge(RosOptions options, const Extrinsics& extrinsics)
    : impl_(std::make_unique<Impl>(std::move(options), extrinsics)) {}
RosBridge::~RosBridge() { try { close(); } catch (...) {} }
const RosOptions& RosBridge::options() const { return impl_->options; }
const std::string& RosBridge::command_topic() const { return impl_->command_topic; }
const std::string& RosBridge::node_namespace() const { return impl_->node_namespace; }
Vec3 RosBridge::command() const {
  std::lock_guard<std::mutex> lock(impl_->command_mutex);
  if (!impl_->received_at || std::chrono::duration<double>(SteadyClock::now() - *impl_->received_at).count() >= impl_->options.command_timeout_s)
    return {};
  return impl_->latest_command;
}
void RosBridge::spin_once(double timeout_s) {
  if (!std::isfinite(timeout_s) || timeout_s < 0.) throw std::invalid_argument("spin timeout must be nonnegative and finite");
  std::lock_guard<std::mutex> lock(impl_->publication_mutex);
  if (!impl_->closed && impl_->context->is_valid())
    impl_->executor->spin_once(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timeout_s)));
}
std::size_t RosBridge::subscriber_count(const std::string& topic) const {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex);
  impl_->require_open(); return impl_->node->count_subscribers(topic);
}
std::size_t RosBridge::command_publisher_count() const {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex);
  impl_->require_open(); return impl_->commands->get_publisher_count();
}
void RosBridge::publish_clock(std::uint64_t ns) {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  rosgraph_msgs::msg::Clock message; message.clock = stamp(ns); impl_->clock->publish(message);
}
void RosBridge::publish_status(const std::string& json) {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  std_msgs::msg::String message; message.data = json; impl_->status->publish(message);
}
// A physical binary32 storage point is part of the Livox wire contract.
// GCC 11 -O3 SLP otherwise elides double->float->double for paired axes;
// retain SI separately and force only the six UDP-equivalent values to round.
static double wire_float32(double value) {
  volatile float stored = static_cast<float>(value);
  return stored;
}
void RosBridge::publish_imu(const ImuSample& sample) {
  finite(sample.angular_velocity); finite(sample.specific_force);
  Vec3 gyro{}, acceleration_g{};
  for (int axis = 0; axis < 3; ++axis) {
    gyro[axis] = wire_float32(sample.angular_velocity[axis]);
    acceleration_g[axis] = wire_float32(sample.specific_force[axis] / kG0);
  }
  finite(gyro); finite(acceleration_g);
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  sensor_msgs::msg::Imu raw;
  raw.header = header(sample.stamp_ns, impl_->options.lidar_frame);
  xyz(raw.angular_velocity, gyro); xyz(raw.linear_acceleration, acceleration_g);
  // Driver default only: fixed identity is neither simulator truth nor an IMU estimate.
  raw.orientation.w = 1.;
  impl_->imu->publish(raw);
  sensor_msgs::msg::Imu si;
  si.header = header(sample.stamp_ns, impl_->options.imu_frame);
  xyz(si.angular_velocity, sample.angular_velocity); xyz(si.linear_acceleration, sample.specific_force);
  si.orientation.w = 0.; si.orientation_covariance[0] = -1.;
  impl_->imu_si->publish(si);
}
void RosBridge::publish_scan(const LidarScan& scan) {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  const auto count = scan.xyz.size();
  if (scan.offsets_ns.size() != count || scan.lines.size() != count)
    throw std::invalid_argument("scan offsets and lines must match xyz count");
  if (count > std::numeric_limits<std::uint32_t>::max() / 26)
    throw std::invalid_argument("scan exceeds ROS pointcloud row_step range");
  for (const auto& xyz_value : scan.xyz) for (float value : xyz_value)
    if (!std::isfinite(value)) throw std::invalid_argument("scan xyz must be finite");
  livox_ros_driver2::msg::CustomMsg custom;
  custom.header = header(scan.stamp_ns, impl_->options.lidar_frame);
  custom.timebase = scan.stamp_ns; custom.point_num = static_cast<std::uint32_t>(count);
  custom.lidar_id = 0; custom.points.resize(count);
  sensor_msgs::msg::PointCloud2 cloud;
  if (impl_->points) {
    cloud.header = custom.header; cloud.height = 1; cloud.width = static_cast<std::uint32_t>(count);
    using Field = sensor_msgs::msg::PointField;
    const std::array<const char*, 7> names{"x", "y", "z", "intensity", "tag", "line", "timestamp"};
    const std::array<std::uint32_t, 7> offsets{0, 4, 8, 12, 16, 17, 18};
    const std::array<std::uint8_t, 7> datatypes{Field::FLOAT32, Field::FLOAT32, Field::FLOAT32,
                                             Field::FLOAT32, Field::UINT8, Field::UINT8, Field::FLOAT64};
    cloud.fields.resize(7);
    for (int field = 0; field < 7; ++field) {
      cloud.fields[field].name = names[field]; cloud.fields[field].offset = offsets[field];
      cloud.fields[field].datatype = datatypes[field]; cloud.fields[field].count = 1;
    }
    cloud.is_bigendian = false; cloud.is_dense = true; cloud.point_step = 26;
    cloud.row_step = static_cast<std::uint32_t>(count * 26); cloud.data.resize(count * 26);
  }
  for (std::size_t index = 0; index < count; ++index) {
    auto& point = custom.points[index];
    point.x = scan.xyz[index][0]; point.y = scan.xyz[index][1]; point.z = scan.xyz[index][2];
    point.offset_time = scan.offsets_ns[index]; point.line = scan.lines[index];
    point.tag = 0; point.reflectivity = 100;
    if (impl_->points) {
      auto* target = cloud.data.data() + index * 26;
      little_endian(target, point.x); little_endian(target + 4, point.y); little_endian(target + 8, point.z);
      little_endian(target + 12, 100.f); target[16] = 0; target[17] = point.line;
      little_endian(target + 18, static_cast<double>(scan.stamp_ns + point.offset_time));
    }
  }
  impl_->lidar->publish(custom);
  if (impl_->points) impl_->points->publish(cloud);
}
void RosBridge::publish_odometry(std::uint64_t ns, const Pose& pose,
                                 const Vec3& linear_velocity, const Vec3& angular_velocity) {
  valid_pose(pose); finite(linear_velocity); finite(angular_velocity);
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  nav_msgs::msg::Odometry message;
  message.header = header(ns, impl_->options.odom_frame); message.child_frame_id = impl_->options.base_frame;
  xyz(message.pose.pose.position, pose.position); quaternion(message.pose.pose.orientation, pose.quaternion);
  xyz(message.twist.twist.linear, linear_velocity); xyz(message.twist.twist.angular, angular_velocity);
  impl_->odom->publish(message);
  impl_->tf->sendTransform(transform(ns, impl_->options.odom_frame, impl_->options.base_frame, pose));
}
void RosBridge::publish_camera(std::uint64_t ns, const CameraFrame& frame, bool panorama) {
  if (frame.width <= 0 || frame.height <= 0 || frame.width > std::numeric_limits<int>::max() / 4)
    throw std::invalid_argument("camera width and height must be positive and fit ROS row stride");
  const std::size_t count = static_cast<std::size_t>(frame.width) * frame.height;
  if (frame.rgb.size() != count * 3 || frame.depth.size() != count)
    throw std::invalid_argument("camera RGB/depth sizes must match width and height");
  if (frame.radial_depth != panorama)
    throw std::invalid_argument("camera radial_depth must match panorama kind");
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  if (!impl_->has_camera) throw std::invalid_argument("camera extrinsics are required to publish camera images");
  const int kind = panorama ? 1 : 0;
  sensor_msgs::msg::Image rgb;
  rgb.header = header(ns, impl_->options.camera_frame); rgb.height = frame.height; rgb.width = frame.width;
  rgb.encoding = "rgb8"; rgb.is_bigendian = false; rgb.step = frame.width * 3; rgb.data = frame.rgb;
  sensor_msgs::msg::Image depth;
  depth.header = rgb.header; depth.height = frame.height; depth.width = frame.width;
  depth.encoding = "32FC1"; depth.is_bigendian = false; depth.step = frame.width * 4;
  depth.data.resize(count * sizeof(float));
  for (std::size_t index = 0; index < count; ++index)
    little_endian(depth.data.data() + index * sizeof(float), frame.depth[index]);
  impl_->rgb[kind]->publish(rgb); impl_->depth[kind]->publish(depth);
  if (!panorama) {
    sensor_msgs::msg::CameraInfo info;
    info.header = rgb.header; info.height = frame.height; info.width = frame.width;
    const double focal = frame.height / (2. * std::tan(impl_->options.perspective_fov_y_deg * std::acos(-1.) / 360.));
    const double cx = (frame.width - 1.) / 2., cy = (frame.height - 1.) / 2.;
    info.distortion_model = "plumb_bob"; info.d.assign(5, 0.);
    info.k = {focal, 0., cx, 0., focal, cy, 0., 0., 1.};
    info.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    info.p = {focal, 0., cx, 0., 0., focal, cy, 0., 0., 0., 1., 0.};
    impl_->camera_info->publish(info);
  }
}
void RosBridge::close() {
  if (!impl_) return;
  std::lock_guard<std::mutex> lock(impl_->publication_mutex);
  if (impl_->closed) return;
  impl_->closed = true;
  impl_->executor->cancel(); impl_->executor->remove_node(impl_->node);
  impl_->commands.reset(); impl_->clock.reset(); impl_->lidar.reset(); impl_->points.reset();
  impl_->imu.reset(); impl_->imu_si.reset(); impl_->odom.reset();
  for (auto& pub : impl_->rgb) pub.reset();
  for (auto& pub : impl_->depth) pub.reset();
  impl_->camera_info.reset(); impl_->status.reset(); impl_->tf.reset(); impl_->static_tf.reset();
  impl_->executor.reset(); impl_->node.reset(); impl_->context->shutdown("ROS bridge closed");
  std::lock_guard<std::mutex> command_lock(impl_->command_mutex);
  impl_->received_at.reset(); impl_->latest_command = {};
}
}  // namespace simple_rbt
