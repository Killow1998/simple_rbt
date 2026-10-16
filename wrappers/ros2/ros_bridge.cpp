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
#include "../common/wire.hpp"

namespace simple_rbt {
using namespace wire;
namespace {
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
      odom = node->create_publisher<nav_msgs::msg::Odometry>(options.odom_topic, 10);
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
  void publish_transform(const messages::Transform& value) { tf->sendTransform(value); }
  void require_open() const {
    if (closed || !context->is_valid()) throw std::runtime_error("ROS bridge is closed");
  }
};

RosBridge::RosBridge(RosOptions options, const Extrinsics& extrinsics)
    : impl_(std::make_unique<Impl>(std::move(options), extrinsics)) {}
RosBridge::~RosBridge() { try { close(); } catch (...) {} }
const RosOptions& RosBridge::options() const { return impl_->options; }
const char* RosBridge::backend() const { return "ros2"; }
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
#include "../common/publish.inc"
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
