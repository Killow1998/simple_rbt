#include "../common/wire.hpp"
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <map>
#include <mutex>
#include <utility>

namespace simple_rbt {
using namespace wire;
struct RosBridge::Impl {
  RosOptions options;
  std::string command_topic, node_namespace;
  mutable std::mutex publication_mutex, command_mutex;
  Vec3 latest_command{};
  JointCommand latest_joints;
  std::optional<SteadyClock::time_point> joints_received;
  std::string camera_parent,
      external_frame = "external_camera_optical_frame",
      external_depth_frame = "external_depth_optical_frame";
  bool has_external_camera = false;
  std::optional<SteadyClock::time_point> received_at;
  bool closed = false, has_camera = false, separate_camera_frames = false, has_camera_imu = false;
  ros::CallbackQueue callbacks;
  std::unique_ptr<ros::NodeHandle> node;
  ros::Subscriber commands, joint_commands;
  ros::Publisher clock, lidar, points, imu, imu_si, odom, camera_info, depth_info, camera_imu, status;
  ros::Publisher tf, static_tf, joints, external_info, external_depth_info;
  std::array<ros::Publisher, 3> rgb, depth;
  std::map<std::string, ros::Publisher> publishers;
  template<class Message> ros::Publisher advertise(const std::string& topic,
                                                   int queue, bool latch = false) {
    auto publisher = node->advertise<Message>(topic, queue, latch);
    publishers.emplace(topic, publisher); return publisher;
  }
  Impl(RosOptions given, const Extrinsics& extrinsics) : options(std::move(given)),
      command_topic(robot_command_topic(options.robot_name)),
      node_namespace("/" + options.robot_name), has_camera(extrinsics.has_camera), separate_camera_frames(extrinsics.separate_camera_frames),
      has_camera_imu(extrinsics.has_camera_imu) {
    if (options.domain_id) throw std::invalid_argument("ROS 1 uses ROS_MASTER_URI, not a DDS domain");
    camera_parent = extrinsics.camera_parent_frame;
    has_external_camera = extrinsics.has_external_camera;
    const auto *spec = find_robot_spec(options.robot_name);
    const bool arm = spec && spec->joint_control;
    const std::string camera_prefix =
        arm ? node_namespace + "/wrist_camera" : "/camera";
    const std::string external_prefix = node_namespace + "/external_camera";
    if (!std::isfinite(options.command_timeout_s) || options.command_timeout_s <= 0.)
      throw std::invalid_argument("command timeout must be positive and finite");
    if (!std::isfinite(options.perspective_fov_y_deg) || options.perspective_fov_y_deg <= 0. ||
        options.perspective_fov_y_deg >= 180.)
      throw std::invalid_argument("perspective field of view must lie between 0 and 180 degrees");
    valid_pose(extrinsics.lidar); valid_pose(extrinsics.imu);
    if (has_camera) valid_pose(extrinsics.camera);
    if (has_camera && separate_camera_frames) valid_pose(extrinsics.camera_depth);
    if (has_camera_imu) valid_pose(extrinsics.camera_imu);
    std::vector<std::string> frames{options.odom_frame, options.base_frame, options.lidar_frame, options.imu_frame};
    if (has_camera) frames.push_back(options.camera_frame);
    if (has_camera && separate_camera_frames) frames.push_back(options.camera_depth_frame);
    if (has_camera_imu) frames.push_back(options.camera_imu_frame);
    if (!camera_parent.empty()) frames.push_back(camera_parent);
    if (has_external_camera) {
      valid_pose(extrinsics.external_camera);
      valid_pose(extrinsics.external_depth);
      frames.push_back(external_frame);
      frames.push_back(external_depth_frame);
    }
    for (const auto& frame : frames)
      if (frame.empty() || frame.front() == '/') throw std::invalid_argument("TF frames must be nonempty without leading slash");
    if (std::set<std::string>(frames.begin(), frames.end()).size() != frames.size())
      throw std::invalid_argument("TF frame names must be distinct");
    if (!ros::isInitialized()) ros::init(ros::M_string{}, options.node_name,
        ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    node = std::make_unique<ros::NodeHandle>(node_namespace);
    node->setCallbackQueue(&callbacks);
    clock = advertise<messages::Clock>("/clock", 1);
    lidar = advertise<messages::Custom>("/livox/lidar", 10);
    if (options.publish_pointcloud2) points = advertise<messages::PointCloud>("/livox/points", 10);
    imu = advertise<messages::Imu>("/livox/imu", 64);
    imu_si = advertise<messages::Imu>("/livox/imu_si", 64);
    if (has_camera_imu)
      camera_imu = advertise<messages::Imu>(camera_prefix + "/imu", 64);
    odom = advertise<messages::Odometry>(options.odom_topic, 10);
    if (has_camera) {
      for (int index = 0; index < 2; ++index) {
        const std::string prefix =
            index == 0 ? camera_prefix : camera_prefix + "/panorama";
        rgb[index] = advertise<messages::Image>(prefix + "/image_raw", 2);
        depth[index] = advertise<messages::Image>(prefix + "/depth/image_raw", 2);
      }
      camera_info =
          advertise<messages::CameraInfo>(camera_prefix + "/camera_info", 2);
      depth_info = advertise<messages::CameraInfo>(
          camera_prefix + "/depth/camera_info", 2);
    }
    if (has_external_camera) {
      rgb[2] = advertise<messages::Image>(external_prefix + "/image_raw", 2);
      depth[2] =
          advertise<messages::Image>(external_prefix + "/depth/image_raw", 2);
      external_info =
          advertise<messages::CameraInfo>(external_prefix + "/camera_info", 2);
      external_depth_info = advertise<messages::CameraInfo>(
          external_prefix + "/depth/camera_info", 2);
    }
    if (arm) {
      joints = advertise<messages::Joints>("joint_states", 10);
      joint_commands = node->subscribe<messages::Joints>(
          "joint_position_command", 10,
          [this](const messages::Joints::ConstPtr &message) {
            std::lock_guard<std::mutex> lock(command_mutex);
            latest_joints = {message->name, message->position, false};
            joints_received = SteadyClock::now();
          },
          ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    }
    if (!arm)
      commands = node->subscribe<geometry_msgs::Twist>(
          "cmd_vel", 10,
          [this](const geometry_msgs::Twist::ConstPtr &message) {
            const Vec3 command{message->linear.x, message->linear.y,
                               message->angular.z};
            std::lock_guard<std::mutex> lock(command_mutex);
            if (robot_command_supported(options.robot_name, command)) {
              latest_command = command;
              received_at = SteadyClock::now();
            } else {
              latest_command = {};
              received_at.reset();
            }
          },
          ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    status = advertise<messages::String>(options.status_topic, 1, true);
    tf = advertise<tf2_msgs::TFMessage>("/tf", 10);
    static_tf = advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);
    tf2_msgs::TFMessage fixed;
    fixed.transforms = {transform(0, options.base_frame, options.lidar_frame, extrinsics.lidar),
                        transform(0, options.base_frame, options.imu_frame, extrinsics.imu)};
    if (has_camera)
      fixed.transforms.push_back(transform(
          0, (camera_parent.empty() ? options.base_frame : camera_parent),
          options.camera_frame, extrinsics.camera));
    if (has_camera && separate_camera_frames)
      fixed.transforms.push_back(transform(
          0, (camera_parent.empty() ? options.base_frame : camera_parent),
          options.camera_depth_frame, extrinsics.camera_depth));
    if (has_camera_imu)
      fixed.transforms.push_back(transform(
          0, (camera_parent.empty() ? options.base_frame : camera_parent),
          options.camera_imu_frame, extrinsics.camera_imu));
    if (has_external_camera) {
      fixed.transforms.push_back(transform(
          0, options.odom_frame, external_frame, extrinsics.external_camera));
      fixed.transforms.push_back(transform(0, options.odom_frame,
                                           external_depth_frame,
                                           extrinsics.external_depth));
    }
    static_tf.publish(fixed);
  }
  void require_open() const {
    if (closed || !ros::ok()) throw std::runtime_error("ROS wrapper is closed");
  }
  void publish_transform(const messages::Transform& value) {
    tf2_msgs::TFMessage message; message.transforms.push_back(value); tf.publish(message);
  }
};
RosBridge::RosBridge(RosOptions options, const Extrinsics& extrinsics)
    : impl_(std::make_unique<Impl>(std::move(options), extrinsics)) {}
RosBridge::~RosBridge() { try { close(); } catch (...) {} }
const RosOptions& RosBridge::options() const { return impl_->options; }
const char* RosBridge::backend() const { return "ros1"; }
const std::string& RosBridge::command_topic() const { return impl_->command_topic; }
const std::string& RosBridge::node_namespace() const { return impl_->node_namespace; }
Vec3 RosBridge::command() const {
  std::lock_guard<std::mutex> lock(impl_->command_mutex);
  if (!impl_->received_at || SteadyClock::now() - *impl_->received_at >=
      std::chrono::duration<double>(impl_->options.command_timeout_s)) return {};
  return impl_->latest_command;
}
void RosBridge::spin_once(double timeout_s) {
  if (!std::isfinite(timeout_s) || timeout_s < 0.)
    throw std::invalid_argument("spin timeout must be nonnegative and finite");
  std::lock_guard<std::mutex> lock(impl_->publication_mutex);
  if (!impl_->closed) impl_->callbacks.callAvailable(ros::WallDuration(timeout_s));
}
std::size_t RosBridge::subscriber_count(const std::string& topic) const {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  const auto found = impl_->publishers.find(topic);
  return found == impl_->publishers.end() ? 0 : found->second.getNumSubscribers();
}
std::size_t RosBridge::command_publisher_count() const {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  return impl_->joint_commands ? impl_->joint_commands.getNumPublishers()
                               : impl_->commands.getNumPublishers();
}
void RosBridge::publish_clock(std::uint64_t ns) {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  messages::Clock message; message.clock = stamp(ns); impl_->clock.publish(message);
}
#include "../common/publish.inc"
void RosBridge::close() {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex);
  if (impl_->closed) return;
  impl_->closed = true;
  impl_->commands.shutdown();
  impl_->joint_commands.shutdown();
  for (auto& item : impl_->publishers) item.second.shutdown();
  impl_->publishers.clear(); impl_->node.reset(); impl_->callbacks.clear();
  std::lock_guard<std::mutex> command_lock(impl_->command_mutex);
  impl_->latest_command = {}; impl_->received_at.reset();
  impl_->latest_joints = {};
  impl_->joints_received.reset();
}
} // namespace simple_rbt
