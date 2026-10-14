// CPU-only localhost DDS contract test. Optional argv[1] writes received CDR
// fixtures for an independent Python RosBridge oracle comparison.
#include "simple_rbt/ros_bridge.hpp"
#include "simple_rbt/robots.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <thread>

namespace {
using namespace simple_rbt;
using namespace std::chrono_literals;
int checks = 0;
std::size_t test_domain() {
  const char* raw = std::getenv("ROS_DOMAIN_ID");
  if (!raw) return 89;  // Keep standalone regression off the user/default 83.
  const std::string text(raw);
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
    throw std::invalid_argument("ROS_DOMAIN_ID must be an integer in [0,232]");
  const auto value = std::stoul(text);
  if (value > 232) throw std::invalid_argument("ROS_DOMAIN_ID must be an integer in [0,232]");
  return value;
}
void require(bool condition, const std::string& what) {
  ++checks;
  if (!condition) throw std::runtime_error(what);
}
template<class F> void rejects(F&& function, const std::string& what) {
  bool rejected = false;
  try { function(); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, what);
}
std::uint64_t ns(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}
template<class T> T value_at(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  T result; std::memcpy(&result, bytes.data() + offset, sizeof(T)); return result;
}
template<class Message> void fixture(const std::string& directory, const std::string& name, const Message& message) {
  if (directory.empty()) return;
  rclcpp::Serialization<Message> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&message, &serialized);
  const auto& raw = serialized.get_rcl_serialized_message();
  std::ofstream output(directory + "/" + name + ".cdr", std::ios::binary);
  output.write(reinterpret_cast<const char*>(raw.buffer), raw.buffer_length);
  require(output.good(), "write CDR fixture " + name);
}
LidarScan scan(std::uint64_t stamp, std::size_t count, float shift) {
  LidarScan result; result.stamp_ns = stamp; result.rays_cast = count;
  for (std::size_t index = 0; index < count; ++index) {
    result.xyz.push_back({shift + static_cast<float>(index), -2.5f, 3.75f});
    result.offsets_ns.push_back(static_cast<std::uint32_t>(index * 5000));
    result.lines.push_back(static_cast<std::uint8_t>(index % 4));
  }
  return result;
}
struct Peer {
  rclcpp::Context::SharedPtr context = std::make_shared<rclcpp::Context>();
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  explicit Peer(std::size_t domain) {
    rclcpp::InitOptions init; init.set_domain_id(domain); context->init(0, nullptr, init);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("simple_rbt_cpp_transport_test", options);
    rclcpp::ExecutorOptions executor_options; executor_options.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor->add_node(node);
  }
  template<class Message, class Callback> void subscribe(const std::string& topic, Callback callback,
                                                        const rclcpp::QoS& qos = rclcpp::QoS(10)) {
    subscriptions.push_back(node->create_subscription<Message>(topic, qos,
        [callback](typename Message::ConstSharedPtr message) { callback(message); }));
  }
  std::string discovery(const RosBridge& bridge, const rclcpp::PublisherBase* publisher = nullptr) const {
    std::ostringstream text;
    text << "domain=" << test_domain() << " topic=" << bridge.command_topic()
         << " bridge_matched_publishers=" << bridge.command_publisher_count();
    if (publisher) text << " peer_matched_subscribers=" << publisher->get_subscription_count();
    const auto endpoints = [&](const char* label, const auto& values) {
      text << ' ' << label << '=' << values.size() << '[';
      for (const auto& value : values) {
        text << value.node_namespace() << '/' << value.node_name() << " gid=";
        for (auto byte : value.endpoint_gid()) text << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
        text << std::dec << ';';
      }
      text << ']';
    };
    endpoints("graph_publishers", node->get_publishers_info_by_topic(bridge.command_topic()));
    endpoints("graph_subscriptions", node->get_subscriptions_info_by_topic(bridge.command_topic()));
    return text.str();
  }
  void until(const std::function<bool()>& condition, RosBridge& bridge, const std::string& what, double timeout = 5.,
             const rclcpp::PublisherBase* publisher = nullptr) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    while (!condition() && std::chrono::steady_clock::now() < end) {
      bridge.spin_once(); executor->spin_once(5ms);
    }
    const bool ready = condition();
    if (!ready) std::cerr << "DDS wait failed: " << discovery(bridge, publisher) << std::endl;
    require(ready, what);
  }
  ~Peer() {
    executor->cancel(); executor->remove_node(node); subscriptions.clear(); executor.reset(); node.reset();
    context->shutdown("transport test complete");
  }
};
const std::vector<std::string> robot_names = [] {
  std::vector<std::string> names;
  for (const auto& spec : robot_specs()) names.push_back(spec.name);
  return names;
}();
void command_probe(const std::string& name, const Extrinsics& extrinsics) {
  const auto iterator = std::find(robot_names.begin(), robot_names.end(), name);
  if (iterator == robot_names.end()) throw std::invalid_argument("Unknown command probe robot");
  const auto index = static_cast<std::size_t>(iterator - robot_names.begin());
  Peer peer(test_domain());
  RosOptions options; options.robot_name = name; options.domain_id = test_domain(); options.command_timeout_s = .15;
  RosBridge bridge(options, extrinsics);
  auto global = peer.node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> pubs;
  for (const auto& robot : robot_names)
    pubs.push_back(peer.node->create_publisher<geometry_msgs::msg::Twist>(robot_command_topic(robot), 10));
  peer.until([&] { return pubs[index]->get_subscription_count() == 1 && bridge.command_publisher_count() == 1; },
             bridge, "isolated command discovery " + name, 5., pubs[index].get());
  std::cerr << "DDS ready: " << peer.discovery(bridge, pubs[index].get()) << std::endl;
  require(bridge.command_topic() == "/" + name + "/cmd_vel", "namespaced command topic");
  require(bridge.node_namespace() == "/" + name, "node namespace");
  const auto& spec = robot_spec(name);
  const bool lateral = spec.allows_lateral;
  auto send = [&](Vec3 command, Vec3 expected) {
    geometry_msgs::msg::Twist message;
    message.linear.x = command[0]; message.linear.y = command[1]; message.angular.z = command[2];
    pubs[index]->publish(message);
    peer.until([&] { return bridge.command() == expected; }, bridge, "command delivery " + name, .1, pubs[index].get());
  };
  send({.3, lateral ? .2 : 0., .2}, {.3, lateral ? .2 : 0., .2});
  const Vec3 own_command = bridge.command();
  // One bridge per simulator process is the production contract. All other
  // robot publishers remain real DDS endpoints: exact graph/match counts and
  // attempted delivery prove that other namespaces cannot command this robot.
  for (std::size_t other = 0; other < robot_names.size(); ++other) if (other != index) {
    require(pubs[other]->get_subscription_count() == 0 &&
            peer.node->get_subscriptions_info_by_topic(robot_command_topic(robot_names[other])).empty(),
            "other namespace has no matching command receiver");
    geometry_msgs::msg::Twist foreign; foreign.linear.x = .9; foreign.angular.z = -.4;
    pubs[other]->publish(foreign); bridge.spin_once(.002);
    require(bridge.command() == own_command, "foreign namespace cannot overwrite actual motion");
  }
  send({std::numeric_limits<double>::quiet_NaN(), 0., 0.}, {});
  send({-.3, lateral ? -.1 : 0., .2}, {-.3, lateral ? -.1 : 0., .2});
  if (!lateral) {
    send({.3, 1.e-12, .2}, {});
    send({.3, 0., .2}, {.3, 0., .2});
  }
  if (!spec.allows_in_place_turn) {
    for (double vx : {0., 1.e-12, -1.e-12, 1.e-9, -1.e-9}) {
      send({vx, 0., .2}, {});
      send({-.3, 0., .2}, {-.3, 0., .2});
    }
  }
  require(global->get_subscription_count() == 0, "global command must have no receivers");
  const auto old_command = bridge.command();
  geometry_msgs::msg::Twist global_command; global_command.linear.x = .9;
  global->publish(global_command); bridge.spin_once();
  require(bridge.command() == old_command, "global command must not overwrite namespaced motion");
  std::this_thread::sleep_for(160ms);
  require(bridge.command() == Vec3{}, "wall-clock deadman expires while simulation time is paused");
  bridge.close(); bridge.close();
}
}  // namespace

int main(int argc, char** argv) {
  try {
    ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
    const bool probe = argc == 3 && std::string(argv[1]) == "--command-probe";
    const std::string directory = argc > 1 && !probe ? argv[1] : "";
    if (!directory.empty()) std::filesystem::create_directories(directory);
    for (const std::string name : {"", "/go2", "2go", "go-2", "go2/left"})
      rejects([&] { robot_command_topic(name); }, "invalid robot name");
    require(robot_command_topic("Xian_robot_2") == "/Xian_robot_2/cmd_vel", "generic legal ROS name");
    Extrinsics extrinsics;
    extrinsics.lidar.position = {.1, 0., .2}; extrinsics.imu.position = {.111, .02329, .15588};
    extrinsics.camera.position = {.2, 0., .3}; extrinsics.camera.quaternion = {.5, -.5, .5, -.5};
    if (probe) {
      command_probe(argv[2], extrinsics);
      std::cout << "{\"passed\":true,\"checks\":" << checks << ",\"domain_id\":" << test_domain()
                << ",\"robot\":\"" << argv[2] << "\"}" << std::endl;
      return 0;
    }
    RosOptions options; options.robot_name = "go2w"; options.domain_id = test_domain();
    {
      RosOptions bad = options; bad.command_timeout_s = 0.;
      rejects([&] { RosBridge bridge(bad, extrinsics); }, "reject zero deadman");
      Extrinsics bad_pose = extrinsics; bad_pose.lidar.quaternion = {};
      rejects([&] { RosBridge bridge(options, bad_pose); }, "reject nonunit WXYZ quaternion");
    }
    {
    Peer peer(test_domain());
    RosBridge bridge(options, extrinsics);
    std::vector<livox_ros_driver2::msg::CustomMsg::ConstSharedPtr> scans;
    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
    rosgraph_msgs::msg::Clock::ConstSharedPtr clock;
    sensor_msgs::msg::Imu::ConstSharedPtr imu, imu_si;
    nav_msgs::msg::Odometry::ConstSharedPtr odom;
    tf2_msgs::msg::TFMessage::ConstSharedPtr tf, static_tf;
    std::map<std::string, sensor_msgs::msg::Image::ConstSharedPtr> images;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
    std_msgs::msg::String::ConstSharedPtr status;
    peer.subscribe<rosgraph_msgs::msg::Clock>("/clock", [&](auto msg) { clock = msg; }, rclcpp::QoS(1).best_effort());
    peer.subscribe<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", [&](auto msg) { scans.push_back(msg); });
    peer.subscribe<sensor_msgs::msg::PointCloud2>("/livox/points", [&](auto msg) { clouds.push_back(msg); });
    peer.subscribe<sensor_msgs::msg::Imu>("/livox/imu", [&](auto msg) { imu = msg; });
    peer.subscribe<sensor_msgs::msg::Imu>("/livox/imu_si", [&](auto msg) { imu_si = msg; });
    peer.subscribe<nav_msgs::msg::Odometry>("/odom", [&](auto msg) { odom = msg; });
    peer.subscribe<tf2_msgs::msg::TFMessage>("/tf", [&](auto msg) { tf = msg; });
    peer.subscribe<tf2_msgs::msg::TFMessage>("/tf_static", [&](auto msg) { static_tf = msg; }, rclcpp::QoS(1).transient_local());
    for (const auto& topic : {"/camera/image_raw", "/camera/depth/image_raw", "/camera/panorama/image_raw", "/camera/panorama/depth/image_raw"})
      peer.subscribe<sensor_msgs::msg::Image>(topic, [&, topic](auto msg) { images[topic] = msg; }, rclcpp::QoS(2));
    peer.subscribe<sensor_msgs::msg::CameraInfo>("/camera/camera_info", [&](auto msg) { camera_info = msg; }, rclcpp::QoS(2));
    peer.subscribe<std_msgs::msg::String>("/full_run/status", [&](auto msg) { status = msg; }, rclcpp::QoS(1).transient_local());
    peer.until([&] {
      for (const auto& subscription : peer.subscriptions)
        if (bridge.subscriber_count(subscription->get_topic_name()) != 1 ||
            peer.node->count_publishers(subscription->get_topic_name()) != 1) return false;
      return static_cast<bool>(static_tf);
    }, bridge, "all publisher/subscriber discovery and late-join static TF");
    auto qos_check = [&](const std::string& topic, std::size_t depth, rmw_qos_reliability_policy_t reliability,
                         rmw_qos_durability_policy_t durability) {
      const auto endpoints = peer.node->get_publishers_info_by_topic(topic);
      require(endpoints.size() == 1, "single publisher QoS " + topic);
      const auto qos = endpoints.front().qos_profile().get_rmw_qos_profile();
      // Humble Fast DDS discovery omits history/depth (reports depth=0).
      // Enforce any reported depth; reliability/durability are always exposed.
      if (qos.depth == 0)
        std::cerr << "DDS discovery does not report history depth for " << topic << '\n';
      require((qos.depth == 0 || qos.depth == depth) && qos.reliability == reliability && qos.durability == durability,
              "reported QoS " + topic);
    };
    qos_check("/clock", 1, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_check("/livox/lidar", 10, RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_check("/camera/image_raw", 2, RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_check("/tf", 100, RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_check("/tf_static", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    constexpr std::uint64_t stamp = 7999999500ULL;
    LidarScan initial; initial.stamp_ns = stamp; initial.rays_cast = 2;
    initial.xyz = {{1.25f, -2.5f, 3.75f}, {-4.f, 5.f, -6.f}};
    initial.offsets_ns = {0, 900}; initial.lines = {0, 3};
    const ImuSample sample{stamp, {.1, .2, .3}, {0., 0., 9.80665}};
    const Pose pose{{1., 2., .3}, {std::sqrt(.5), 0., 0., std::sqrt(.5)}};
    CameraFrame camera; camera.width = 3; camera.height = 2;
    for (std::uint8_t value = 0; value < 18; ++value) camera.rgb.push_back(value);
    camera.depth = {std::numeric_limits<float>::infinity(), 2.5f, 3.f, 4.f, 5.f, 6.f};
    bridge.publish_clock(stamp); bridge.publish_scan(initial); bridge.publish_imu(sample);
    bridge.publish_odometry(stamp, pose, {.4, 0., 0.}, {0., 0., .2});
    bridge.publish_camera(stamp, camera, false); camera.radial_depth = true;
    bridge.publish_camera(stamp, camera, true); bridge.publish_status("{\"ready\":true}");
    camera.rgb.assign(18, 255); initial.xyz[0][0] = 999.f;  // caller buffers may be reused now
    peer.until([&] { return clock && scans.size() == 1 && clouds.size() == 1 && imu && imu_si && odom &&
                            tf && static_tf && images.size() == 4 && camera_info && status; }, bridge, "all sensor messages delivered");
    require(ns(clock->clock) == stamp, "clock uses complete nanoseconds");
    require(scans[0]->timebase == stamp && ns(scans[0]->header.stamp) == stamp && scans[0]->point_num == 2,
            "CustomMsg timestamp/count");
    require(scans[0]->points[0].x == 1.25f && scans[0]->points[1].offset_time == 900 &&
            scans[0]->points[1].line == 3 && scans[0]->points[1].tag == 0 && scans[0]->points[1].reflectivity == 100,
            "CustomMsg exact values and owned storage");
    require(clouds[0]->point_step == 26 && clouds[0]->row_step == 52 && clouds[0]->fields.size() == 7,
            "PointCloud2 packed size");
    require(clouds[0]->fields.back().offset == 18 && clouds[0]->fields.back().datatype == 8 &&
            value_at<double>(clouds[0]->data, 44) == 8000000400., "absolute per-point timestamp crosses second");
    require(imu->header.frame_id == "livox_frame" && imu->linear_acceleration.z == 1. &&
            imu->angular_velocity.x == static_cast<double>(static_cast<float>(.1)), "raw IMU units/float32 wire precision");
    require(imu->orientation.w == 1. && imu->orientation.x == 0. && imu->orientation_covariance[0] == 0.,
            "raw IMU unavailable orientation identity placeholder");
    require(imu_si->header.frame_id == "imu_frame" && imu_si->linear_acceleration.z == 9.80665 &&
            imu_si->angular_velocity.x == .1 && imu_si->orientation.w == 0. && imu_si->orientation_covariance[0] == -1.,
            "SI IMU precise units/unavailable orientation");
    require(odom->header.frame_id == "odom" && odom->child_frame_id == "base_link" &&
            odom->pose.pose.orientation.w == std::sqrt(.5) && odom->pose.pose.orientation.x == 0. &&
            odom->pose.pose.orientation.z == std::sqrt(.5), "odom WXYZ to XYZW conversion");
    require(tf->transforms[0].child_frame_id == "base_link" && ns(tf->transforms[0].header.stamp) == stamp,
            "dynamic TF time and frames");
    require(static_tf->transforms.size() == 3 && static_tf->transforms[2].transform.rotation.x == -.5 &&
            static_tf->transforms[2].transform.rotation.w == .5, "measured camera optical extrinsics conversion");
    require(images["/camera/image_raw"]->data.front() == 0 && images["/camera/image_raw"]->encoding == "rgb8",
            "RGB payload owns input bytes");
    require(std::isinf(value_at<float>(images["/camera/depth/image_raw"]->data, 0)) &&
            images["/camera/depth/image_raw"]->encoding == "32FC1", "no-hit depth infinity survives DDS");
    require(camera_info->width == 3 && camera_info->height == 2 &&
            std::abs(camera_info->k[0] - std::sqrt(3.)) < 1.e-14, "perspective pinhole calibration");
    require(peer.node->count_publishers("/camera/panorama/camera_info") == 0, "panorama has no fake pinhole calibration");
    require(status->data == "{\"ready\":true}", "fullcourse status transport");
    fixture(directory, "clock", *clock); fixture(directory, "custom", *scans[0]); fixture(directory, "points", *clouds[0]);
    fixture(directory, "imu", *imu); fixture(directory, "imu_si", *imu_si); fixture(directory, "odom", *odom);
    fixture(directory, "tf", *tf); fixture(directory, "tf_static", *static_tf);
    fixture(directory, "rgb", *images["/camera/image_raw"]); fixture(directory, "depth", *images["/camera/depth/image_raw"]);
    fixture(directory, "panorama_rgb", *images["/camera/panorama/image_raw"]);
    fixture(directory, "panorama_depth", *images["/camera/panorama/depth/image_raw"]);
    fixture(directory, "camera_info", *camera_info); fixture(directory, "status", *status);
    // Expected IEEE binary32 bits come from Python struct.pack('<f'), not
    // the same C++ double->float->double expression that GCC -O3 SLP removed.
    // Every axis differs from the original SI double, including the SIMD pair.
    const std::array<std::array<double, 6>, 3> precision_inputs{{
      {{.7676144758470572, -.18092352295455252, .8759101923113392,
        .8846144758470572, -3.428200333474639, 10.474264475847058}},
      {{.10000000000000003, -.20000000000000007, .3000000000000001,
        1.234567891, -8.76543219, 9.999999999}},
      {{-1.7976931348623157e-7, 2.718281828459045, 3.141592653589793,
        -.000021736529, 12.3456789123, -9.8765432123}}
    }};
    const std::array<std::array<std::uint32_t, 6>, 3> expected_bits{{
      {{0x3f448262u, 0xbe394404u, 0x3f603ba6u, 0x3db8bdb3u, 0xbeb2fc0au, 0x3f88b6c5u}},
      {{0x3dcccccdu, 0xbe4ccccdu, 0x3e99999au, 0x3e00e98bu, 0xbf64d1bdu, 0x3f828610u}},
      {{0xb441069du, 0x402df854u, 0x40490fdbu, 0xb614bf56u, 0x3fa123edu, 0xbf80e98bu}}
    }};
    for (std::size_t index = 0; index < precision_inputs.size(); ++index) {
      const auto& values = precision_inputs[index];
      ImuSample dynamic_sample{stamp + (index + 1) * 5000000ULL,
          {values[0], values[1], values[2]}, {values[3], values[4], values[5]}};
      bridge.publish_imu(dynamic_sample);
      peer.until([&] { return ns(imu->header.stamp) == dynamic_sample.stamp_ns &&
                             ns(imu_si->header.stamp) == dynamic_sample.stamp_ns; }, bridge, "precision IMU pair received");
      const std::array<double, 6> raw_values{imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z,
          imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z};
      const std::array<double, 6> si_values{imu_si->angular_velocity.x, imu_si->angular_velocity.y, imu_si->angular_velocity.z,
          imu_si->linear_acceleration.x, imu_si->linear_acceleration.y, imu_si->linear_acceleration.z};
      for (std::size_t axis = 0; axis < 6; ++axis) {
        float expected; std::memcpy(&expected, &expected_bits[index][axis], sizeof(expected));
        require(raw_values[axis] == static_cast<double>(expected), "each raw axis equals independent binary32 wire value");
        require(si_values[axis] == values[axis], "SI axes preserve original sampled doubles");
      }
    }
    LidarScan bad = initial; bad.offsets_ns.pop_back();
    rejects([&] { bridge.publish_scan(bad); }, "bad array lengths rejected");
    bad = initial; bad.xyz[0][0] = std::numeric_limits<float>::infinity();
    rejects([&] { bridge.publish_scan(bad); }, "nonfinite point rejected");
    rejects([&] { bridge.publish_clock(std::numeric_limits<std::uint64_t>::max()); }, "out-of-range ROS timestamp rejected");
    rejects([&] { bridge.publish_camera(stamp, camera, false); }, "depth convention mismatch rejected");
    std::map<std::uint64_t, LidarScan> expected;
    for (auto count : {4, 1, 0, 3}) {
      const auto timestamp = stamp + 100000000ULL * (expected.size() + 1);
      auto frame = scan(timestamp, count, static_cast<float>(count));
      expected.emplace(timestamp, frame); bridge.publish_scan(frame);
    }
    peer.until([&] { return scans.size() == 5 && clouds.size() == 5; }, bridge, "all four queued scans survive reuse");
    std::vector<std::thread> threads;
    for (int index = 0; index < 3; ++index) {
      auto frame = scan(stamp + 1000000000ULL + index, 1000 + index, static_cast<float>(index + 10));
      expected.emplace(frame.stamp_ns, frame);
      threads.emplace_back([&, frame] { bridge.publish_scan(frame); });
    }
    for (auto& thread : threads) thread.join();
    peer.until([&] { return scans.size() == 8 && clouds.size() == 8; }, bridge, "concurrent queued scans delivered");
    for (std::size_t index = 1; index < scans.size(); ++index) {
      const auto& message = *scans[index]; const auto& expected_scan = expected.at(message.timebase);
      require(message.point_num == expected_scan.xyz.size() && clouds[index]->width == message.point_num &&
              ns(clouds[index]->header.stamp) == message.timebase, "scan/cloud pairs retain same ordering and counts");
      for (std::size_t point = 0; point < message.points.size(); ++point)
        require(message.points[point].x == expected_scan.xyz[point][0] &&
                message.points[point].offset_time == expected_scan.offsets_ns[point], "queued/concurrent point data unchanged");
    }
    bridge.close(); bridge.close();
    }  // Destroy the sensor peer/subscriptions before the command-only phase.
    std::cout << "{\"passed\":true,\"checks\":" << checks << ",\"fixture_messages\":14,\"domain_id\":" << test_domain() << "}" << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "ROS transport failed after " << checks << " checks: " << error.what() << std::endl;
    return 1;
  }
}
