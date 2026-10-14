#include "simple_rbt/ros_bridge.hpp"
#include "simple_rbt/robots.hpp"
#include <regex>
#include <stdexcept>
namespace simple_rbt {
std::string robot_command_topic(const std::string& robot_name) {
  static const std::regex name("[A-Za-z_][A-Za-z0-9_]*");
  if (!std::regex_match(robot_name, name))
    throw std::invalid_argument("robot_name must be one ROS name and cannot begin with a digit");
  return "/" + robot_name + "/cmd_vel";
}
bool robot_command_supported(const std::string& robot_name, const Vec3& command) {
  return supported_command(robot_name, command);
}
} // namespace simple_rbt
