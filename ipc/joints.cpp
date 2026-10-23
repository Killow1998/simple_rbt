#include "simple_rbt/ipc.hpp"
#include <cmath>
#include <set>
namespace simple_rbt::ipc {
Packet encode_joint_command(const simple_rbt::JointCommand &c) {
  return json_packet(
      Kind::JointCommand,
      {{"name", c.name}, {"position", c.position}, {"hold", c.hold}});
}
simple_rbt::JointCommand decode_joint_command(const Packet &p) {
  const auto j = decode_json(p);
  simple_rbt::JointCommand c{j.value("name", std::vector<std::string>{}),
                             j.value("position", std::vector<double>{}),
                             j.value("hold", false)};
  if (c.name.size() != c.position.size() || c.name.size() > 64 ||
      (c.name.empty() && !c.hold))
    throw std::invalid_argument(
        "Joint command must contain matching names and positions");
  std::set<std::string> names;
  for (std::size_t i = 0; i < c.name.size(); ++i)
    if (c.name[i].empty() || c.name[i].size() > 128 ||
        !names.insert(c.name[i]).second || !std::isfinite(c.position[i]))
      throw std::invalid_argument("Invalid or repeated joint command");
  return c;
}
Packet encode_joints(std::uint64_t stamp, const simple_rbt::JointState &s) {
  return json_packet(Kind::JointState,
                     {{"name", s.name},
                      {"position", s.position},
                      {"velocity", s.velocity},
                      {"effort", s.effort},
                      {"tool_pose",
                       {{"position", s.tool_pose.position},
                        {"quaternion", s.tool_pose.quaternion}}}},
                     stamp);
}
simple_rbt::JointState decode_joints(const Packet &p) {
  const auto j = decode_json(p);
  simple_rbt::JointState s{j.at("name").get<std::vector<std::string>>(),
                           j.at("position").get<std::vector<double>>(),
                           j.at("velocity").get<std::vector<double>>(),
                           j.at("effort").get<std::vector<double>>(),
                           {}};
  if (s.name.size() > 64 || s.name.size() != s.position.size() ||
      s.name.size() != s.velocity.size() || s.name.size() != s.effort.size())
    throw std::invalid_argument("Inconsistent joint state arrays");
  s.tool_pose.position = j.at("tool_pose").at("position").get<Vec3>();
  s.tool_pose.quaternion = j.at("tool_pose").at("quaternion").get<Quat>();
  return s;
}
} // namespace simple_rbt::ipc
