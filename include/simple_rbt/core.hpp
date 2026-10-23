#pragma once
#include "simple_rbt/robots.hpp"
#include "simple_rbt/types.hpp"
#include <filesystem>
#include <memory>
#include <optional>
#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>
#include <string>

namespace simple_rbt {
using Json = nlohmann::json;
struct SimulationOptions {
  Json scene;
  std::filesystem::path project_root;
  std::string robot = "go2", controller = "auto", terrain = "flat",
              terrain_level = "standard";
  std::string control_mode = "tracked", go2_torque_mode = "configured",
              go2_startup = "crouched";
  double difficulty = 1., friction = -1.;
  std::uint64_t seed = 0;
  std::optional<Pose> mid360_mount, camera_mount, external_camera_mount;
  bool external_camera_enabled = false;
  std::string camera_model = "d435";
  Pose camera_color_offset{{0.,.015,0.}}, camera_depth_offset{},
       camera_imu_offset{{-.01174,-.00552,.0051}};
};
class Simulation {
public:
  explicit Simulation(const SimulationOptions &);
  ~Simulation();
  Simulation(const Simulation &) = delete;
  Simulation &operator=(const Simulation &) = delete;
  void set_command(const Vec3 &);
  void set_joints(const JointCommand &);
  void hold_joints();
  JointState joints() const;
  Json joint_controls() const;
  Pose relative_pose(const std::string &site,
                     const std::string &parent = "base_origin") const;
  Pose mount_parent(const std::string &module) const;
  void step();
  // Configuration only; the controller body IMU is never modified.
  void set_mount(const std::string& module, const Pose&);
  mjModel *model() const;
  mjData *data() const;
  std::uint64_t stamp_ns() const;
  Pose pose(const std::string &site = "base_origin") const;
  Pose extrinsics(const std::string &) const;
  OpticalPose camera_pose(const std::string &site = "camera_color") const;
  bool has_camera_imu() const;
  ImuSample camera_imu() const;
  Vec3 linear_velocity() const;
  Vec3 angular_velocity() const;
  // Specific force and angular rate averaged over the last native physics
  // interval, in the current sensor frame. At time zero this is the instantaneous native sensor.
  ImuSample imu() const;
  const Vec3 &command() const;
  const Vec3 &command_limits() const;
  bool startup_complete() const;
  std::string startup_phase() const;
  std::uint64_t startup_steps() const;
  const std::string &controller_name() const;
  const Json &terrain_metadata() const;
  Json deployment_metadata() const;
  Json tracker_metadata() const;
  std::string xml() const;

private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};
std::string sha256_file(const std::filesystem::path &);
} // namespace simple_rbt
