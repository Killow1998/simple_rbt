#pragma once
#include "simple_rbt/types.hpp"
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

namespace simple_rbt {
class PoseHistory;

// Owns one run's guarded directory and files; it never steps or renders.
class ArtifactWriter {
public:
  ArtifactWriter(const std::filesystem::path &project_root,
                 const std::filesystem::path &requested_output,
                 bool summary_only, bool precreated);
  const std::filesystem::path &output() const noexcept { return output_; }
  void model(const std::string &xml);
  void open_streams();
  void imu(const ImuSample &sample);
  void trajectory(std::uint64_t stamp, const Pose &, const Vec3 &linear,
                  const Vec3 &angular);
  void close_streams();
  void camera(std::uint64_t stamp, const OpticalPose &, const OpticalPose &depth, const CameraPair &);
  void lidar(const LidarScan &, const Pose &reference, const PoseHistory &,
             bool rolling, const nlohmann::json &scan_model);
  void summary(const nlohmann::json &value);

private:
  std::filesystem::path output_;
  bool summary_only_;
  std::ofstream trajectory_, imu_csv_;
};
} // namespace simple_rbt
