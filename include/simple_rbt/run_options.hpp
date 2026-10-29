#pragma once
#include "simple_rbt/core.hpp"
#include <filesystem>
namespace simple_rbt {
struct RunOptions {
  SimulationOptions simulation;
  std::string backend = "cpu", sensors = "lidar", lidar_motion = "rolling";
  double duration = 10., settle = 1., camera_hz = 1.;
  std::size_t rays = 0; // Zero selects one full 100 ms scan of the device.
  Vec3 simulation_command{};
  bool realtime = false, viewer = false, summary_only = false, indefinite = false;
  bool mid360_enabled = true, camera_enabled = true, depth_enabled = true, panorama_enabled = true;
  bool output_precreated =
      false; // Only the validator's newly allocated folder.
  std::filesystem::path output;
};
}
