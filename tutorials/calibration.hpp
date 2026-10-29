#pragma once
#include "simple_rbt/ipc.hpp"
namespace simple_rbt::tutorial {
using Json = ipc::Json;
Json frontend_parameters(Json base, const Extrinsics &,
                         const std::string &algorithm,
                         const std::string &lidar_model);
Json camera_parameters(int width, int height, double fov_y_deg);
} // namespace simple_rbt::tutorial
