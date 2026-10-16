#pragma once
#include "simple_rbt/core.hpp"
#include <tinyxml2.h>
namespace simple_rbt {
void add_sensor_devices(tinyxml2::XMLDocument &, const SimulationOptions &);
void move_sensor_device(mjModel *, const std::string &, const Pose &,
                        const SimulationOptions &);
Extrinsics sensor_extrinsics(Simulation &, bool camera);
} // namespace simple_rbt
