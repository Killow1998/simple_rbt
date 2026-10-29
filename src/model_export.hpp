#pragma once
#include <mujoco/mujoco.h>
#include <string>
namespace simple_rbt {
std::string export_model(const std::string &, const mjModel *, bool flight);
}
