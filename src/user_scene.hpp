#pragma once
#include "simple_rbt/core.hpp"
#include <tinyxml2.h>
namespace simple_rbt {
inline constexpr std::size_t max_scene_boxes = 2048;
Json opening_boxes(const Json &object);
Json scene_boxes(const Json &scene);
void validate_user_scene(const Json &scene);
void append_user_scene(tinyxml2::XMLDocument &, const Json &scene);
void apply_scene_spawn(mjModel *, mjData *, const Json &scene);
} // namespace simple_rbt
