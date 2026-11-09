#pragma once
#include "scene.hpp"
#include <map>
#include <tinyxml2.h>
namespace simple_rbt {
inline constexpr std::size_t max_scene_boxes = 2048;
Json opening_boxes(const Json &object);
Json scene_boxes(const Json &scene);
struct PreparedUserScene {
  int version;
  Json geometry;
  std::map<std::string, double> floor_heights;
  ScenePlacement spawn;
};
PreparedUserScene prepare_user_scene(const Json &, const std::string &spawn);
void append_user_scene(tinyxml2::XMLDocument &, const PreparedUserScene &);
void apply_scene_spawn(mjModel *, mjData *, const ScenePlacement &spawn);
} // namespace simple_rbt
