#pragma once
#include <array>
#include <mujoco/mujoco.h>
#include <vector>
namespace simple_rbt {
struct MeshLevel {
  std::vector<unsigned> indices;
  float error = 0; // meshoptimizer's absolute object-space error estimate
};
struct CameraMeshData {
  using Vertex = std::array<float, 8>; // position, normal, UV
  std::vector<Vertex> vertices;
  std::vector<unsigned> indices, depth, shadow;
  std::vector<std::array<float, 3>> positions;
  std::vector<MeshLevel> levels;
  Vertex last{};
  bool textured = false;
};
CameraMeshData camera_mesh_data(const mjModel *, int mesh);
} // namespace simple_rbt
