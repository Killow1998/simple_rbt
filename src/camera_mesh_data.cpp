#include "camera_mesh_data.hpp"
#include "meshoptimizer/meshoptimizer.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
namespace simple_rbt {
namespace {
template <size_t N> using Key = std::array<unsigned, N>;
template <size_t N> Key<N> key(const float *value) {
  Key<N> result;
  static_assert(sizeof(unsigned) == sizeof(float));
  std::memcpy(result.data(), value, sizeof(result));
  return result;
}
// Match MuJoCo 3.15.0 render_context.c mesh upload arithmetic, including sharp
// untextured face normals. See third_party/mujoco_classic for source/license.
std::array<float, 3> face_normal(const float *a, const float *b,
                                 const float *c) {
  float ab[3], ac[3];
  for (int k = 0; k < 3; k++) {
    ab[k] = b[k] - a[k];
    ac[k] = c[k] - a[k];
  }
  std::array<float, 3> n;
  for (int k = 0; k < 3; k++)
    n[k] =
        ab[(k + 1) % 3] * ac[(k + 2) % 3] - ab[(k + 2) % 3] * ac[(k + 1) % 3];
  float length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  if (length < 1e-10f)
    return {0, 0, 1};
  float inverse = 1.f / length;
  for (auto &v : n)
    v *= inverse;
  return n;
}
} // namespace
CameraMeshData camera_mesh_data(const mjModel *m, int mesh) {
  CameraMeshData data;
  // Small meshes gain nothing from LOD; retain their native display lists.
  if (m->mesh_facenum[mesh] < 256)
    return data;
  int va = m->mesh_vertadr[mesh], na = m->mesh_normaladr[mesh];
  int ta = m->mesh_texcoordadr[mesh];
  data.textured = ta >= 0;
  std::map<Key<8>, unsigned> vertices;
  for (int f = m->mesh_faceadr[mesh];
       f < m->mesh_faceadr[mesh] + m->mesh_facenum[mesh]; f++) {
    auto position = [&](int i) {
      return m->mesh_vert + 3 * (va + m->mesh_face[3 * f + i]);
    };
    auto n = face_normal(position(0), position(1), position(2));
    for (int i = 0; i < 3; i++) {
      CameraMeshData::Vertex v{};
      const float *normal =
          m->mesh_normal + 3 * (na + m->mesh_facenormal[3 * f + i]);
      if (ta < 0 && normal[0] * n[0] + normal[1] * n[1] + normal[2] * n[2] < .8)
        normal = n.data();
      std::copy_n(position(i), 3, v.begin());
      std::copy_n(normal, 3, v.begin() + 3);
      if (ta >= 0)
        std::copy_n(m->mesh_texcoord +
                        2 * (ta + m->mesh_facetexcoord[3 * f + i]),
                    2, v.begin() + 6);
      auto [it, added] =
          vertices.emplace(key<8>(v.data()), data.vertices.size());
      if (added)
        data.vertices.push_back(v);
      data.indices.push_back(it->second);
      data.last = v;
    }
  }
  std::map<Key<3>, unsigned> positions;
  for (auto i : data.indices) {
    const auto &v = data.vertices[i];
    auto [it, added] =
        positions.emplace(key<3>(v.data()), data.positions.size());
    if (added)
      data.positions.push_back({v[0], v[1], v[2]});
    data.depth.push_back(
        it->second); // Every original triangle, in original order.
  }
  // Stop at the error limit, without an arbitrary triangle-count floor.
  data.shadow.resize(data.depth.size());
  data.shadow.resize(meshopt_simplify(
      data.shadow.data(), data.depth.data(), data.depth.size(),
      data.positions[0].data(), data.positions.size(),
      sizeof(data.positions[0]), 0, .001f, meshopt_SimplifyErrorAbsolute));
  const float weights[] = {.01f, .01f, .01f, 1.f, 1.f};
  for (float tolerance :
       {.000025f, .00005f, .0001f, .0002f, .0004f, .0008f, .0016f, .0032f}) {
    MeshLevel level;
    level.indices.resize(data.indices.size());
    level.indices.resize(meshopt_simplifyWithAttributes(
        level.indices.data(), data.indices.data(), data.indices.size(),
        data.vertices[0].data(), data.vertices.size(), sizeof(data.vertices[0]),
        data.vertices[0].data() + 3, sizeof(data.vertices[0]), weights,
        ta >= 0 ? 5 : 3, nullptr, 0, tolerance,
        meshopt_SimplifyErrorAbsolute |
            (ta < 0 ? meshopt_SimplifyPermissive : 0),
        &level.error));
    if (level.indices.size() < data.indices.size())
      data.levels.push_back(std::move(level));
  }
  return data;
}
} // namespace simple_rbt
