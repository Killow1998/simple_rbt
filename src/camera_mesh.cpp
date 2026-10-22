#define GL_GLEXT_PROTOTYPES
#include "camera_mesh.hpp"
#include "camera_mesh_data.hpp"
#include <GL/gl.h>
#include <GL/glext.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace simple_rbt {
namespace {
struct Buffer {
  GLuint id = 0;
  GLsizei count = 0;
  Buffer() = default;
  Buffer(const Buffer &) = delete;
  Buffer &operator=(const Buffer &) = delete;
  Buffer(Buffer &&b) noexcept : id(b.id), count(b.count) { b.id = 0; }
  ~Buffer() {
    if (id)
      glDeleteBuffers(1, &id);
  }
  template <class T> void upload(GLenum target, const std::vector<T> &values) {
    glGenBuffers(1, &id);
    glBindBuffer(target, id);
    glBufferData(target, values.size() * sizeof(T), values.data(),
                 GL_STATIC_DRAW);
    GLint64 allocated = 0;
    glGetBufferParameteri64v(target, GL_BUFFER_SIZE, &allocated);
    glBindBuffer(target, 0);
    if (allocated != static_cast<GLint64>(values.size() * sizeof(T)))
      throw std::runtime_error("Cannot allocate software camera mesh buffer");
    count = values.size();
  }
};
struct Level {
  Buffer index;
  float error;
};
struct Mesh {
  Buffer vertices, index, positions, depth, shadow;
  std::vector<Level> levels;
  CameraMeshData::Vertex last;
  bool textured = false;
};
float error_budget(const mjvGeom *g, float near) {
  GLint vp[4];
  float mv[16], p[16];
  glGetIntegerv(GL_VIEWPORT, vp);
  glGetFloatv(GL_MODELVIEW_MATRIX, mv);
  glGetFloatv(GL_PROJECTION_MATRIX, p);
  // Perspective only. Conservative projected estimate over the visible frustum,
  // including off-axis projection; 10% margin. A metric estimate is not an
  // exact Hausdorff bound or a bound on lighting/texture pixel differences.
  if (p[15] != 0 || p[0] == 0 || p[5] == 0)
    return 0;
  float z = -mv[14];
  for (int k = 0; k < 3; k++)
    z -= std::abs(mv[4 * k + 2]) * g->size[k];
  float x = (1 + std::abs(p[8])) / std::abs(p[0]);
  float y = (1 + std::abs(p[9])) / std::abs(p[5]);
  float focal = .5f * std::max(vp[2] * std::abs(p[0]), vp[3] * std::abs(p[5]));
  float scale = std::sqrt(mv[0] * mv[0] + mv[1] * mv[1] + mv[2] * mv[2]);
  return std::min(.25f, .5f / (1.1f * std::sqrt(1 + x * x + y * y))) *
         std::max(z, near) / (focal * scale);
}
} // namespace
struct CameraMeshes::Impl {
  std::vector<Mesh> meshes;
};
CameraMeshes::CameraMeshes(const mjModel *model)
    : p_(std::make_unique<Impl>()) {
  if (!model)
    return;
  p_->meshes.reserve(model->nmesh);
  for (int i = 0; i < model->nmesh; i++) {
    auto data = camera_mesh_data(model, i);
    p_->meshes.emplace_back();
    auto &b = p_->meshes.back();
    if (data.indices.empty())
      continue;
    b.vertices.upload(GL_ARRAY_BUFFER, data.vertices);
    b.index.upload(GL_ELEMENT_ARRAY_BUFFER, data.indices);
    b.positions.upload(GL_ARRAY_BUFFER, data.positions);
    b.depth.upload(GL_ELEMENT_ARRAY_BUFFER, data.depth);
    b.shadow.upload(GL_ELEMENT_ARRAY_BUFFER, data.shadow);
    b.last = data.last;
    b.textured = data.textured;
    for (auto &level : data.levels) {
      b.levels.emplace_back();
      b.levels.back().index.upload(GL_ELEMENT_ARRAY_BUFFER, level.indices);
      b.levels.back().error = level.error;
    }
  }
}
CameraMeshes::~CameraMeshes() = default;
bool CameraMeshes::contains(const mjvGeom *g) const {
  return g->type == mjGEOM_MESH && g->dataid >= 0 && !(g->dataid % 2) &&
         size_t(g->dataid / 2) < p_->meshes.size() &&
         p_->meshes[g->dataid / 2].vertices.id;
}
bool CameraMeshes::draw(const mjvGeom *g, int mode, float near) const {
  if (!contains(g) || (mode != 2 && g->transparent))
    return false;
  const auto &b = p_->meshes[g->dataid / 2];
  const Buffer *indices = &b.index;
  if (mode >= 2) {
    glBindBuffer(GL_ARRAY_BUFFER, b.positions.id);
    glVertexPointer(3, GL_FLOAT, 0, nullptr);
    indices = mode == 2 ? &b.shadow : &b.depth;
  } else {
    float budget = error_budget(g, near);
    for (const auto &level : b.levels)
      if (level.error <= budget && level.index.count < indices->count)
        indices = &level.index;
    glBindBuffer(GL_ARRAY_BUFFER, b.vertices.id);
    glVertexPointer(3, GL_FLOAT, 8 * sizeof(float), nullptr);
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, 8 * sizeof(float),
                    reinterpret_cast<void *>(3 * sizeof(float)));
    if (b.textured) {
      glEnableClientState(GL_TEXTURE_COORD_ARRAY);
      glTexCoordPointer(2, GL_FLOAT, 8 * sizeof(float),
                        reinterpret_cast<void *>(6 * sizeof(float)));
    }
  }
  glEnableClientState(GL_VERTEX_ARRAY);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices->id);
  glDrawElements(GL_TRIANGLES, indices->count, GL_UNSIGNED_INT, nullptr);
  glDisableClientState(GL_VERTEX_ARRAY);
  if (mode < 2) {
    glDisableClientState(GL_NORMAL_ARRAY);
    if (b.textured)
      glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  }
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  // Display lists leave their last normal/UV current; array draws do not.
  glNormal3fv(b.last.data() + 3);
  if (b.textured)
    glTexCoord2fv(b.last.data() + 6);
  return true;
}
} // namespace simple_rbt
