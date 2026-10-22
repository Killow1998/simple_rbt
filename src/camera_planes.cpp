#define GL_GLEXT_PROTOTYPES
#include "camera_planes.hpp"
#include <GL/gl.h>
#include <GL/glext.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace simple_rbt {
namespace {
// Match the finite-plane grid arithmetic in MuJoCo 3.15.0 render_context.c.
// Infinite/default planes retain the native display list. Shadow casting uses
// the complete grid; view rejection only removes wholly invisible strips.
void upload(GLenum target, GLsizeiptr bytes, const void *data, GLenum usage) {
  glBufferData(target, bytes, data, usage);
  GLint64 allocated = 0;
  glGetBufferParameteri64v(target, GL_BUFFER_SIZE, &allocated);
  if (allocated != bytes) {
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    throw std::runtime_error("Cannot allocate software camera plane buffer");
  }
}
struct Plane {
  GLuint vertices = 0, indices = 0;
  std::vector<unsigned> elements, visible;
  struct Tile {
    size_t offset;
    double x, y, hx, hy;
  };
  std::vector<Tile> tiles;
  double last[2]{};
  ~Plane() {
    glDeleteBuffers(1, &vertices);
    glDeleteBuffers(1, &indices);
  }
};
} // namespace
struct CameraPlanes::Impl {
  std::vector<std::unique_ptr<Plane>> planes;
  explicit Impl(const mjModel *m) {
    if (!m)
      return;
    for (int geom = 0; geom < m->ngeom; geom++)
      if (m->geom_type[geom] == mjGEOM_PLANE) {
        planes.push_back(std::make_unique<Plane>());
        auto &p = *planes.back();
        double size[2] = {m->geom_size[3 * geom], m->geom_size[3 * geom + 1]};
        if (size[0] <= 0 || size[1] <= 0)
          continue;
        std::vector<double> grid[2];
        int count[2];
        for (int k = 0; k < 2; k++) {
          double sz2 = std::max(double(m->geom_size[3 * geom + 2]),
                                size[k] / (mjMAXPLANEGRID - 2));
          int n = int(floor(size[k] / std::max(double(mjMINVAL), sz2)));
          n -= n % 2;
          double pad = size[k] - n * sz2;
          n += 2;
          count[k] = n;
          grid[k].resize(n + 1);
          for (int i = 0; i < n; i++) {
            double left =
                i == 0 ? -size[k] : -size[k] + pad + (i - 1) * sz2 * 2;
            double right = left + (i == 0 || i == n - 1 ? pad : sz2 * 2);
            grid[k][i] = left;
            grid[k][i + 1] = std::max(left, right);
          }
        }
        std::vector<float> vertices;
        for (int x = 0; x <= count[0]; x++)
          for (int y = 0; y <= count[1]; y++) {
            vertices.insert(
                vertices.end(),
                {float(grid[0][x]), float(grid[1][y]), 0,
                 float((grid[0][x] + size[0]) / (2 * size[0])),
                 float(1 - (grid[1][y] + size[1]) / (2 * size[1]))});
          }
        // Original primitive order: eight quads per culling group along each
        // row.
        for (int x = 0; x < count[0]; x++)
          for (int y = 0; y < count[1]; y += 8) {
            int end = std::min(y + 8, count[1]);
            p.tiles.push_back({p.elements.size(),
                               (grid[0][x] + grid[0][x + 1]) / 2,
                               (grid[1][y] + grid[1][end]) / 2,
                               (grid[0][x + 1] - grid[0][x]) / 2,
                               (grid[1][end] - grid[1][y]) / 2});
            for (int j = y; j < end; j++) {
              unsigned v = x * (count[1] + 1) + j, n = v + count[1] + 1;
              p.elements.insert(p.elements.end(), {v, n, n + 1, v + 1});
            }
          }
        p.tiles.push_back({p.elements.size(), 0, 0, 0, 0});
        p.last[0] = (grid[0][count[0] - 1] + size[0]) / (2 * size[0]);
        p.last[1] = 1 - (grid[1][count[1]] + size[1]) / (2 * size[1]);
        glGenBuffers(1, &p.vertices);
        glBindBuffer(GL_ARRAY_BUFFER, p.vertices);
        upload(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
               vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glGenBuffers(1, &p.indices);
        p.visible.reserve(p.elements.size());
      }
  }
  bool draw(const mjvGeom *g, int mode, const float planes[5][4]) {
    if (g->dataid < 0 || size_t(g->dataid) >= this->planes.size())
      return false;
    auto &p = *this->planes[g->dataid];
    if (!p.vertices)
      return false;
    p.visible.clear();
    double local[5][3];
    for (int n = 0; n < 5; n++) {
      local[n][0] = local[n][1] = 0;
      local[n][2] = planes[n][3];
      for (int k = 0; k < 3; k++) {
        local[n][0] += planes[n][k] * g->mat[3 * k];
        local[n][1] += planes[n][k] * g->mat[3 * k + 1];
        local[n][2] += planes[n][k] * g->pos[k];
      }
    }
    for (size_t i = 0; i + 1 < p.tiles.size(); i++) {
      const auto &t = p.tiles[i];
      bool skip = false;
      if (mode != 2)
        for (auto &q : local) {
          if (q[0] * t.x + q[1] * t.y + q[2] <
              -fabs(q[0]) * t.hx - fabs(q[1]) * t.hy - 1e-4) {
            skip = true;
            break;
          }
        }
      if (!skip)
        p.visible.insert(p.visible.end(), p.elements.begin() + t.offset,
                         p.elements.begin() + p.tiles[i + 1].offset);
    }
    glBindBuffer(GL_ARRAY_BUFFER, p.vertices);
    glVertexPointer(3, GL_FLOAT, 5 * sizeof(float), nullptr);
    glEnableClientState(GL_VERTEX_ARRAY);
    glNormal3d(0, 0, 1);
    if (mode < 2) {
      glTexCoordPointer(2, GL_FLOAT, 5 * sizeof(float),
                        (void *)(3 * sizeof(float)));
      glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, p.indices);
    upload(GL_ELEMENT_ARRAY_BUFFER, p.visible.size() * sizeof(unsigned),
           p.visible.data(), GL_STREAM_DRAW);
    glDrawElements(GL_QUADS, p.visible.size(), GL_UNSIGNED_INT, nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDisableClientState(GL_VERTEX_ARRAY);
    if (mode < 2)
      glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoord2dv(p.last);
    return true;
  }
};
CameraPlanes::CameraPlanes(const mjModel *m) : p_(std::make_unique<Impl>(m)) {}
CameraPlanes::~CameraPlanes() = default;
bool CameraPlanes::draw(const mjvGeom *g, int mode, const float planes[5][4]) {
  return p_->draw(g, mode, planes);
}
} // namespace simple_rbt
