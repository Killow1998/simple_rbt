#define GL_GLEXT_PROTOTYPES
#include "camera_render.hpp"
#include <GL/gl.h>
#include <GL/glext.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <mujoco/mjmacro.h>
namespace simple_rbt {
namespace {
struct Batch {
  bool fresh = true, cull = true, clip_control = false;
  const mjrContext *context = nullptr;
  float near = 0, far = 0, planes[4][4]{};
};
#if mjVERSION_HEADER == 3015000
// The classic renderer has private functions, so keep the pinned adaptation
// isolated. Public MuJoCo APIs and the physics library are never overridden.
thread_local Batch *active = nullptr;
bool outside(const mjvGeom *g) {
  if (!active->cull || g->objtype != mjOBJ_GEOM || g->type == mjGEOM_PLANE ||
      g->type == mjGEOM_HFIELD || g->type == mjGEOM_FLEX ||
      g->type == mjGEOM_SKIN || !(g->modelrbound > 0))
    return false;
  for (const auto &p : active->planes) {
    double radius = g->modelrbound;
    if (g->type == mjGEOM_MESH || g->type == mjGEOM_BOX) {
      radius = 0;
      for (int i = 0; i < 3; i++)
        radius += std::abs(p[0] * g->mat[i] + p[1] * g->mat[3 + i] +
                           p[2] * g->mat[6 + i]) *
                  g->size[i];
    }
    if (p[0] * g->pos[0] + p[1] * g->pos[1] + p[2] * g->pos[2] + p[3] <
        -radius - 1e-4)
      return true;
  }
  return false;
}
void set_planes(const float *projection, const float *view) {
  double matrix[16]{};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        matrix[4 * j + i] += double(projection[4 * k + i]) * view[4 * j + k];
  // Side planes suffice; omit near/far rejection to remain conservative at
  // clipping boundaries. Include the scene transform through the view matrix.
  for (int n = 0; n < 4; n++) {
    double p[4];
    for (int j = 0; j < 4; j++)
      p[j] = matrix[4 * j + 3] + (n % 2 ? -1 : 1) * matrix[4 * j + n / 2];
    double length = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    for (int j = 0; j < 4; j++)
      active->planes[n][j] = p[j] / length;
  }
}
#include "mujoco_classic/engine_sort.h"
#include "mujoco_classic/helpers.inc"
#include "mujoco_classic/primitives.inc"
#include "mujoco_classic/render.inc"
#endif
} // namespace
struct CameraRender::Impl {
  bool software = false, supported = false;
  Batch batch;
};
CameraRender::CameraRender(const std::string &renderer)
    : p_(std::make_unique<Impl>()) {
  p_->software = renderer.find("llvmpipe") != std::string::npos ||
                 renderer.find("softpipe") != std::string::npos;
#if mjVERSION_HEADER == 3015000
  p_->supported = p_->software && mj_version() == mjVERSION_HEADER;
#endif
  const auto *extensions =
      reinterpret_cast<const char *>(glGetString(GL_EXTENSIONS));
  p_->batch.clip_control =
      extensions && std::strstr(extensions, "GL_ARB_clip_control");
}
CameraRender::~CameraRender() = default;
void CameraRender::begin() {
  p_->batch.fresh = true;
  p_->batch.context = nullptr;
}
void CameraRender::render(mjrRect viewport, mjvScene *scene,
                          const mjrContext *context) {
#if mjVERSION_HEADER == 3015000
  if (p_->supported) {
    struct Restore {
      Batch *previous = active;
      ~Restore() { active = previous; }
    } restore;
    active = &p_->batch;
    render_classic(viewport, scene, context);
    return;
  }
#endif
  mjr_render(viewport, scene, context);
}
int CameraRender::shadow_size(int requested) const {
  return p_->software ? std::min(requested, 2048) : requested;
}
} // namespace simple_rbt
