#define GL_GLEXT_PROTOTYPES
#include "camera_render.hpp"
#include "camera_lighting.hpp"
#include "camera_mesh.hpp"
#include "camera_planes.hpp"
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
  float near = 0, far = 0, planes[5][4]{};
  CameraMeshes *meshes = nullptr;
  CameraLighting *lighting = nullptr;
  CameraPlanes *geometry = nullptr;
  bool fuse = false;
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
  // Four sides and the reverse-Z near plane. This also skips high-poly sensor
  // housings entirely behind the near clip. Keep the 0.1 mm boundary margin.
  // Far-plane rejection is unnecessary; shadow casters bypass all view planes.
  for (int n = 0; n < 5; n++) {
    double p[4];
    for (int j = 0; j < 4; j++)
      p[j] = matrix[4 * j + 3] +
             (n == 4 ? -matrix[4 * j + 2]
                     : (n % 2 ? -1 : 1) * matrix[4 * j + n / 2]);
    double length = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    for (int j = 0; j < 4; j++)
      active->planes[n][j] = p[j] / length;
  }
}
bool fused_geom(const mjvGeom *g, const mjrContext *c) {
  return active->cull && active->fuse && active->meshes->contains(g) &&
         active->lighting->supports(g, c);
}
void draw_mesh(const mjvGeom *g, int mode, const mjrContext *c) {
  if (!active->cull || !active->meshes ||
      !active->meshes->draw(g, mode, active->near))
    glCallList(c->baseMesh + g->dataid);
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
  std::unique_ptr<CameraMeshes> meshes;
  std::unique_ptr<CameraLighting> lighting;
  std::unique_ptr<CameraPlanes> geometry;
};
CameraRender::CameraRender(const std::string &renderer, const mjModel *model)
    : p_(std::make_unique<Impl>()) {
  p_->software = renderer.find("llvmpipe") != std::string::npos ||
                 renderer.find("softpipe") != std::string::npos;
#if mjVERSION_HEADER == 3015000
  p_->supported = p_->software && mj_version() == mjVERSION_HEADER;
  if (p_->supported) {
    p_->meshes = std::make_unique<CameraMeshes>(model);
    p_->lighting = std::make_unique<CameraLighting>();
    p_->geometry = std::make_unique<CameraPlanes>(model);
    p_->batch.geometry = p_->geometry.get();
    p_->batch.meshes = p_->meshes.get();
    p_->batch.lighting = p_->lighting.get();
  }
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
    active->fuse = p_->lighting->supports(scene, context);
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
