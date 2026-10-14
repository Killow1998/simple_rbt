#include "simple_rbt/cameras.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace simple_rbt {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr std::array<Mat3, 6> kFaces{{
    {1.,0.,0., 0.,1.,0., 0.,0.,1.},
    {0.,0.,1., 0.,1.,0., -1.,0.,0.},
    {-1.,0.,0., 0.,1.,0., 0.,0.,-1.},
    {0.,0.,-1., 0.,1.,0., 1.,0.,0.},
    {1.,0.,0., 0.,0.,-1., 0.,1.,0.},
    {1.,0.,0., 0.,0.,1., 0.,-1.,0.}}};

std::size_t pixels(int width, int height) {
  if (width <= 0 || height <= 0)
    throw std::invalid_argument("Camera dimensions must be positive integers");
  const auto size = static_cast<std::uint64_t>(width) * height;
  if (size > std::numeric_limits<std::uint32_t>::max())
    throw std::invalid_argument("Camera dimensions exceed the pixel-index capacity");
  return static_cast<std::size_t>(size);
}
CameraConfig checked(CameraConfig config) {
  pixels(config.perspective_width, config.perspective_height);
  pixels(config.panorama_width, config.panorama_height);
  pixels(config.cube_face_size, config.cube_face_size);
  if (!std::isfinite(config.perspective_fov_y_deg) || config.perspective_fov_y_deg <= 0. ||
      config.perspective_fov_y_deg >= 180.)
    throw std::invalid_argument("Camera vertical FOV must be finite and between 0 and 180 degrees");
  return config;
}
mjModel* checked(mjModel* model) {
  if (!model) throw std::invalid_argument("CameraRig requires a model");
  return model;
}
void check_pose(const OpticalPose& pose) {
  for (double value : pose.position)
    if (!std::isfinite(value)) throw std::invalid_argument("Camera position must be finite");
  for (double value : pose.rotation)
    if (!std::isfinite(value)) throw std::invalid_argument("Camera rotation must be finite");
  const auto& r = pose.rotation;
  for (int a = 0; a < 3; ++a)
    for (int b = 0; b < 3; ++b) {
      double dot = 0.;
      for (int i = 0; i < 3; ++i) dot += r[3*i+a] * r[3*i+b];
      if (std::abs(dot - (a == b ? 1. : 0.)) > 1e-6)
        throw std::invalid_argument("Camera rotation must be orthonormal");
    }
  const double determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) -
      r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
  if (std::abs(determinant - 1.) > 1e-6)
    throw std::invalid_argument("Camera rotation must be proper");
}

// Device-display selection and config match the installed MuJoCo EGL backend.
// Only EGL's device path is used; no window/X11 or software fallback is added.
struct EglDisplay {
  EGLDisplay handle = EGL_NO_DISPLAY;
  EglDisplay() {
    auto query_devices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
    auto platform_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    if (!query_devices || !platform_display)
      throw std::runtime_error("EGL device platform extensions are unavailable");
    EGLint count = 0;
    if (!query_devices(0, nullptr, &count) || count <= 0)
      throw std::runtime_error("EGL returned no devices");
    std::vector<EGLDeviceEXT> devices(static_cast<std::size_t>(count));
    if (!query_devices(count, devices.data(), &count))
      throw std::runtime_error("EGL device enumeration failed");
    long first = 0, last = count;
    if (const char* selected = std::getenv("MUJOCO_EGL_DEVICE_ID")) {
      char* end = nullptr;
      errno = 0;
      first = std::strtol(selected, &end, 10);
      if (errno || end == selected || *end || first < 0 || first >= count)
        throw std::invalid_argument("MUJOCO_EGL_DEVICE_ID is outside the EGL device list");
      last = first + 1;
    }
    for (long index = first; index < last; ++index) {
      eglGetError();
      EGLDisplay candidate = platform_display(EGL_PLATFORM_DEVICE_EXT, devices[index], nullptr);
      if (candidate != EGL_NO_DISPLAY && eglInitialize(candidate, nullptr, nullptr) == EGL_TRUE) {
        handle = candidate;
        break;
      }
    }
    if (handle == EGL_NO_DISPLAY) throw std::runtime_error("Cannot initialize an EGL device display");
  }
  ~EglDisplay() { if (handle != EGL_NO_DISPLAY) eglTerminate(handle); }
};
std::shared_ptr<EglDisplay> acquire_display() {
  static std::mutex mutex;
  static std::weak_ptr<EglDisplay> cached;
  std::lock_guard<std::mutex> lock(mutex);
  auto display = cached.lock();
  if (!display) { display = std::make_shared<EglDisplay>(); cached = display; }
  return display;
}
struct EglContext {
  std::shared_ptr<EglDisplay> display;
  EGLContext context = EGL_NO_CONTEXT;
  const std::thread::id owner = std::this_thread::get_id();
  std::string renderer;
  EglContext() : display(acquire_display()) {
    const EGLint attributes[] = {
        EGL_RED_SIZE,8, EGL_GREEN_SIZE,8, EGL_BLUE_SIZE,8, EGL_ALPHA_SIZE,8,
        EGL_DEPTH_SIZE,24, EGL_STENCIL_SIZE,8, EGL_COLOR_BUFFER_TYPE,EGL_RGB_BUFFER,
        EGL_SURFACE_TYPE,EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE,EGL_OPENGL_BIT, EGL_NONE};
    EGLConfig config{};
    EGLint count = 0;
    if (!eglChooseConfig(display->handle, attributes, &config, 1, &count) || count < 1 ||
        !eglBindAPI(EGL_OPENGL_API))
      throw std::runtime_error("Cannot select the MuJoCo EGL OpenGL configuration");
    context = eglCreateContext(display->handle, config, EGL_NO_CONTEXT, nullptr);
    if (context == EGL_NO_CONTEXT) throw std::runtime_error("Cannot create EGL OpenGL context");
    try {
      make_current();
      const auto* name = glGetString(GL_RENDERER);
      renderer = name ? reinterpret_cast<const char*>(name) : "unknown";
    } catch (...) {
      eglDestroyContext(display->handle, context);
      context = EGL_NO_CONTEXT;
      throw;
    }
  }
  void make_current() {
    if (std::this_thread::get_id() != owner)
      throw std::logic_error("CameraRig is bound to its construction thread");
    if (!eglMakeCurrent(display->handle, EGL_NO_SURFACE, EGL_NO_SURFACE, context))
      throw std::runtime_error("Cannot make the camera EGL context current");
  }
  void release_current() noexcept {
    // GLVND rejects EGL/GLX transitions while the other API remains current.
    // Only detach this rig's context, never an unrelated caller's context.
    if (eglGetCurrentContext() == context)
      eglMakeCurrent(display->handle, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
  }
  ~EglContext() {
    if (context != EGL_NO_CONTEXT) {
      release_current();
      eglDestroyContext(display->handle, context);
    }
  }
};

struct Scene {
  mjvScene value{};
  explicit Scene(mjModel* model) { mjv_defaultScene(&value); mjv_makeScene(model, &value, 10000); }
  ~Scene() { mjv_freeScene(&value); }
};
struct RenderContext {
  mjrContext value{};
  RenderContext(mjModel* model, int width, int height) {
    // The model's default framebuffer can be much larger than these views.
    // Restore even on an exception; no physics or other visual settings change.
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    struct Restore {
      mjModel* model;
      int width, height;
      ~Restore() { model->vis.global.offwidth = width; model->vis.global.offheight = height; }
    } restore{model, model->vis.global.offwidth, model->vis.global.offheight};
    model->vis.global.offwidth = width;
    model->vis.global.offheight = height;
    mjr_defaultContext(&value);
    mjr_makeContext(model, &value, mjFONTSCALE_150);
    value.readDepthMap = mjDEPTH_ZEROFAR;
    mjr_setBuffer(mjFB_OFFSCREEN, &value);
  }
  ~RenderContext() { mjr_freeContext(&value); }
};
struct FlagRestore {
  mjvScene& scene;
  std::array<mjtByte, mjNRNDFLAG> flags{};
  explicit FlagRestore(mjvScene& value) : scene(value) {
    std::copy_n(scene.flags, mjNRNDFLAG, flags.begin());
  }
  ~FlagRestore() { std::copy(flags.begin(), flags.end(), scene.flags); }
};
struct LookupEntry { std::uint32_t destination, source; double factor; };
using Lookup = std::array<std::vector<LookupEntry>, 6>;

Lookup make_lookup(const CameraConfig& config) {
  const int width = config.panorama_width, height = config.panorama_height;
  const int size = config.cube_face_size;
  const std::size_t count = pixels(width, height);
  std::vector<double> sin_lon(width), cos_lon(width), sin_lat(height), cos_lat(height);
  for (int x = 0; x < width; ++x) {
    const double angle = ((x + .5) / width - .5) * (2. * kPi);
    sin_lon[x] = std::sin(angle); cos_lon[x] = std::cos(angle);
  }
  for (int y = 0; y < height; ++y) {
    const double angle = (.5 - (y + .5) / height) * kPi;
    sin_lat[y] = std::sin(angle); cos_lat[y] = std::cos(angle);
  }
  auto ray_at = [&](int x, int y) -> Vec3 {
    return {sin_lon[x] * cos_lat[y], -sin_lat[y], cos_lon[x] * cos_lat[y]};
  };
  std::vector<std::uint8_t> selected(count);
  std::array<std::size_t, 6> counts{};
  for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
    const auto ray = ray_at(x, y);
    double best = -std::numeric_limits<double>::infinity();
    std::uint8_t face = 0;
    for (std::uint8_t f = 0; f < 6; ++f) {
      const auto& r = kFaces[f];
      const double score = (ray[0]*r[2] + ray[1]*r[5]) + ray[2]*r[8];
      if (score > best) { best = score; face = f; }  // strict: first tie wins
    }
    selected[static_cast<std::size_t>(y)*width+x] = face;
    ++counts[face];
  }
  Lookup lookup;
  for (int f = 0; f < 6; ++f) lookup[f].resize(counts[f]);
  counts.fill(0);
  const double focal = size / (2. * std::tan((90. * (kPi / 180.)) / 2.));
  std::vector<double> factors(pixels(size, size));
  for (int y = 0; y < size; ++y) for (int x = 0; x < size; ++x) {
    const double px = (x + .5 - size / 2.) / focal;
    const double py = (y + .5 - size / 2.) / focal;
    factors[static_cast<std::size_t>(y)*size+x] = std::sqrt((px*px + py*py) + 1.);
  }
  for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
    const auto destination = static_cast<std::uint32_t>(static_cast<std::size_t>(y)*width+x);
    const auto face = selected[destination];
    const auto ray = ray_at(x, y);
    const auto& r = kFaces[face];
    Vec3 local{};
    for (int i = 0; i < 3; ++i)
      local[i] = (ray[0]*r[i] + ray[1]*r[3+i]) + ray[2]*r[6+i];
    const int col = std::clamp(static_cast<int>(std::floor((local[0]/local[2] + 1.) * size / 2.)), 0, size-1);
    const int row = std::clamp(static_cast<int>(std::floor((local[1]/local[2] + 1.) * size / 2.)), 0, size-1);
    const auto source = static_cast<std::uint32_t>(static_cast<std::size_t>(row)*size+col);
    lookup[face][counts[face]++] = {destination, source, factors[source]};
  }
  return lookup;
}
void frame_size(CameraFrame& frame, int width, int height, bool radial) {
  const auto count = pixels(width, height);
  frame.width = width; frame.height = height; frame.radial_depth = radial;
  frame.rgb.resize(count * 3); frame.depth.resize(count);
}
}  // namespace

struct CameraRig::Impl {
  mjModel* model;
  CameraConfig config;
  EglContext egl;
  Scene scene;
  RenderContext perspective_context, face_context;
  Lookup lookup;
  mjvOption option{};
  mjvCamera abstract_camera{};
  std::vector<std::uint8_t> no_surface, segmentation_rgb;
  std::vector<float> raw_depth;
  CameraFrame face;

  Impl(mjModel* input_model, CameraConfig input_config)
      : model(checked(input_model)), config(checked(input_config)), egl(), scene(model),
        perspective_context(model, config.perspective_width, config.perspective_height),
        face_context(model, config.cube_face_size, config.cube_face_size), lookup(make_lookup(config)) {
    mjv_defaultOption(&option);
    mjv_defaultCamera(&abstract_camera);
    mjv_defaultFreeCamera(model, &abstract_camera);
    egl.release_current();
  }
  ~Impl() {
    // Member destruction frees both MuJoCo contexts before the EGL context.
    if (std::this_thread::get_id() == egl.owner)
      eglMakeCurrent(egl.display->handle, EGL_NO_SURFACE, EGL_NO_SURFACE, egl.context);
  }
  void update_scene(mjData* data) {
    mjv_updateScene(model, data, &option, nullptr, &abstract_camera, mjCAT_ALL, &scene.value);
    no_surface.assign(static_cast<std::size_t>(scene.value.ngeom)+1, 1);
    for (int i = 0; i < scene.value.ngeom; ++i) {
      const auto& geom = scene.value.geoms[i];
      if (geom.segid != -1) no_surface.at(geom.segid+1) = geom.objid == -1;
    }
  }
  void render_view(const OpticalPose& pose, int width, int height, double fov,
                   RenderContext& context, CameraFrame& frame) {
    frame_size(frame, width, height, false);
    const double near = static_cast<double>(model->vis.map.znear) * model->stat.extent;
    const double far = static_cast<double>(model->vis.map.zfar) * model->stat.extent;
    const double half_height = near * std::tan((fov * (kPi / 180.)) / 2.);
    for (auto& camera : scene.value.camera) {
      for (int i = 0; i < 3; ++i) {
        camera.pos[i] = static_cast<float>(pose.position[i]);
        camera.forward[i] = static_cast<float>(pose.rotation[3*i+2]);
        camera.up[i] = static_cast<float>(-pose.rotation[3*i+1]);
      }
      camera.orthographic = 0;
      camera.frustum_near = static_cast<float>(near);
      camera.frustum_far = static_cast<float>(far);
      camera.frustum_center = 0.; camera.frustum_width = 0.;
      camera.frustum_bottom = static_cast<float>(-half_height);
      camera.frustum_top = static_cast<float>(half_height);
    }
    const mjrRect viewport{0, 0, width, height};
    mjr_setBuffer(mjFB_OFFSCREEN, &context.value);
    mjr_render(viewport, &scene.value, &context.value);
    mjr_readPixels(frame.rgb.data(), nullptr, viewport, &context.value);
    const std::size_t count = pixels(width, height);
    raw_depth.resize(count); segmentation_rgb.resize(count*3);
    {
      FlagRestore restore(scene.value);
      scene.value.flags[mjRND_SEGMENT] = 1;
      scene.value.flags[mjRND_IDCOLOR] = 1;
      mjr_render(viewport, &scene.value, &context.value);
      mjr_readPixels(segmentation_rgb.data(), raw_depth.data(), viewport, &context.value);
    }
    // Match classic Renderer's float32 projection coefficients, double metric
    // conversion and final float32 store. Compile without fast-math/FMA folding.
    const float zfar = static_cast<float>(far), znear = static_cast<float>(near);
    float c = -(zfar + znear) / (zfar - znear);
    float d = -(2.f * zfar * znear) / (zfar - znear);
    c = -.5f * c - .5f;
    d = -.5f * d;
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
      const std::size_t source = static_cast<std::size_t>(height-1-y)*width+x;
      const std::size_t destination = static_cast<std::size_t>(y)*width+x;
      const auto* id = segmentation_rgb.data() + source*3;
      const std::uint32_t segment = id[0] + (static_cast<std::uint32_t>(id[1]) << 8) +
                                    (static_cast<std::uint32_t>(id[2]) << 16);
      frame.depth[destination] = no_surface.at(segment) ? std::numeric_limits<float>::infinity() :
          static_cast<float>(static_cast<double>(d) / (static_cast<double>(raw_depth[source]) + c));
    }
    const std::size_t row_bytes = static_cast<std::size_t>(width)*3;
    for (int y = 0; y < height/2; ++y)
      std::swap_ranges(frame.rgb.begin()+y*row_bytes, frame.rgb.begin()+(y+1)*row_bytes,
                       frame.rgb.begin()+(height-1-y)*row_bytes);
  }
  void render_into(mjData* data, const OpticalPose& pose, CameraPair& output) {
    if (!data) throw std::invalid_argument("CameraRig requires data");
    check_pose(pose);
    egl.make_current();
    struct ReleaseCurrent {
      EglContext& context;
      ~ReleaseCurrent() { context.release_current(); }
    } release{egl};
    update_scene(data);
    render_view(pose, config.perspective_width, config.perspective_height,
                config.perspective_fov_y_deg, perspective_context, output.perspective);
    // Use the same scene for all six cube faces of this physical snapshot.
    // Rebuild once to match the Python perspective/panorama scene boundary.
    update_scene(data);
    frame_size(output.panorama, config.panorama_width, config.panorama_height, true);
    for (int f = 0; f < 6; ++f) {
      OpticalPose face_pose = pose;
      for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        double value = 0.;
        for (int k = 0; k < 3; ++k) value += pose.rotation[3*i+k] * kFaces[f][3*k+j];
        face_pose.rotation[3*i+j] = value;
      }
      render_view(face_pose, config.cube_face_size, config.cube_face_size, 90., face_context, face);
      for (const auto& entry : lookup[f]) {
        std::copy_n(face.rgb.data()+3*static_cast<std::size_t>(entry.source), 3,
                    output.panorama.rgb.data()+3*static_cast<std::size_t>(entry.destination));
        output.panorama.depth[entry.destination] = static_cast<float>(
            static_cast<double>(face.depth[entry.source]) * entry.factor);
      }
    }
  }
};

CameraRig::CameraRig(mjModel* model, CameraConfig config) : impl_(std::make_unique<Impl>(model, config)) {}
CameraRig::~CameraRig() = default;
void CameraRig::render_into(mjData* data, const OpticalPose& pose, CameraPair& output) {
  impl_->render_into(data, pose, output);
}
const std::string& CameraRig::renderer_name() const noexcept { return impl_->egl.renderer; }

}  // namespace simple_rbt
