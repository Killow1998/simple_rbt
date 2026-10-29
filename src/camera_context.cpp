#include "camera_context.hpp"
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <vector>
namespace simple_rbt::camera_detail {
struct EglDisplay {
  EGLDisplay handle = EGL_NO_DISPLAY;
  EglDisplay() {
    // Small sensor views suffer from llvmpipe worker synchronization when Mesa
    // starts a worker per CPU. Bound its default pool; explicit tuning wins.
    // Hardware GL drivers ignore this Mesa software-renderer setting.
    if (!std::getenv("LP_NUM_THREADS")) {
      auto workers =
          std::min(4u, std::max(1u, std::thread::hardware_concurrency()));
      setenv("LP_NUM_THREADS", std::to_string(workers).c_str(), 0);
    }
    auto query_devices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(
        eglGetProcAddress("eglQueryDevicesEXT"));
    auto platform_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
        eglGetProcAddress("eglGetPlatformDisplayEXT"));
    if (!query_devices || !platform_display)
      throw std::runtime_error(
          "EGL device platform extensions are unavailable");
    EGLint count = 0;
    if (!query_devices(0, nullptr, &count) || count <= 0)
      throw std::runtime_error("EGL returned no devices");
    std::vector<EGLDeviceEXT> devices(static_cast<std::size_t>(count));
    if (!query_devices(count, devices.data(), &count))
      throw std::runtime_error("EGL device enumeration failed");
    long first = 0, last = count;
    if (const char *selected = std::getenv("MUJOCO_EGL_DEVICE_ID")) {
      char *end = nullptr;
      errno = 0;
      first = std::strtol(selected, &end, 10);
      if (errno || end == selected || *end || first < 0 || first >= count)
        throw std::invalid_argument(
            "MUJOCO_EGL_DEVICE_ID is outside the EGL device list");
      last = first + 1;
    }
    for (long index = first; index < last; ++index) {
      eglGetError();
      EGLDisplay candidate =
          platform_display(EGL_PLATFORM_DEVICE_EXT, devices[index], nullptr);
      if (candidate != EGL_NO_DISPLAY &&
          eglInitialize(candidate, nullptr, nullptr) == EGL_TRUE) {
        handle = candidate;
        break;
      }
    }
    if (handle == EGL_NO_DISPLAY)
      throw std::runtime_error("Cannot initialize an EGL device display");
  }
  ~EglDisplay() { eglTerminate(handle); }
};
std::shared_ptr<EglDisplay> acquire_display() {
  static std::mutex mutex;
  static std::weak_ptr<EglDisplay> cached;
  std::lock_guard<std::mutex> lock(mutex);
  auto display = cached.lock();
  if (!display) {
    display = std::make_shared<EglDisplay>();
    cached = display;
  }
  return display;
}
EglContext::EglContext() : display(acquire_display()) {
  const EGLint attributes[] = {EGL_RED_SIZE,
                               8,
                               EGL_GREEN_SIZE,
                               8,
                               EGL_BLUE_SIZE,
                               8,
                               EGL_ALPHA_SIZE,
                               8,
                               EGL_DEPTH_SIZE,
                               24,
                               EGL_STENCIL_SIZE,
                               8,
                               EGL_COLOR_BUFFER_TYPE,
                               EGL_RGB_BUFFER,
                               EGL_SURFACE_TYPE,
                               EGL_PBUFFER_BIT,
                               EGL_RENDERABLE_TYPE,
                               EGL_OPENGL_BIT,
                               EGL_NONE};
  EGLConfig config{};
  EGLint count = 0;
  if (!eglChooseConfig(display->handle, attributes, &config, 1, &count) ||
      count < 1 || !eglBindAPI(EGL_OPENGL_API))
    throw std::runtime_error(
        "Cannot select the MuJoCo EGL OpenGL configuration");
  context = eglCreateContext(display->handle, config, EGL_NO_CONTEXT, nullptr);
  if (context == EGL_NO_CONTEXT)
    throw std::runtime_error("Cannot create EGL OpenGL context");
  try {
    make_current();
    const auto *name = glGetString(GL_RENDERER);
    renderer = name ? reinterpret_cast<const char *>(name) : "unknown";
  } catch (...) {
    eglDestroyContext(display->handle, context);
    throw;
  }
}
void EglContext::make_current() {
  if (std::this_thread::get_id() != owner)
    throw std::logic_error("CameraRig is bound to its construction thread");
  if (!eglMakeCurrent(display->handle, EGL_NO_SURFACE, EGL_NO_SURFACE, context))
    throw std::runtime_error("Cannot make the camera EGL context current");
}
void EglContext::release_current() noexcept {
  // GLVND rejects EGL/GLX transitions while the other API remains current.
  // Only detach this rig's context, never an unrelated caller's context.
  if (eglGetCurrentContext() == context)
    eglMakeCurrent(display->handle, EGL_NO_SURFACE, EGL_NO_SURFACE,
                   EGL_NO_CONTEXT);
}
void EglContext::prepare_cleanup() noexcept {
  if (std::this_thread::get_id() == owner)
    eglMakeCurrent(display->handle, EGL_NO_SURFACE, EGL_NO_SURFACE, context);
}
EglContext::~EglContext() {
  release_current();
  eglDestroyContext(display->handle, context);
}
} // namespace simple_rbt::camera_detail
