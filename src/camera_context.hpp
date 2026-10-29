#pragma once
#include <EGL/egl.h>
#include <memory>
#include <string>
#include <thread>
namespace simple_rbt::camera_detail {
struct EglDisplay;
class EglContext {
  std::shared_ptr<EglDisplay> display;
  EGLContext context = EGL_NO_CONTEXT;
  const std::thread::id owner = std::this_thread::get_id();

public:
  std::string renderer;
  EglContext();
  ~EglContext();
  void make_current();
  void release_current() noexcept;
  void prepare_cleanup() noexcept;
};
} // namespace simple_rbt::camera_detail
