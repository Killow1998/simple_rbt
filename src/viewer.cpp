#include "simple_rbt/viewer.hpp"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <stdexcept>
namespace simple_rbt {
struct Viewer::Impl {
  mjModel *m;
  mjData *d;
  GLFWwindow *w = nullptr;
  mjvCamera camera{};
  mjvOption option{};
  mjvScene scene{};
  mjrContext context{};
  double mx = 0, my = 0;
  Impl(mjModel *model, mjData *data) : m(model), d(data) {
    if (!glfwInit())
      throw std::runtime_error(
          "GLFW initialization failed; --viewer requires a working display");
    w = glfwCreateWindow(1280, 720, "simple_rbt — native C++", nullptr,
                         nullptr);
    if (!w) {
      glfwTerminate();
      throw std::runtime_error("Cannot create viewer window");
    }
    glfwMakeContextCurrent(w);
    glfwSwapInterval(0);
    mjv_defaultCamera(&camera);
    mjv_defaultFreeCamera(m, &camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    mjv_makeScene(m, &scene, 10000);
    mjr_makeContext(m, &context, mjFONTSCALE_150);
    glfwSetWindowUserPointer(w, this);
    glfwSetKeyCallback(w, [](GLFWwindow *w, int key, int, int action, int) {
      if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(w, true);
    });
    glfwSetCursorPosCallback(w, [](GLFWwindow *w, double x, double y) {
      auto *p = static_cast<Impl *>(glfwGetWindowUserPointer(w));
      int width, height;
      glfwGetWindowSize(w, &width, &height);
      double dx = x - p->mx, dy = y - p->my;
      p->mx = x;
      p->my = y;
      if (height <= 0)
        return;
      bool left = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS,
           right = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS,
           middle =
               glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
      if (!left && !right && !middle)
        return;
      bool shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                   glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
      int a = right  ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
              : left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V)
                     : mjMOUSE_ZOOM;
      mjv_moveCamera(p->m, a, dx / height, dy / height, &p->camera);
    });
    glfwSetScrollCallback(w, [](GLFWwindow *w, double, double y) {
      auto *p = static_cast<Impl *>(glfwGetWindowUserPointer(w));
      mjv_moveCamera(p->m, mjMOUSE_ZOOM, 0, -.05 * y, &p->camera);
    });
    // Leave no GLX context current before the camera's EGL renderer runs.
    glfwMakeContextCurrent(nullptr);
  }
  ~Impl() {
    if (w) {
      glfwMakeContextCurrent(w);
      mjr_freeContext(&context);
      mjv_freeScene(&scene);
      glfwMakeContextCurrent(nullptr);
      glfwDestroyWindow(w);
    }
    glfwTerminate();
  }
};
Viewer::Viewer(mjModel *m, mjData *d) : p_(std::make_unique<Impl>(m, d)) {}
Viewer::~Viewer() = default;
bool Viewer::running() const { return !glfwWindowShouldClose(p_->w); }
void Viewer::sync() {
  auto &p = *p_;
  glfwMakeContextCurrent(p.w);
  struct ReleaseCurrent {
    ~ReleaseCurrent() { glfwMakeContextCurrent(nullptr); }
  } release;
  int width, height;
  glfwGetFramebufferSize(p.w, &width, &height);
  mjv_updateScene(p.m, p.d, &p.option, nullptr, &p.camera, mjCAT_ALL, &p.scene);
  mjrRect viewport{0, 0, width, height};
  mjr_setBuffer(mjFB_WINDOW, &p.context);
  mjr_render(viewport, &p.scene, &p.context);
  glfwSwapBuffers(p.w);
  glfwPollEvents();
}
} // namespace simple_rbt
