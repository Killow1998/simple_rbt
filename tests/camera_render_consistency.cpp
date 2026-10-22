// Compare complete native OpenGL passes across changing snapshots and cameras.
// The oracle is MuJoCo's unmodified renderer in its own framebuffer/shadow map.
#include "camera_render.hpp"
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <mujoco/mujoco.h>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <vector>
namespace {
struct EGL {
  EGLDisplay display = EGL_NO_DISPLAY;
  EGLContext context = EGL_NO_CONTEXT;
  EGL() {
    auto platform = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
        eglGetProcAddress("eglGetPlatformDisplayEXT"));
    if (!platform)
      throw std::runtime_error("EGL platform extension unavailable");
    display =
        platform(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    if (!eglInitialize(display, nullptr, nullptr) ||
        !eglBindAPI(EGL_OPENGL_API))
      throw std::runtime_error("Software EGL initialization failed");
    const EGLint attrs[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
                            EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
    EGLConfig config;
    EGLint count = 0;
    if (!eglChooseConfig(display, attrs, &config, 1, &count) || count != 1)
      throw std::runtime_error("EGL config unavailable");
    context = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
    if (context == EGL_NO_CONTEXT ||
        !eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context))
      throw std::runtime_error("EGL context unavailable");
  }
  ~EGL() {
    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(display, context);
    eglTerminate(display);
  }
};
struct Scene {
  mjvScene value{};
  explicit Scene(mjModel *model) {
    mjv_defaultScene(&value);
    mjv_makeScene(model, &value, 1000);
  }
  ~Scene() { mjv_freeScene(&value); }
};
struct Context {
  mjrContext value{};
  explicit Context(mjModel *model) {
    mjr_defaultContext(&value);
    mjr_makeContext(model, &value, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &value);
  }
  ~Context() { mjr_freeContext(&value); }
};
struct Pixels {
  std::vector<unsigned char> rgb;
  std::vector<float> depth;
  explicit Pixels(mjrRect viewport)
      : rgb(viewport.width * viewport.height * 3),
        depth(viewport.width * viewport.height) {}
  void read(mjrRect viewport, const mjrContext *context) {
    mjr_readPixels(rgb.data(), depth.data(), viewport, context);
  }
};
// Self-contained dense CAD-like torus: exercise LOD, exact indexed depth and
// fused lighting in CI without downloading robot assets.
std::string dense_mesh(bool textured) {
  std::ostringstream vertices, uv, faces;
  constexpr int rings = 48, sides = 24;
  constexpr double pi = 3.14159265358979323846;
  for (int i = 0; i <= rings; i++)
    for (int j = 0; j <= sides; j++) {
      double a = 2 * pi * i / rings, b = 2 * pi * j / sides;
      vertices << (.7 + .2 * std::cos(b)) * std::cos(a) << ' '
               << (.7 + .2 * std::cos(b)) * std::sin(a) << ' '
               << .2 * std::sin(b) << ' ';
      uv << double(i) / rings << ' ' << double(j) / sides << ' ';
      if (i < rings && j < sides) {
        int v = i * (sides + 1) + j, n = v + sides + 1;
        faces << v << ' ' << n << ' ' << v + 1 << ' ' << v + 1 << ' ' << n
              << ' ' << n + 1 << ' ';
      }
    }
  return "<mesh name=\"tetra\" vertex=\"" + vertices.str() + "\" face=\"" +
         faces.str() + "\"" +
         (textured ? " texcoord=\"" + uv.str() + "\"" : "") + "/>";
}
void check(int lights, bool reflected, int &comparisons, bool dense = false,
           bool textured = false, bool planes = false) {
  std::string xml =
      R"(<mujoco><visual><global offwidth="320" offheight="256"/><quality shadowsize="2048"/></visual>
  <asset><texture name="checker" type="2d" builtin="checker" width="64" height="64" rgb1=".1 .2 .3" rgb2=".8 .7 .6"/>
  <material name="textured" texture="checker"/><material name="floor" reflectance="REFLECT"/><mesh name="tetra" vertex="0 0 0  1 0 0  0 1 0  0 0 1" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset>
  <worldbody><light pos="1 2 4" dir="-.2 -.1 -1" directional="true"/>)";
  if (lights == 2)
    xml += R"(<light pos="-3 1 4" dir=".4 0 -1"/>)";
  xml.replace(xml.find("REFLECT"), 7, reflected ? ".3" : "0");
  xml +=
      R"(<geom type="plane" size="8 8 .1" rgba=".4 .4 .4 1" material="floor"/>)";
  if (planes)
    xml += R"(<geom type="plane" size="3.37 2.19 .17" pos="0 1 .6" euler="12 -8 17" material="textured"/>
      <geom type="plane" size="1.17 1.29 .13" pos="-2 0 1" euler="-24 13 0" rgba=".6 .2 .3 .5"/>
      <geom type="plane" size="0 0 .1" pos="0 0 -.1"/>)";
  xml += R"(<geom type="box" size=".4 .7 .6" pos="2 -1 .6" material="textured"/>
  <geom type="sphere" size=".5" pos="-1 1 .8" rgba=".1 .8 .2 .4"/>
  <geom type="mesh" mesh="tetra" pos="-2 -1 .5"/>
  <body pos="0 0 .8"><joint type="slide" axis="1 0 0"/><geom type="box" size=".3 .2 .8" rgba=".8 .1 .2 1"/></body>
  </worldbody></mujoco>)";
  if (dense) {
    auto start = xml.find("<mesh name=");
    xml.replace(start, xml.find("/>", start) + 2 - start, dense_mesh(textured));
    if (textured) {
      start = xml.find("mesh=\"tetra\" pos=");
      xml.insert(start, "material=\"textured\" ");
    }
  }
  char error[2048]{};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error)),
      mj_deleteSpec);
  if (!spec)
    throw std::runtime_error(error);
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_compile(spec.get(), nullptr), mj_deleteModel);
  if (!model)
    throw std::runtime_error(mjs_getError(spec.get()));
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
      mj_makeData(model.get()), mj_deleteData);
  Scene scene(model.get());
  Context native(model.get()), batched(model.get());
  simple_rbt::CameraRender renderer(
      reinterpret_cast<const char *>(glGetString(GL_RENDERER)), model.get());
  mjvOption option;
  mjv_defaultOption(&option);
  mjvCamera camera;
  mjv_defaultFreeCamera(model.get(), &camera);
  camera.distance = 3;
  camera.elevation = -20;
  for (int sample = 0; sample < 4; sample++) {
    data->qpos[0] = sample * .4;
    model->light_pos[0] = 1 + sample * .2;
    mj_forward(model.get(), data.get());
    mjv_updateScene(model.get(), data.get(), &option, nullptr, &camera,
                    mjCAT_ALL, &scene.value);
    renderer.begin();
    for (int view = 0; view < 7; view++) {
      camera.azimuth = view * 60 + sample * 13;
      camera.elevation = view == 6 ? 70 : -20;
      mjv_updateCamera(model.get(), data.get(), &camera, &scene.value);
      mjrRect viewport{0, 0, view ? 256 : 320, view ? 256 : 240};
      for (bool segment : {false, true}) {
        scene.value.flags[mjRND_SEGMENT] = segment;
        scene.value.flags[mjRND_IDCOLOR] = segment;
        Pixels a(viewport), b(viewport);
        mjr_setBuffer(mjFB_OFFSCREEN, &native.value);
        mjr_render(viewport, &scene.value, &native.value);
        a.read(viewport, &native.value);
        mjr_setBuffer(mjFB_OFFSCREEN, &batched.value);
        renderer.render(viewport, &scene.value, &batched.value);
        b.read(viewport, &batched.value);
        bool mismatch = a.rgb != b.rgb || a.depth != b.depth;
        if (dense && !segment) {
          // Runtime reads sensor depth from the original-geometry segmentation
          // pass, never the RGB LOD depth attachment.
          double error_sum = 0;
          size_t outliers = 0;
          for (size_t pixel = 0; pixel < a.rgb.size(); pixel += 3) {
            int largest = 0;
            for (int c = 0; c < 3; c++) {
              int delta =
                  std::abs(int(a.rgb[pixel + c]) - int(b.rgb[pixel + c]));
              largest = std::max(largest, delta);
              error_sum += delta;
            }
            outliers += largest > 10;
          }
          mismatch = error_sum / a.rgb.size() > 1 ||
                     double(outliers) / (a.rgb.size() / 3) > .02;
        }
        if (mismatch) {
          auto pixels = std::inner_product(a.rgb.begin(), a.rgb.end(),
                                           b.rgb.begin(), 0, std::plus<int>(),
                                           std::not_equal_to<unsigned char>());
          throw std::runtime_error(
              "Native mismatch: lights=" + std::to_string(lights) +
              " reflection=" + std::to_string(reflected) + " sample=" +
              std::to_string(sample) + " view=" + std::to_string(view) +
              " segment=" + std::to_string(segment) +
              " RGB differences=" + std::to_string(pixels));
        }
        comparisons++;
      }
    }
  }
}
} // namespace
int main() {
  try {
    EGL context;
    int comparisons = 0;
    for (int lights : {1, 2})
      for (bool reflected : {false, true})
        check(lights, reflected, comparisons);
    std::cout << comparisons << " native RGB/depth passes identical; ";
    comparisons = 0;
    for (int lights : {1, 2})
      for (bool reflected : {false, true})
        check(lights, reflected, comparisons, false, false, true);
    std::cout << comparisons << " tilted/textured/infinite plane passes identical; ";
    comparisons = 0;
    for (bool textured : {false, true})
      check(1, false, comparisons, true, textured);
    std::cout << comparisons
              << " dense mesh RGB quality / exact segmentation-depth passes; "
                 "renderer="
              << glGetString(GL_RENDERER) << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
