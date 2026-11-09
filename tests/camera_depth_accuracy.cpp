#include "simple_rbt/cameras.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

int main() {
  try {
    const char *xml = R"(<mujoco>
      <visual><global offwidth="320" offheight="240"/>
        <quality offsamples="4"/><map znear=".005" zfar="50"/></visual>
      <worldbody><light pos="0 0 4" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="20 20 .1" rgba=".2 .5 .2 1"/>
        <geom name="wall" type="box" pos="14 0 1" size=".1 10 2" rgba=".8 .1 .1 1"/>
      </worldbody></mujoco>)";
    char error[2048]{};
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
        mj_parseXMLString(xml, nullptr, error, sizeof(error)), mj_deleteSpec);
    if (!spec) throw std::runtime_error(error);
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_compile(spec.get(), nullptr), mj_deleteModel);
    if (!model) throw std::runtime_error(mjs_getError(spec.get()));
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        mj_makeData(model.get()), mj_deleteData);
    mj_forward(model.get(), data.get());

    simple_rbt::CameraConfig config;
    config.render_panorama = true;
    const simple_rbt::OpticalPose pose{{0, 0, 1.2}, {0, 0, 1, -1, 0, 0, 0, -1, 0}};
    simple_rbt::CameraRig camera(model.get(), config);
    simple_rbt::CameraPair frame;
    camera.render_into(data.get(), pose, frame);
    const int floor = mj_name2id(model.get(), mjOBJ_GEOM, "floor");
    const double focal = .5 * config.perspective_height /
                         std::tan(config.perspective_fov_y_deg * std::acos(-1.) / 360.);
    const double near = model->vis.map.znear * model->stat.extent;
    double max_error = 0.;
    int samples = 0;
    for (int y = 130; y < config.perspective_height - 10; y += 9) {
      for (int x = 20; x < config.perspective_width - 20; x += 11) {
        const double lx = (x + .5 - .5 * config.perspective_width) / focal;
        const double ly = (y + .5 - .5 * config.perspective_height) / focal;
        const double norm = std::sqrt(lx * lx + ly * ly + 1.);
        const double local[3]{lx / norm, ly / norm, 1. / norm};
        double direction[3]{}, origin[3]{};
        for (int row = 0; row < 3; ++row) {
          for (int col = 0; col < 3; ++col)
            direction[row] += pose.rotation[3 * row + col] * local[col];
          origin[row] = pose.position[row] + near / local[2] * direction[row];
        }
        int hit = -1;
        const double distance = mj_ray(model.get(), data.get(), origin, direction,
                                       nullptr, 1, -1, &hit, nullptr);
        if (hit != floor || distance < 0.) continue;
        const double expected = (distance + near / local[2]) * local[2];
        if (expected > 10.) continue;
        const double actual = frame.perspective.depth[y * config.perspective_width + x];
        if (!std::isfinite(actual)) throw std::runtime_error("Finite floor depth missing");
        max_error = std::max(max_error, std::abs(actual - expected));
        ++samples;
      }
    }
    std::cout << "renderer=" << camera.renderer_name() << " samples=" << samples
              << " max_depth_error_m=" << max_error << std::endl;
    if (samples < 100 || max_error > .002)
      throw std::runtime_error("Pixel-centre floor depth error exceeds 2 mm");
    if (frame.panorama.depth.size() !=
        static_cast<std::size_t>(config.panorama_width * config.panorama_height))
      throw std::runtime_error("Panorama depth missing");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
