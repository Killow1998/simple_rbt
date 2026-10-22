// A renderer must observe its submitted sample even while live physics
// advances.
#include "runtime_camera_worker.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace simple_rbt;

void same(const CameraFrame &a, const CameraFrame &b) {
  if (a.width != b.width || a.height != b.height ||
      a.radial_depth != b.radial_depth || a.rgb != b.rgb || a.depth != b.depth)
    throw std::runtime_error("Camera worker changed a frozen sample");
}
int main() {
  try {
    char error[2048]{};
    const char *xml = R"(<mujoco><worldbody>
      <light pos="0 0 4" dir="0 0 -1" directional="true"/>
      <geom type="plane" size="5 5 .1"/>
      <body pos="2 0 .5"><joint type="slide" axis="0 1 0"/>
        <geom type="box" size=".1 .5 .5" rgba=".8 .1 .1 1"/>
      </body></worldbody></mujoco>)";
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
        mj_parseXMLString(xml, nullptr, error, sizeof(error)), mj_deleteSpec);
    if (!spec)
      throw std::runtime_error(error);
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_compile(spec.get(), nullptr), mj_deleteModel);
    if (!model)
      throw std::runtime_error(mjs_getError(spec.get()));
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        mj_makeData(model.get()), mj_deleteData);
    mj_forward(model.get(), data.get());
    CameraConfig config;
    OpticalPose pose{{0, 0, .4}, {0, 0, 1, -1, 0, 0, 0, -1, 0}};
    CameraRig oracle(model.get(), config);
    CameraPair expected;
    {
      CameraWorker worker(model.get(), data.get(), config, pose, pose);
      for (int sample = 0; sample < 6; ++sample) {
        data->qpos[0] = -.5 + .2 * sample;
        mj_forward(model.get(), data.get());
        auto color = pose, depth = pose;
        color.position[1] = .03 * sample;
        depth.position[1] = color.position[1] - .05;
        oracle.render_into(data.get(), color, depth, expected);
        worker.submit(data.get(), color, depth);
        data->qvel[0] = 4;
        for (int step = 0; step < 100; ++step)
          mj_step(model.get(), data.get());
        worker.wait();
        same(expected.perspective, worker.frames().perspective);
        same(expected.panorama, worker.frames().panorama);
      }
      // Destruction also joins an outstanding render on its GL owner thread.
      worker.submit(data.get(), pose, pose);
    }
    {
      CameraWorker worker(model.get(), data.get(), config, pose, pose);
      auto invalid = pose;
      invalid.position[0] = std::numeric_limits<double>::quiet_NaN();
      worker.submit(data.get(), invalid, pose);
      bool failed = false;
      try {
        worker.wait();
      } catch (const std::invalid_argument &) {
        failed = true;
      }
      if (!failed)
        throw std::runtime_error("Worker exception was lost");
    }
    std::cout << "6 frozen RGB/depth/panorama samples identical; shutdown and "
                 "error propagation pass\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
