// Independent native renderer harness. Inputs contain a compiled model and an
// exact Python-oracle state/optical pose; this executable never launches Python.
#include "simple_rbt/cameras.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
template<class T> void read_exact(std::ifstream& input, T* data, std::size_t count) {
  if (!input.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(count*sizeof(T))))
    throw std::runtime_error("Truncated oracle input");
}
void end_of_file(std::ifstream& input) {
  if (input.peek() != std::char_traits<char>::eof()) throw std::runtime_error("Unexpected oracle trailing bytes");
}
template<class T> void write(const std::string& path, const std::vector<T>& values) {
  std::ofstream out(path, std::ios::binary);
  if (!out.write(reinterpret_cast<const char*>(values.data()), static_cast<std::streamsize>(values.size()*sizeof(T))))
    throw std::runtime_error("Cannot write " + path);
}
void write_frame(const std::string& prefix, const simple_rbt::CameraFrame& frame) {
  write(prefix+"-rgb.u8", frame.rgb);
  write(prefix+"-depth.f32", frame.depth);
}
std::size_t invalid_depths(const simple_rbt::CameraFrame& frame) {
  std::size_t invalid = 0;
  for (float value : frame.depth)
    invalid += std::isnan(value) || value == -INFINITY || value < 0.f;
  return invalid;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 5 && argc != 11) {
      std::cerr << "usage: camera_oracle model.mjb state.bin pose.bin output_prefix [pw ph ew eh face fovy]\n";
      return 2;
    }
    simple_rbt::CameraConfig config;
    if (argc == 11) {
      config.perspective_width = std::stoi(argv[5]);
      config.perspective_height = std::stoi(argv[6]);
      config.panorama_width = std::stoi(argv[7]);
      config.panorama_height = std::stoi(argv[8]);
      config.cube_face_size = std::stoi(argv[9]);
      config.perspective_fov_y_deg = std::stod(argv[10]);
    }
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_loadModel(argv[1], nullptr), mj_deleteModel);
    if (!model) throw std::runtime_error("Cannot load oracle model");
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    const int state_size = mj_stateSize(model.get(), mjSTATE_INTEGRATION);
    std::vector<mjtNum> state(static_cast<std::size_t>(state_size));
    std::ifstream state_file(argv[2], std::ios::binary);
    std::uint64_t saved_count = 0;
    read_exact(state_file, &saved_count, 1);
    if (saved_count != state.size()) throw std::runtime_error("Oracle state size differs from compiled model");
    read_exact(state_file, state.data(), state.size()); end_of_file(state_file);
    mj_setState(model.get(), data.get(), state.data(), mjSTATE_INTEGRATION);
    mj_forward(model.get(), data.get());
    simple_rbt::OpticalPose pose;
    std::ifstream pose_file(argv[3], std::ios::binary);
    read_exact(pose_file, pose.position.data(), pose.position.size());
    read_exact(pose_file, pose.rotation.data(), pose.rotation.size()); end_of_file(pose_file);
    const auto original_visual = model->vis;
    const std::vector<mjtNum> qpos(data->qpos, data->qpos+model->nq), qvel(data->qvel, data->qvel+model->nv);
    const double original_time = data->time;
    simple_rbt::CameraPair first, repeated;
    std::string renderer;
    {
      simple_rbt::CameraRig camera(model.get(), config);
      camera.render_into(data.get(), pose, first);
      renderer = camera.renderer_name();
      camera.render_into(data.get(), pose, repeated);
      if (first.perspective.depth != repeated.perspective.depth || first.panorama.depth != repeated.panorama.depth)
        throw std::runtime_error("Repeated physical snapshot changed depth");
    }
    if (std::memcmp(&original_visual, &model->vis, sizeof(original_visual)) ||
        !std::equal(qpos.begin(), qpos.end(), data->qpos) || !std::equal(qvel.begin(), qvel.end(), data->qvel) ||
        original_time != data->time)
      throw std::runtime_error("Rendering changed model visuals or physical state");
    if (invalid_depths(first.perspective) || invalid_depths(first.panorama))
      throw std::runtime_error("Invalid negative or NaN depth");
    const std::string prefix(argv[4]);
    write_frame(prefix+"-perspective", first.perspective);
    write_frame(prefix+"-panorama", first.panorama);
    std::ofstream meta(prefix+"-metadata.json");
    meta << "{\n  \"renderer\": \"" << renderer << "\",\n"
         << "  \"perspective_width\": " << config.perspective_width << ",\n"
         << "  \"perspective_height\": " << config.perspective_height << ",\n"
         << "  \"panorama_width\": " << config.panorama_width << ",\n"
         << "  \"panorama_height\": " << config.panorama_height << ",\n"
         << "  \"cube_face_size\": " << config.cube_face_size << ",\n"
         << "  \"perspective_fov_y_deg\": " << config.perspective_fov_y_deg << ",\n"
         << "  \"perspective_depth_kind\": \"z\",\n  \"panorama_depth_kind\": \"range\",\n"
         << "  \"depth_dtype\": \"float32\",\n  \"no_surface\": \"positive infinity\",\n"
         << "  \"physical_state_and_model_visuals_unchanged\": true,\n"
         << "  \"repeat_depth_identical\": true\n}\n";
    if (!meta) throw std::runtime_error("Cannot write oracle metadata");
    std::cout << "camera_oracle rendered " << prefix << " with " << renderer << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "camera_oracle: " << error.what() << '\n';
    return 1;
  }
}
