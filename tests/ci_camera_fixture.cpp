// Independent analytic fixture for the existing camera_oracle binary interface.
#include <mujoco/mujoco.h>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("Usage: ci-camera-fixture OUTPUT_DIRECTORY");
    const std::filesystem::path output(argv[1]);
    std::filesystem::create_directories(output);
    std::ofstream xml(output / "model.xml");
    xml << R"(<mujoco model="ci-camera-analytic-wall">
      <visual><global offwidth="1024" offheight="1024"/>
        <map znear=".005" zfar="50"/></visual>
      <worldbody><light pos="0 0 4" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="5 5 .1" rgba=".2 .5 .2 1"/>
        <geom name="wall" type="box" pos="2.1 0 .5" size=".1 2 2" rgba=".8 .1 .1 1"/>
      </worldbody></mujoco>)";
    xml.close();
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_loadXML((output / "model.xml").c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) throw std::runtime_error(error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    mj_forward(model.get(), data.get());
    mj_saveModel(model.get(), (output / "model.mjb").c_str(), nullptr, 0);
    const std::uint64_t size = mj_stateSize(model.get(), mjSTATE_INTEGRATION);
    std::vector<mjtNum> state(size);
    mj_getState(model.get(), data.get(), state.data(), mjSTATE_INTEGRATION);
    std::ofstream snapshot(output / "state.bin", std::ios::binary);
    snapshot.write(reinterpret_cast<const char*>(&size), sizeof(size));
    snapshot.write(reinterpret_cast<const char*>(state.data()), state.size()*sizeof(mjtNum));
    // Optical forward +X, right -Y, down -Z. The wall's front is exactly X=2m.
    const std::array<double, 12> pose{0,0,.4, 0,0,1, -1,0,0, 0,-1,0};
    std::ofstream optical(output / "pose.bin", std::ios::binary);
    optical.write(reinterpret_cast<const char*>(pose.data()), sizeof(pose));
    if (!snapshot || !optical) throw std::runtime_error("Cannot write fixture");
    std::cout << "Independent analytic camera fixture: center-wall depth 2m\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
