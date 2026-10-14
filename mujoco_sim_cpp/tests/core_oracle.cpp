// Offline comparison harness. The simulator implementation never starts Python.
#include "simple_rbt/core.hpp"
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
namespace fs = std::filesystem;
using simple_rbt::Json;
using simple_rbt::Vec3;
template<class T> void write(std::ofstream& stream, const T* data, std::size_t count) {
  if (!stream.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(sizeof(T)*count)))
    throw std::runtime_error("Cannot write oracle output");
}
}
int main(int argc, char** argv) {
  try {
    if (argc != 3) {
      std::cerr << "usage: core_oracle case.json output_directory\n";
      return 2;
    }
    Json test; std::ifstream input(argv[1]); input >> test;
    simple_rbt::SimulationOptions options;
    const auto& source = test.at("options");
    options.project_root = source.at("project_root").get<std::string>();
    options.robot = source.at("robot");
    options.controller = source.value("controller", "auto");
    options.terrain = source.value("terrain", "flat");
    options.terrain_level = source.value("terrain_level", "standard");
    options.control_mode = source.value("control_mode", "native");
    options.go2_torque_mode = source.value("go2_torque_mode", "configured");
    options.go2_startup = source.value("go2_startup", "crouched");
    options.difficulty = source.value("difficulty", 1.);
    options.seed = source.value("seed", std::uint64_t{0});
    options.friction = source.value("friction", -1.);
    simple_rbt::Simulation sim(options);
    const auto* model = sim.model();
    auto* data = sim.data();
    fs::path output(argv[2]); fs::create_directories(output);
    mj_saveModel(model, (output/"native-model.mjb").c_str(), nullptr, 0);
    Json rejected = Json::array();
    for (const auto& command : test.value("reject_commands", Json::array())) {
      sim.set_command({.2, 0., 0.});
      bool caught = false;
      try { sim.set_command(command.get<Vec3>()); }
      catch (const std::invalid_argument&) { caught = true; }
      rejected.push_back({{"requested", command}, {"rejected", caught}, {"remaining_command", sim.command()}});
      if (!caught || sim.command() != Vec3{}) throw std::runtime_error("Unsupported command did not safely stop");
    }
    sim.set_command({});
    std::uint64_t rows = 1;
    for (const auto& segment : test.at("segments")) rows += segment.at("ticks").get<std::uint64_t>();
    const std::uint64_t header[] = {rows, static_cast<std::uint64_t>(model->nq),
        static_cast<std::uint64_t>(model->nv), static_cast<std::uint64_t>(model->nu),
        static_cast<std::uint64_t>(model->nsensordata)};
    std::ofstream binary(output/"native-trajectory.f64", std::ios::binary);
    write(binary, header, 5);
    Json stamps = Json::array(), startup = Json::array(), commands = Json::array();
    auto capture = [&] {
      write(binary, data->qpos, model->nq); write(binary, data->qvel, model->nv);
      write(binary, data->ctrl, model->nu); write(binary, data->sensordata, model->nsensordata);
      stamps.push_back(sim.stamp_ns()); commands.push_back(sim.command());
      startup.push_back({sim.startup_complete(), sim.startup_phase(), sim.startup_steps()});
    };
    capture();
    for (const auto& segment : test.at("segments")) {
      sim.set_command(segment.at("command").get<Vec3>());
      for (std::uint64_t tick=0; tick<segment.at("ticks").get<std::uint64_t>(); ++tick) {
        sim.step(); capture();
      }
    }
    Json metadata{{"schema",1}, {"rows",rows}, {"columns",{"qpos","qvel","ctrl","sensordata"}},
      {"dimensions",{model->nq,model->nv,model->nu,model->nsensordata}},
      {"dtype","native little-endian IEEE float64; five uint64 header values"},
      {"physics_dt",model->opt.timestep}, {"controller",sim.controller_name()},
      {"stamp_ns",stamps}, {"startup",startup}, {"command",commands},
      {"rejected_commands",rejected}, {"command_limits",sim.command_limits()},
      {"deployment",sim.deployment_metadata()}, {"tracker",sim.tracker_metadata()},
      {"terrain",sim.terrain_metadata()}};
    std::ofstream meta(output/"native-metadata.json"); meta << metadata.dump(2) << '\n';
    if (!meta) throw std::runtime_error("Cannot write metadata");
    std::cout << options.robot << '/' << sim.controller_name() << ": " << rows << " snapshots\n";
  } catch (const std::exception& error) {
    std::cerr << "core_oracle: " << error.what() << '\n'; return 1;
  }
}
