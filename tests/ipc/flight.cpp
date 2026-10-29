#include "simple_rbt/core.hpp"
#include "simple_rbt/ipc.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <unistd.h>
using namespace simple_rbt;
using namespace simple_rbt::ipc;
namespace {
void wait() { std::this_thread::sleep_for(std::chrono::milliseconds(25)); }
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
} // namespace
int main() {
  try {
    const auto dir = std::filesystem::temp_directory_path() /
                     ("srbt-flight-ipc-" + std::to_string(getpid()));
    TransportOptions options;
    options.robot_name = "skydio_x2";
    options.command_timeout_s = .15;
    SessionServer server(dir, options, {});
    server.set_running(true);
    server.set_source("gui");
    Client gui, program;
    check(gui.connect(dir, "gui", {"metadata"}), "GUI connection");
    check(program.connect(dir, "program", {}), "program connection");
    wait();
    FlightCommand takeoff;
    takeoff.action = "takeoff";
    gui.send_packet(encode_flight_command(takeoff));
    wait();
    SimulationOptions setup;
    setup.robot = "skydio_x2";
    setup.project_root = SIMPLE_RBT_PROJECT_ROOT;
    Simulation sim(setup);
    auto advance = [&](double seconds) {
      for (int i = 0; i < seconds / sim.model()->opt.timestep; ++i) {
        sim.set_flight(server.flight_command());
        sim.step();
      }
    };
    advance(8);
    check(sim.pose().position[2] > .9, "IPC takeoff");
    FlightCommand move;
    move.action = "velocity";
    move.velocity = {.5, 0, 0};
    gui.send_packet(encode_flight_command(move));
    wait();
    advance(3);
    std::this_thread::sleep_for(std::chrono::milliseconds(180));
    check(server.flight_command().action == "hold",
          "timeout did not request hold");
    advance(4);
    const auto held = sim.pose().position;
    advance(4);
    check(std::abs(sim.pose().position[0] - held[0]) < .05,
          "timeout did not brake to hover");
    server.set_source("program");
    program.send_packet(encode_flight_command(move));
    wait();
    FlightCommand cut;
    cut.action = "disarm";
    gui.send_packet(encode_flight_command(cut));
    wait();
    check(server.flight_command().action == "velocity",
          "GUI focus/motor cut overrode program owner");
    program.close();
    wait();
    check(server.flight_command().action == "hold", "disconnect did not hold");
    server.set_source("gui");
    gui.send_packet(json_packet(
        Kind::FlightCommand,
        {{"frame", "body"}, {"action", "velocity"}, {"velocity", {1, 0, 0}}}));
    wait();
    bool rejected = false;
    for (const auto &p : gui.poll())
      rejected |= p.kind == Kind::Error;
    check(rejected && server.flight_command().action == "hold",
          "ambiguous frame was accepted");
    for (int variant = 0; variant < 2; ++variant) {
      auto invalid = decode_json(encode_flight_command(move));
      if (variant == 0)
        invalid["roll_rate"] = .1;
      else
        invalid["velocity"] = {1, 0, 0, .1, 0, 0};
      gui.send_packet(json_packet(Kind::FlightCommand, invalid));
      wait();
      bool refused = false;
      for (const auto &packet : gui.poll())
        refused |= packet.kind == Kind::Error;
      check(refused && server.flight_command().action == "hold",
            "unsupported flight axis was ignored");
    }
    gui.send_command({1, 0, 0});
    wait();
    check(server.flight_command().action == "hold",
          "legacy planar command entered flight");
    server.set_running(false);
    check(server.command_status()["state"] == "paused", "pause state missing");
    gui.close();
    server.close();
    std::filesystem::remove_all(dir);
    std::cout << "flight IPC: takeoff, expiry hover, owner isolation, "
                 "disconnect and frame rejection passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
