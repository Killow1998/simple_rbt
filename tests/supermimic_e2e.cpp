#include "simple_rbt/core.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

using namespace simple_rbt;
int supermimic_case(const char *, const char *, const char *, const char *, bool);
int main(int argc, char **argv) {
  if (argc == 6 && (std::string(argv[1]) == "--case" || std::string(argv[1]) == "--inspect"))
    return supermimic_case(argv[2], argv[3], argv[4], argv[5], std::string(argv[1]) == "--inspect");
  if (argc < 3 || argc > 4 || (argc == 4 && std::string(argv[3]) != "--courses")) {
    std::cerr << "Usage: supermimic_e2e ASSET_ROOTS NEW_OUTPUT_DIRECTORY [--courses]\n"
                 "       supermimic_e2e --case|--inspect MANIFEST CASE_ID ASSET_ROOT NEW_OUTPUT\n";
    return 2;
  }
  const std::filesystem::path output = argv[2];
  if (!std::filesystem::create_directory(output)) return 2;
  const bool courses = argc == 4;
  struct Case { const char *name; Vec3 command; const char *scene; bool course = false; };
  const Case cases[]{
      {"zero", {}, "flat"}, {"forward", {.5, 0, 0}, "flat"},
      {"reverse", {-.3, 0, 0}, "flat"},
      {"left", {0, .2, 0}, "flat"}, {"right", {0, -.2, 0}, "flat"},
      {"yaw_left", {0, 0, .6}, "flat"}, {"yaw_right", {0, 0, -.6}, "flat"},
      {"small_forward", {.05, 0, 0}, "flat"},
      {"slope_plus", {}, "slope_plus"}, {"slope_minus", {}, "slope_minus"},
      {"terrain_up", {.5, 0, 0}, "terrain_up", true},
      {"terrain_down", {.5, 0, 0}, "terrain_down", true},
      {"terrain_slope", {.5, 0, 0}, "terrain_slope", true},
      {"terrain_rubble", {.5, 0, 0}, "terrain_rubble", true}};
  Json results = Json::array();
  bool all_passed = true;
  for (const auto &test : cases) for (double heading : {0., -.2, .2}) {
    if (test.course != courses || (test.course && heading != 0.)) continue;
    const auto name = std::string(test.name) + "_" + std::to_string(heading);
    Json row{{"case", name}, {"command", test.command}, {"heading_rad", heading}};
    try {
      SimulationOptions options;
      options.robot = "go2w"; options.controller = "supermimic";
      options.control_mode = "native";
      options.project_root = std::filesystem::absolute(argv[1]) / test.scene;
      std::ifstream(options.project_root / "validation.scene.json") >> options.scene;
      Simulation sim(options);
      auto *model = sim.model(); auto *data = sim.data();
      data->qpos[3] = std::cos(heading / 2);
      data->qpos[6] = std::sin(heading / 2);
      mj_forward(model, data);
      if (heading == 0.) std::ofstream(output / (std::string(test.scene) + "-model.xml")) << sim.xml();
      std::ofstream trace(output / (name + ".csv"));
      trace << "time,x,y,z,roll,pitch,vx,vy,wz\n";
      trace.precision(17);
      Vec3 error{}, velocity_sum{}, at_eight{};
      double min_height = 1e9, max_tilt = 0, stop_speed = 0, stop_yaw = 0;
      int command_samples = 0, stop_samples = 0;
      double finish_time = -1.;
      const int steps = std::lround((test.course ? 20. : 12.) / model->opt.timestep);
      for (int step = 0; step < steps; ++step) {
        const double t = step * model->opt.timestep;
        if (test.course && finish_time < 0. && sim.pose().position[0] >= 5.4) finish_time = t;
        sim.set_command(t >= 2. && (test.course ? finish_time < 0. : t < 8.) ? test.command : Vec3{});
        sim.step();
        const auto pose = sim.pose();
        const auto v = sim.linear_velocity(), w = sim.angular_velocity();
        mjtNum r[9]; mju_quat2Mat(r, pose.quaternion.data());
        const double roll = std::atan2(r[7], r[8]);
        const double pitch = std::asin(std::clamp(-r[6], -1., 1.));
        min_height = std::min(min_height, pose.position[2]);
        max_tilt = std::max(max_tilt, std::max(std::abs(roll), std::abs(pitch)));
        if (t >= 6. && t < 8.) {
          const Vec3 actual{v[0], v[1], w[2]};
          for (int i = 0; i < 3; ++i) {
            error[i] += std::abs(actual[i] - test.command[i]);
            velocity_sum[i] += actual[i];
          }
          ++command_samples;
        }
        if (step + 1 == std::lround(8. / model->opt.timestep)) at_eight = pose.position;
        if (test.course ? finish_time >= 0. && t >= finish_time + 1. : t >= 10.) {
          stop_speed += std::hypot(v[0], v[1]); stop_yaw += std::abs(w[2]); ++stop_samples;
        }
        if ((step + 1) % std::lround(.02 / model->opt.timestep) == 0)
          trace << data->time << ',' << pose.position[0] << ',' << pose.position[1] << ','
                << pose.position[2] << ',' << roll << ',' << pitch << ','
                << v[0] << ',' << v[1] << ',' << w[2] << '\n';
        if (test.course && finish_time >= 0. && data->time >= finish_time + 2.) break;
      }
      for (int i = 0; i < 3; ++i) { error[i] /= command_samples; velocity_sum[i] /= command_samples; }
      if (stop_samples) { stop_speed /= stop_samples; stop_yaw /= stop_samples; }
      const auto end = sim.pose().position;
      const double drift = std::hypot(end[0] - at_eight[0], end[1] - at_eight[1]);
      const bool slope = std::string(test.scene).rfind("slope_", 0) == 0;
      const bool safe = max_tilt <= .8 && (slope || min_height >= .15);
      const bool tracking = error[0] <= .15 && error[1] <= .15 && error[2] <= .25;
      const bool small = std::string(test.name) != "small_forward" || velocity_sum[0] >= .02;
      const bool stopped = stop_samples > 0 && stop_speed <= .05 && stop_yaw <= .10;
      row.update({{"passed", safe && (test.course ? finish_time >= 0. : tracking && small) && stopped && (!slope || drift <= .10)},
                  {"finish_time_s", finish_time}, {"simulation_seconds", data->time},
                  {"end_position", end}, {"stop_measurement_seconds", test.course ? 1 : 2},
                  {"safe", safe}, {"mean_absolute_tracking_error", error},
                  {"mean_command_segment_velocity", velocity_sum},
                  {"stop_planar_speed", stop_speed}, {"stop_abs_yaw_rate", stop_yaw},
                  {"min_height", min_height},
                  {"max_roll_pitch", max_tilt}, {"deployment", sim.deployment_metadata()},
                  {"tracker", sim.tracker_metadata()}});
      if (!test.course) row["last_four_seconds_planar_drift"] = drift;
    } catch (const std::exception &e) {
      row.update({{"passed", false}, {"error", e.what()}});
    }
    all_passed = all_passed && row.at("passed").get<bool>();
    results.push_back(row);
    std::cout << name << ": " << row.at("passed") << '\n';
    std::ofstream(output / "report.json") << Json{{"passed", all_passed}, {"mujoco", mj_versionString()}, {"cases", results}}.dump(2) << '\n';
  }
  return all_passed ? 0 : 1;
}
