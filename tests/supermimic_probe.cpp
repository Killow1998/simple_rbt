#include "../src/controllers/supermimic.hpp"
#include "../src/controller_policy.hpp"
#include <chrono>
#include <fstream>
#include <iostream>

using namespace simple_rbt;
namespace simple_rbt {
struct SuperMimicReplayAccess {
  static void restore_command(SuperMimicController &controller, const Vec3 &command) {
    controller.pending_command_ = command;
  }
  static void restore_action(SuperMimicController &controller, const Json &observation) {
    // Test-only restoration of actor state; retain ORT's bound output buffer.
    for (int i = 0; i < 16; ++i)
      controller.policy_->last[controller.env_order_[i]] = observation.at((controller.actor_wheel_vel_ ? 234 : 214) + i).get<float>();
  }
};
} // namespace simple_rbt
namespace {
Json read(const std::filesystem::path &path) {
  std::ifstream file(path);
  return Json::parse(file);
}
double error(const std::vector<float> &actual, const Json &expected) {
  if (actual.size() != expected.size())
    throw std::invalid_argument("Fixture dimension mismatch");
  double result = 0.;
  for (std::size_t i = 0; i < actual.size(); ++i)
    result = std::max(result, std::abs(actual[i] - expected[i].get<double>()));
  return result;
}
} // namespace
int main(int argc, char **argv) {
  if (argc < 5 || argc > 6 || (argc == 6 && std::string(argv[5]) != "--complete-state")) {
    std::cerr << "Usage: supermimic_probe MODEL_XML CONTROLLER_JSON FRAMES_JSON REPORT_JSON [--complete-state]\n";
    return 2;
  }
  try {
    char message[2048]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(argv[1], nullptr, message, sizeof(message)), mj_deleteModel);
    if (!model) throw std::runtime_error(message);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        mj_makeData(model.get()), mj_deleteData);
    const auto config = read(argv[2]), frames = read(argv[3]);
    SimulationOptions options;
    options.robot = "go2w";
    options.control_mode = "native";
    options.project_root = std::filesystem::absolute(argv[2]).parent_path();
    const std::string name = "supermimic";
    const Json catalog = Json::object();
    std::unique_ptr<SuperMimicController> controller;
    Json rows = Json::array();
    double observation_error = 0., action_error = 0., normalized_error = 0.;
    for (const auto &frame : frames) {
      if (!controller || frame.value("reset", false)) {
        mj_resetData(model.get(), data.get());
        controller = std::make_unique<SuperMimicController>(
            ControllerContext{options, name, config, catalog, model.get(), data.get()});
      }
      const auto q = frame.at("qpos").get<std::vector<double>>();
      const auto v = frame.at("qvel").get<std::vector<double>>();
      if (q.size() != std::size_t(model->nq) || v.size() != std::size_t(model->nv))
        throw std::invalid_argument("Fixture state does not match model");
      std::copy(q.begin(), q.end(), data->qpos);
      std::copy(v.begin(), v.end(), data->qvel);
      mj_forward(model.get(), data.get());
      const auto command = frame.at("command").get<Vec3>();
      // Fixtures already contain the consumed command, not the incoming request.
      SuperMimicReplayAccess::restore_command(*controller, command);
      if (argc == 6) SuperMimicReplayAccess::restore_action(*controller, frame.at("observation"));
      controller->update(command);
      const double oe = error(controller->observation(), frame.at("observation"));
      const double ae = error(controller->action(), frame.at("action"));
      double magnitude = 0.;
      for (double value : frame.at("action")) magnitude = std::max(magnitude, std::abs(value));
      const double ne = ae / (1. + magnitude);
      observation_error = std::max(observation_error, oe);
      action_error = std::max(action_error, ae);
      normalized_error = std::max(normalized_error, ne);
      rows.push_back({{"observation_max_error", oe}, {"action_max_error", ae},
                      {"action_normalized_linf", ne}, {"action", controller->action()},
                      {"observation", controller->observation()},
                      {"ctrl", std::vector<double>(data->ctrl, data->ctrl + model->nu)}});
      // Advance the controller clock without evolving the independently saved state.
      for (int i = 1; i < std::lround(.02 / model->opt.timestep); ++i)
        controller->update(command);
    }
    const bool passed = !frames.empty() && observation_error <= 1e-5 && normalized_error <= 2e-6;
    Policy inference(options.project_root / config.at("policy").get<std::string>(), 16);
    std::vector<float> input(config.value("actor_wheel_vel", true) ? 265 : 245);
    std::vector<double> timing;
    for (const auto &frame : frames) {
      for (std::size_t i = 0; i < input.size(); ++i) input[i] = frame.at("observation").at(i);
      if (timing.empty()) for (int i = 0; i < 20; ++i) inference.run_into(input);
      const auto start = std::chrono::steady_clock::now();
      inference.run_into(input);
      timing.push_back(std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - start).count());
    }
    std::sort(timing.begin(), timing.end());
    const Json report{{"passed", passed}, {"frames", frames.size()},
                      {"replay_mode", argc == 6 ? "complete_state" : "recursive_action_history"},
                      {"mujoco", mj_versionString()},
                      {"onnxruntime", OrtGetApiBase()->GetVersionString()},
                      {"inference_cpu", {{"threads", 1}, {"samples", timing.size()},
                        {"median_us", timing.at(timing.size() / 2)},
                        {"p95_us", timing.at(std::size_t(.95 * (timing.size() - 1)))}}},
                      {"observation_max_error", observation_error},
                      {"action_max_error", action_error},
                      {"action_normalized_linf", normalized_error}, {"rows", rows}};
    std::ofstream(argv[4]) << report.dump(2) << '\n';
    auto summary = report; summary.erase("rows");
    std::cout << summary.dump(2) << '\n';
    return passed ? 0 : 1;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
