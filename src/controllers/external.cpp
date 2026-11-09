#include "../controller.hpp"
#include "simple_rbt/controller_plugin.h"
#include <cmath>
#include <dlfcn.h>

namespace simple_rbt {
namespace {
class ExternalController final : public Controller {
  std::unique_ptr<void, int (*)(void *)> library_{nullptr, dlclose};
  SimpleRbtControllerV1 plugin_{};
  mjModel *model_;
  mjData *data_;
  int decimation_;

public:
  explicit ExternalController(const ControllerContext &c)
      : model_(c.model), data_(c.data) {
    if (c.options.control_mode != "native")
      throw std::invalid_argument("External controllers require native control mode");
    state.policy_period = c.config.value("period_s", .02);
    if (!std::isfinite(state.policy_period) || state.policy_period <= 0.)
      throw std::invalid_argument("External controller period must be positive");
    decimation_ = std::lround(state.policy_period / model_->opt.timestep);
    if (decimation_ < 1 ||
        std::abs(decimation_ * model_->opt.timestep - state.policy_period) > 1e-9)
      throw std::invalid_argument("External controller period must be a multiple of the physics timestep");
    const auto path = c.options.project_root / c.config.at("library").get<std::string>();
    library_.reset(dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL));
    if (!library_)
      throw std::runtime_error("Cannot load controller plugin " + path.string() + ": " + dlerror());
    auto create = reinterpret_cast<SimpleRbtControllerCreateV1 *>(
        dlsym(library_.get(), "simple_rbt_controller_create_v1"));
    if (!create)
      throw std::runtime_error("Controller plugin entry missing: " + std::string(dlerror()));
    const auto parameters = c.config.value("config", Json::object()).dump();
    char error[1024]{};
    if (create(mj_version(), model_, parameters.c_str(), &plugin_, error, sizeof(error)))
      throw std::runtime_error("Controller plugin creation failed: " + std::string(error));
    if (!plugin_.update) {
      if (plugin_.destroy) plugin_.destroy(plugin_.context);
      throw std::runtime_error("Controller plugin has no update callback");
    }
  }
  ~ExternalController() override {
    if (plugin_.destroy) plugin_.destroy(plugin_.context);
  }
  void update(const Vec3 &command) override {
    if (state.policy_steps % decimation_ == 0) {
      char error[1024]{};
      if (plugin_.update(plugin_.context, model_, data_, command.data(), data_->ctrl,
                         error, sizeof(error)))
        throw std::runtime_error("Controller plugin update failed: " + std::string(error));
      for (int i = 0; i < model_->nu; ++i)
        if (!std::isfinite(data_->ctrl[i]))
          throw std::runtime_error("Controller plugin returned a nonfinite actuator control");
      state.observation_command = command;
    }
    ++state.policy_steps;
  }
  void after_step() override {
    if (plugin_.after_step) {
      char error[1024]{};
      if (plugin_.after_step(plugin_.context, model_, data_, error, sizeof(error)))
        throw std::runtime_error("Controller plugin recorder failed: " + std::string(error));
    }
  }
};
} // namespace
std::unique_ptr<Controller> make_external_controller(const ControllerContext &c) {
  return std::make_unique<ExternalController>(c);
}
} // namespace simple_rbt
