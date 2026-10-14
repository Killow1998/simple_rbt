#pragma once
#include "simple_rbt/core.hpp"
namespace simple_rbt {
struct ControlState {
  bool startup_done = true;
  std::string phase = "policy";
  std::uint64_t policy_steps = 0, pd_calls = 0, startup_count = 0;
  double policy_period = 0.;
};
class Controller {
public:
  virtual ~Controller() = default;
  virtual void update(const Vec3 &command) = 0;
  ControlState state;
};
struct ControllerContext {
  const SimulationOptions &options;
  const std::string &name;
  const Json &config;
  const Json &catalog;
  mjModel *model;
  mjData *data;
};
std::unique_ptr<Controller> make_controller(const ControllerContext &);
} // namespace simple_rbt
