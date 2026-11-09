#include "supermimic.hpp"
#include "../controller_policy.hpp"
#include <array>

namespace simple_rbt {
SuperMimicController::SuperMimicController(const ControllerContext &c)
    : model_(c.model), data_(c.data) {
  const auto &config = c.config;
  actor_wheel_vel_ = config.value("actor_wheel_vel", true);
  observation_.assign(actor_wheel_vel_ ? 265 : 245, 0.f);
  const auto names = config.at("joint_names").get<std::vector<std::string>>();
  defaults_ = config.at("default_dof_pos").get<std::vector<float>>();
  scales_ = config.at("action_scale").get<std::vector<float>>();
  offsets_ = config.at("action_offset").get<std::vector<float>>();
  env_order_ = config.at("env_action_order").get<std::vector<int>>();
  if (names.size() != 16 || defaults_.size() != 16 || scales_.size() != 16 ||
      offsets_.size() != 16 || env_order_.size() != 16)
    throw std::invalid_argument("supermimic requires 16 canonical joints/actions");
  std::array<bool, 16> seen{};
  for (int i : env_order_) {
    if (i < 0 || i >= 16 || seen[i])
      throw std::invalid_argument("env_action_order must be a permutation of 0..15");
    seen[i] = true;
  }
  for (const auto &name : names) add(model_, name);
  base_ = control_detail::id(model_, mjOBJ_BODY,
                            config.at("base_body").get<std::string>());
  state.policy_period = .02;
  state.command_observation_delay_s = state.policy_period;
  decimation_ = std::lround(state.policy_period / model_->opt.timestep);
  if (decimation_ < 1 ||
      std::abs(decimation_ * model_->opt.timestep - state.policy_period) > 1e-9)
    throw std::invalid_argument("supermimic physics timestep must divide 20 ms");
  const auto path = c.options.project_root / config.at("policy").get<std::string>();
  if (sha256_file(path) != config.at("policy_sha256"))
    throw std::runtime_error("Policy SHA256 mismatch");
  policy_ = std::make_unique<Policy>(path, 16);
}
SuperMimicController::~SuperMimicController() = default;
Json SuperMimicController::diagnostics() const {
  return {{"observation", observation_}, {"action", policy_->last}};
}
const std::vector<float> &SuperMimicController::action() const {
  return policy_->last;
}
void SuperMimicController::update(const Vec3 &command) {
  if (state.policy_steps % decimation_ == 0) {
    state.observation_command = pending_command_;
    pending_command_ = command;
    std::array<float, 53> frame{};
    const int action_start = actor_wheel_vel_ ? 34 : 30;
    const int command_start = action_start + 16;
    mjtNum velocity[6];
    // XBODY selects the link frame; BODY would use the inertial principal axes.
    mj_objectVelocity(model_, data_, mjOBJ_XBODY, base_, velocity, 1);
    const auto *rotation = data_->xmat + 9 * base_;
    for (int i = 0; i < 3; ++i) {
      frame[i] = static_cast<float>(velocity[i]);
      frame[3 + i] = static_cast<float>(-rotation[6 + i]);
      frame[command_start + i] = static_cast<float>(state.observation_command[i]);
    }
    for (int i = 0; i < 12; ++i)
      frame[6 + i] = static_cast<float>(data_->qpos[qpos[i]]) - defaults_[i];
    for (int i = 0; i < 16; ++i) {
      if (actor_wheel_vel_ || i < 12)
        frame[18 + i] = static_cast<float>(data_->qvel[qvel[i]]) * .05f;
      frame[action_start + i] = policy_->last[env_order_[i]];
    }
    // Each term stores five chronological frames; reset repeats the first frame.
    int h = 0, f = 0;
    const std::array<int, 7> widths{3, 3, 12, 12,
                                    actor_wheel_vel_ ? 4 : 0, 16, 3};
    for (int width : widths) {
      auto begin = observation_.begin() + h;
      if (state.policy_steps == 0) {
        for (int age = 0; age < 5; ++age)
          std::copy_n(frame.begin() + f, width, begin + age * width);
      } else {
        std::move(begin + width, begin + 5 * width, begin);
        std::copy_n(frame.begin() + f, width, begin + 4 * width);
      }
      h += 5 * width;
      f += width;
    }
    policy_->run_into(observation_);
    // Original position/velocity actuators own gains, limits and armature.
    for (int i = 0; i < 16; ++i)
      data_->ctrl[actuators[i]] = offsets_[i] + scales_[i] * policy_->last[i];
  }
  ++state.policy_steps;
}
std::unique_ptr<Controller> make_supermimic_controller(const ControllerContext &c) {
  return std::make_unique<SuperMimicController>(c);
}
} // namespace simple_rbt
