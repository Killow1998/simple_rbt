#include "../controller.hpp"
#include <algorithm>
#include <cmath>
#include <set>

namespace simple_rbt {
namespace {
class ArmController final : public Controller {
  struct Axis {
    std::string name;
    int joint, actuator;
    double minimum, maximum, speed, scale, target, reference, home;
  };
  mjModel *model_;
  mjData *data_;
  std::vector<Axis> axes_;
  std::vector<int> observed_;
  bool holding_ = true;

public:
  explicit ArmController(const ControllerContext &c)
      : model_(c.model), data_(c.data) {
    for (const auto &entry : c.config.at("axes")) {
      const std::string name = entry.at("joint");
      const std::string actuator = entry.at("actuator");
      int joint = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
      int motor = mj_name2id(model_, mjOBJ_ACTUATOR, actuator.c_str());
      if (joint < 0 || motor < 0 || !model_->jnt_limited[joint])
        throw std::invalid_argument(
            "Arm axis must have a bounded joint and actuator");
      const double q = data_->qpos[model_->jnt_qposadr[joint]];
      axes_.push_back({name, joint, motor, model_->jnt_range[2 * joint],
                       model_->jnt_range[2 * joint + 1], entry.at("speed"),
                       entry.value("scale", 1.), q, q, q});
    }
    for (const auto &name : c.config.at("state_joints")) {
      int joint =
          mj_name2id(model_, mjOBJ_JOINT, name.get<std::string>().c_str());
      if (joint < 0)
        throw std::invalid_argument("Missing observed arm joint");
      observed_.push_back(joint);
    }
  }
  void set_joints(const JointCommand &command) override {
    if (command.hold) {
      hold_joints();
      return;
    }
    if (command.name.empty() || command.name.size() != command.position.size())
      throw std::invalid_argument(
          "Joint command requires matching names and positions");
    std::set<std::string> seen;
    for (std::size_t i = 0; i < command.name.size(); ++i) {
      auto axis = std::find_if(axes_.begin(), axes_.end(), [&](const Axis &a) {
        return a.name == command.name[i];
      });
      const double q = command.position[i];
      if (axis == axes_.end() || !seen.insert(command.name[i]).second ||
          !std::isfinite(q) || q < axis->minimum || q > axis->maximum)
        throw std::invalid_argument(
            "Unknown, repeated or out-of-range arm joint");
    }
    for (std::size_t i = 0; i < command.name.size(); ++i)
      for (auto &axis : axes_)
        if (axis.name == command.name[i])
          axis.target = command.position[i];
    holding_ = false;
  }
  void hold_joints() override {
    if (holding_)
      return;
    for (auto &a : axes_)
      a.target = a.reference = std::clamp(
          data_->qpos[model_->jnt_qposadr[a.joint]], a.minimum, a.maximum);
    holding_ = true;
  }
  void update(const Vec3 &) override {
    for (auto &a : axes_) {
      const double change = a.speed * model_->opt.timestep;
      a.reference += std::clamp(a.target - a.reference, -change, change);
      data_->ctrl[a.actuator] = a.reference * a.scale;
    }
    ++state.pd_calls;
  }
  JointState joints() const override {
    JointState out;
    for (int id : observed_) {
      const int dof = model_->jnt_dofadr[id];
      out.name.emplace_back(mj_id2name(model_, mjOBJ_JOINT, id));
      out.position.push_back(data_->qpos[model_->jnt_qposadr[id]]);
      out.velocity.push_back(data_->qvel[dof]);
      out.effort.push_back(data_->qfrc_actuator[dof]);
    }
    return out;
  }
  Json joint_controls() const override {
    Json out = Json::array();
    for (const auto &a : axes_)
      out.push_back({{"name", a.name},
                     {"minimum", a.minimum},
                     {"maximum", a.maximum},
                     {"target", a.target},
                     {"home", a.home},
                     {"speed", a.speed},
                     {"linear", model_->jnt_type[a.joint] == mjJNT_SLIDE}});
    return out;
  }
};
} // namespace
std::unique_ptr<Controller> make_arm_controller(const ControllerContext &c) {
  return std::make_unique<ArmController>(c);
}
} // namespace simple_rbt
