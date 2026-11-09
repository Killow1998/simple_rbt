#pragma once
#include "../controller_internal.hpp"

namespace simple_rbt {
class Policy;
class SuperMimicController final : public Controller,
                                 private control_detail::JointBinding {
public:
  Json diagnostics() const override;
  explicit SuperMimicController(const ControllerContext &);
  ~SuperMimicController() override;
  void update(const Vec3 &) override;
  const std::vector<float> &observation() const { return observation_; }
  const std::vector<float> &action() const;

private:
  friend struct SuperMimicReplayAccess;
  mjModel *model_;
  mjData *data_;
  int base_, decimation_;
  bool actor_wheel_vel_ = true;
  Vec3 pending_command_{};
  std::unique_ptr<Policy> policy_;
  std::vector<float> defaults_, scales_, offsets_;
  std::vector<int> env_order_;
  std::vector<float> observation_ = std::vector<float>(265, 0.f);
};
} // namespace simple_rbt
