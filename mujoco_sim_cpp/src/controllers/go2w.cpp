#include "../controller_internal.hpp"
#include "../controller_policy.hpp"
namespace simple_rbt {
namespace {
using namespace control_detail;
class Go2WController final : public Controller, private JointBinding {
  mjModel *m;
  mjData *d;
  std::unique_ptr<Policy> policy;
  std::vector<double> default_pos, scales;
  std::vector<float> target_pos, target_vel, obs;
  int gyro_address = -1, imu_id = -1;
  double next_pd = 0;

public:
  explicit Go2WController(const ControllerContext &c) : m(c.model), d(c.data) {
    const Json &config = c.config;
    for (const auto &name : config["joint_names"])
      add(m, name.get<std::string>());
    const auto path =
        c.options.project_root / config["policy"].get<std::string>();
    if (sha256_file(path) != config["policy_sha256"])
      throw std::runtime_error("Policy SHA256 mismatch");
    policy = std::make_unique<Policy>(path, 16);
    default_pos = config["default_dof_pos"].get<std::vector<double>>();
    scales = config["action_scale"].get<std::vector<double>>();
    target_pos.assign(default_pos.begin(), default_pos.end());
    target_vel.assign(16, 0.f);
    state.policy_period = .02;
    if (qpos.size() != 16 || default_pos.size() != 16 || scales.size() != 16)
      throw std::runtime_error("Invalid Go2W controller dimensions");
    obs.reserve(57);
    gyro_address = m->sensor_adr[id(m, mjOBJ_SENSOR, "gyro")];
    imu_id = id(m, mjOBJ_SITE, "imu");
  }
  void update(const Vec3 &policy_command) override {
    int every = std::lround(.02 / m->opt.timestep);
    if (state.policy_steps % every == 0) {
      Vec3 gyro{d->sensordata[gyro_address], d->sensordata[gyro_address + 1],
                d->sensordata[gyro_address + 2]};
      const auto *rot = d->site_xmat + 9 * imu_id;
      Vec3 gravity{-rot[6], -rot[7], -rot[8]};
      obs.clear();

      for (double v : gyro)
        obs.push_back(static_cast<float>(v) * .25f);
      for (double v : gravity)
        obs.push_back(static_cast<float>(v));
      for (double v : policy_command)
        obs.push_back(static_cast<float>(v));
      for (int i = 0; i < 16; i++)
        obs.push_back(i >= 12 ? 0.f
                              : static_cast<float>(d->qpos[qpos[i]]) -
                                    static_cast<float>(default_pos[i]));
      for (int i = 0; i < 16; i++)
        obs.push_back(static_cast<float>(d->qvel[qvel[i]]) * .05f);
      obs.insert(obs.end(), policy->last.begin(), policy->last.end());
      for (float &v : obs)
        v = std::min(100.f, std::max(-100.f, v));
      policy->run_into(obs);
      for (float &v : policy->last)
        v = std::min(100.f, std::max(-100.f, v));
      for (int i = 0; i < 12; i++)
        target_pos[i] = static_cast<float>(default_pos[i]) +
                        policy->last[i] * static_cast<float>(scales[i]);
      for (int i = 12; i < 16; i++)
        target_vel[i] = policy->last[i] * static_cast<float>(scales[i]);
    }

    if (state.policy_steps * m->opt.timestep + 1e-10 >= next_pd) {
      for (int i = 0; i < 16; i++) {
        double tau = .5 * (target_vel[i] - d->qvel[qvel[i]]);
        if (i < 12)
          tau += 20. * (target_pos[i] - d->qpos[qpos[i]]);
        d->ctrl[actuators[i]] = tau;
      }
      state.pd_calls++;
      next_pd = state.pd_calls * .005;
    }

    state.policy_steps++;
  }
};
} // namespace
std::unique_ptr<Controller> make_go2w_controller(const ControllerContext &c) {
  return std::make_unique<Go2WController>(c);
}
} // namespace simple_rbt
