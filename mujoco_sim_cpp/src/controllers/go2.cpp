#include "../controller_internal.hpp"
#include "../controller_policy.hpp"
#include <array>
namespace simple_rbt {
namespace {
using namespace control_detail;
namespace fs = std::filesystem;
class Go2Controller final : public Controller, private JointBinding {
  enum class Observation {
    Commands,
    AngularVelocity,
    Gravity,
    JointPosition,
    JointVelocity,
    Actions
  };
  bool legacy, configured_torque;
  std::vector<Observation> observation_terms;
  std::vector<int> history_indices;
  Vec3 commands_scale{};
  double angular_scale = 0, position_scale = 0, velocity_scale = 0;
  float observation_bound = 0;
  std::vector<double> lower_actions, upper_actions;
  std::vector<float> obs, input, history;
  int gyro_address = -1, imu_id = -1, linear_address = -1;
  mjModel *m;
  mjData *d;
  std::unique_ptr<Policy> policy;
  std::vector<int> sdkq, sdkv, sdka;
  std::vector<double> default_pos, scales, kp, kd, torque_limits;
  std::vector<float> start_pos, getup_target;
  float pre_percent = 0, stand_percent = 0;

public:
  explicit Go2Controller(const ControllerContext &c)
      : legacy(c.name == "legacy"),
        configured_torque(c.options.go2_torque_mode == "configured"),
        m(c.model), d(c.data) {
    const Json &config = c.config;
    const Json &catalog = c.catalog;
    gyro_address = m->sensor_adr[id(m, mjOBJ_SENSOR, "gyro")];
    imu_id = id(m, mjOBJ_SITE, "imu");
    if (legacy)
      linear_address = m->sensor_adr[id(m, mjOBJ_SENSOR, "local_linvel")];
    obs.reserve(48);
    state.policy_period = .02;

    if (!legacy)
      for (auto &name : config["joint_names"])
        add(m, name.get<std::string>());
    else
      for (int j = 1; j < m->njnt; j++)
        add(m, mj_id2name(m, mjOBJ_JOINT, j));
    std::size_t n = 12;
    fs::path path =
        c.options.project_root / config["policy"].get<std::string>();
    if (sha256_file(path) != config["policy_sha256"])
      throw std::runtime_error("Policy SHA256 mismatch");
    policy = std::make_unique<Policy>(
        path, n, legacy ? "continuous_actions" : "actions");
    if (legacy)
      default_pos.assign(m->key_qpos + 7, m->key_qpos + 19);
    else
      default_pos = config["default_dof_pos"].get<std::vector<double>>();
    if (!legacy) {
      scales = config["action_scale"].get<std::vector<double>>();
      kp = config["rl_kp"].get<std::vector<double>>();
      kd = config["rl_kd"].get<std::vector<double>>();
      torque_limits = config["torque_limits"].get<std::vector<double>>();
      history_indices =
          config.at("observations_history").get<std::vector<int>>();
      if (history_indices.empty())
        history_indices.push_back(0);
      for (int index : history_indices)
        if (index < 0)
          throw std::runtime_error("Negative observation history index");
      const std::size_t nh =
          static_cast<std::size_t>(*std::max_element(history_indices.begin(),
                                                     history_indices.end())) +
          1;
      history.assign(nh * 45, 0.f);
      input.reserve(history_indices.size() * 45);
      commands_scale = config.at("commands_scale").get<Vec3>();
      angular_scale = config.at("ang_vel_scale").get<double>();
      position_scale = config.at("dof_pos_scale").get<double>();
      velocity_scale = config.at("dof_vel_scale").get<double>();
      observation_bound = config.at("clip_obs").get<float>();
      lower_actions =
          config.at("clip_actions_lower").get<std::vector<double>>();
      upper_actions =
          config.at("clip_actions_upper").get<std::vector<double>>();
      std::size_t dimension = 0;
      for (const auto &item : config.at("observations")) {
        const auto term = item.get<std::string>();
        if (term == "commands") {
          observation_terms.push_back(Observation::Commands);
          dimension += 3;
        } else if (term == "ang_vel") {
          observation_terms.push_back(Observation::AngularVelocity);
          dimension += 3;
        } else if (term == "gravity_vec") {
          observation_terms.push_back(Observation::Gravity);
          dimension += 3;
        } else if (term == "dof_pos") {
          observation_terms.push_back(Observation::JointPosition);
          dimension += 12;
        } else if (term == "dof_vel") {
          observation_terms.push_back(Observation::JointVelocity);
          dimension += 12;
        } else if (term == "actions") {
          observation_terms.push_back(Observation::Actions);
          dimension += 12;
        } else
          throw std::runtime_error("Unknown observation term");
      }
      if (dimension != 45 || scales.size() != 12 || kp.size() != 12 ||
          kd.size() != 12 || torque_limits.size() != 12 ||
          lower_actions.size() != 12 || upper_actions.size() != 12)
        throw std::runtime_error("Invalid Go2 controller dimensions");
      if (c.options.go2_startup == "upstream_getup") {
        state.startup_done = false;
        state.phase = "pre_getup";
        for (auto &name : catalog["go2_manifest"]["sdk_joint_names"]) {
          int j = id(m, mjOBJ_JOINT, name);
          sdkq.push_back(m->jnt_qposadr[j]);
          sdkv.push_back(m->jnt_dofadr[j]);
          for (int a = 0; a < m->nu; a++)
            if (m->actuator_trnid[2 * a] == j)
              sdka.push_back(a);
          start_pos.push_back(static_cast<float>(d->qpos[m->jnt_qposadr[j]]));
        }
        getup_target = start_pos;
      }
    }
    if (qpos.size() != 12 || default_pos.size() != 12)
      throw std::runtime_error("Invalid Go2 joint dimensions");
  }
  void getup() {
    static constexpr std::array<float, 12> pre{0.f,    1.36f,  -2.65f, 0.f,
                                               1.36f,  -2.65f, 0.f,    1.36f,
                                               -2.65f, 0.f,    1.36f,  -2.65f};
    static constexpr std::array<float, 12> stand{
        0.f, .8f, -1.5f, 0.f, .8f, -1.5f, 0.f, .8f, -1.5f, 0.f, .8f, -1.5f};
    auto interp = [&](float &p, const auto &a, const auto &b, float duration) {
      if (p >= 1.f)
        return;
      if (p == 0.f) {
        float max = 0;
        for (int i = 0; i < 12; i++)
          max = std::max(max, std::abs(a[i] - b[i]));
        if (max < .1f)
          p = 1.f;
      }
      int frames = std::max(1, static_cast<int>(std::ceil(duration / .005f)));
      p = std::min(1.f, p + 1.f / frames);
      for (int i = 0; i < 12; i++)
        getup_target[i] = (1.f - p) * a[i] + p * b[i];
    };
    state.startup_count++;
    interp(pre_percent, start_pos, pre, 1.f);
    if (pre_percent >= 1.f) {
      state.phase = "getup";
      interp(stand_percent, pre, stand, 2.f);
      if (stand_percent >= 1.f) {
        state.startup_done = true;
        state.phase = "complete";
      }
    }
    for (int i = 0; i < 12; i++)
      d->ctrl[sdka[i]] = 80. * (double(getup_target[i]) - d->qpos[sdkq[i]]) -
                         3. * d->qvel[sdkv[i]];
  }
  void update(const Vec3 &policy_command) override {
    if (!state.startup_done) {
      getup();
      return;
    }
    int every = std::lround(.02 / m->opt.timestep);
    if (state.policy_steps % every == 0) {
      Vec3 gyro{d->sensordata[gyro_address], d->sensordata[gyro_address + 1],
                d->sensordata[gyro_address + 2]};
      const auto *rot = d->site_xmat + 9 * imu_id;
      Vec3 gravity{-rot[6], -rot[7], -rot[8]};
      obs.clear();
      if (legacy) {
        Vec3 lin{d->sensordata[linear_address],
                 d->sensordata[linear_address + 1],
                 d->sensordata[linear_address + 2]};
        for (double v : lin)
          obs.push_back(v);
        for (double v : gyro)
          obs.push_back(v);
        for (double v : gravity)
          obs.push_back(v);
        for (int i = 0; i < 12; i++)
          obs.push_back(d->qpos[qpos[i]] - default_pos[i]);
        for (int i = 0; i < 12; i++)
          obs.push_back(d->qvel[qvel[i]]);
        obs.insert(obs.end(), policy->last.begin(), policy->last.end());
        for (double v : policy_command)
          obs.push_back(v);
        policy->run_into(obs);
      } else {
        for (const auto term : observation_terms) {
          if (term == Observation::Commands) {
            const auto &scale = commands_scale;
            for (int i = 0; i < 3; i++)
              obs.push_back(policy_command[i] * scale[i]);
          } else if (term == Observation::AngularVelocity)
            for (double v : gyro)
              obs.push_back(v * angular_scale);
          else if (term == Observation::Gravity)
            for (double v : gravity)
              obs.push_back(v);
          else if (term == Observation::JointPosition)
            for (int i = 0; i < 12; i++)
              obs.push_back((d->qpos[qpos[i]] - default_pos[i]) *
                            position_scale);
          else if (term == Observation::JointVelocity)
            for (int i = 0; i < 12; i++)
              obs.push_back(d->qvel[qvel[i]] * velocity_scale);
          else if (term == Observation::Actions)
            obs.insert(obs.end(), policy->last.begin(), policy->last.end());
          else
            throw std::runtime_error("Unknown observation term");
        }
        float bound = observation_bound;
        for (float &v : obs)
          v = std::min(bound, std::max(-bound, v));
        std::move_backward(history.begin(), history.end() - 45, history.end());
        std::copy(obs.begin(), obs.end(), history.begin());
        input.clear();
        for (int i : history_indices)
          input.insert(input.end(), history.begin() + i * 45,
                       history.begin() + (i + 1) * 45);
        policy->run_into(input);
        for (int i = 0; i < 12; i++)
          policy->last[i] =
              clamp(policy->last[i], lower_actions[i], upper_actions[i]);
      }
    }
    if (legacy)
      for (int i = 0; i < 12; i++)
        d->ctrl[actuators[i]] = default_pos[i] + .5 * policy->last[i];
    else
      for (int i = 0; i < 12; i++) {
        double tau = kp[i] * (default_pos[i] + policy->last[i] * scales[i] -
                              d->qpos[qpos[i]]) -
                     kd[i] * d->qvel[qvel[i]];
        d->ctrl[actuators[i]] =
            configured_torque ? clamp(tau, -torque_limits[i], torque_limits[i])
                              : tau;
      }
    state.policy_steps++;
  }
};
} // namespace
std::unique_ptr<Controller> make_go2_controller(const ControllerContext &c) {
  return std::make_unique<Go2Controller>(c);
}
} // namespace simple_rbt
