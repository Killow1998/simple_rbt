#include "simple_rbt/core.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>

using namespace simple_rbt;
namespace fs = std::filesystem;
namespace {
Json read(const fs::path &p) { std::ifstream f(p); return Json::parse(f); }
void write(const fs::path &p, const Json &j) { std::ofstream(p) << j.dump(2) << '\n'; }
template<class T> Json array(const T *p, int n) { return std::vector<T>(p, p + n); }
double difference(const Json &a, const Json &b) {
  if (a.is_number() && b.is_number()) return std::abs(a.get<double>() - b.get<double>());
  if (a.size() != b.size()) return 1e100;
  double result = 0;
  if (a.is_array()) for (std::size_t i = 0; i < a.size(); ++i) result = std::max(result, difference(a[i], b[i]));
  else for (const auto &v : a.items()) result = std::max(result, difference(v.value(), b.at(v.key())));
  return result;
}
Json options(const mjOption &o) {
  return {{"timestep", o.timestep}, {"integrator", o.integrator}, {"solver", o.solver},
          {"iterations", o.iterations}, {"ls_iterations", o.ls_iterations},
          {"tolerance", o.tolerance}, {"ls_tolerance", o.ls_tolerance}, {"cone", o.cone},
          {"impratio", o.impratio}, {"ccd_iterations", o.ccd_iterations},
          {"gravity", array(o.gravity, 3)}, {"disableflags", o.disableflags}, {"enableflags", o.enableflags}};
}
Json terrain(const mjModel *m, const mjData *d) {
  Json result = Json::array();
  for (int i = 0; i < m->ngeom; ++i) {
    if (m->body_weldid[m->geom_bodyid[i]] || !(m->geom_contype[i] || m->geom_conaffinity[i])) continue;
    result.push_back({{"type", m->geom_type[i]}, {"size", array(m->geom_size + 3*i, 3)},
      {"pos", array(d->geom_xpos + 3*i, 3)}, {"mat", array(d->geom_xmat + 9*i, 9)},
      {"friction", array(m->geom_friction + 3*i, 3)}, {"solref", array(m->geom_solref + mjNREF*i, mjNREF)},
      {"solimp", array(m->geom_solimp + mjNIMP*i, mjNIMP)}});
  }
  return result;
}
void restore(const Json &j, mjtNum *target, int count) {
  const auto values = j.get<std::vector<double>>();
  if (values.size() != std::size_t(count)) throw std::invalid_argument("Initial state size mismatch");
  std::copy(values.begin(), values.end(), target);
}
} // namespace

int supermimic_case(const char *manifest_path, const char *case_id, const char *asset_root,
                    const char *output_path, bool inspect) {
  const fs::path output(output_path);
  if (!fs::create_directory(output)) return 2;
  try {
    const auto manifest = read(manifest_path);
    Json test;
    for (const auto &v : manifest.at("cases")) if (v.at("id") == case_id) test = v;
    if (test.is_null()) throw std::invalid_argument("Case is absent from manifest");
    const auto contract_path = fs::path(manifest.at("original_root").get<std::string>()) / case_id / "contract.json";
    const auto contract = read(contract_path);
    SimulationOptions settings;
    settings.robot = "go2w"; settings.controller = "supermimic"; settings.control_mode = "native";
    settings.project_root = fs::absolute(asset_root);
    settings.scene = read(settings.project_root / "validation.scene.json");
    Simulation sim(settings);
    auto *m = sim.model(); auto *d = sim.data();
    mj_resetData(m, d);
    restore(contract.at("init_qpos"), d->qpos, m->nq);
    restore(contract.at("init_qvel"), d->qvel, m->nv);
    restore(contract.at("init_act"), d->act, m->na);
    restore(contract.at("init_ctrl"), d->ctrl, m->nu);
    mj_forward(m, d);
    std::ofstream(output / "compiled-model.xml") << sim.xml();
    Json initial{{"qpos", array(d->qpos,m->nq)}, {"qvel", array(d->qvel,m->nv)},
                 {"act", array(d->act,m->na)}, {"ctrl", array(d->ctrl,m->nu)}};
    write(output / "initial-state.json", initial);
    Json actual_terrain = terrain(m, d), actual_options = options(m->opt);
    Json audit{{"static_collision_geoms", actual_terrain}, {"solver_options", actual_options},
               {"static_max_error", difference(actual_terrain, contract.at("static_collision_geoms"))},
               {"solver_max_error", difference(actual_options, contract.at("solver_options"))},
               {"initial_qpos_max_error", difference(initial["qpos"], contract.at("init_qpos"))},
               {"initial_qvel_max_error", difference(initial["qvel"], contract.at("init_qvel"))}};
    audit["matched"] = audit["static_max_error"].get<double>() <= 1e-12 && audit["solver_max_error"].get<double>() <= 1e-12;
    write(output / "input-audit.json", audit);
    if (!audit.at("matched").get<bool>()) throw std::runtime_error("Imported terrain/options differ from original contract");
    if (inspect) return 0;
    const double dt = manifest.at("control_dt"), physics_dt = manifest.at("physics_dt");
    const int decimation = std::lround(dt / physics_dt), count = test.at("steps");
    if (std::abs(m->opt.timestep - physics_dt) > 1e-12 ||
        std::abs(dt - decimation * physics_dt) > 1e-12 || manifest.at("command_delay_s") != dt)
      throw std::invalid_argument("Unsupported control clock/delay contract");
    const auto config = read(settings.project_root / "assets/catalog.json").at("controllers").at("go2w-supermimic");
    const int base = mj_name2id(m, mjOBJ_BODY, config.at("base_body").get<std::string>().c_str());
    std::vector<int> q, v;
    for (const auto &name : config.at("joint_names")) {
      const int j = mj_name2id(m, mjOBJ_JOINT, name.get<std::string>().c_str());
      q.push_back(m->jnt_qposadr[j]); v.push_back(m->jnt_dofadr[j]);
    }
    auto snapshot = [&]() {
      const double *r = d->xmat + 9*base, *p = d->xpos + 3*base;
      mjtNum velocity[6]; mj_objectVelocity(m, d, mjOBJ_XBODY, base, velocity, 1);
      const double roll = std::atan2(r[7], r[8]), pitch = std::asin(std::clamp(-r[6], -1., 1.));
      std::vector<double> state{p[2], -r[6], -r[7], -r[8], velocity[3], velocity[4], velocity[5], velocity[0], velocity[1], velocity[2]};
      for (int i=0;i<12;++i) state.push_back(d->qpos[q[i]]-config.at("default_dof_pos")[i].get<double>());
      for (int i=0;i<16;++i) state.push_back(d->qvel[v[i]]);
      return Json{{"xyz", array(p,3)}, {"pose", {p[0],p[1],std::atan2(r[3],r[0])}},
                  {"state", state}, {"roll", roll}, {"pitch", pitch},
                  {"gravity_tilt", std::acos(std::clamp(r[8],-1.,1.))},
                  {"body_velocity", {velocity[3],velocity[4],velocity[2]}},
                  {"qpos", array(d->qpos,m->nq)}, {"qvel", array(d->qvel,m->nv)}};
    };
    const auto origin = snapshot().at("xyz").get<Vec3>();
    Vec3 consumed{}, request{};
    const int settle = std::lround(manifest.at("settle_s").get<double>() / dt);
    const int stop = std::lround(manifest.at("stop_s").get<double>() / dt);
    const double distance = test.at("course_distance"), limit = manifest.at("bounds_abs_xy_m");
    const double angle = manifest.at("body_tilt_termination_rad");
    int reached = -1;
    std::string reason = "time_limit";
    Json rows = Json::array(), terminal;
    const auto start = std::chrono::steady_clock::now();
    for (int i=0; i<count; ++i) {
      auto row = snapshot();
      if (reached < 0 && row["xyz"][0].get<double>() - origin[0] >= distance) reached = i;
      request = i >= settle && reached < 0 ? test.at("command").get<Vec3>() : Vec3{};
      row.update({{"step",i}, {"time",i*dt}, {"physics_time",d->time},
                  {"command",request}, {"consumed_command",consumed}, {"ctrl_before",array(d->ctrl,m->nu)}});
      sim.set_command(request);
      for (int k=0; k<decimation; ++k) sim.step();
      consumed = request;
      terminal = snapshot();
      const bool finite = std::all_of(d->qpos,d->qpos+m->nq,[](double x){return std::isfinite(x);}) &&
                          std::all_of(d->qvel,d->qvel+m->nv,[](double x){return std::isfinite(x);});
      const bool fall = finite && terminal["gravity_tilt"].get<double>() > angle;
      const bool bounds = finite && (std::abs(terminal["xyz"][0].get<double>()-origin[0]) > limit ||
                                    std::abs(terminal["xyz"][1].get<double>()-origin[1]) > limit);
      terminal["time"] = (i+1)*dt;
      const bool stop_complete = reached >= 0 && i >= reached + stop - 1;
      row.update({{"ctrl_after",array(d->ctrl,m->nu)}, {"terminated_after_action",fall || bounds || !finite},
                  {"terminated",fall || !finite}, {"truncated",bounds || (i+1==count && !stop_complete)}});
      rows.push_back(std::move(row));
      if (!finite || fall || bounds) { reason = !finite ? "nonfinite" : fall ? "fall" : "bounds"; break; }
      if (stop_complete) { reason = "goal_stop"; break; }
    }
    std::ofstream(output / "trajectory.json") << rows.dump() << '\n';
    std::ofstream csv(output / "trajectory.csv"); csv.precision(17);
    csv << "step,time,x,y,z,roll,pitch,gravity_tilt,vx,vy,wz,requested_vx,requested_vy,requested_wz,consumed_vx,consumed_vy,consumed_wz,terminated,truncated\n";
    for (const auto &row : rows) {
      csv << row["step"] << ',' << row["time"];
      for (double x : row["xyz"]) csv << ',' << x;
      for (const auto *key : {"roll","pitch","gravity_tilt"}) csv << ',' << row[key];
      for (const auto *key : {"body_velocity","command","consumed_command"}) for (double x : row[key]) csv << ',' << x;
      csv << ',' << row["terminated"] << ',' << row["truncated"] << '\n';
    }
    write(output / "terminal-state.json", terminal);
    write(output / "summary.json", {{"case",test}, {"mujoco",mj_versionString()},
      {"contract_sha256",sha256_file(contract_path)}, {"manifest_sha256",sha256_file(manifest_path)},
      {"policy_sha256",config.at("policy_sha256")}, {"sampling","50 Hz before action; body-link velocity"},
      {"steps",rows.size()}, {"requested_steps",count}, {"reached_step",reached < 0 ? Json(nullptr) : Json(reached)},
      {"termination_reason",reason}, {"simulation_seconds",d->time}, {"command_delay_s",dt},
      {"wall_seconds",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()},
      {"tracker_updates",sim.tracker_metadata().at("updates")}, {"tracker",sim.tracker_metadata()}, {"deployment",sim.deployment_metadata()},
      {"interpretation","Raw outcome only; strict acceptance belongs to the shared analyzer"}});
    std::cout << case_id << ": " << reason << " at " << d->time << " s\n";
    return 0;
  } catch (const std::exception &e) {
    write(output / "infrastructure-error.json", {{"error",e.what()}});
    std::cerr << e.what() << '\n';
    return 2;
  }
}
