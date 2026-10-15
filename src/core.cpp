#include "simple_rbt/core.hpp"
#include "controller.hpp"
#include "simple_rbt/paths.hpp"
#include <tinyxml2.h>
#include <sstream>
#include <iomanip>
#include "scene.hpp"
#include "tracking.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace simple_rbt {
namespace {
double clamp(double x, double lo, double hi) {
  return std::min(std::max(x, lo), hi);
}
int id(const mjModel *m, int type, const std::string &n) {
  int i = mj_name2id(m, type, n.c_str());
  if (i < 0)
    throw std::runtime_error("Missing MuJoCo name " + n);
  return i;
}
Vec3 sensor(const mjData *d, int a) {
  return {d->sensordata[a], d->sensordata[a + 1], d->sensordata[a + 2]};
}
Pose sitepose(const mjData *d, int i) {
  Pose p;
  std::copy_n(d->site_xpos + 3 * i, 3, p.position.begin());
  mju_mat2Quat(p.quaternion.data(), d->site_xmat + 9 * i);
  return p;
}
} // namespace
struct Simulation::Impl {
  SimulationOptions o;
  const RobotSpec &spec;
  std::string controller, scene_xml;
  Json terrain;
  int base_site = -1, lidar_site = -1, camera_site = -1, imu_site = -1,
      mid360_site = -1;
  int linear_address = -1, angular_address = -1, gyro_address = -1,
      accel_address = -1;
  int site_id(const std::string &name) const {
    if (name == "base_origin")
      return base_site;
    if (name == "lidar")
      return lidar_site;
    if (name == "camera_mount")
      return camera_site;
    if (name == "imu")
      return imu_site;
    if (name == "mid360_imu")
      return mid360_site;
    return id(m, mjOBJ_SITE, name);
  }
  mjModel *m = nullptr;
  mjData *d = nullptr;
  Vec3 command{}, policy_command{}, limits{1, .6, 1};
  CommandTracker tracker;
  std::unique_ptr<Controller> control;
  std::uint64_t steps = 0;
  Impl(const SimulationOptions &options)
      : o(options), spec(robot_spec(options.robot)) {
    if (o.project_root.empty())
      o.project_root = default_project_root();
    const Json catalog = load_catalog(o.project_root);
    controller = o.controller;
    if (controller == "auto")
      controller = spec.default_controller;
    if (!catalog["models"].contains(o.robot + "-" + controller))
      throw std::invalid_argument("Unsupported robot/controller combination");
    if (spec.continuous_terrain && o.terrain != "flat" && o.terrain != "ramp" &&
        o.terrain != "hills")
      throw std::invalid_argument("Pure wheel robots require flat/ramp/hills");
    if (!catalog["terrain_presets"].contains(o.terrain + "-" + o.terrain_level))
      throw std::invalid_argument("Unknown terrain/level");
    if (!std::isfinite(o.difficulty) || o.difficulty <= 0 || o.difficulty > 3 ||
        (o.terrain_level != "standard" && o.difficulty != 1))
      throw std::invalid_argument("Invalid terrain difficulty");
    if (o.friction != -1 &&
        (!std::isfinite(o.friction) || o.friction <= 0 || o.friction > 2))
      throw std::invalid_argument("Invalid friction");
    if (o.control_mode != "native" && o.control_mode != "tracked")
      throw std::invalid_argument("Invalid control mode");
    if (o.go2_torque_mode != "configured" && o.go2_torque_mode != "upstream")
      throw std::invalid_argument("Invalid Go2 torque mode");
    if (o.go2_startup != "crouched" && o.go2_startup != "upstream_getup")
      throw std::invalid_argument("Invalid Go2 startup");
    if ((o.robot != "go2" || controller == "legacy") &&
        (o.go2_torque_mode != "configured" || o.go2_startup != "crouched"))
      throw std::invalid_argument(
          "Go2 options require canonical Go2 controller");
    limits = spec.command_limits;
    tracker.limits = limits;
    if (!spec.allows_lateral)
      tracker.limits[1] = .6;
    tracker.lateral = spec.allows_lateral;
    // Pure wheel chassis have physical joint brakes; legged policies still
    // need chassis feedback while holding a zero-speed stance.
    tracker.brake_on_zero = spec.continuous_terrain;
    scene_xml = build_scene(o, catalog, controller, terrain);
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    mj_addBufferVFS(&vfs, "scene.xml", scene_xml.data(), scene_xml.size());
    char error[2048]{};
    m = mj_loadXML("scene.xml", &vfs, error, sizeof(error));
    mj_deleteVFS(&vfs);
    if (!m)
      throw std::runtime_error(error);
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_guard(
        m, mj_deleteModel);
    d = mj_makeData(m);
    if (!d)
      throw std::runtime_error("mj_makeData failed");
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_guard(d,
                                                                 mj_deleteData);
    if (m->nkey)
      mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);
    const Json config = catalog["controllers"].value(o.robot + "-" + controller,
                                                     Json::object());
    control = make_controller({o, controller, config, catalog, m, d});
    base_site = id(m, mjOBJ_SITE, "base_origin");
    lidar_site = id(m, mjOBJ_SITE, "lidar");
    camera_site = id(m, mjOBJ_SITE, "camera_mount");
    imu_site = id(m, mjOBJ_SITE, "imu");
    mid360_site = id(m, mjOBJ_SITE, "mid360_imu");
    linear_address = m->sensor_adr[id(m, mjOBJ_SENSOR, "base_linvel")];
    angular_address = m->sensor_adr[id(m, mjOBJ_SENSOR, "base_angvel")];
    gyro_address = m->sensor_adr[id(m, mjOBJ_SENSOR, "mid360_gyro")];
    accel_address = m->sensor_adr[id(m, mjOBJ_SENSOR, "mid360_accelerometer")];
    // Only selected typed controller parameters and terrain metadata survive.
    // The complete catalog/config are construction-local and are released here.
    data_guard.release();
    model_guard.release();
  }
  ~Impl() {
    control.reset();
    if (d)
      mj_deleteData(d);
    if (m)
      mj_deleteModel(m);
  }
};
Simulation::Simulation(const SimulationOptions &o)
    : p_(std::make_unique<Impl>(o)) {
  if (o.mid360_mount) set_mount("mid360", *o.mid360_mount);
  if (o.camera_mount) set_mount("camera", *o.camera_mount);
}
Simulation::~Simulation() = default;
mjModel *Simulation::model() const { return p_->m; }
mjData *Simulation::data() const { return p_->d; }
void Simulation::set_mount(const std::string& module, const Pose& requested) {
  Pose pose = requested;
  for (double v:pose.position) if (!std::isfinite(v) || std::abs(v)>10.)
    throw std::invalid_argument("Mount translation must be finite and within 10 metres");
  double norm=0.;
  for (double v:pose.quaternion) { if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite mount rotation"); norm+=v*v; }
  if (norm<1e-12) throw std::invalid_argument("Zero mount quaternion");
  for (double& v:pose.quaternion) v/=std::sqrt(norm);
  int target = module=="mid360" ? p_->lidar_site : module=="camera" ? p_->camera_site : -1;
  if (target<0) throw std::invalid_argument("Only MID360 assembly and camera mounts are editable");
  auto assign=[&](int site,const Pose& value){
    std::copy(value.position.begin(),value.position.end(),p_->m->site_pos+site*3);
    std::copy(value.quaternion.begin(),value.quaternion.end(),p_->m->site_quat+site*4);
  };
  assign(target,pose);
  if(module=="mid360") {
    Pose internal=pose;
    const double offset[3]={.011,.02329,-.04412}; double transformed[3];
    mju_rotVecQuat(transformed,offset,pose.quaternion.data());
    for(int i=0;i<3;++i) internal.position[i]+=transformed[i];
    assign(p_->mid360_site,internal);
  }
  mj_forward(p_->m,p_->d);
}
void Simulation::set_command(const Vec3 &v) {
  if (!supported_command(p_->o.robot, v)) {
    p_->command = {};
    throw std::invalid_argument("Unsupported/nonfinite velocity command");
  }
  for (int i = 0; i < 3; i++)
    p_->command[i] = clamp(v[i], -p_->limits[i], p_->limits[i]);
}
void Simulation::step() {
  auto &p = *p_;
  p.policy_command = p.command;
  bool starting = !p.control->state.startup_done;
  if (starting) {
    p.policy_command = {};
    p.control->update(p.policy_command);
  } else {
    if (p.o.control_mode == "tracked" && p.spec.command_tracking) {
      int s = p.base_site;
      const double *r = p.d->site_xmat + 9 * s;
      p.policy_command =
          p.tracker.update(p.command, linear_velocity(), angular_velocity(),
                           std::atan2(r[3], r[0]), p.m->opt.timestep);
      if (!p.spec.allows_lateral)
        p.policy_command[1] = 0;
      if (p.spec.min_turning_radius > 0.) {
        if (p.command == Vec3{})
          p.policy_command = {};
        else {
          double limit =
              std::abs(p.policy_command[0]) / p.spec.min_turning_radius;
          p.policy_command[2] = clamp(p.policy_command[2], -limit, limit);
          if (std::abs(p.policy_command[0]) <= 1e-9)
            p.policy_command[2] = 0;
        }
      }
    }
    p.control->update(p.policy_command);
  }
  mj_step(p.m, p.d);
  p.steps++;
  mj_forward(p.m, p.d);
  for (int i = 0; i < p.m->nq; i++)
    if (!std::isfinite(p.d->qpos[i]))
      throw std::runtime_error("Nonfinite physics state");
  if (std::abs(p.d->time - p.steps * p.m->opt.timestep) > 1e-6)
    throw std::runtime_error("MuJoCo reset time");
}
std::uint64_t Simulation::stamp_ns() const {
  return std::llround(p_->steps * p_->m->opt.timestep * 1e9);
}
Pose Simulation::pose(const std::string &s) const {
  return sitepose(p_->d, p_->site_id(s));
}
Pose Simulation::extrinsics(const std::string &s) const {
  int i = p_->site_id(s);
  Pose p;
  std::copy_n(p_->m->site_pos + i * 3, 3, p.position.begin());
  std::copy_n(p_->m->site_quat + i * 4, 4, p.quaternion.begin());
  return p;
}
OpticalPose Simulation::camera_pose() const {
  int i = p_->camera_site;
  OpticalPose p;
  std::copy_n(p_->d->site_xpos + 3 * i, 3, p.position.begin());
  const double *r = p_->d->site_xmat + 9 * i;
  for (int row = 0; row < 3; row++) {
    p.rotation[3 * row] = -r[3 * row + 1];
    p.rotation[3 * row + 1] = -r[3 * row + 2];
    p.rotation[3 * row + 2] = r[3 * row];
  }
  return p;
}
Vec3 Simulation::linear_velocity() const {
  return sensor(p_->d, p_->linear_address);
}
Vec3 Simulation::angular_velocity() const {
  return sensor(p_->d, p_->angular_address);
}
ImuSample Simulation::imu() const {
  return {stamp_ns(), sensor(p_->d, p_->gyro_address),
          sensor(p_->d, p_->accel_address)};
}
const Vec3 &Simulation::command() const { return p_->command; }
const Vec3 &Simulation::command_limits() const { return p_->limits; }
bool Simulation::startup_complete() const {
  return p_->control->state.startup_done;
}
std::string Simulation::startup_phase() const {
  return p_->control->state.startup_done ? "policy" : p_->control->state.phase;
}
std::uint64_t Simulation::startup_steps() const {
  return p_->control->state.startup_count;
}
const std::string &Simulation::controller_name() const {
  return p_->controller;
}
const Json &Simulation::terrain_metadata() const { return p_->terrain; }
Json Simulation::deployment_metadata() const {
  return {
      {"robot", p_->o.robot},
      {"controller", p_->controller},
      {"runtime_language", "C++17"},
      {"go2_torque_mode", p_->o.go2_torque_mode},
      {"go2_startup", p_->o.go2_startup},
      {"policy_period_s", p_->control->state.policy_period},
      {"physics_period_s", p_->m->opt.timestep},
      {"policy_calls", p_->control->state.policy_steps
                           ? 1 + (p_->control->state.policy_steps - 1) /
                                     std::lround(.02 / p_->m->opt.timestep)
                           : 0},
      {"pd_calls", p_->control->state.pd_calls},
      {"startup_steps", p_->control->state.startup_count},
      {"source_model",
       "offline frozen MJCF; original contact/inertial/actuator parameters"}};
}
Json Simulation::tracker_metadata() const {
  return {{"saturation_fraction",
           p_->tracker.updates
               ? double(p_->tracker.saturated) / p_->tracker.updates
               : 0.},
          {"saturation_count", p_->tracker.saturated},
          {"updates", p_->tracker.updates},
          {"limitation", "Ideal simulator body velocity/heading feedback; no "
                         "hardware estimator"}};
}
std::string Simulation::xml() const {
  tinyxml2::XMLDocument doc; doc.Parse(p_->scene_xml.c_str());
  auto numbers=[](const double* v,int count){std::ostringstream out;out<<std::setprecision(17);for(int i=0;i<count;++i){if(i)out<<' ';out<<v[i];}return out.str();};
  auto visit=[&](auto&& self,tinyxml2::XMLElement* e)->void {
    if(std::string(e->Name())=="site") {
      const char* name=e->Attribute("name");
      if(name && (std::string(name)=="lidar" || std::string(name)=="mid360_imu" || std::string(name)=="camera_mount")) {
        int i=p_->site_id(name);e->SetAttribute("pos",numbers(p_->m->site_pos+3*i,3).c_str());e->SetAttribute("quat",numbers(p_->m->site_quat+4*i,4).c_str());
      }
    }
    for(auto* child=e->FirstChildElement();child;child=child->NextSiblingElement())self(self,child);
  };
  visit(visit,doc.RootElement());tinyxml2::XMLPrinter out;doc.Print(&out);return out.CStr();
}
} // namespace simple_rbt
