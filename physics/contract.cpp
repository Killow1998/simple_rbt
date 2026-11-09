#include "simple_rbt/core.hpp"
#include <fstream>
#include <iostream>
using namespace simple_rbt;
namespace fs=std::filesystem;
template<class T> Json array(const T *p,int n){return std::vector<T>(p,p+n);}
void restore_trace(Simulation&,const fs::path&,int);
bool force_trace(Simulation&,const fs::path&);
int main(int argc,char**argv){
 try {
  if(argc!=4&&argc!=6)throw std::invalid_argument("Usage: simple-rbt-physics-contract ASSET_ROOT SESSION_JSON NEW_OUTPUT [INITIAL_CONTRACT ROW]");
  fs::path root=fs::absolute(argv[1]),out=fs::absolute(argv[3]);
  Json c;std::ifstream(argv[2])>>c;
  if(c.at("robot")!="go2w"||c.at("controller")!="supermimic"||c.at("control_mode")!="native")
    throw std::invalid_argument("Shared physics contract currently supports native Go2W SuperMimic");
  SimulationOptions o;o.project_root=root;o.robot="go2w";o.controller="supermimic";o.control_mode="native";
  o.scene=c.value("scene",Json());o.scene_spawn=c.value("scene_spawn",std::string());
  o.camera_model=c.value("camera_model",std::string("d435"));
  Simulation sim(o);auto*m=sim.model();auto*d=sim.data();
  if(argc==6)restore_trace(sim,argv[4],std::stoi(argv[5]));
  Json catalog;std::ifstream(root/"assets/catalog.json")>>catalog;
  Json controller=catalog.at("controllers").at("go2w-supermimic");
  if(controller.value("actor_wheel_vel",true))throw std::invalid_argument("This shared contract requires the 49D current policy");
  controller["policy"]=fs::absolute(root/controller.at("policy").get<std::string>()).lexically_normal().string();
  if(!fs::create_directories(out))throw std::invalid_argument("Output already exists");
  std::ofstream(out/"model.xml")<<sim.xml();
  Json fields;
#define FIELD(name,count) fields[#name]=array(m->name,count)
  FIELD(qpos0,m->nq);FIELD(body_mass,m->nbody);FIELD(body_inertia,3*m->nbody);FIELD(body_ipos,3*m->nbody);FIELD(body_iquat,4*m->nbody);
  FIELD(dof_armature,m->nv);FIELD(dof_damping,m->nv);FIELD(dof_frictionloss,m->nv);
  FIELD(jnt_range,2*m->njnt);FIELD(jnt_limited,m->njnt);FIELD(geom_type,m->ngeom);FIELD(geom_size,3*m->ngeom);
  FIELD(geom_friction,3*m->ngeom);FIELD(geom_solref,2*m->ngeom);FIELD(geom_solimp,5*m->ngeom);
  FIELD(geom_contype,m->ngeom);FIELD(geom_conaffinity,m->ngeom);FIELD(geom_condim,m->ngeom);FIELD(geom_priority,m->ngeom);FIELD(geom_margin,m->ngeom);FIELD(geom_gap,m->ngeom);
  FIELD(actuator_gainprm,10*m->nu);FIELD(actuator_biasprm,10*m->nu);FIELD(actuator_ctrlrange,2*m->nu);FIELD(actuator_forcerange,2*m->nu);FIELD(actuator_ctrllimited,m->nu);FIELD(actuator_forcelimited,m->nu);FIELD(actuator_gear,6*m->nu);
#undef FIELD
  Json initial{{"qpos",array(d->qpos,m->nq)},{"qvel",array(d->qvel,m->nv)},{"ctrl",array(d->ctrl,m->nu)}};
  Json geoms=Json::array();
  for(int i=0;i<m->ngeom;++i)if(m->body_weldid[m->geom_bodyid[i]]==0&&(m->geom_contype[i]||m->geom_conaffinity[i])) {
    mjtNum quat[4];mju_mat2Quat(quat,d->geom_xmat+9*i);
    geoms.push_back({{"type",m->geom_type[i]},{"size",array(m->geom_size+3*i,3)},{"pos",array(d->geom_xpos+3*i,3)},{"quat",array(quat,4)},
      {"friction",array(m->geom_friction+3*i,3)},{"solref",array(m->geom_solref+2*i,2)},{"solimp",array(m->geom_solimp+5*i,5)},
      {"contype",m->geom_contype[i]},{"conaffinity",m->geom_conaffinity[i]},{"condim",m->geom_condim[i]},{"priority",m->geom_priority[i]},{"margin",m->geom_margin[i]},{"gap",m->geom_gap[i]}});
  }
  Json solver{{"timestep",m->opt.timestep},{"integrator",int(m->opt.integrator)},{"cone",int(m->opt.cone)},{"solver",int(m->opt.solver)},{"impratio",m->opt.impratio},{"gravity",array(m->opt.gravity,3)},
    {"iterations",m->opt.iterations},{"ls_iterations",m->opt.ls_iterations},{"tolerance",m->opt.tolerance},{"ls_tolerance",m->opt.ls_tolerance},{"ccd_iterations",m->opt.ccd_iterations},
    {"disableflags",m->opt.disableflags},{"enableflags",m->opt.enableflags}};
  Json contract{{"format","sprobo.physics.v1"},{"model","model.xml"},{"model_sha256",sha256_file(out/"model.xml")},{"controller",controller},{"initial",initial},{"model_fields",fields},{"static_geoms",geoms},{"solver",solver},
    {"physics",{{"backend","mujoco"},{"device","cpu"},{"version",mj_versionString()}}},{"units","SI; right-handed Z up; quaternion WXYZ; body-frame vx/vy/yaw_rate"},
    {"command_limits",sim.command_limits()}, {"clock",{{"physics_s",.005},{"control_s",.02},{"command_delay_s",.02}}},{"observation",{{"frame_width",49},{"history",5},{"term_widths",{3,3,12,12,16,3}},{"order","term-major oldest to newest; leg velocity x0.05; norm frozen in actor"}}},
    {"reset",{{"qpos","initial"},{"qvel","initial"},{"history","repeat initial frame"},{"previous_action",0},{"command_latch",0},{"time",0}}},
    {"termination",{{"nonfinite","hard failure"},{"tilt_rad",1.2217304763960306},{"timeout_policy_steps",300},{"reset","explicit subset; no hidden auto-reset"}}}};
  Json geom_names=Json::array(),geom_bodies=Json::array();
  for(int i=0;i<m->ngeom;++i){const char*g=mj_id2name(m,mjOBJ_GEOM,i);const char*b=mj_id2name(m,mjOBJ_BODY,m->geom_bodyid[i]);geom_names.push_back(g?g:"");geom_bodies.push_back(b?b:"world");}
  contract["geom_names"]=geom_names;contract["geom_bodies"]=geom_bodies;
  Json actuator_schema=Json::array();
  for(int i=0;i<m->nu;++i){const char*name=mj_id2name(m,mjOBJ_JOINT,m->actuator_trnid[2*i]);const std::string joint=name?name:"";
    actuator_schema.push_back({{"index",i},{"joint",joint},{"target_units",joint.find("wheel_joint")!=std::string::npos?"rad/s":"rad"},{"force_range",array(m->actuator_forcerange+2*i,2)}});}
  contract["actuators"]=actuator_schema;
  std::ofstream(out/"contract.json")<<contract.dump(2)<<'\n';std::ofstream(out/"controller.json")<<controller.dump(2)<<'\n';
  if(argc==6)return force_trace(sim,out)?0:1;
  Json frames=Json::array();bool finite=true;double max_tilt=0.;
  for(int step=0;step<300;++step){
    Vec3 command=step<50||step>=250?Vec3{}:step<150?Vec3{.3,0,0}:step<200?Vec3{0,0,.3}:Vec3{};
    Json f{{"qpos",array(d->qpos,m->nq)},{"qvel",array(d->qvel,m->nv)},{"command",command}};
    sim.set_command(command);sim.step();auto diag=sim.controller_diagnostics();f.update(diag);
    f["observed_command"]=sim.deployment_metadata()["observation_command"];
    f["ctrl"]=array(d->ctrl,m->nu);f["actuator_force"]=array(d->actuator_force,m->nu);
    f["qfrc_actuator"]=array(d->qfrc_actuator,m->nv);f["contacts"]=d->ncon;
    Json contacts=Json::array();
    for(int i=0;i<d->ncon;++i){
      const auto &contact=d->contact[i];mjtNum force[6];mj_contactForce(m,d,i,force);
      Json bodies=Json::array();for(int g:contact.geom){const char*name=mj_id2name(m,mjOBJ_BODY,m->geom_bodyid[g]);bodies.push_back(name?name:"world");}
      contacts.push_back({{"geom",array(contact.geom,2)},{"bodies",bodies},{"contact_frame_force_torque",array(force,6)},{"distance_m",contact.dist}});
    }
    f["contact_details"]=contacts;
    for(int i=0;i<m->nq;++i)finite=finite&&std::isfinite(d->qpos[i]);
    for(int i=0;i<m->nv;++i)finite=finite&&std::isfinite(d->qvel[i]);
    const auto &obs=diag["observation"];const double z=obs[29].get<double>();
    max_tilt=std::max(max_tilt,std::acos(std::clamp(-z,-1.,1.)));
    frames.push_back(f);for(int k=1;k<4;++k)sim.step();
  }
  std::ofstream(out/"cpu-frames.json")<<frames.dump()<<'\n';
  Json checks{{"finite",finite},{"upright",max_tilt<1.2217304763960306},{"frames",frames.size()==300}};
  const bool passed=finite&&checks["upright"].get<bool>();
  std::ofstream(out/"report.json")<<Json{{"passed",passed},{"checks",checks},{"physics_backend","mujoco"},{"physics_device","cpu"},{"version",mj_versionString()},{"max_tilt_rad",max_tilt},{"policy_steps",300}}.dump(2)<<'\n';
  std::cout<<contract.at("physics").dump()<<'\n';return passed?0:1;
 } catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
