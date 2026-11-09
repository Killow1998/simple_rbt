#include "simple_rbt/core.hpp"
#include <chrono>
#include <fstream>
#include <sys/resource.h>
using namespace simple_rbt;
namespace fs=std::filesystem;
namespace {
template<class T>Json arr(const T*p,int n){return std::vector<T>(p,p+n);}
}
void restore_trace(Simulation &sim,const fs::path &path,int row){
 Json c;std::ifstream(path)>>c;auto*m=sim.model();auto*d=sim.data();
 if(c.at("rows").at(row).at("policy")!="four_hour"||c["rows"][row]["kind"]!="up"||c["rows"][row]["variant"]!=0||c["level"]!=5)
  throw std::invalid_argument("Trace requires frozen 4h level5 up nominal input");
 auto copy=[&](const char*key,double*p,int n){auto v=c.at(key).at(row).get<std::vector<double>>();if(int(v.size())!=n)throw std::invalid_argument("Initial state shape mismatch");std::copy(v.begin(),v.end(),p);};
 copy("init_qpos",d->qpos,m->nq);copy("init_qvel",d->qvel,m->nv);copy("init_act",d->act,m->na);copy("init_ctrl",d->ctrl,m->nu);mj_forward(m,d);
}
bool force_trace(Simulation &sim,const fs::path &out){
 auto*m=sim.model();auto*d=sim.data();std::ofstream file(out/"force-trace.jsonl");
 auto start=std::chrono::steady_clock::now();int count=0;std::string reason="duration";
 for(int i=0;i<4000;++i){
  const Vec3 request=i<400?Vec3{}:Vec3{.5,0,0};Json pre{{"qpos",arr(d->qpos,m->nq)},{"qvel",arr(d->qvel,m->nv)}};
  sim.set_command(request);sim.step();++count;
  Json contacts=Json::array();
  for(int j=0;j<d->ncon;++j){const auto&c=d->contact[j];mjtNum force[6];mj_contactForce(m,d,j,force);
   Json bodies=Json::array();bool wheel=false,nonwheel=false,world=false;
   for(int g:c.geom){const auto b=m->geom_bodyid[g];const char*n=mj_id2name(m,mjOBJ_BODY,b);std::string name=n?n:"world";bodies.push_back(name);
    if(m->body_weldid[b]==0)world=true;else if(name.find("wheel_link")!=std::string::npos)wheel=true;else nonwheel=true;}
   contacts.push_back({{"geom",arr(c.geom,2)},{"bodies",bodies},{"wheel_ground",world&&wheel},{"nonwheel_ground",world&&nonwheel},
    {"frame",arr(c.frame,9)},{"pos",arr(c.pos,3)},{"distance_m",c.dist},{"force_torque",arr(force,6)}});
  }
  const auto pose=sim.pose();mjtNum r[9];mju_quat2Mat(r,pose.quaternion.data());const double tilt=std::acos(std::clamp(r[8],-1.,1.));
  file<<Json{{"step",i},{"stamp_ns",sim.stamp_ns()},{"requested",request},{"consumed",sim.deployment_metadata()["observation_command"]},
   {"pre",pre},{"post_qpos",arr(d->qpos,m->nq)},{"post_qvel",arr(d->qvel,m->nv)},
   {"ctrl",arr(d->ctrl,m->nu)},{"actuator_force",arr(d->actuator_force,m->nu)},{"qfrc_actuator",arr(d->qfrc_actuator,m->nv)},
   {"base_position",pose.position},{"tilt_rad",tilt},{"contacts",contacts}}.dump()<<'\n';
  if(tilt>1.2217304763960306){reason="tilt";break;}
  struct rusage usage{};getrusage(RUSAGE_SELF,&usage);
  if(usage.ru_maxrss>2*1024*1024||std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()>120||file.tellp()>100000000){reason="resource_budget";break;}
 }
 file.close();
 Json report{{"passed",reason=="duration"},{"reason",reason},{"physics_backend","mujoco"},{"physics_device","cpu"},{"version",mj_versionString()},
  {"physics_steps",count},{"simulation_s",sim.stamp_ns()*1e-9},
  {"force_sample_phase","post mj_step + mj_forward at integrated state; instantaneous MuJoCo solution, not interval impulse or applied-step average"},
  {"ctrl_units","raw MuJoCo actuator order, identified by contract.json actuators; joint-position targets rad and wheel-velocity targets rad/s; never torque"},
  {"actuator_force_units","native actuator force (N*m for joint motors); qfrc_actuator in generalized coordinates"},
  {"contact_units","contact-frame normal/tangent forces N, torques N*m; raw frame and bodies retained"}};
 std::ofstream(out/"trace-report.json")<<report.dump(2)<<'\n';return reason=="duration";
}
