#include "simple_rbt/core.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <map>
using namespace simple_rbt;
namespace fs=std::filesystem;
Json read(const fs::path&p){std::ifstream f(p);return Json::parse(f);}
template<class T>Json arr(const T*p,int n){return std::vector<T>(p,p+n);}
void restore(const Json&j,mjtNum*p,int n){auto v=j.get<std::vector<double>>();if(int(v.size())!=n)throw std::runtime_error("State size mismatch");std::copy(v.begin(),v.end(),p);}
int main(int argc,char**argv){
 if(argc!=6&&argc!=7){std::cerr<<"profile contract index output inspect [native|tracked]\n";return 2;}
 fs::path out(argv[4]);if(!fs::create_directories(out))return 2;
 try{
  const auto contract=read(argv[2]);const int index=std::stoi(argv[3]);auto test=contract.at("rows").at(index);
  SimulationOptions opt;opt.robot="go2w";opt.controller="supermimic";opt.control_mode=argc==7?argv[6]:"native";
  opt.project_root=fs::absolute(argv[1]);opt.scene=read(opt.project_root/"validation.scene.json");
  Simulation sim(opt);auto*m=sim.model();auto*d=sim.data();mj_resetData(m,d);
  restore(contract.at("init_qpos")[index],d->qpos,m->nq);restore(contract.at("init_qvel")[index],d->qvel,m->nv);
  restore(contract.at("init_act")[index],d->act,m->na);restore(contract.at("init_ctrl")[index],d->ctrl,m->nu);mj_forward(m,d);
  std::ofstream(out/"compiled-model.xml")<<sim.xml();
  std::ofstream(out/"initial.json")<<Json{{"qpos",arr(d->qpos,m->nq)},{"qvel",arr(d->qvel,m->nv)},{"act",arr(d->act,m->na)},{"ctrl",arr(d->ctrl,m->nu)}}.dump();
  if(std::stoi(argv[5]))return 0;
  const auto cfg=read(opt.project_root/"assets/catalog.json").at("controllers").at("go2w-supermimic");
  const int base=mj_name2id(m,mjOBJ_BODY,cfg.at("base_body").get<std::string>().c_str());
  const int decimation=std::lround(.02/m->opt.timestep);if(std::abs(decimation*m->opt.timestep-.02)>1e-9)throw std::runtime_error("Clock mismatch");
  std::vector<int> q,v;for(auto&name:cfg.at("joint_names")){int j=mj_name2id(m,mjOBJ_JOINT,name.get<std::string>().c_str());q.push_back(m->jnt_qposadr[j]);v.push_back(m->jnt_dofadr[j]);}
  auto snapshot=[&](){const auto*r=d->xmat+9*base;const auto*p=d->xpos+3*base;mjtNum vel[6];mj_objectVelocity(m,d,mjOBJ_XBODY,base,vel,1);
   std::vector<double>s{p[2],-r[6],-r[7],-r[8],vel[3],vel[4],vel[5],vel[0],vel[1],vel[2]};
   for(int i=0;i<12;++i)s.push_back(d->qpos[q[i]]-cfg.at("default_dof_pos")[i].get<double>());
   for(int i=0;i<16;++i)s.push_back(d->qvel[v[i]]);
   return Json{{"state",s},{"position",arr(p,3)}};
  };
  auto origin=contract.at("origins")[index].get<Vec3>();std::string mode=test.at("mode"),kind=test.at("kind");
  const int level=contract.at("level");double height[6]={.02,.04,.08,.12,.16,.20};
  Json rows=Json::array();int reached=-1,stop=-1;bool cleared=false;std::string reason="timeout";
  for(int step=0;step<1750;++step){
   Vec3 cmd{};if((mode=="traverse"&&step>=100&&step<1600&&reached<0)||(mode=="tracking"&&step>=100&&step<400))cmd=test.at("command").get<Vec3>();
   if(mode=="tracking"&&step>=400&&step<550&&test.value("release_lateral_only",false)){cmd=test.at("command").get<Vec3>();cmd[1]=0.;}
   if(stop<0&&((mode=="traverse"&&(reached>=0||step>=1600))||(mode=="tracking"&&step>=400)||(mode=="hold"&&step>=100)))stop=step;
   sim.set_command(cmd);double peak=0.;std::set<int>wheels;
   for(int sub=0;sub<decimation;++sub){
    sim.step();std::map<int,Vec3> forces;wheels.clear();
    for(int c=0;c<d->ncon;++c){auto&co=d->contact[c];int a=m->geom_bodyid[co.geom[0]],b=m->geom_bodyid[co.geom[1]];
     int body=-1,sign=1;if(m->body_weldid[a]==0&&m->body_weldid[b]!=0)body=b;else if(m->body_weldid[b]==0&&m->body_weldid[a]!=0){body=a;sign=-1;}else continue;
     std::string name=mj_id2name(m,mjOBJ_BODY,body);if(name.find("wheel_link")!=std::string::npos){wheels.insert(body);continue;}
     mjtNum force[6],world[3];mj_contactForce(m,d,c,force);mju_mulMatTVec(world,co.frame,force,3,3);
     auto&f=forces[body];for(int k=0;k<3;++k)f[k]+=sign*world[k];
    }
    for(auto&f:forces)peak=std::max(peak,std::sqrt(f.second[0]*f.second[0]+f.second[1]*f.second[1]+f.second[2]*f.second[2]));
   }
   auto row=snapshot();auto s=row.at("state").get<std::vector<double>>();auto pos=row.at("position").get<Vec3>();
   for(double x:s)if(!std::isfinite(x))throw std::runtime_error("Nonfinite physics: hard stop");
   double x=pos[0]-origin[0],y=pos[1]-origin[1],z=pos[2]-origin[2];double expect=(kind=="up"?1:-1)*5*height[level];
   cleared=cleared||(std::abs(x)>2.8&&std::abs(y)<2.5&&z>expect+.2&&z<expect+.6&&std::hypot(s[1],s[2])<.4&&wheels.size()>=2);
   bool dead=peak>10||std::acos(std::clamp(-s[3],-1.,1.))>1.2217304763960306||std::abs(x)>6.5||std::abs(y)>2.7;
   if(reached<0&&mode=="traverse"&&x>=5.4)reached=step;
   row.update({{"command",cmd},{"nonwheel_peak",peak},{"terminated",dead},{"ctrl",arr(d->ctrl,m->nu)}});rows.push_back(row);
   if(dead){reason=peak>10?"illegal_nonwheel_contact":std::abs(y)>2.7?"lane_exit":"orientation_or_bounds";break;}
   if((mode=="traverse"&&reached>=0&&stop>=0&&step>=stop+149)||(mode=="tracking"&&step>=549)||(mode=="hold"&&step>=599)){reason="completed";break;}
  }
  const bool tracked=opt.control_mode=="tracked";
  if((sim.tracker_metadata().at("updates").get<int>()!=0)!=tracked)
   throw std::runtime_error("Tracker state does not match control mode");
  std::ofstream(out/"trajectory.json")<<rows.dump();
  std::ofstream(out/"report.json")<<Json{{"case",test},{"level",level},{"reason",reason},{"reached_step",reached},{"stop_step",stop},{"stair_clear",cleared},
   {"success",reason=="completed"&&((kind!="up"&&kind!="down")||mode!="traverse"||cleared)},
   {"steps",rows.size()},{"mujoco",mj_versionString()},{"tracker_updates",sim.tracker_metadata().at("updates")},{"deployment",sim.deployment_metadata()}}.dump(2);
  return 0;
 }catch(const std::exception&e){std::ofstream(out/"failure.json")<<Json{{"error",e.what()}}.dump(2);std::cerr<<e.what()<<'\n';return 2;}
}
