#include "simple_rbt/lidar.hpp"
#include "../src/lidar_cuda.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>

using namespace simple_rbt;
namespace {
void check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
void near(double value, double expected, double tolerance, const char* message) {
  if (!(std::abs(value-expected) <= tolerance)) throw std::runtime_error(std::string(message)+": "+std::to_string(value)+" versus "+std::to_string(expected));
}
template<class F> void rejects(F&& f, const char* message) {
  bool thrown=false; try { f(); } catch (const std::exception&) { thrown=true; }
  check(thrown,message);
}
struct Fixture {
  std::string path;
  mjModel* model = nullptr;
  mjData* data = nullptr;
  explicit Fixture(const std::string& extra="") {
    path="/tmp/simple-rbt-lidar-"+std::to_string(getpid())+".xml";
    std::ofstream xml(path);
    xml << "<mujoco><worldbody><geom name='wall' type='plane' pos='5 0 0' quat='.7071067811865476 0 -.7071067811865476 0' size='0 0 .1'/>"
        << "<body name='robot'><freejoint/><geom type='sphere' size='.2' mass='1'/><site name='lidar'/>"
        << "<body name='leg' pos='1 0 0'><joint axis='0 0 1'/><geom type='sphere' size='.2' mass='.1'/></body></body>"
        << extra << "</worldbody></mujoco>";
    xml.close(); char error[1024]{}; model=mj_loadXML(path.c_str(),nullptr,error,sizeof(error));
    if (!model) throw std::runtime_error(error);
    data=mj_makeData(model); mj_forward(model,data);
  }
  ~Fixture() { mj_deleteData(data); mj_deleteModel(model); std::remove(path.c_str()); }
};
struct PatternFile {
  std::string path="/tmp/simple-rbt-pattern-"+std::to_string(getpid())+".csv";
  PatternFile() { std::ofstream(path) << "Time/s,Azimuth/deg,Zenith/deg\n1,0,90\n2,180,90\n3,15,90\n4,180,90\n"; }
  ~PatternFile() { std::remove(path.c_str()); }
};
Pose yaw_pose(double x, double yaw) { return Pose{{x,0,0},{std::cos(yaw/2),0,0,std::sin(yaw/2)}}; }
PoseHistory motion(std::uint64_t start=0) {
  PoseHistory history;
  for (std::uint64_t t=0;t<=250000000;t+=5000000) history.add(start+t,yaw_pose(t*1e-9,.5*t*1e-9));
  return history;
}
void write_directions(const std::string& csv, const std::string& file) {
  CsvPattern pattern(csv); std::ofstream out(file,std::ios::binary);
  for (std::size_t i=0;i<pattern.size();++i) {
    const auto& direction=pattern.ray(i).direction;
    out.write(reinterpret_cast<const char*>(direction.data()),3*sizeof(float));
  }
  std::cout << "pattern_rows=" << pattern.size() << " output=" << file << '\n';
}
void compare_cuda() {
  Fixture fixture("<geom type='box' pos='3 2 0' size='.2 .2 .4'/>"); PatternFile file;
  CsvPattern a(file.path),b(file.path); auto history=motion();
  RollingLidar cpu(fixture.model,LidarBackend::Cpu,a),gpu(fixture.model,LidarBackend::Cuda,b);
  LidarScan x,y;
  cpu.scan(fixture.data,0,40001,history,x); gpu.scan(fixture.data,0,40001,history,y);
  check(x.ray_indices==y.ray_indices && x.offsets_ns==y.offsets_ns && x.lines==y.lines,"CPU/CUDA ray identity mismatch");
  double error=0.; for (std::size_t i=0;i<x.xyz.size();++i) for(int k=0;k<3;++k) error=std::max(error,std::abs(double(x.xyz[i][k])-y.xyz[i][k]));
  check(error<=2e-5,"CPU/CUDA XYZ mismatch");
  check(gpu.ray_capacity()>=40001,"Ray growth truncated input");
  std::cout << "cuda_moving_40001 max_xyz_error_m=" << error << " points=" << x.xyz.size() << '\n';
  // Independent mju_rayGeom contract for inside-box exits, plane sidedness,
  // zero-size infinite plane axes and oblique/parallel boundary rays.
  std::vector<detail::RayGeometry> geoms(1); auto& g=geoms[0];
  g.rotation[0]=g.rotation[4]=g.rotation[8]=1.;
  for (int kind : {int(mjGEOM_PLANE),int(mjGEOM_BOX)}) {
    g.type=kind; g.size[0]=g.size[1]=g.size[2]=1.;
    std::vector<double> origins{0,0,1, 2,0,1, 0,0,-1, 0,0,0, 1,0,0, 2,1,0};
    std::vector<double> rays{0,0,-1, 0,0,-1, 0,0,1, 1,0,0, -1,0,0, -1,0,0};
    auto* state=detail::make_cuda_rays(geoms); std::vector<double> ranges(6);
    detail::trace_cuda_rays(state,origins.data(),rays.data(),6,40.,ranges.data()); detail::free_cuda_rays(state);
    for (std::size_t i=0;i<6;++i) {
      const double ref=mju_rayGeom(g.position,g.rotation,g.size,origins.data()+3*i,rays.data()+3*i,kind,nullptr);
      near(ranges[i],ref,1e-12,"CUDA primitive contract mismatch");
    }
  }
}
void scene_oracle(const std::string& xml, const std::string& csv, const std::string& prefix, bool cuda) {
  char error[2048]{};
  std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(xml.c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
  if(!model)throw std::runtime_error(error);
  std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
  mj_forward(model.get(),data.get());
  CsvPattern a(csv),b(csv); RollingLidar cpu(model.get(),LidarBackend::Cpu,a);
  std::unique_ptr<RollingLidar> gpu;
  if(cuda)gpu=std::make_unique<RollingLidar>(model.get(),LidarBackend::Cuda,b);
  double max_error=0.; std::size_t differences=0,points=0; std::vector<double> cpu_ms,gpu_ms;
  for(unsigned frame=0;frame<40;++frame) {
    PoseHistory history;
    const std::uint64_t start=frame*100000000ULL;
    for(std::uint64_t offset=0;offset<=100000000;offset+=5000000) {
      const double t=(start+offset)*1e-9;
      const double cy=std::cos((.4+.7*t)/2), sy=std::sin((.4+.7*t)/2);
      const double cp=std::cos(.35/2), sp=std::sin(.35/2);
      history.add(start+offset,Pose{{1.5+.6*t,-.15+.05*t,1.2+.02*t},{cy*cp,-sy*sp,cy*sp,sy*cp}});
    }
    LidarScan x,y; auto before=std::chrono::steady_clock::now();
    cpu.scan(data.get(),start,20000,history,x);
    cpu_ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count());
    if(gpu) {
      before=std::chrono::steady_clock::now(); gpu->scan(data.get(),start,20000,history,y);
      gpu_ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count());
      check(x.ray_indices==y.ray_indices && x.offsets_ns==y.offsets_ns && x.lines==y.lines,"Full CSV CPU/CUDA ray identity mismatch");
      for(std::size_t i=0;i<x.xyz.size();++i)for(int k=0;k<3;++k) {
        const double e=std::abs(double(x.xyz[i][k])-y.xyz[i][k]);
        max_error=std::max(max_error,e); differences+=e!=0.;
      }
    }
    points+=x.xyz.size();
    if(frame==0) {
      const auto dump=[&](const std::string& suffix,const auto& values) {
        std::ofstream out(prefix+suffix,std::ios::binary);
        out.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(values[0]));
        check(bool(out),"Cannot write oracle artifact");
      };
      dump(".xyz.bin",x.xyz);dump(".offsets.bin",x.offsets_ns);dump(".lines.bin",x.lines);dump(".indices.bin",x.ray_indices);
    }
  }
  check(max_error<=2e-5,"Full CSV CPU/CUDA geometry discrepancy");
  const auto median=[](std::vector<double> a){std::sort(a.begin(),a.end());return (a[(a.size()-1)/2]+a[a.size()/2])/2;};
  std::cout << "scene=" << xml << " rays=800000 valid=" << points << " max_xyz_error=" << max_error
            << " differing_xyz_components=" << differences << " cpu_median_ms=" << median(cpu_ms)
            << " cuda_median_ms=" << (gpu_ms.empty()?0.:median(gpu_ms)) << '\n';
}
void test_cpu() {
  PatternFile file; CsvPattern pattern(file.path);
  check(pattern.size()==4 && pattern.ray(0).direction==std::array<float,3>{1,0,0},"CSV azimuth/zenith direction");
  pattern.advance(6); check(pattern.cursor()==2 && pattern.emitted()==6,"CSV wrap"); pattern.reset();
  const std::string invalid=file.path+".invalid";
  std::ofstream(invalid) << "Time/s,Azimuth/deg,Zenith/deg\n1,0,90\n3,0,90\n";
  rejects([&]{CsvPattern skipped(invalid);},"Nonconsecutive CSV IDs accepted");
  std::remove(invalid.c_str());
  PoseHistory h; for(std::uint64_t i=0;i<=100;++i) h.add(i*5000000,yaw_pose(i,0));
  check(h.bounds()[0]==245000000 && h.size()==52,"History retention lost predecessor");
  near(h.interpolate(247500000).position[0],49.5,1e-12,"History translation");
  rejects([&]{h.interpolate(244999999);},"History extrapolated");
  rejects([&]{h.add(500000000,yaw_pose(0,0));},"History duplicate accepted");
  rejects([&]{h.add(505000000,Pose{{0,0,0},{1e308,1e308,1e308,1e308}});},"Huge quaternion accepted");
  const std::uint64_t large=9000000000000000000ULL;
  PoseHistory turn; turn.add(large,yaw_pose(0,179.*std::acos(-1.)/180.));
  Pose other=yaw_pose(2,-179.*std::acos(-1.)/180.); for(double& v:other.quaternion)v=-v;
  turn.add(large+10,other); auto middle=turn.interpolate(large+5);
  near(middle.position[0],1.,1e-12,"Large timestamp precision"); near(std::abs(middle.quaternion[3]),1.,1e-12,"SLERP shortest arc");
  Fixture fixture; auto history=motion();
  std::vector<int> original_groups(fixture.model->geom_group,fixture.model->geom_group+fixture.model->ngeom);
  RollingLidar lidar(fixture.model,LidarBackend::Cpu,pattern); LidarScan scan;
  rejects([&]{lidar.scan(fixture.data,250000000,4,history,scan);},"Missing history accepted");
  check(pattern.cursor()==0,"Failed scan consumed pattern");
  pattern.advance(2); lidar.scan(fixture.data,0,6,history,scan);
  check(scan.ray_indices==std::vector<std::uint64_t>{2,0,2},"Miss/wrap ray indices");
  check(scan.offsets_ns==std::vector<std::uint32_t>{0,10000,20000},"Miss/wrap timestamps");
  check(scan.lines==std::vector<std::uint8_t>{2,0,2},"Miss/wrap lines");
  pattern.reset(); lidar.scan(fixture.data,0,20000,history,scan);
  check(scan.rays_cast==20000 && scan.xyz.size()==10000,"Complete 20k frame lost rays");
  double snapshot_error=0.;
  for(std::size_t i=0;i<scan.xyz.size();++i) {
    const double t=scan.offsets_ns[i]*1e-9, angle=.5*t;
    near(t+std::cos(angle)*scan.xyz[i][0]-std::sin(angle)*scan.xyz[i][1],5.,1e-6,"Per-point moving wall geometry");
    snapshot_error=std::max(snapshot_error,std::abs(.1+std::cos(.05)*scan.xyz[i][0]-std::sin(.05)*scan.xyz[i][1]-5.));
  }
  check(snapshot_error>.08,"Scan-end negative control did not distinguish rolling");
  check(std::equal(original_groups.begin(),original_groups.end(),fixture.model->geom_group),"Live model mutated");
  fixture.data->geom_xpos[0]+=.01;
  rejects([&]{lidar.scan(fixture.data,0,4,history,scan);},"Changed static environment accepted");
  Fixture curved("<geom type='sphere' pos='5 2 0' size='.2'/>");
  RollingLidar curved_lidar(curved.model,LidarBackend::Cpu,pattern);
  check(curved_lidar.environment_geom_count()==lidar.environment_geom_count()+1,"Curved environment omitted");
  Fixture moving("<body name='obstacle' pos='3 2 0'><freejoint/><geom type='box' size='.2 .2 .2'/></body>");
  rejects([&]{RollingLidar bad(moving.model,LidarBackend::Cpu,pattern);},"Dynamic obstacle accepted");
  std::cout << "CPU rolling/time/wrap/negative-control/static-state tests passed\n";
}
}
int main(int argc,char** argv) {
  try {
    if(argc==4 && std::string(argv[1])=="--dump-directions") {write_directions(argv[2],argv[3]);return 0;}
    if(argc>=5 && std::string(argv[1])=="--scene-oracle") {scene_oracle(argv[2],argv[3],argv[4],argc==6 && std::string(argv[5])=="cuda");return 0;}
    test_cpu();
    if(argc==2 && std::string(argv[1])=="--cuda") compare_cuda();
    return 0;
  } catch(const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
}
