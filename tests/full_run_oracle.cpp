// CPU-only formal-gate regression fixtures. These deliberately place a compiled
// free body at boundary states via mj_forward; they are not trajectory or
// performance evidence. Only the production CourseMonitor is exercised.
#include "simple_rbt/course.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <set>
#include <unistd.h>

namespace {
using namespace simple_rbt;
namespace fs = std::filesystem;
constexpr double pi = 3.14159265358979323846;
int assertions = 0;
void require(bool condition, const std::string& message) {
  ++assertions;
  if (!condition) throw std::runtime_error(message);
}
void save(const fs::path& path, const Json& value) {
  std::ofstream stream(path); stream << value.dump(2) << '\n';
  if (!stream) throw std::runtime_error("Cannot write fixture " + path.string());
}
struct Fixture {
  fs::path output;
  std::unique_ptr<Simulation> simulation;
  std::unique_ptr<CourseMonitor> observer;
  int free_address = -1, velocity_address = -1;
  Fixture(const fs::path& root, const std::string& name, const std::string& robot,
          double speed = .3, double settle = 0., double friction = -1.,
          const fs::path& model_root = SIMPLE_RBT_PROJECT_ROOT) : output(root / name) {
    if (!fs::create_directories(output)) throw std::runtime_error("Fixture output exists: " + output.string());
    SimulationOptions options; options.project_root = model_root;
    options.robot = robot; options.control_mode = "native"; options.friction = friction;
    simulation = std::make_unique<Simulation>(options);
    auto* model = simulation->model();
    for (int joint = 0; joint < model->njnt; ++joint) if (model->jnt_type[joint] == mjJNT_FREE) {
      free_address = model->jnt_qposadr[joint]; velocity_address = model->jnt_dofadr[joint]; break;
    }
    require(free_address >= 0, "fixture needs actual free body");
    Json config{{"robot", robot}, {"speed_m_s", speed}, {"settle_s", settle},
      {"commanded_distance_budget_m", 8.}, {"friction", friction > 0 ? Json(friction) : Json(nullptr)}};
    observer = std::make_unique<CourseMonitor>(*simulation, config);
  }
  void state(const Vec3& position, double roll = 0., double yaw = 0., Vec3 velocity = {}, double yaw_rate = 0.) {
    auto* model = simulation->model(); auto* data = simulation->data();
    const double cr = std::cos(roll / 2.), sr = std::sin(roll / 2.);
    const double cy = std::cos(yaw / 2.), sy = std::sin(yaw / 2.);
    const Quat quaternion{cy * cr, cy * sr, sy * sr, sy * cr};
    for (int axis = 0; axis < 4; ++axis) data->qpos[free_address + 3 + axis] = quaternion[axis];
    mj_forward(model, data);
    const Pose actual = simulation->pose();
    for (int axis = 0; axis < 3; ++axis) data->qpos[free_address + axis] += position[axis] - actual.position[axis];
    for (int axis = 0; axis < 6; ++axis) data->qvel[velocity_address + axis] = axis < 3 ? velocity[axis] : axis == 5 ? yaw_rate : 0.;
    mj_forward(model, data);
  }
  void sample() { observer->observe(*simulation); }
  Json finish() {
    Json result{{"formal_course", observer->report()}, {"friction", observer->friction_report()},
                {"fixture_only", true}, {"fixture_scope", "formal boundary states; no ROS or resource measurement"}};
    save(output / "course.json", result); return result;
  }
};
void failure_is(const Json& result, const std::string& reason) {
  require(result.at("formal_course").at("failure_reason") == reason, "expected failure: " + reason);
  require(!result.at("formal_course").at("success").get<bool>(), "failure cannot pass route");
}
} // namespace

int main(int argc, char** argv) {
  try {
    const fs::path output = argc > 1 ? fs::path(argv[1]) : fs::temp_directory_path() / ("simple-rbt-formal-fixtures-" + std::to_string(getpid()));
    if (!fs::create_directories(output)) throw std::runtime_error("Output already exists");
    Json results = Json::object();
    {
      for (double angle : {44.9, 45.1}) {
        const std::string name = angle < 45 ? "wheel_tilt_below" : "wheel_tilt_above";
        Fixture fixture(output, name, "scout_mini"); fixture.state({0, 0, 2}, angle * pi / 180.); fixture.sample();
        auto result = fixture.finish(); results[name] = result["formal_course"];
        require(result["formal_course"]["thresholds"]["max_tilt_deg"] == 45., "wheel tilt gate remains 45 degrees");
        if (angle > 45) failure_is(result, "tilt_limit"); else require(result["formal_course"]["failure_reason"].is_null(), "44.9 degree wheel body is within gate");
      }
      Fixture contact(output, "chassis_contact", "scout_mini"); contact.state({0, 0, .02}); contact.sample();
      auto result = contact.finish(); results["chassis_contact"] = result["formal_course"];
      failure_is(result, "torso_environment_contact");
      require(!result["formal_course"]["final"]["torso_contact_geoms"].empty(), "actual chassis contact named");
      Fixture friction(output, "explicit_friction", "scout_mini", .3, 0., .4);
      for (int step = 0; step < 60; ++step) { friction.simulation->step(); friction.sample(); }
      result = friction.finish(); results["explicit_friction"] = result["friction"];
      require(result["friction"]["passed"].get<bool>(), "actual contacts preserve explicit friction");
      require(result["friction"]["effective_friction_min"][0] == .4, "effective sliding coefficient is requested .4");
    }
    {
      double widths[2]{};
      for (int turn = 0; turn < 2; ++turn) {
        const std::string name = turn ? "yaw90_world_corridor" : "yaw0_world_corridor";
        Fixture fixture(output, name, "fr_mini"); fixture.state({2, .65, 2}, 0., turn * pi / 2.); fixture.sample();
        auto result = fixture.finish(); results[name] = result["formal_course"];
        widths[turn] = result["formal_course"]["thresholds"]["corridor_half_width_m"];
        if (turn) failure_is(result, "left_course_corridor"); else require(result["formal_course"]["failure_reason"].is_null(), "unrotated FR inside corridor");
      }
      require(widths[1] < widths[0] - .2, "rotated long rear-origin body increases world Y extent");
    }
    {
      for (double angle : {49.9, 50.1}) {
        const std::string name = angle < 50 ? "legacy_tilt_below" : "legacy_tilt_above";
        Fixture fixture(output, name, "diffdrive"); fixture.state({0, 0, 2}, angle * pi / 180.); fixture.sample();
        auto result = fixture.finish(); results[name] = result["formal_course"];
        require(result["formal_course"]["thresholds"]["max_tilt_deg"] == 50., "legacy non-wheel envelope gate remains 50 degrees");
        if (angle > 50) failure_is(result, "tilt_limit"); else require(result["formal_course"]["failure_reason"].is_null(), "49.9 degree body within legacy gate");
      }
      // Actual compiled contacts: support balls touch at z=.154 while chassis
      // clearance and legacy tilt gates remain valid. Naming cannot excuse a
      // different sphere or the real chassis touching the floor.
      Fixture supports(output, "diffdrive_support_contacts", "diffdrive");
      supports.state({0, 0, .154});
      std::set<int> touched_supports;
      const auto* support_model = supports.simulation->model();
      for (int index = 0; index < supports.simulation->data()->ncon; ++index) {
        const auto& contact = supports.simulation->data()->contact[index];
        for (int geom : contact.geom) {
          const char* name = geom >= 0 ? mj_id2name(support_model, mjOBJ_GEOM, geom) : nullptr;
          if (name && (std::string(name) == "front_caster_collision" || std::string(name) == "rear_caster_collision"))
            touched_supports.insert(geom);
        }
      }
      require(touched_supports.size() == 2, "both actual support balls touch ground");
      supports.sample(); auto support_result = supports.finish();
      results["diffdrive_support_contacts"] = support_result["formal_course"];
      require(support_result["formal_course"]["failure_reason"].is_null(), "normal rolling/support contacts are not torso collision");
      require(support_result["formal_course"]["final"]["torso_contact_geoms"].empty(), "support IDs excluded precisely");
      require(support_result["formal_course"]["thresholds"]["max_tilt_deg"] == 50. &&
              support_result["formal_course"]["thresholds"]["minimum_base_clearance_m"] == .1,
              "support semantics do not change legacy safety thresholds");
      Fixture torso(output, "diffdrive_true_chassis_contact", "diffdrive");
      torso.state({0, 0, .14}, pi / 4.); torso.sample();
      auto torso_result = torso.finish(); results["diffdrive_true_chassis_contact"] = torso_result["formal_course"];
      failure_is(torso_result, "torso_environment_contact");
      require(torso_result["formal_course"]["final"]["base_clearance_m"].get<double>() > .1 &&
              torso_result["formal_course"]["final"]["tilt_deg"].get<double>() < 50.,
              "true contact fails independently of clearance/tilt gates");
      // Compile a fresh test-only XML so collision BVH/bounds reflect the
      // added sphere. Never mutate a compiled geom type with stale BVH data.
      const fs::path sphere_root = output / "nonwheel-sphere-model";
      const fs::path sphere_assets = sphere_root / "assets";
      fs::create_directories(sphere_assets / "models");
      fs::create_directory_symlink(fs::path(SIMPLE_RBT_PROJECT_ROOT)/"assets/visual",sphere_assets/"visual");
      fs::copy_file(fs::path(SIMPLE_RBT_PROJECT_ROOT) / "assets/catalog.json", sphere_assets / "catalog.json");
      std::ifstream original(fs::path(SIMPLE_RBT_PROJECT_ROOT) / "assets/models/diffdrive-none.xml");
      std::string sphere_xml{std::istreambuf_iterator<char>(original), {}};
      const auto marker = sphere_xml.find("<site name=\"base_origin\"");
      require(marker != std::string::npos, "real diffdrive fixture source found");
      sphere_xml.insert(marker, "<geom name=\"body_sphere\" type=\"sphere\" size=\".025\" pos=\"0 .1 -.13\" mass=\".01\" friction=\".001 .001 .001\"/>\n");
      std::ofstream(sphere_assets / "models/diffdrive-none.xml") << sphere_xml;
      Fixture sphere(output, "diffdrive_nonwheel_sphere_contact", "diffdrive", .3, 0., -1., sphere_root);
      sphere.state({0, 0, .154}); sphere.sample();
      auto sphere_result = sphere.finish(); results["diffdrive_nonwheel_sphere_contact"] = sphere_result["formal_course"];
      failure_is(sphere_result, "torso_environment_contact");
      require(sphere_result["formal_course"]["final"]["torso_contact_geoms"] == Json::array({"body_sphere / floor"}),
              "same-shape nonwheel sphere is still identified as body contact");
      for (const std::string change : {"missing", "wrong_shape", "wrong_body", "disabled"}) {
        const fs::path variant_root = output / ("invalid-support-" + change + "-model");
        const fs::path variant_assets = variant_root / "assets";
        fs::create_directories(variant_assets / "models");
        fs::create_directory_symlink(fs::path(SIMPLE_RBT_PROJECT_ROOT)/"assets/visual",variant_assets/"visual");
        fs::copy_file(fs::path(SIMPLE_RBT_PROJECT_ROOT) / "assets/catalog.json", variant_assets / "catalog.json");
        std::ifstream source(fs::path(SIMPLE_RBT_PROJECT_ROOT) / "assets/models/diffdrive-none.xml");
        std::string xml{std::istreambuf_iterator<char>(source), {}};
        const auto start = xml.find("<geom name=\"front_caster_collision\"");
        const auto end = xml.find("/>", start) + 2;
        std::string tag = xml.substr(start, end - start);
        if (change == "missing")
          tag.replace(tag.find("front_caster_collision"), std::string("front_caster_collision").size(), "ordinary_front_sphere");
        else if (change == "wrong_shape") {
          tag.replace(tag.find("type=\"sphere\""), std::string("type=\"sphere\"").size(), "type=\"box\"");
          tag.replace(tag.find("size=\".025\""), std::string("size=\".025\"").size(), "size=\".025 .025 .025\"");
        } else if (change == "disabled")
          tag.insert(tag.size() - 2, " contype=\"0\" conaffinity=\"0\"");
        xml.replace(start, end - start, change == "wrong_body" ? "" : tag);
        if (change == "wrong_body") {
          const auto wheel = xml.find("<body name=\"left_wheel\"");
          xml.insert(xml.find('>', wheel) + 1, tag);
        }
        std::ofstream(variant_assets / "models/diffdrive-none.xml") << xml;
        bool rejected_role = false;
        try { Fixture invalid(output, "invalid_support_" + change, "diffdrive", .3, 0., -1., variant_root); }
        catch (const std::runtime_error& error) {
          rejected_role = std::string(error.what()).find("support caster role") != std::string::npos;
        }
        require(rejected_role, "partial/invalid support role must fail closed: " + change);
        results["invalid_support_" + change] = {{"rejected", rejected_role}};
      }
      Fixture no_casters(output, "wheel_without_casters", "fr_mini");
      require(mj_name2id(no_casters.simulation->model(), mjOBJ_GEOM, "front_caster_collision") < 0 &&
              mj_name2id(no_casters.simulation->model(), mjOBJ_GEOM, "rear_caster_collision") < 0,
              "ordinary wheel model has no optional caster roles");
      for (int step = 0; step < 50; ++step) no_casters.simulation->step();
      require(no_casters.simulation->data()->ncon > 0, "ordinary wheels actually contact floor");
      no_casters.sample(); auto no_caster_result = no_casters.finish();
      require(no_caster_result["formal_course"]["failure_reason"].is_null(), "wheel model without casters remains valid");
      results["wheel_without_casters"] = no_caster_result["formal_course"];
      Fixture low(output, "legacy_clearance", "diffdrive"); low.state({0, 0, .099}); low.sample();
      auto result = low.finish(); results["legacy_clearance"] = result["formal_course"]; failure_is(result, "base_clearance_limit");
      Fixture collision_before_finish(output, "finish_does_not_override_tilt", "diffdrive");
      collision_before_finish.state({6.1, 0, 2}, 51 * pi / 180.); collision_before_finish.sample();
      result = collision_before_finish.finish(); failure_is(result, "tilt_limit"); results["finish_does_not_override_tilt"] = result["formal_course"];
      struct TrackingCase { const char* name; double forward, side, yaw; int steps; bool passed; };
      const std::vector<TrackingCase> cases{{"speed_exact", .3, 0, 0, 300, true}, {"speed_error", .36, 0, 0, 300, false},
          {"lateral_absolute", .3, .11, 0, 300, false}, {"yaw_absolute", .3, 0, .11, 300, false},
          {"short_window", .3, 0, 0, 50, false}};
      for (const auto& test : cases) {
        Fixture fixture(output, test.name, "diffdrive");
        for (int step = 0; step < test.steps; ++step) {
          fixture.simulation->step(); fixture.state({step + 1 == test.steps ? 6.1 : 0., 0, 2}, 0., 0.,
              {test.forward, step % 2 ? test.side : -test.side, 0}, step % 2 ? test.yaw : -test.yaw); fixture.sample();
        }
        result = fixture.finish(); results[test.name] = result["formal_course"];
        require(result["formal_course"]["success"].get<bool>(), "fixture reaches finish with no safety violation");
        require(result["formal_course"]["speed_tracking"]["passed"].get<bool>() == test.passed, std::string("original speed gate: ") + test.name);
        require(result["formal_course"]["speed_tracking"]["thresholds"]["mean_forward_error_m_s"] == .05, "no speed threshold relaxation");
        require(result["formal_course"]["speed_tracking"]["samples"] == test.steps, "only actual observed samples counted");
        if (test.passed) {
          require(!fixture.observer->should_stop(*fixture.simulation), "finish retains deadman observation tail");
          for (int step = 0; step < static_cast<int>(std::ceil(2. / fixture.simulation->model()->opt.timestep)); ++step) { fixture.simulation->step(); fixture.state({6.1, 0, 2}); fixture.sample(); }
          require(fixture.observer->should_stop(*fixture.simulation), "stop after actual 2 second tail");
          result = fixture.finish();
          require(result["formal_course"]["speed_tracking"]["samples"] == test.steps, "deadman tail excluded from speed window");
        }
      }
    }
    save(output / "results.json", {{"passed", true}, {"assertions", assertions}, {"cases", results},
        {"scope", "CPU synthetic boundary fixtures against real compiled model / public CourseMonitor; no trajectory or performance claim"}});
    std::cout << "full_run_oracle: " << assertions << " assertions passed; " << output << '\n';
    return 0;
  } catch (const std::exception& error) { std::cerr << "full_run_oracle failed: " << error.what() << '\n'; return 1; }
}
