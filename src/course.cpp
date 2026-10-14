#include "simple_rbt/course.hpp"
#include "course_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace simple_rbt {
namespace {
constexpr double kPi = 3.14159265358979323846;
bool finite(const Vec3& values) { for (double x : values) if (!std::isfinite(x)) return false; return true; }
}  // namespace
struct CourseMonitor::Impl {
  Json config, profile, parameters, thresholds, last = nullptr, final_clearance = nullptr;
  Json transitions = Json::array();
  std::string phase, failure;
  CourseGeometry geometry;
  FrictionMonitor friction;
  Simulation* sim = nullptr;
  bool success = false, entered = false;
  std::uint64_t finish_stamp = 0, tracking_samples = 0, contact_steps = 0;
  double timestep = 0., finish_x = 0., entry_x = 0., exit_x = 0., width = 0., duration = 0.;
  double minimum_clearance = std::numeric_limits<double>::infinity(), maximum_tilt = 0., minimum_corridor = 0.;
  Vec3 previous_position{}, velocity_sum{}, angular_sum{}, linear_error_squared{};
  double absolute_lateral_sum = 0., absolute_yaw_sum = 0., yaw_squared = 0.;
  std::string phase_at(double x) const {
    if (x >= finish_x) return "complete";
    if (x < entry_x) return "approach";
    const std::string name = profile.at("name");
    if (name == "ramp" || name == "stairs") {
      if (x < parameters.at("ascent_end_x_m").get<double>()) return "ascent";
      if (x < parameters.at("landing_end_x_m").get<double>()) return "top";
      if (x < parameters.at("descent_end_x_m").get<double>()) return "descent";
      return "exit";
    }
    if ((name == "rough" || name == "hills") && x >= exit_x) return "exit";
    return "traversal";
  }
  Json clearance(const Vec3& position, bool contact) const {
    const double x = position[0], y = position[1], height = geometry.height_at(x, y);
    const double before = geometry.height_at(x - 1.e-5, y), after = geometry.height_at(x + 1.e-5, y);
    const double previous = geometry.height_at(previous_position[0], previous_position[1]);
    std::vector<double> edges, crossed;
    if (profile.at("name") == "stairs") {
      const double tread = parameters.at("tread_length_m");
      for (int index = 0; index < parameters.at("ascending_steps").get<int>(); ++index)
        edges.push_back(parameters.at("ascent_start_x_m").get<double>() + index * tread);
      for (int index = 0; index < parameters.at("descending_risers").get<int>(); ++index)
        edges.push_back(parameters.at("descent_start_x_m").get<double>() + index * tread);
    }
    double nearest = std::numeric_limits<double>::infinity();
    for (double edge : edges) {
      nearest = std::min(nearest, std::abs(x - edge));
      if (std::min(previous_position[0], x) < edge && edge <= std::max(previous_position[0], x)) crossed.push_back(edge);
    }
    return {{"terrain_height_at_base_xy_m", height}, {"base_origin_height_m", position[2]},
            {"base_clearance_m", position[2] - height}, {"torso_contact", contact},
            {"previous_terrain_height_at_base_xy_m", previous}, {"terrain_height_change_since_previous_step_m", height - previous},
            {"height_before_after_10um_m", {before, after}},
            {"at_vertical_height_discontinuity", (profile.at("name") == "stairs" || profile.at("name") == "rough") && std::abs(after - before) > 1.e-8},
            {"nearest_stair_riser_distance_m", std::isfinite(nearest) ? Json(nearest) : Json(nullptr)},
            {"crossed_stair_riser_x_m", crossed},
            {"definition", "Vertical gap at base-origin XY; higher surface at an edge. Low gap alone is not proof of falling or torso contact."}};
  }
  void monitor(std::uint64_t stamp, const Pose& pose, const Vec3& linear, const Vec3& angular) {
    if (success || !failure.empty()) return;
    const auto* data = sim->data(); const double* rotation = data->site_xmat + 9 * geometry.base_site;
    bool valid = finite(pose.position) && finite(linear) && finite(angular);
    for (double value : pose.quaternion) valid = valid && std::isfinite(value);
    for (int index = 0; index < 9; ++index) valid = valid && std::isfinite(rotation[index]);
    if (!valid) { failure = "nonfinite_state"; return; }
    if (geometry.wheel_safety) {
      const double corridor = width / 2. - geometry.half_width(*sim) - .1;
      thresholds["corridor_half_width_m"] = corridor; minimum_corridor = std::min(minimum_corridor, corridor);
    }
    const auto contacts = geometry.body_contacts(*sim);
    const double height = geometry.height_at(pose.position[0], pose.position[1]);
    if (!std::isfinite(height)) { failure = "nonfinite_terrain_height"; return; }
    const double clearance_m = pose.position[2] - height;
    const double tilt = std::acos(std::clamp(rotation[8], -1., 1.)) * 180. / kPi;
    minimum_clearance = std::min(minimum_clearance, clearance_m); maximum_tilt = std::max(maximum_tilt, tilt);
    entered = entered || pose.position[0] >= entry_x; contact_steps += !contacts.empty();
    const std::string next_phase = phase_at(pose.position[0]);
    if (next_phase != phase) transitions.push_back({{"phase", next_phase}, {"stamp_ns", stamp}, {"base_x_m", pose.position[0]}});
    phase = next_phase;
    if (stamp * 1.e-9 >= config.at("settle_s").get<double>()) {
      ++tracking_samples;
      const double speed = config.at("speed_m_s");
      for (int axis = 0; axis < 3; ++axis) {
        velocity_sum[axis] += linear[axis]; angular_sum[axis] += angular[axis];
        const double error = linear[axis] - (axis == 0 ? speed : 0.); linear_error_squared[axis] += error * error;
      }
      absolute_lateral_sum += std::abs(linear[1]); absolute_yaw_sum += std::abs(angular[2]); yaw_squared += angular[2] * angular[2];
    }
    if (tilt > thresholds.at("max_tilt_deg").get<double>()) failure = "tilt_limit";
    else if (!geometry.wheel_safety && clearance_m < .1) failure = "base_clearance_limit";
    else if (!contacts.empty()) failure = "torso_environment_contact";
    else if (entered && std::abs(pose.position[1]) > thresholds.at("corridor_half_width_m").get<double>()) failure = "left_course_corridor";
    else if (pose.position[0] >= finish_x) { success = true; finish_stamp = stamp; }
    else if (stamp * 1.e-9 + 1.e-12 >= duration) failure = "time_limit_before_finish";
    last = {{"stamp_ns", stamp}, {"position_m", pose.position}, {"tilt_deg", tilt}, {"terrain_height_m", height},
            {"base_clearance_m", clearance_m}, {"torso_contact_geoms", contacts}, {"course_phase", phase}};
    if (success || !failure.empty()) final_clearance = clearance(pose.position, !contacts.empty());
    previous_position = pose.position;
  }
  Json tracking() const {
    const double limit = std::max(.05, .15 * config.at("speed_m_s").get<double>());
    Json result{{"thresholds", {{"mean_forward_error_m_s", limit}, {"mean_absolute_lateral_velocity_m_s", .1},
                                {"mean_absolute_yaw_rate_rad_s", .1}, {"minimum_tracking_window_s", 1.}, {"rmse_is_a_pass_criterion", false}}},
                {"samples", tracking_samples}, {"window", "after settle through formal finish/failure; excludes deadman tail"}, {"passed", false}};
    if (!tracking_samples) return result;
    const double count = static_cast<double>(tracking_samples), seconds = count * timestep;
    const double error = std::abs(velocity_sum[0] / count - config.at("speed_m_s").get<double>());
    const double lateral = absolute_lateral_sum / count, yaw = absolute_yaw_sum / count;
    std::array<double, 6> mean{}; Vec3 rmse{};
    for (int axis = 0; axis < 3; ++axis) {
      mean[axis] = velocity_sum[axis] / count; mean[axis + 3] = angular_sum[axis] / count;
      rmse[axis] = std::sqrt(linear_error_squared[axis] / count);
    }
    result.update({{"tracking_seconds", seconds}, {"mean_forward_error_m_s", error},
                   {"mean_absolute_lateral_velocity_m_s", lateral}, {"mean_absolute_yaw_rate_rad_s", yaw},
                   {"mean_velocity_base_m_s_rad_s", mean}, {"linear_rmse_by_axis_m_s", rmse},
                   {"yaw_rate_rmse_rad_s", std::sqrt(yaw_squared / count)},
                   {"passed", seconds >= 1. && error <= limit && lateral <= .1 && yaw <= .1}});
    return result;
  }
  Json course_report() const {
    return {{"success", success}, {"failure_reason", failure.empty() ? Json(nullptr) : Json(failure)},
            {"finish_stamp_ns", success ? Json(finish_stamp) : Json(nullptr)}, {"thresholds", thresholds},
            {"phase_transitions", transitions}, {"final", last},
            {"minimum_base_clearance_m", std::isfinite(minimum_clearance) ? Json(minimum_clearance) : Json(nullptr)},
            {"max_tilt_deg", maximum_tilt}, {"torso_contact_steps", contact_steps}, {"minimum_corridor_half_width_m", minimum_corridor},
            {"final_clearance_observation", final_clearance},
            {"failure_interpretation", "Safety-gate failures do not all mean a physical fall; the reason and actual contact are recorded."},
            {"speed_tracking", tracking()}};
  }
};
CourseMonitor::CourseMonitor(Simulation& sim, const Json& config) : impl_(std::make_unique<Impl>()) {
  auto& p = *impl_; p.sim = &sim; p.config = config;
  p.friction.requested = config.at("friction");
  p.profile = sim.terrain_metadata(); p.parameters = p.profile.at("parameters");
  p.geometry.initialize(sim, p.config.at("robot")); p.timestep = sim.model()->opt.timestep;
  p.finish_x = p.profile.at("finish_x_m"); p.entry_x = p.profile.at("entry_x_m");
  p.exit_x = p.profile.at("exit_x_m"); p.width = p.profile.at("width_m");
  const double budget = std::max(p.config.at("commanded_distance_budget_m").get<double>(), p.finish_x + 2.);
  p.duration = p.config.at("settle_s").get<double>() + budget / p.config.at("speed_m_s").get<double>();
  const double corridor = p.width / 2. - (p.geometry.wheel_safety ? p.geometry.half_width(sim) + .1 : .25);
  p.minimum_corridor = corridor;
  p.thresholds = {{"finish_x_m", p.finish_x}, {"max_tilt_deg", p.geometry.wheel_safety ? 45. : 50.},
      {"minimum_base_clearance_m", p.geometry.wheel_safety ? Json(nullptr) : Json(.1)},
      {"base_clearance_is_diagnostic_only", p.geometry.wheel_safety}, {"corridor_half_width_m", corridor},
      {"corridor_enforcement", "after first entry, including backtracking"}, {"torso_environment_contact_is_failure", true},
      {"maximum_simulation_duration_s", p.duration}, {"effective_distance_budget_m", budget}};
  p.previous_position = sim.pose().position; p.entered = p.previous_position[0] >= p.entry_x;
  p.phase = p.phase_at(p.previous_position[0]);
  p.transitions.push_back({{"phase", p.phase}, {"stamp_ns", 0}, {"base_x_m", p.previous_position[0]}});
}
CourseMonitor::~CourseMonitor() = default;
void CourseMonitor::observe(Simulation& sim) {
  impl_->friction.observe(sim);
  impl_->monitor(sim.stamp_ns(), sim.pose(), sim.linear_velocity(), sim.angular_velocity());
}
bool CourseMonitor::should_stop(const Simulation& sim) const {
  return !impl_->failure.empty() || (impl_->success && sim.stamp_ns() >= impl_->finish_stamp + 2000000000ULL);
}
void CourseMonitor::mark_error() {
  impl_->success = false;
  if (impl_->failure.empty()) impl_->failure = "simulation_or_sensor_error";
}
Json CourseMonitor::report() const { return impl_->course_report(); }
Json CourseMonitor::friction_report() const { return impl_->friction.report(); }
const Json& CourseMonitor::profile() const { return impl_->profile; }
double CourseMonitor::timestep() const { return impl_->timestep; }
double CourseMonitor::finish_x() const { return impl_->finish_x; }
}  // namespace simple_rbt
