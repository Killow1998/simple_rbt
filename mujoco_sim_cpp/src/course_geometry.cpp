#include "course_geometry.hpp"
#include "simple_rbt/robots.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace simple_rbt {
namespace {
std::string geom_name(const mjModel* model, int id) {
  const char* name = mj_id2name(model, mjOBJ_GEOM, id); return name ? name : std::to_string(id);
}
std::vector<bool> descendants(const mjModel* model, std::vector<bool> result) {
  for (int id = 1; id < model->nbody; ++id) if (result[model->body_parentid[id]]) result[id] = true;
  return result;
}
// Optional base-attached support-ball roles. Resolve model IDs independently
// of robot registration names; arbitrary spheres remain body geometry.
std::vector<int> caster_support_ids(const mjModel* model, int base) {
  const char* names[] = {"front_caster_collision", "rear_caster_collision"};
  std::vector<int> result;
  for (const char* name : names) {
    const int geom = mj_name2id(model, mjOBJ_GEOM, name);
    if (geom >= 0) result.push_back(geom);
  }
  if (result.empty()) return result;
  if (result.size() != 2) throw std::runtime_error("Incomplete support caster roles");
  for (int geom : result)
    if (model->geom_bodyid[geom] != base || model->geom_type[geom] != mjGEOM_SPHERE ||
        !(model->geom_contype[geom] || model->geom_conaffinity[geom]))
      throw std::runtime_error("Invalid support caster role: " + geom_name(model, geom));
  return result;
}
}  // namespace
void CourseGeometry::initialize(const Simulation& sim, const std::string& robot) {
    const auto* model = sim.model(); const auto* data = sim.data();
    base_site = mj_name2id(model, mjOBJ_SITE, "base_origin");
    if (base_site < 0) throw std::runtime_error("Formal gates need base_origin");
    base_body = model->site_bodyid[base_site];
    wheel_safety = robot_spec(robot).body_envelope_safety;
    robot_bodies.assign(model->nbody, false); wheel_geoms.assign(model->ngeom, false);
    rolling_contact_gate = robot_spec(robot).continuous_terrain;
    if (rolling_contact_gate) {
      const int root = mj_name2id(model, mjOBJ_BODY, "base");
      if (root <= 0) throw std::runtime_error("Wheel gates need robot base subtree");
      robot_bodies[root] = true; robot_bodies = descendants(model, robot_bodies);
      std::vector<bool> wheels(model->nbody, false);
      for (int body = 1; body < model->nbody; ++body) {
        const char* raw = mj_id2name(model, mjOBJ_BODY, body); const std::string name = raw ? raw : "";
        if (robot_bodies[body] && name.size() >= 6 && name.compare(name.size() - 6, 6, "_wheel") == 0) wheels[body] = true;
      }
      if (std::none_of(wheels.begin(), wheels.end(), [](bool value) { return value; }))
        throw std::runtime_error("Wheel contact gate found no *_wheel assemblies");
      wheels = descendants(model, wheels);
      for (int geom = 0; geom < model->ngeom; ++geom) {
        const bool active = model->geom_contype[geom] || model->geom_conaffinity[geom];
        if (active && robot_bodies[model->geom_bodyid[geom]]) collision_ids.push_back(geom);
        wheel_geoms[geom] = active && wheels[model->geom_bodyid[geom]];
      }
      if (collision_ids.empty()) throw std::runtime_error("No wheel robot collision envelope");
      for (int geom : caster_support_ids(model, root)) wheel_geoms[geom] = true;
    }
    for (int geom = 0; geom < model->ngeom; ++geom) {
      if (model->geom_bodyid[geom] != 0 || geom_name(model, geom).rfind("terrain_", 0) != 0) continue;
      if (model->geom_type[geom] != mjGEOM_BOX) throw std::runtime_error("Formal terrain height requires exact boxes");
      Box box;
      std::copy_n(data->geom_xpos + 3 * geom, 3, box.centre.begin());
      std::copy_n(model->geom_size + 3 * geom, 3, box.half.begin());
      std::copy_n(data->geom_xmat + 9 * geom, 9, box.rotation.begin());
      boxes.push_back(box);
    }
  }
double CourseGeometry::height_at(double x, double y) const {
    if (!std::isfinite(x) || !std::isfinite(y)) throw std::runtime_error("Nonfinite terrain coordinates");
    double result = 0.;
    for (const auto& box : boxes) {
      const Vec3 offset{x - box.centre[0], y - box.centre[1], -box.centre[2]};
      double low = -std::numeric_limits<double>::infinity(), high = std::numeric_limits<double>::infinity();
      bool inside = true;
      for (int axis = 0; axis < 3; ++axis) {
        const double local = box.rotation[axis] * offset[0] + box.rotation[3 + axis] * offset[1] + box.rotation[6 + axis] * offset[2];
        const double direction = box.rotation[6 + axis];
        if (std::abs(direction) < 1.e-14) { if (std::abs(local) > box.half[axis]) inside = false; }
        else {
          const double first = (-box.half[axis] - local) / direction, second = (box.half[axis] - local) / direction;
          low = std::max(low, std::min(first, second)); high = std::min(high, std::max(first, second));
        }
      }
      if (inside && low <= high) result = std::max(result, high);
    }
    return result;
  }
double CourseGeometry::half_width(const Simulation& sim) const {
    const auto* model = sim.model(); const auto* data = sim.data();
    double lower = std::numeric_limits<double>::infinity(), upper = -lower;
    for (int geom : collision_ids) {
      const double* rotation = data->geom_xmat + 9 * geom + 3;  // world Y row
      const double* size = model->geom_size + 3 * geom;
      double support = 0.;
      switch (model->geom_type[geom]) {
        case mjGEOM_BOX: for (int axis = 0; axis < 3; ++axis) support += std::abs(rotation[axis]) * size[axis]; break;
        case mjGEOM_ELLIPSOID: for (int axis = 0; axis < 3; ++axis) support += std::pow(rotation[axis] * size[axis], 2); support = std::sqrt(support); break;
        case mjGEOM_SPHERE: support = size[0]; break;
        case mjGEOM_CYLINDER: support = size[0] * std::hypot(rotation[0], rotation[1]) + size[1] * std::abs(rotation[2]); break;
        case mjGEOM_CAPSULE: support = size[0] + size[1] * std::abs(rotation[2]); break;
        default: throw std::runtime_error("Unsupported collision shape in exact wheel envelope: " + geom_name(model, geom));
      }
      const double centre = data->geom_xpos[3 * geom + 1] - data->site_xpos[3 * base_site + 1];
      lower = std::min(lower, centre - support); upper = std::max(upper, centre + support);
    }
    return std::max(std::abs(lower), std::abs(upper));
  }
std::set<std::string> CourseGeometry::body_contacts(const Simulation& sim) const {
    const auto* model = sim.model(); const auto* data = sim.data(); std::set<std::string> result;
    for (int index = 0; index < data->ncon; ++index) {
      const auto& contact = data->contact[index]; const int a = contact.geom[0], b = contact.geom[1];
      if (a < 0 || b < 0) continue;
      const int ba = model->geom_bodyid[a], bb = model->geom_bodyid[b];
      if (rolling_contact_gate) {
        if (contact.dist > 1.e-6) continue;
        const int robot = ba == 0 && robot_bodies[bb] ? b : bb == 0 && robot_bodies[ba] ? a : -1;
        if (robot >= 0 && !wheel_geoms[robot]) result.insert(geom_name(model, robot) + " / " + geom_name(model, robot == a ? b : a));
      } else if ((ba == 0 && bb == base_body) || (bb == 0 && ba == base_body)) {
        const int robot = ba == base_body ? a : b; const char* name = mj_id2name(model, mjOBJ_GEOM, robot);
        result.insert(name ? name : "unnamed");
      }
    }
    return result;
  }
void FrictionMonitor::Stats::add(const mjtNum* values) {
      if (!count) for (int axis = 0; axis < 5; ++axis) minimum[axis] = maximum[axis] = values[axis];
      else for (int axis = 0; axis < 5; ++axis) { minimum[axis] = std::min(minimum[axis], values[axis]); maximum[axis] = std::max(maximum[axis], values[axis]); }
      ++count;
    }
Json FrictionMonitor::Stats::report() const { return {{"samples", count}, {"effective_friction_min", count ? Json(minimum) : Json(nullptr)}, {"effective_friction_max", count ? Json(maximum) : Json(nullptr)}}; }
void FrictionMonitor::observe(const Simulation& sim) {
    const auto* model = sim.model(); const auto* data = sim.data();
    for (int index = 0; index < data->ncon; ++index) {
      const auto& contact = data->contact[index]; int a = contact.geom[0], b = contact.geom[1];
      if (a < 0 || b < 0) continue;
      const bool world_a = model->geom_bodyid[a] == 0, world_b = model->geom_bodyid[b] == 0;
      if (world_a == world_b) continue;
      const int world = world_a ? a : b, robot = world_a ? b : a;
      total.add(contact.friction); pairs[geom_name(model, world) + "|" + geom_name(model, robot)].add(contact.friction);
      if (!requested.is_null()) {
        const double mu = requested.get<double>(); const std::array<double, 5> target{mu, mu, .02, .01, .01};
        bool mismatch = false;
        for (int axis = 0; axis < 5; ++axis) mismatch = mismatch || !std::isfinite(contact.friction[axis]) || std::abs(contact.friction[axis] - target[axis]) > 1.e-10;
        mismatches += mismatch;
      }
    }
  }
Json FrictionMonitor::report() const {
    Json values = Json::object(); for (const auto& pair : pairs) values[pair.first] = pair.second.report();
    return {{"requested_mu", requested}, {"mixing", requested.is_null() ? "source defaults" : "environment priority2, condim6, (mu,.02,.01)"},
            {"observed_contact_samples", total.count}, {"mismatched_samples", mismatches},
            {"effective_friction_min", total.count ? Json(total.minimum) : Json(nullptr)},
            {"effective_friction_max", total.count ? Json(total.maximum) : Json(nullptr)},
            {"contact_pairs", values}, {"passed", total.count > 0 && mismatches == 0}};
  }
}  // namespace simple_rbt
