#include "user_scene.hpp"
#include <cmath>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
namespace simple_rbt {
namespace {
void require(bool ok, const char *message) {
  if (!ok)
    throw std::invalid_argument(message);
}
std::map<std::string, double> floors(const Json &s) {
  std::map<std::string, double> out;
  for (const auto &f : s.at("floors"))
    out.emplace(f.at("id"), f.at("height"));
  return out;
}
template <class T> std::string numbers(const T &v) {
  std::ostringstream out;
  out << std::setprecision(17);
  for (auto x : v)
    out << x << ' ';
  return out.str();
}
void vector(const Json &v, unsigned count, double bound) {
  require(v.is_array() && v.size() == count, "Scene vector has wrong size");
  for (const auto &x : v) {
    double n = x.get<double>();
    require(std::isfinite(n) && std::abs(n) <= bound,
            "Invalid scene coordinate");
  }
}
void identity(const Json &x, std::set<std::string> &ids) {
  auto id = x.at("id").get<std::string>(),
       name = x.at("name").get<std::string>();
  require(!id.empty() && id.size() <= 80 && ids.insert(id).second,
          "Scene IDs must be unique");
  require(!name.empty() && name.size() <= 160,
          "Scene name is empty or too long");
}
void place_fixed_base(tinyxml2::XMLElement *world, const Json &scene) {
  for (auto *body = world->FirstChildElement("body"); body;
       body = body->NextSiblingElement("body")) {
    if (body->FirstChildElement("joint") ||
        body->FirstChildElement("freejoint"))
      continue;
    for (auto *site = body->FirstChildElement("site"); site;
         site = site->NextSiblingElement("site")) {
      const char *name = site->Attribute("name");
      if (!name || std::string(name) != "base_origin")
        continue;
      Vec3 position{};
      Quat quaternion{1, 0, 0, 0};
      if (auto *text = body->Attribute("pos")) {
        std::istringstream input(text);
        for (auto &v : position)
          input >> v;
      }
      if (auto *text = body->Attribute("quat")) {
        std::istringstream input(text);
        for (auto &v : quaternion)
          input >> v;
      }
      const auto spawn = scene.at("spawn").at("position").get<Vec3>();
      for (int i = 0; i < 3; ++i)
        position[i] += spawn[i];
      position[2] += floors(scene).at(scene.at("spawn").at("floor"));
      const double half =
          scene.at("spawn").at("yaw_deg").get<double>() * std::acos(-1.) / 360.;
      const Quat yaw{std::cos(half), 0, 0, std::sin(half)};
      Quat result;
      mju_mulQuat(result.data(), yaw.data(), quaternion.data());
      // Place fixed bodies before compilation so static collision bounds and
      // the exported model use the same transform as the rendered scene.
      body->SetAttribute("pos", numbers(position).c_str());
      body->SetAttribute("quat", numbers(result).c_str());
      if (auto *statistic = world->Parent()->FirstChildElement("statistic")) {
        if (const char *text = statistic->Attribute("center")) {
          Vec3 center{}, placed{};
          std::istringstream input(text);
          for (auto &v : center)
            input >> v;
          mju_rotVecQuat(placed.data(), center.data(), yaw.data());
          for (int i = 0; i < 3; ++i)
            placed[i] += spawn[i];
          placed[2] += floors(scene).at(scene.at("spawn").at("floor"));
          statistic->SetAttribute("center", numbers(placed).c_str());
        }
      }
      return;
    }
  }
}
} // namespace
void validate_user_scene(const Json &s) {
  require(s.at("version") == 1, "Unsupported scene version");
  std::set<std::string> ids;
  identity(s, ids);
  require(s.at("floors").is_array() && !s.at("floors").empty() &&
              s.at("floors").size() <= 32,
          "Scene requires 1..32 floors");
  for (const auto &f : s.at("floors")) {
    identity(f, ids);
    double h = f.at("height");
    require(std::isfinite(h) && std::abs(h) <= 100, "Invalid floor height");
  }
  auto heights = floors(s);
  require(s.at("objects").is_array() && s.at("objects").size() <= max_scene_boxes,
          "Scene exceeds its physical geometry budget");
  for (const auto &o : s.at("objects")) {
    identity(o, ids);
    require(heights.count(o.at("floor")), "Unknown object floor");
    auto kind = o.at("kind").get<std::string>();
    require(kind == "floor" || kind == "wall" || kind == "ramp" ||
                kind == "obstacle" || kind == "stairs",
            "Unknown scene object kind");
    vector(o.at("position"), 3, 100);
    vector(o.at("size"), 3, 200);
    vector(o.at("quaternion"), 4, 1);
    for (double v : o.at("size"))
      require(v >= .02, "Scene dimensions must be at least 0.02 m");
    double norm = 0;
    for (double v : o.at("quaternion"))
      norm += v * v;
    require(std::abs(norm - 1) < 1e-6, "Scene quaternion must be normalized");
  }
  (void)scene_boxes(s);
  const auto &spawn = s.at("spawn");
  require(heights.count(spawn.at("floor")), "Unknown spawn floor");
  vector(spawn.at("position"), 3, 100);
  double yaw = spawn.at("yaw_deg");
  require(std::isfinite(yaw) && std::abs(yaw) <= 180, "Invalid spawn yaw");
}
void append_user_scene(tinyxml2::XMLDocument &doc, const Json &s) {
  validate_user_scene(s);
  auto heights = floors(s);
  auto *world = doc.RootElement()->FirstChildElement("worldbody");
  place_fixed_base(world, s);
  int index = 0;
  for (const auto &o : scene_boxes(s)) {
    auto pos = o.at("position").get<Vec3>(), size = o.at("size").get<Vec3>();
    pos[2] += heights.at(o.at("floor"));
    for (auto &v : size)
      v *= .5;
    auto *g = doc.NewElement("geom");
    g->SetAttribute("name",
                    ("scene_object_" + std::to_string(index++)).c_str());
    g->SetAttribute("type", "box");
    g->SetAttribute("group", "1");
    g->SetAttribute("pos", numbers(pos).c_str());
    g->SetAttribute("size", numbers(size).c_str());
    g->SetAttribute("quat", numbers(o.at("quaternion").get<Quat>()).c_str());
    const std::string kind = o.at("kind");
    g->SetAttribute("rgba", kind == "wall"    ? ".63 .67 .72 1"
                            : kind == "ramp"  ? ".47 .62 .44 1"
                            : kind == "floor" ? ".4 .48 .57 1"
                                              : ".74 .52 .34 1");
    g->SetAttribute("friction", "1 .01 .001");
    world->InsertEndChild(g);
  }
}
void apply_scene_spawn(mjModel *m, mjData *d, const Json &s) {
  validate_user_scene(s);
  int joint = -1;
  for (int i = 0; i < m->njnt; i++)
    if (m->jnt_type[i] == mjJNT_FREE) {
      joint = i;
      break;
    }
  if (joint < 0) {
    const int site = mj_name2id(m, mjOBJ_SITE, "base_origin");
    require(site >= 0, "Fixed robot requires a base origin");
    const int body = m->site_bodyid[site];
    require(body > 0 && m->body_parentid[body] == 0 &&
                m->body_jntnum[body] == 0,
            "Invalid fixed robot root");
    return;
  }
  int q = m->jnt_qposadr[joint];
  auto pos = s.at("spawn").at("position").get<Vec3>();
  pos[2] += floors(s).at(s.at("spawn").at("floor"));
  d->qpos[q] = pos[0];
  d->qpos[q + 1] = pos[1];
  d->qpos[q + 2] += pos[2];
  double half =
      s.at("spawn").at("yaw_deg").get<double>() * 3.14159265358979323846 / 360.;
  mjtNum yaw[4]{std::cos(half), 0, 0, std::sin(half)}, out[4];
  mju_mulQuat(out, yaw, d->qpos + q + 3);
  mju_copy(d->qpos + q + 3, out, 4);
}
} // namespace simple_rbt
