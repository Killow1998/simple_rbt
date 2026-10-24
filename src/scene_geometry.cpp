#include "scene_geometry.hpp"
#include "user_scene.hpp"
#include <cmath>
#include <functional>
#include <map>
#include <regex>
#include <set>
namespace simple_rbt::structured {
void require(bool ok, const std::string &message) {
  if (!ok)
    throw std::invalid_argument("Scene v2: " + message);
}
void fields(const Json &j, std::initializer_list<const char *> required,
            std::initializer_list<const char *> optional) {
  require(j.is_object(), "expected object");
  std::set<std::string> allowed;
  for (auto key : required) {
    require(j.contains(key), std::string("missing ") + key);
    allowed.insert(key);
  }
  for (auto key : optional)
    allowed.insert(key);
  for (auto &entry : j.items())
    require(allowed.count(entry.key()), "unknown field " + entry.key());
}
double dimension(const Json &j, const char *key, bool zero) {
  double v = j.at(key).get<double>();
  require(std::isfinite(v) && (zero ? v >= 0 : v > 0) && v <= 1000,
          std::string("invalid dimension ") + key + " (maximum 1000 m)");
  return v;
}
void vector(const Json &j, std::size_t n) {
  require(j.is_array() && j.size() == n, "invalid vector size");
  for (double v : j)
    require(std::isfinite(v) && std::abs(v) <= 10000,
            "nonfinite or out-of-bounds vector");
}
namespace {
void id(const std::string &s) {
  static const std::regex pattern("[A-Za-z_][A-Za-z0-9_.:-]*");
  require(s.size() <= 160 && std::regex_match(s, pattern), "invalid ID " + s);
}
Pose pose(const Json &j) {
  fields(j, {"position_m", "quaternion_wxyz"});
  vector(j.at("position_m"), 3);
  vector(j.at("quaternion_wxyz"), 4);
  Pose p{j.at("position_m").get<Vec3>(), j.at("quaternion_wxyz").get<Quat>()};
  double norm = 0;
  for (double v : p.quaternion)
    norm += v * v;
  require(std::abs(norm - 1) <= 1e-6, "quaternion must be normalized");
  for (double &v : p.quaternion)
    v /= std::sqrt(norm);
  for (double v : p.quaternion)
    if (std::abs(v) > 1e-15) {
      if (v < 0)
        for (double &q : p.quaternion)
          q = -q;
      break;
    }
  return p;
}
Pose compose(const Pose &a, const Pose &b) {
  Pose out;
  mju_rotVecQuat(out.position.data(), b.position.data(), a.quaternion.data());
  for (int i = 0; i < 3; ++i)
    out.position[i] += a.position[i];
  mju_mulQuat(out.quaternion.data(), a.quaternion.data(), b.quaternion.data());
  return out;
}
} // namespace
} // namespace simple_rbt::structured
namespace simple_rbt {
Json resolve_structured_scene(const Json &s) {
  using namespace structured;
  fields(s,
         {"format", "version", "id", "name", "units", "coordinate_system",
          "objects"},
         {"geometry_templates"});
  require(s.at("format") == "sprobo.scene" && s.at("version") == 2,
          "unsupported format/version");
  require(s.at("units") == Json{{"length", "m"}, {"angle", "rad"}} &&
              s.at("coordinate_system") == "right_handed_z_up",
          "unsupported units/axes");
  id(s.at("id"));
  require(!s.at("name").get<std::string>().empty(), "empty scene name");
  const auto templates = s.value("geometry_templates", Json::object());
  require(templates.is_object(), "templates must be a map");
  for (const auto &t : templates.items()) {
    id(t.key());
    (void)primitive(t.value());
  }
  require(s.at("objects").is_array() && s.at("objects").size() <= 8192,
          "node budget exceeded (8192)");
  std::map<std::string, const Json *> nodes;
  std::map<std::string, Pose> poses;
  std::set<std::string> visiting;
  for (const auto &o : s.at("objects")) {
    fields(o, {"id", "name", "kind", "parent", "pose"},
           {"geometry", "evidence", "semantic"});
    const std::string key = o.at("id");
    id(key);
    id(o.at("parent"));
    require(key != "world" && nodes.emplace(key, &o).second,
            "duplicate/reserved object ID " + key);
    require(!o.at("name").get<std::string>().empty(), "empty object name");
    const std::string kind = o.at("kind");
    require(kind == "primitive" || kind == "group", "unknown object kind");
    require(kind == "primitive"
                ? o.contains("geometry") && o.contains("evidence")
                : !o.contains("geometry"),
            "invalid group/primitive fields");
    if (o.contains("semantic")) {
      static const std::regex semantic("[a-z][a-z0-9_]*");
      require(std::regex_match(o.at("semantic").get<std::string>(), semantic),
              "invalid semantic");
    }
    if (o.contains("evidence")) {
      const auto &e = o.at("evidence");
      fields(e, {"basis"}, {"report_ref", "note"});
      if (e.contains("report_ref"))
        require(e.at("report_ref").is_string() &&
                    !e.at("report_ref").get<std::string>().empty(),
                "invalid evidence report reference");
      if (e.contains("note"))
        require(e.at("note").is_string(), "invalid evidence note");
      const std::string basis = e.at("basis");
      require(basis == "manual" || basis == "fitted" || basis == "inferred" ||
                  basis == "mixed",
              "unknown evidence basis");
      require(basis == "manual" ||
                  (e.contains("report_ref") &&
                   !e.at("report_ref").get<std::string>().empty()),
              "source evidence report required");
    }
  }
  std::function<Pose(const std::string &, int)> world =
      [&](const std::string &key, int depth) {
        if (key == "world")
          return Pose{};
        require(depth < 128, "hierarchy exceeds 128 levels");
        if (poses.count(key))
          return poses.at(key);
        require(nodes.count(key), "unknown parent " + key);
        require(visiting.insert(key).second, "parent cycle at " + key);
        const auto &o = *nodes.at(key);
        Pose p = compose(world(o.at("parent"), depth + 1), pose(o.at("pose")));
        visiting.erase(key);
        poses[key] = p;
        return p;
      };
  Json result{{"parts", Json::array()}, {"nodes", Json::object()}};
  std::size_t vertices = 0;
  for (const auto &o : s.at("objects")) {
    const std::string key = o.at("id");
    const auto p = world(key, 0);
    result["nodes"][key] = {{"position", p.position},
                            {"quaternion", p.quaternion}};
    if (o.at("kind") == "group")
      continue;
    Json geometry = o.at("geometry");
    if (geometry.contains("template_id")) {
      fields(geometry, {"template_id"}, {"overrides"});
      const std::string ref = geometry.at("template_id");
      require(templates.contains(ref), "unknown template " + ref);
      const auto overrides = geometry.value("overrides", Json::object());
      require(overrides.is_object() && !overrides.contains("type"),
              "invalid template overrides");
      geometry = templates.at(ref);
      for (const auto &v : overrides.items())
        geometry[v.key()] = v.value();
    }
    Json parts;
    try {
      parts = primitive(geometry);
    } catch (const std::exception &e) {
      throw std::invalid_argument("Object " + key + ": " + e.what());
    }
    for (auto part : parts) {
      require(result["parts"].size() < max_parts,
              "physical geometry budget exceeded (4096)");
      part = preview(std::move(part));
      vertices += part.at("vertices").size();
      require(vertices <= 262144, "display vertex budget exceeded (262144)");
      part["object_id"] = key;
      part["semantic"] = o.value("semantic", std::string());
      part["position"] = p.position;
      part["quaternion"] = p.quaternion;
      result["parts"].push_back(std::move(part));
    }
  }
  result["geometry_count"] = result["parts"].size();
  return result;
}
} // namespace simple_rbt
