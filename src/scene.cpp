#include "scene.hpp"
#include "user_scene.hpp"
#include "sensor_devices.hpp"
#include "simple_rbt/random.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <openssl/sha.h>
#include <sstream>
#include <stdexcept>
#include <tinyxml2.h>
namespace simple_rbt {
namespace fs = std::filesystem;
namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
std::string read(const fs::path &p) {
  std::ifstream f(p, std::ios::binary);
  if (!f)
    throw std::runtime_error("Cannot open " + p.string());
  return {std::istreambuf_iterator<char>(f), {}};
}
std::string numbers(std::initializer_list<double> x) {
  std::ostringstream s;
  s << std::setprecision(17);
  bool first = true;
  for (double v : x) {
    if (!first)
      s << ' ';
    s << v;
    first = false;
  }
  return s.str();
}
} // namespace
std::string build_scene(const SimulationOptions &o, const Json &catalog,
                        const std::string &controller, Json &profile) {
  std::string key = o.robot + "-" + controller;
  std::string raw = read(o.project_root / "assets" /
                         catalog["models"][key]["file"].get<std::string>());
  const std::string marker = "@PROJECT_ROOT@";
  std::size_t offset = 0;
  while ((offset = raw.find(marker, offset)) != std::string::npos) {
    raw.replace(offset, marker.size(), o.project_root.string());
    offset += o.project_root.string().size();
  }
  tinyxml2::XMLDocument doc;
  if (doc.Parse(raw.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("Invalid model XML");
  auto *root = doc.RootElement();
  auto *world = root->FirstChildElement("worldbody");
  auto add =
      [&](tinyxml2::XMLElement *parent, const char *tag,
          std::initializer_list<std::pair<const char *, std::string>> attrs) {
        auto *x = doc.NewElement(tag);
        for (auto &a : attrs)
          x->SetAttribute(a.first, a.second.c_str());
        parent->InsertEndChild(x);
        return x;
      };
  add(world, "light",
      {{"pos", "0 0 6"}, {"dir", "0 0 -1"}, {"directional", "true"}});
  if (o.scene.is_null()) {
    add(world, "geom",
        {{"name", "floor"},
         {"type", "plane"},
         {"size", "12 12 .1"},
         {"rgba", ".35 .4 .45 1"},
         {"friction", "1 .01 .001"},
         {"group", "1"}});
    const char *names[] = {"north", "south", "east", "west"};
    const char *positions[] = {"0 6 1.5", "0 -6 1.5", "12 0 1.5", "-12 0 1.5"};
    const char *sizes[] = {"12 .1 1.5", "12 .1 1.5", ".1 6 1.5", ".1 6 1.5"};
    const char *colors[] = {".7 .4 .3 1", ".3 .6 .7 1", ".7 .7 .4 1",
                            ".4 .7 .4 1"};
    for (int i = 0; i < 4; i++)
      add(world, "geom",
          {{"name", names[i]},
           {"type", "box"},
           {"pos", positions[i]},
           {"size", sizes[i]},
           {"rgba", colors[i]},
           {"group", "1"}});
  } else {
    append_user_scene(doc, o.scene);
  }
  add_sensor_devices(doc,o);
  auto *visual = root->FirstChildElement("visual");
  if (!visual)
    visual = add(root, "visual", {});
  add(visual, "global", {{"offwidth", "1024"}, {"offheight", "1024"}});
  add(visual, "map", {{"znear", ".005"}, {"zfar", "50"}});
  Json terrain = catalog["terrain_presets"][o.terrain + "-" + o.terrain_level];
  profile = terrain["metadata"];
  profile["difficulty"] = o.difficulty;
  profile["seed"] = o.seed;
  auto box = [&](const std::string &name, double x, double y, double z,
                 double hx, double hy, double hz, double slope) {
    std::string color = o.terrain == "ramp"     ? ".51 .58 .36 1"
                        : o.terrain == "stairs" ? ".60 .48 .32 1"
                        : o.terrain == "rough"  ? ".49 .48 .44 1"
                                                : ".45 .56 .39 1";
    add(world, "geom",
        {{"name", name},
         {"type", "box"},
         {"pos", numbers({x, y, z})},
         {"size", numbers({hx, hy, hz})},
         {"quat", numbers({std::cos(slope / 2), 0, -std::sin(slope / 2), 0})},
         {"rgba", color},
         {"friction", "1 .01 .001"},
         {"group", "1"},
         {"contype", "1"},
         {"conaffinity", "1"},
         {"condim", "3"}});
  };
  if (!o.scene.is_null()) {
    profile = {{"type", "user_scene"},
               {"scene_id", o.scene.at("id")},
               {"name", o.scene.at("name")}};
  } else if (o.difficulty == 1. && (o.terrain != "rough" || o.seed == 0)) {
    for (const auto &b : terrain["boxes"]) {
      auto c = b["centre"].get<Vec3>(), h = b["half_size"].get<Vec3>();
      box(b["name"], c[0], c[1], c[2], h[0], h[1], h[2], b["slope"]);
    }
  } else {
    // Recompute scaled geometry while retaining the standard course dimensions.
    auto levelbox = [&](std::string n, double a, double b, double w,
                        double height) {
      box(n, (a + b) / 2, 0, (height - .10) / 2, (b - a) / 2, w / 2,
          (height + .10) / 2, 0);
    };
    auto slopebox = [&](std::string n, double a, double b, double height,
                        double angle) {
      double c = std::cos(angle), s = std::sin(angle);
      box(n, (a + b) / 2 + s * .075, 0,
          height + (b - a) * std::tan(angle) / 2 - c * .075, (b - a) / (2 * c),
          1.2, .075, angle);
    };
    if (o.terrain == "ramp") {
      double deg = o.terrain_level == "standard"
                       ? 8 * o.difficulty
                       : profile["parameters"]["slope_degrees"].get<double>();
      double run = profile["parameters"]["horizontal_run_m"],
             landing = profile["parameters"]["landing_length_m"],
             angle = deg * pi / 180, peak = run * std::tan(angle);
      slopebox("terrain_ramp_up", 1, 1 + run, 0, angle);
      levelbox("terrain_ramp_landing", 1 + run, 1 + run + landing, 2.4, peak);
      slopebox("terrain_ramp_down", 1 + run + landing, 1 + 2 * run + landing,
               peak, -angle);
      profile["parameters"]["slope_degrees"] = deg;
      profile["max_height_m"] = peak;
    } else if (o.terrain == "stairs") {
      int count = profile["parameters"]["ascending_steps"];
      double tread = profile["parameters"]["tread_length_m"],
             rise = profile["parameters"]["step_height_m"].get<double>() *
                    o.difficulty,
             landing = profile["parameters"]["landing_length_m"];
      for (int i = 0; i < count; i++)
        levelbox("terrain_stair_up_" + std::to_string(i), 1 + i * tread,
                 1 + (i + 1) * tread, 2.4, (i + 1) * rise);
      levelbox("terrain_stair_landing", 1 + count * tread,
               1 + count * tread + landing, 2.4, count * rise);
      for (int i = 0; i < count - 1; i++)
        levelbox("terrain_stair_down_" + std::to_string(i),
                 1 + count * tread + landing + i * tread,
                 1 + count * tread + landing + (i + 1) * tread, 2.4,
                 (count - i - 1) * rise);
      profile["parameters"]["step_height_m"] = rise;
      profile["max_height_m"] = count * rise;
    } else if (o.terrain == "hills") {
      double amplitude =
          profile["parameters"]["amplitude_m"].get<double>() * o.difficulty;
      std::array<double, 49> x{}, h{};
      for (int i = 0; i <= 48; i++) {
        x[i] = 1 + 4. * i / 48;
        h[i] = amplitude * std::pow(std::sin(2 * pi * i / 48), 2);
      }
      h[0] = h[48] = 0;
      for (int i = 0; i < 48; i++)
        slopebox("terrain_hills_" + std::to_string(i), x[i], x[i + 1], h[i],
                 std::atan2(h[i + 1] - h[i], x[i + 1] - x[i]));
      profile["parameters"]["amplitude_m"] = amplitude;
      profile["parameters"]["node_x_m"] = x;
      profile["parameters"]["node_height_m"] = h;
      profile["max_height_m"] = amplitude;
    } else if (o.terrain == "rough") {
      NumpyPcg64 rng(o.seed);
      double maximum =
          profile["parameters"]["maximum_cell_height_m"].get<double>() *
          o.difficulty;
      Json heights = Json::array();
      double highest = 0;
      for (int col = 0; col < 10; col++) {
        Json row = Json::array();
        for (int r = 0; r < 6; r++) {
          double height = rng.uniform_next() * maximum;
          highest = std::max(highest, height);
          row.push_back(height);
          double x0 = std::max(1., 1 + col * .4 - .00001),
                 x1 = std::min(5., 1 + (col + 1) * .4 + .00001),
                 y0 = std::max(-1.2, -1.2 + r * .4 - .00001),
                 y1 = std::min(1.2, -1.2 + (r + 1) * .4 + .00001);
          box("terrain_rough_" + std::to_string(col) + "_" + std::to_string(r),
              (x0 + x1) / 2, (y0 + y1) / 2, (height - .10) / 2, (x1 - x0) / 2,
              (y1 - y0) / 2, (height + .10) / 2, 0);
        }
        heights.push_back(row);
      }
      profile["parameters"]["cell_heights_m"] = heights;
      profile["parameters"]["maximum_cell_height_m"] = maximum;
      profile["max_height_m"] = highest;
    }
  }
  if (o.friction > 0) {
    for (auto *g = world->FirstChildElement("geom"); g;
         g = g->NextSiblingElement("geom")) {
      const char *n = g->Attribute("name");
      std::string name = n ? n : "";
      if (name == "floor" || name.rfind("terrain_", 0) == 0) {
        g->SetAttribute("friction", numbers({o.friction, .02, .01}).c_str());
        g->SetAttribute("priority", "2");
        g->SetAttribute("condim", "6");
      }
    }
    profile["friction"] = {o.friction, .02, .01};
    profile["parameters"]["friction_override"] = {
        {"sliding", o.friction},
        {"torsional", .02},
        {"rolling", .01},
        {"surface_priority", 2},
        {"contact_dimension", 6},
        {"scope", "floor and course; robot and walls unchanged"}};
  }
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}
std::string sha256_file(const fs::path &p) {
  std::string text = read(p);
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(text.data()), text.size(),
         hash);
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (auto x : hash)
    out << std::setw(2) << static_cast<int>(x);
  return out.str();
}
Json load_catalog(const fs::path &project_root) {
  return Json::parse(read(project_root / "assets/catalog.json"));
}
} // namespace simple_rbt
