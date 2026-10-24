#include "scene_geometry.hpp"
#include "user_scene.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
namespace simple_rbt::structured {
Json polyhedron(const std::vector<Vec3> &v,
                const std::vector<std::vector<int>> &polygons) {
  Vec3 center{};
  for (const auto &p : v)
    for (int i = 0; i < 3; ++i)
      center[i] += p[i] / v.size();
  Json faces = Json::array();
  for (const auto &ring : polygons)
    for (std::size_t i = 1; i + 1 < ring.size(); ++i) {
      int a = ring[0], b = ring[i], c = ring[i + 1];
      Vec3 u{}, w{}, n{}, out{};
      for (int j = 0; j < 3; ++j) {
        u[j] = v[b][j] - v[a][j];
        w[j] = v[c][j] - v[a][j];
        out[j] = v[a][j] - center[j];
      }
      mju_cross(n.data(), u.data(), w.data());
      if (mju_dot3(n.data(), out.data()) < 0)
        std::swap(b, c);
      faces.push_back({a, b, c});
    }
  return {{"type", "mesh"}, {"vertices", v}, {"faces", faces}};
}
namespace {
Json slab(double x0, double x1, double width, double top0, double top1,
          double bottom0, double bottom1) {
  std::vector<Vec3> v{{x0, -width / 2, bottom0}, {x1, -width / 2, bottom1},
                      {x1, width / 2, bottom1},  {x0, width / 2, bottom0},
                      {x0, -width / 2, top0},    {x1, -width / 2, top1},
                      {x1, width / 2, top1},     {x0, width / 2, top0}};
  return polyhedron(v, {{0, 1, 2, 3},
                        {4, 5, 6, 7},
                        {0, 1, 5, 4},
                        {1, 2, 6, 5},
                        {2, 3, 7, 6},
                        {3, 0, 4, 7}});
}
Json frustum(double bottom, double top, double height) {
  const double r = std::max(bottom, top), pi = std::acos(-1.);
  const int n = std::max(32, static_cast<int>(std::ceil(
                                 pi / std::acos(std::max(-1., 1 - .001 / r)))));
  require(n <= 1024, "frustum exceeds 1 mm tessellation budget");
  std::vector<Vec3> v;
  std::vector<std::vector<int>> f;
  // Conical tips have one vertex, avoiding degenerate triangles.
  for (int layer = 0; layer < 2; ++layer) {
    double radius = layer ? top : bottom;
    for (int i = 0; i < (radius == 0 ? 1 : n); ++i)
      v.push_back({radius * std::cos(2 * pi * i / n),
                   radius * std::sin(2 * pi * i / n), (layer - .5) * height});
  }
  const int offset = bottom == 0 ? 1 : n;
  for (int i = 0; i < n; ++i) {
    int a = i, b = (i + 1) % n;
    if (bottom == 0)
      f.push_back({0, offset + a, offset + b});
    else if (top == 0)
      f.push_back({a, b, offset});
    else
      f.push_back({a, b, offset + b, offset + a});
  }
  for (int layer = 0; layer < 2; ++layer)
    if ((layer ? top : bottom) > 0) {
      std::vector<int> cap(n);
      std::iota(cap.begin(), cap.end(), layer ? offset : 0);
      f.push_back(cap);
    }
  auto part = polyhedron(v, f);
  part["surface_error_bound_m"] = r * (1 - std::cos(pi / n));
  return part;
}
} // namespace
Json primitive(const Json &g) {
  const std::string type = g.at("type");
  Json parts = Json::array();
  auto native = [&](const std::string &kind, const Vec3 &size) {
    parts.push_back({{"type", kind}, {"size", size}});
  };
  if (type == "box") {
    fields(g, {"type", "length_m", "width_m", "height_m"});
    native(type, {dimension(g, "length_m") / 2, dimension(g, "width_m") / 2,
                  dimension(g, "height_m") / 2});
  } else if (type == "sphere") {
    fields(g, {"type", "radius_m"});
    native(type, {dimension(g, "radius_m"), 0, 0});
  } else if (type == "ellipsoid") {
    fields(g, {"type", "radii_m"});
    vector(g.at("radii_m"), 3);
    auto radii = g.at("radii_m").get<Vec3>();
    for (double r : radii)
      require(r > 0 && r <= 1000, "invalid ellipsoid radius");
    native(type, radii);
  } else if (type == "cylinder" || type == "capsule") {
    const char *length = type == "cylinder" ? "height_m" : "cylinder_length_m";
    fields(g, {"type", "radius_m", length});
    double r = dimension(g, "radius_m"),
           h = dimension(g, length, type == "capsule");
    native(h == 0 ? "sphere" : type, {r, h / 2, 0});
  } else if (type == "frustum") {
    fields(g, {"type", "bottom_radius_m", "top_radius_m", "height_m"});
    double b = dimension(g, "bottom_radius_m", true),
           t = dimension(g, "top_radius_m", true);
    require(b + t > 0, "frustum radii are both zero");
    parts.push_back(frustum(b, t, dimension(g, "height_m")));
  } else if (type == "planar_region") {
    fields(g, {"type", "outer_xy_m", "holes_xy_m", "thickness_m"});
    return planar_parts(g);
  } else if (type == "ramp" || type == "stair_flight") {
    double width = dimension(g, "width_m"),
           thickness = dimension(g, "slab_thickness_m"), run, rise;
    int steps = 1;
    if (type == "ramp") {
      fields(g, {"type", "width_m", "run_m", "rise_m", "slab_thickness_m"});
      run = dimension(g, "run_m");
      rise = dimension(g, "rise_m");
    } else {
      fields(g, {"type", "width_m", "step_height_m", "tread_depth_m", "steps",
                 "slab_thickness_m"});
      require(g.at("steps").is_number_integer() && g.at("steps") >= 1 &&
                  g.at("steps") <= max_parts,
              "invalid stair count or geometry budget");
      steps = g.at("steps");
      run = dimension(g, "tread_depth_m");
      rise = dimension(g, "step_height_m");
    }
    const double offset = thickness * std::hypot(1., rise / run);
    for (int i = 0; i < steps; ++i)
      parts.push_back(slab(i * run, (i + 1) * run, width,
                           (type == "ramp" ? i : i + 1) * rise, (i + 1) * rise,
                           i * rise - offset, (i + 1) * rise - offset));
  } else
    require(false, "unknown primitive " + type);
  return parts;
}
Json preview(Json part) {
  if (part.at("type") == "mesh")
    return part;
  const std::string type = part.at("type");
  const Vec3 size = part.at("size").get<Vec3>();
  Json mesh;
  if (type == "box")
    mesh = slab(-size[0], size[0], 2 * size[1], size[2], size[2], -size[2],
                -size[2]);
  else if (type == "cylinder")
    mesh = frustum(size[0], size[0], 2 * size[1]);
  else {
    const double pi = std::acos(-1.);
    const int slices = 32, stacks = 16;
    std::vector<Vec3> vertices;
    std::vector<std::vector<int>> faces;
    const int rings = stacks + (type == "capsule" ? 1 : 0);
    for (int j = 0; j <= rings; ++j)
      for (int i = 0; i < slices; ++i) {
        const int latitude_index =
            type == "capsule" && j > stacks / 2 ? j - 1 : j;
        double latitude = -pi / 2 + pi * latitude_index / stacks,
               longitude = 2 * pi * i / slices;
        Vec3 v{std::cos(latitude) * std::cos(longitude),
               std::cos(latitude) * std::sin(longitude), std::sin(latitude)};
        if (type == "ellipsoid")
          for (int k = 0; k < 3; ++k)
            v[k] *= size[k];
        else {
          for (double &x : v)
            x *= size[0];
          if (type == "capsule")
            v[2] += (j <= stacks / 2 ? -size[1] : size[1]);
        }
        vertices.push_back(v);
      }
    for (int j = 0; j < rings; ++j)
      for (int i = 0; i < slices; ++i)
        faces.push_back({j * slices + i, j * slices + (i + 1) % slices,
                         (j + 1) * slices + (i + 1) % slices,
                         (j + 1) * slices + i});
    mesh = polyhedron(vertices, faces);
  }
  part["vertices"] = mesh["vertices"];
  part["faces"] = mesh["faces"];
  return part;
}
} // namespace simple_rbt::structured
