#include "scene_geometry.hpp"
#include "user_scene.hpp"
#include <algorithm>
#include <cmath>
#include <earcut/earcut.hpp>
#include <map>
namespace simple_rbt::structured {
namespace {
using XY = std::array<double, 2>;
using Ring = std::vector<XY>;
double cross(const XY &a, const XY &b, const XY &c) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}
bool touches(const XY &a, const XY &b, const XY &c, const XY &d) {
  if (std::max(a[0], b[0]) + 1e-10 < std::min(c[0], d[0]) ||
      std::max(c[0], d[0]) + 1e-10 < std::min(a[0], b[0]) ||
      std::max(a[1], b[1]) + 1e-10 < std::min(c[1], d[1]) ||
      std::max(c[1], d[1]) + 1e-10 < std::min(a[1], b[1]))
    return false;
  const auto sign = [](double x) {
    return x > 1e-10 ? 1 : x < -1e-10 ? -1 : 0;
  };
  return sign(cross(a, b, c)) * sign(cross(a, b, d)) <= 0 &&
         sign(cross(c, d, a)) * sign(cross(c, d, b)) <= 0;
}
bool inside(const XY &p, const Ring &ring) {
  bool value = false;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
    const auto &a = ring[i], &b = ring[j];
    if ((a[1] > p[1]) != (b[1] > p[1]) &&
        p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0])
      value = !value;
  }
  return value;
}
double area(const Ring &ring) {
  double a = 0;
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const auto &p = ring[i], &q = ring[(i + 1) % ring.size()];
    a += p[0] * q[1] - p[1] * q[0];
  }
  return a / 2;
}
// Merge adjacent triangles only when their union is convex. Boundaries and
// holes stay exact; fewer collision parts also remove unnecessary contact
// seams.
std::vector<std::vector<int>>
convex_cells(const std::vector<std::uint32_t> &indices, const Ring &vertices) {
  std::vector<std::vector<int>> cells;
  for (std::size_t i = 0; i < indices.size(); i += 3) {
    std::vector<int> cell{int(indices[i]), int(indices[i + 1]),
                          int(indices[i + 2])};
    if (cross(vertices[cell[0]], vertices[cell[1]], vertices[cell[2]]) < 0)
      std::reverse(cell.begin(), cell.end());
    cells.push_back(cell);
  }
  bool changed = true;
  while (changed) {
    changed = false;
    std::map<std::pair<int, int>, std::size_t> edges;
    for (std::size_t i = 0; i < cells.size() && !changed; ++i) {
      const auto &a = cells[i];
      for (std::size_t j = 0; j < a.size(); ++j) {
        int u = a[j], v = a[(j + 1) % a.size()];
        auto found = edges.find({v, u});
        if (found == edges.end()) {
          edges[{u, v}] = i;
          continue;
        }
        const auto other = found->second;
        const auto &b = cells[other];
        auto k = std::find(b.begin(), b.end(), v) - b.begin();
        std::vector<int> merged;
        for (std::size_t n = 0; n < a.size(); ++n)
          merged.push_back(a[(j + 1 + n) % a.size()]);
        for (std::size_t n = 2; n < b.size(); ++n)
          merged.push_back(b[(k + n) % b.size()]);
        bool convex = true;
        for (std::size_t n = 0; n < merged.size(); ++n)
          if (cross(vertices[merged[n]],
                    vertices[merged[(n + 1) % merged.size()]],
                    vertices[merged[(n + 2) % merged.size()]]) < -1e-10)
            convex = false;
        if (!convex)
          continue;
        cells[i] = std::move(merged);
        cells[other].clear();
        changed = true;
        break;
      }
    }
  }
  for (auto &cell : cells) {
    bool simplify = true;
    while (simplify && cell.size() > 3) {
      simplify = false;
      for (std::size_t i = 0; i < cell.size(); ++i) {
        if (std::abs(cross(vertices[cell[(i + cell.size() - 1) % cell.size()]],
                           vertices[cell[i]],
                           vertices[cell[(i + 1) % cell.size()]])) < 1e-10) {
          cell.erase(cell.begin() + i);
          simplify = true;
          break;
        }
      }
    }
  }
  cells.erase(std::remove_if(cells.begin(), cells.end(),
                             [](const auto &c) { return c.empty(); }),
              cells.end());
  return cells;
}
} // namespace
Json planar_parts(const Json &g) {
  const double thickness = dimension(g, "thickness_m");
  require(g.at("holes_xy_m").is_array(), "polygon holes must be an array");
  Json input = Json::array({g.at("outer_xy_m")});
  for (const auto &hole : g.at("holes_xy_m"))
    input.push_back(hole);
  std::vector<Ring> rings;
  std::vector<XY> flat;
  double expected = 0;
  for (const auto &raw : input) {
    require(raw.is_array() && raw.size() >= 3,
            "ring needs at least three vertices");
    for (const auto &v : raw)
      vector(v, 2);
    auto ring = raw.get<Ring>();
    require(flat.size() + ring.size() <= 2048,
            "polygon vertex budget exceeded (2048)");
    double a = area(ring);
    require(rings.empty() ? a > 1e-10 : a < -1e-10,
            "outer ring must be CCW and holes CW");
    expected += a;
    for (std::size_t i = 0; i < ring.size(); ++i) {
      const auto next = (i + 1) % ring.size();
      require(ring[i] != ring[next], "repeated polygon vertex");
      for (std::size_t j = i + 1; j < ring.size(); ++j) {
        if (j == next || (j + 1) % ring.size() == i)
          continue;
        require(
            !touches(ring[i], ring[next], ring[j], ring[(j + 1) % ring.size()]),
            "self-intersecting polygon");
      }
      for (const auto &other : rings)
        for (std::size_t j = 0; j < other.size(); ++j)
          require(!touches(ring[i], ring[next], other[j],
                           other[(j + 1) % other.size()]),
                  "polygon rings touch or cross");
    }
    if (!rings.empty()) {
      require(inside(ring[0], rings[0]), "hole outside outer ring");
      for (std::size_t i = 1; i < rings.size(); ++i)
        require(!inside(ring[0], rings[i]) && !inside(rings[i][0], ring),
                "nested/overlapping holes");
    }
    flat.insert(flat.end(), ring.begin(), ring.end());
    rings.push_back(std::move(ring));
  }
  auto indices = mapbox::earcut<std::uint32_t>(rings);
  const auto cells = convex_cells(indices, flat);
  require(cells.size() <= max_parts,
          "polygon exceeds physical geometry budget");
  Json parts = Json::array();
  double actual = 0;
  for (const auto &cell : cells) {
    std::vector<Vec3> v;
    std::vector<std::vector<int>> faces(2);
    Ring ring;
    const int n = cell.size();
    for (int layer = 0; layer < 2; ++layer)
      for (int j = 0; j < n; ++j) {
        const auto &p = flat.at(cell[j]);
        v.push_back({p[0], p[1], layer ? 0 : -thickness});
        faces[layer].push_back(layer * n + j);
      }
    for (int j = 0; j < n; ++j) {
      faces.push_back({j, (j + 1) % n, (j + 1) % n + n, j + n});
      ring.push_back(flat[cell[j]]);
    }
    actual += area(ring);
    parts.push_back(polyhedron(v, faces));
  }
  require(std::abs(actual - expected) <= 1e-8 * std::max(1., expected),
          "polygon triangulation area mismatch");
  return parts;
}
} // namespace simple_rbt::structured
