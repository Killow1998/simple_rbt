#include "lidar_scene.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace simple_rbt::detail {
StaticRayIndex::StaticRayIndex(const mjModel *m, const mjData *d,
                               const std::vector<int> &ids)
    : model_(m), data_(d) {
  for (int g : ids) {
    const int material = m->geom_matid[g];
    if ((material >= 0 ? m->mat_rgba[4 * material + 3]
                       : m->geom_rgba[4 * g + 3]) == 0)
      continue;
    if (m->geom_type[g] == mjGEOM_PLANE) {
      planes_.push_back(g);
      continue;
    }
    const double *aabb = m->geom_aabb + 6 * g, *rotation = d->geom_xmat + 9 * g;
    double center[3];
    mju_mulMatVec3(center, rotation, aabb);
    Bound bound;
    for (int i = 0; i < 3; ++i) {
      center[i] += d->geom_xpos[3 * g + i];
      double half = 0;
      for (int j = 0; j < 3; ++j)
        half += std::abs(rotation[3 * i + j]) * aabb[3 + j];
      bound.low[i] = center[i] - half - 1e-9;
      bound.high[i] = center[i] + half + 1e-9;
    }
    items_.push_back({g, bound});
  }
  if (!items_.empty())
    build(0, items_.size());
}
int StaticRayIndex::build(int begin, int end) {
  Bound bound = items_[begin].bound;
  for (int i = begin + 1; i < end; ++i)
    for (int j = 0; j < 3; ++j) {
      bound.low[j] = std::min(bound.low[j], items_[i].bound.low[j]);
      bound.high[j] = std::max(bound.high[j], items_[i].bound.high[j]);
    }
  int index = nodes_.size();
  nodes_.push_back({bound, begin, end});
  if (end - begin <= 4)
    return index;
  int axis = 0;
  for (int j = 1; j < 3; ++j)
    if (bound.high[j] - bound.low[j] > bound.high[axis] - bound.low[axis])
      axis = j;
  const int middle = (begin + end) / 2;
  std::nth_element(items_.begin() + begin, items_.begin() + middle,
                   items_.begin() + end, [&](const auto &a, const auto &b) {
                     return a.bound.low[axis] + a.bound.high[axis] <
                            b.bound.low[axis] + b.bound.high[axis];
                   });
  const int left = build(begin, middle), right = build(middle, end);
  nodes_[index].left = left;
  nodes_[index].right = right;
  return index;
}
double StaticRayIndex::ray(const double *origin, const double *direction,
                           double cutoff) const {
  double nearest = cutoff;
  bool hit = false;
  auto intersect = [&](int g) {
    double distance;
    if (model_->geom_type[g] == mjGEOM_MESH)
      distance = mj_rayMesh(model_, data_, g, origin, direction, nullptr);
    else if (model_->geom_type[g] == mjGEOM_HFIELD)
      distance = mj_rayHfield(model_, data_, g, origin, direction, nullptr);
    else
      distance = mju_rayGeom(data_->geom_xpos + 3 * g, data_->geom_xmat + 9 * g,
                             model_->geom_size + 3 * g, origin, direction,
                             model_->geom_type[g], nullptr);
    if (distance >= 0 && distance < nearest) {
      nearest = distance;
      hit = true;
    }
  };
  for (int g : planes_)
    intersect(g);
  if (nodes_.empty())
    return hit ? nearest : -1;
  std::array<int, 64> stack{};
  int count = 1;
  stack[0] = 0;
  while (count) {
    const auto &node = nodes_[stack[--count]];
    double low = 0, high = nearest;
    for (int j = 0; j < 3 && low <= high; ++j) {
      if (direction[j] == 0) {
        if (origin[j] < node.bound.low[j] || origin[j] > node.bound.high[j])
          high = -1;
        continue;
      }
      double a = (node.bound.low[j] - origin[j]) / direction[j],
             b = (node.bound.high[j] - origin[j]) / direction[j];
      if (a > b)
        std::swap(a, b);
      low = std::max(low, a);
      high = std::min(high, b);
    }
    if (low > high)
      continue;
    if (node.left < 0)
      for (int i = node.begin; i < node.end; ++i)
        intersect(items_[i].geom);
    else {
      stack[count++] = node.left;
      stack[count++] = node.right;
    }
  }
  return hit ? nearest : -1;
}
} // namespace simple_rbt::detail
