#pragma once
#include <array>
#include <mujoco/mujoco.h>
#include <vector>
namespace simple_rbt::detail {
// Broad phase only. Exact intersections remain MuJoCo's own ray routines.
class StaticRayIndex {
public:
  StaticRayIndex(const mjModel *, const mjData *, const std::vector<int> &);
  double ray(const double *origin, const double *direction,
             double cutoff) const;

private:
  struct Bound {
    std::array<double, 3> low, high;
  };
  struct Item {
    int geom;
    Bound bound;
  };
  struct Node {
    Bound bound;
    int begin, end, left = -1, right = -1;
  };
  int build(int begin, int end);
  const mjModel *model_;
  const mjData *data_;
  std::vector<Item> items_;
  std::vector<Node> nodes_;
  std::vector<int> planes_;
};
} // namespace simple_rbt::detail
