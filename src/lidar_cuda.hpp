#pragma once
#include <cstddef>
#include <vector>

// Private host/CUDA boundary shared by the two implementation files.
namespace simple_rbt::detail {
struct RayGeometry {
  int type = 0;
  double position[3]{}, rotation[9]{}, size[3]{};
};
struct CudaRayState;
CudaRayState *make_cuda_rays(const std::vector<RayGeometry> &geometries);
void free_cuda_rays(CudaRayState *) noexcept;
void trace_cuda_rays(CudaRayState *, const double *origins,
                     const double *directions, std::size_t count, double cutoff,
                     double *ranges);
} // namespace simple_rbt::detail
