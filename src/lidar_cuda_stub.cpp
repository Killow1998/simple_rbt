#include "lidar_cuda.hpp"
#include <stdexcept>

namespace simple_rbt::detail {
CudaRayState *make_cuda_rays(const std::vector<RayGeometry> &) {
  throw std::runtime_error(
      "CUDA LiDAR was not compiled; select CPU or Auto for CPU fallback");
}
void free_cuda_rays(CudaRayState *) noexcept {}
void trace_cuda_rays(CudaRayState *, const double *, const double *,
                     std::size_t, double, double *) {
  throw std::logic_error("CUDA LiDAR is unavailable in this build");
}
} // namespace simple_rbt::detail
