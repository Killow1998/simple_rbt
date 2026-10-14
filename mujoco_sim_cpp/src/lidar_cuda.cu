#include "lidar_cuda.hpp"
#include <mujoco/mujoco.h>
#include <memory>
#include <cuda_runtime.h>
#include <math_constants.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace simple_rbt::detail {
namespace {
void checked(cudaError_t code, const char* operation) {
  if (code != cudaSuccess) throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(code));
}
// Match MuJoCo's pure geometry contract: planes are front-facing, each zero
// half-size axis is infinite, boxes return the EXIT when the ray starts inside.
// Double geometry avoids the float32 slab seam ambiguities of the old Warp path.
__device__ double intersect(const RayGeometry& g, const double* origin, const double* ray) {
  double p[3], v[3];
  for (int axis=0; axis<3; ++axis) {
    p[axis] = 0.; v[axis] = 0.;
    for (int row=0; row<3; ++row) {
      p[axis] += g.rotation[3*row+axis]*(origin[row]-g.position[row]);
      v[axis] += g.rotation[3*row+axis]*ray[row];
    }
  }
  constexpr double tiny = 1e-15;  // mjMINVAL in installed double MuJoCo
  if (g.type == mjGEOM_PLANE) {
    if (v[2] > -tiny) return -1.;
    const double t = -p[2]/v[2];
    if (t < 0.) return -1.;
    for (int axis=0; axis<2; ++axis)
      if (g.size[axis] > 0. && fabs(p[axis]+t*v[axis]) > g.size[axis]) return -1.;
    return t;
  }
  if (g.type != mjGEOM_BOX) return -1.;
  double enter = -CUDART_INF, leave = CUDART_INF;
  for (int axis=0; axis<3; ++axis) {
    if (fabs(v[axis]) < tiny) {
      if (fabs(p[axis]) > g.size[axis]) return -1.;
    } else {
      const double a = (-g.size[axis]-p[axis])/v[axis];
      const double b = (g.size[axis]-p[axis])/v[axis];
      enter = fmax(enter, fmin(a,b)); leave = fmin(leave, fmax(a,b));
    }
  }
  if (enter > leave || leave < 0.) return -1.;
  return enter >= 0. ? enter : leave;
}
__global__ void ray_kernel(const RayGeometry* geometries, std::size_t geoms,
                           const double* origins, const double* directions,
                           std::size_t count, double cutoff, double* ranges) {
  const std::size_t i = blockIdx.x*blockDim.x+threadIdx.x;
  if (i >= count) return;
  double nearest = cutoff;
  for (std::size_t g=0; g<geoms; ++g) {
    const double d = intersect(geometries[g], origins+3*i, directions+3*i);
    if (d >= 0. && d < nearest) nearest = d;
  }
  ranges[i] = nearest < cutoff ? nearest : -1.;
}
}
struct CudaRayState {
  RayGeometry* geometries = nullptr;
  std::size_t geometry_count = 0, capacity = 0;
  double *origins = nullptr, *directions = nullptr, *ranges = nullptr;
  ~CudaRayState() {
    cudaFree(geometries); cudaFree(origins); cudaFree(directions); cudaFree(ranges);
  }
  void reserve(std::size_t count) {
    if (count <= capacity) return;
    // Allocate a complete replacement before releasing the previous capacity.
    double *new_origins=nullptr, *new_directions=nullptr, *new_ranges=nullptr;
    try {
      checked(cudaMalloc(reinterpret_cast<void**>(&new_origins), count*3*sizeof(double)), "allocate ray origins");
      checked(cudaMalloc(reinterpret_cast<void**>(&new_directions), count*3*sizeof(double)), "allocate ray directions");
      checked(cudaMalloc(reinterpret_cast<void**>(&new_ranges), count*sizeof(double)), "allocate ray ranges");
    } catch (...) {
      cudaFree(new_origins); cudaFree(new_directions); cudaFree(new_ranges); throw;
    }
    cudaFree(origins); cudaFree(directions); cudaFree(ranges);
    origins=new_origins; directions=new_directions; ranges=new_ranges; capacity=count;
  }
};
CudaRayState* make_cuda_rays(const std::vector<RayGeometry>& geometries) {
  std::unique_ptr<CudaRayState> state(new CudaRayState);
  state->geometry_count = geometries.size();
  if (!geometries.empty()) {
    checked(cudaMalloc(reinterpret_cast<void**>(&state->geometries), geometries.size()*sizeof(RayGeometry)), "allocate ray scene");
    checked(cudaMemcpy(state->geometries, geometries.data(), geometries.size()*sizeof(RayGeometry), cudaMemcpyHostToDevice), "upload ray scene");
  }
  state->reserve(20000);
  return state.release();
}
void free_cuda_rays(CudaRayState* state) noexcept { delete state; }
void trace_cuda_rays(CudaRayState* state, const double* origins, const double* directions,
                     std::size_t count, double cutoff, double* ranges) {
  if (!state) throw std::logic_error("CUDA ray scene not initialized");
  state->reserve(count);
  checked(cudaMemcpy(state->origins, origins, count*3*sizeof(double), cudaMemcpyHostToDevice), "upload ray origins");
  checked(cudaMemcpy(state->directions, directions, count*3*sizeof(double), cudaMemcpyHostToDevice), "upload ray directions");
  ray_kernel<<<static_cast<unsigned>((count+255)/256),256>>>(state->geometries, state->geometry_count,
      state->origins, state->directions, count, cutoff, state->ranges);
  checked(cudaGetLastError(), "launch rolling ray kernel");
  // Synchronous copy times completed work, including kernel errors, not enqueue.
  checked(cudaMemcpy(ranges, state->ranges, count*sizeof(double), cudaMemcpyDeviceToHost), "read ray ranges");
}
}  // namespace simple_rbt::detail
