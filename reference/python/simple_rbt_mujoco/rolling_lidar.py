"""Per-point moving LiDAR against one static environment, without stepping physics.

Raw points are expressed in the instantaneous sensor frame at their own offset
time. The original CSV supplies directions/sample order, not hardware times.
The static ray scene is a single private model/data copy; all geometry in the
robot root's subtree is masked out. Other moving bodies are unsupported.
"""
import bisect
import copy
import operator

import numpy as np

from .lidar import LidarScan


def _timestamp(value):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError("timestamp must be an integer nanosecond value, not bool")
    try:
        value = operator.index(value)
    except TypeError as error:
        raise ValueError("timestamp must be an integer nanosecond value") from error
    if not 0 <= value <= np.iinfo(np.uint64).max:
        raise ValueError("timestamp must fit nonnegative uint64 nanoseconds")
    return value


class PoseHistory:
    """World poses of the LiDAR itself, with native physics knots retained.

    Quaternions use MuJoCo's wxyz convention. Translation is linear and rotation
    follows the shortest SLERP arc. Retention keeps the knot immediately before
    the window boundary as well as every knot inside it. No extrapolation.
    """

    def __init__(self, retention_ns=250_000_000):
        self.retention_ns = _timestamp(retention_ns)
        if self.retention_ns < 250_000_000:
            raise ValueError("pose history must retain at least 250 ms")
        self._stamps, self._positions, self._quaternions = [], [], []

    def __len__(self):
        return len(self._stamps)

    @property
    def bounds_ns(self):
        return (self._stamps[0], self._stamps[-1]) if self._stamps else None

    def add(self, stamp_ns, position, quaternion_wxyz):
        stamp_ns = _timestamp(stamp_ns)
        position = np.asarray(position, dtype=np.float64)
        quaternion = np.asarray(quaternion_wxyz, dtype=np.float64)
        if position.shape != (3,) or quaternion.shape != (4,):
            raise ValueError("pose requires position[3] and quaternion_wxyz[4]")
        if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
            raise ValueError("pose must be finite")
        norm = np.linalg.norm(quaternion)
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError("pose quaternion must have a finite nonzero norm")
        if self._stamps and stamp_ns <= self._stamps[-1]:
            raise ValueError("pose timestamps must be strictly increasing")
        self._stamps.append(stamp_ns)
        self._positions.append(position.copy())
        self._quaternions.append(quaternion / norm)
        boundary = max(0, stamp_ns - self.retention_ns)
        first = max(0, bisect.bisect_left(self._stamps, boundary) - 1)
        if first:
            del self._stamps[:first]
            del self._positions[:first]
            del self._quaternions[:first]

    def poses(self, stamps):
        values = np.asarray(stamps)
        if values.ndim != 1 or values.dtype.kind not in "iu":
            raise ValueError("query timestamps must be a one-dimensional integer array")
        if values.dtype.kind == "i" and np.any(values < 0):
            raise ValueError("query timestamps must be nonnegative")
        values = values.astype(np.uint64, copy=False)
        if len(values) == 0:
            return np.empty((0, 3)), np.empty((0, 4))
        if not self._stamps or values.min() < self._stamps[0] or values.max() > self._stamps[-1]:
            raise ValueError("pose history does not cover requested times; extrapolation is forbidden")
        times = np.asarray(self._stamps, dtype=np.uint64)
        positions = np.asarray(self._positions)
        quaternions = np.asarray(self._quaternions)
        left = np.clip(np.searchsorted(times, values, side="right") - 1, 0, len(times) - 1)
        right = np.minimum(left + 1, len(times) - 1)
        duration = (times[right] - times[left]).astype(np.float64)
        elapsed = (values - times[left]).astype(np.float64)
        fraction = np.divide(elapsed, duration, out=np.zeros_like(elapsed), where=duration > 0)
        position = positions[left] + fraction[:, None] * (positions[right] - positions[left])
        q0, q1 = quaternions[left], quaternions[right].copy()
        dot = np.sum(q0 * q1, axis=1)
        q1[dot < 0] *= -1
        dot = np.clip(np.abs(dot), 0., 1.)
        close = dot > .9995
        quaternion = q0 + fraction[:, None] * (q1 - q0)
        if np.any(~close):
            angle = np.arccos(dot[~close])
            u = fraction[~close]
            quaternion[~close] = (np.sin((1 - u) * angle)[:, None] * q0[~close]
                                   + np.sin(u * angle)[:, None] * q1[~close]) / np.sin(angle)[:, None]
        quaternion /= np.linalg.norm(quaternion, axis=1, keepdims=True)
        return position, quaternion

    def max_interval_ns(self, start_ns, end_ns):
        """Largest native knot interval used to interpolate this scan window."""
        left = max(0, bisect.bisect_right(self._stamps, start_ns) - 1)
        right = min(len(self._stamps) - 1, bisect.bisect_left(self._stamps, end_ns))
        return max((self._stamps[i + 1] - self._stamps[i] for i in range(left, right)), default=0)


def rotate_vectors(quaternions_wxyz, vectors):
    """Apply corresponding unit quaternions to vectors without Nx3x3 matrices."""
    q = np.asarray(quaternions_wxyz)
    v = np.asarray(vectors)
    uv = 2. * np.cross(q[:, 1:], v)
    return v + q[:, :1] * uv + np.cross(q[:, 1:], uv)


_WORLD_RAY_KERNEL = None


def _world_ray_kernel():
    # Lazy imports leave the CPU reference independent of Warp initialization.
    global _WORLD_RAY_KERNEL
    if _WORLD_RAY_KERNEL is not None:
        return _WORLD_RAY_KERNEL
    import warp as wp
    from mujoco_lidar.core_warp.kernels import read_mat33, ray_mesh_distance, trace_geom

    @wp.kernel
    def world_rays(bvh_id: wp.uint64,
                   types: wp.array(dtype=wp.int32), sizes: wp.array(dtype=wp.vec3),
                   geom_mesh_ids: wp.array(dtype=wp.int32), mesh_ids: wp.array(dtype=wp.uint64),
                   positions: wp.array(dtype=wp.vec3), rotations: wp.array2d(dtype=wp.float32),
                   origins: wp.array(dtype=wp.vec3), directions: wp.array(dtype=wp.vec3),
                   cutoff: float, distances: wp.array(dtype=wp.float32)):
        i = wp.tid()
        origin = origins[i]
        direction = wp.normalize(directions[i])
        best = cutoff
        query = wp.bvh_query_ray(bvh_id, origin, direction)
        geom = int(0)
        while wp.bvh_query_next(query, geom):
            kind = types[geom]
            if kind >= 0:
                rotation = read_mat33(rotations, geom)
                distance = -1.0
                if kind == 1 or kind == 7:
                    mesh = geom_mesh_ids[geom]
                    if mesh >= 0:
                        distance = ray_mesh_distance(mesh_ids[mesh], origin, direction,
                                                     positions[geom], rotation, best)
                else:
                    distance = trace_geom(kind, origin, direction, positions[geom], sizes[geom], rotation)
                if distance >= 0.0 and distance < best:
                    best = distance
        distances[i] = wp.where(best < cutoff, best, -1.0)

    _WORLD_RAY_KERNEL = world_rays
    return world_rays


class RollingCsvLidar:
    """20k time-indexed rays share one immutable environment model and BVH.

    ``history`` contains world poses of ``site_name``, not base poses. Its time
    interval must cover every scheduled ray, even rays that later miss. CPU is
    an independent scalar mj_ray reference; Warp traces all variable-origin
    rays in one kernel. Neither path steps or modifies the live simulation.
    """

    def __init__(self, model, pattern, *, backend="cpu", site_name="lidar",
                 min_range_m=.1, max_range_m=40., device=None, geomgroup=None):
        import mujoco
        if backend not in ("cpu", "warp"):
            raise ValueError("backend must be cpu or warp")
        if not np.isfinite([min_range_m, max_range_m]).all() or not 0 <= min_range_m < max_range_m:
            raise ValueError("require finite 0 <= min_range_m < max_range_m")
        self.pattern, self.backend = pattern, backend
        self.min_range_m, self.max_range_m = float(min_range_m), float(max_range_m)
        site_body = int(model.site_bodyid[model.site(site_name).id])
        robot_root = int(model.body_rootid[site_body])
        robot_bodies = model.body_rootid == robot_root if robot_root else np.zeros(model.nbody, dtype=bool)
        robot_bodies[0] = False
        for body in range(1, model.nbody):
            if not robot_bodies[body] and (model.body_jntnum[body] > 0 or model.body_mocapid[body] >= 0):
                raise ValueError("rolling static environment does not support non-robot dynamic bodies")
        selected = ~robot_bodies[model.geom_bodyid]
        if geomgroup is not None:
            groups = np.asarray(geomgroup)
            if groups.shape != (mujoco.mjNGROUP,):
                raise ValueError("geomgroup must have one entry per MuJoCo geometry group")
            selected &= groups[model.geom_group].astype(bool)
        self.environment_geom_ids = np.flatnonzero(selected)
        # One private copy, not one per ray. Only ray visibility groups change;
        # the live model, controls, collision filters and sensors are untouched.
        self.scene_model = copy.copy(model)
        self.scene_model.geom_group[:] = np.where(selected, 0, 1)
        self.scene_data = mujoco.MjData(self.scene_model)
        mujoco.mj_forward(self.scene_model, self.scene_data)
        self._groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self._warp = None
        if backend == "warp":
            from mujoco_lidar.core_warp.mjlidar_warp import MjLidarWarp
            self._warp = MjLidarWarp(self.scene_model, cutoff_dist=self.max_range_m,
                                    geomgroup=self._groups, device=device, use_bvh=True)
            self._warp.update(self.scene_data)
        self.metadata = {"acquisition_model": "rolling_static_environment",
                         "point_time_model": "uniform_from_csv_sample_order",
                         "sample_rate_hz": pattern.sample_rate_hz,
                         "nominal_point_interval_ns": 1e9 / pattern.sample_rate_hz,
                         "point_coordinates": "instantaneous LiDAR-local frame at each point's own acquisition time",
                         "deskewed": False,
                         "pose_interpolation": "linear_translation_short_arc_slerp",
                         "robot_self_occlusion": False,
                         "excluded_robot_root_body_id": robot_root if robot_root else None,
                         "environment_geom_count": int(selected.sum()),
                         "dynamic_environment_supported": False,
                         "ray_scene_copies": 1,
                         "pattern_timing_source": "synthetic CSV sample order; not recorded hardware timestamps"}

    def _ranges(self, origins, directions):
        if self.backend == "cpu":
            import mujoco
            return np.asarray([mujoco.mj_ray(self.scene_model, self.scene_data, origin, direction,
                                             self._groups, True, -1, None)
                               for origin, direction in zip(origins, directions)])
        import warp as wp
        backend = self._warp
        origins_wp = wp.array(np.asarray(origins, dtype=np.float32), dtype=wp.vec3, device=backend.device)
        directions_wp = wp.array(np.asarray(directions, dtype=np.float32), dtype=wp.vec3, device=backend.device)
        ranges = wp.empty(len(origins), dtype=wp.float32, device=backend.device)
        wp.launch(_world_ray_kernel(), dim=len(origins), inputs=[backend._bvh.id, backend.geom_types,
                  backend.geom_sizes, backend.geom_mesh_ids, backend.mesh_ids,
                  backend._geom_xpos, backend._geom_xmat, origins_wp, directions_wp,
                  self.max_range_m, ranges], device=backend.device)
        # numpy() waits for the device-to-host copy: callers time completed work.
        return ranges.numpy()

    def scan(self, data, stamp_ns, n_rays=20000, *, history):
        stamp_ns = _timestamp(stamp_ns)
        if not isinstance(n_rays, (int, np.integer)) or n_rays <= 0:
            raise ValueError("n_rays must be a positive integer")
        last_offset = round((int(n_rays) - 1) * 1e9 / self.pattern.sample_rate_hz)
        last_stamp = _timestamp(stamp_ns + last_offset)
        # Validate before consuming pattern state. A missing-history error must
        # not silently skip the next part of the physical scanning pattern.
        history.poses(np.array([stamp_ns, last_stamp], dtype=np.uint64))
        ids = self.environment_geom_ids
        if data is not None and (not np.allclose(data.geom_xpos[ids], self.scene_data.geom_xpos[ids], rtol=0, atol=1e-10)
                                 or not np.allclose(data.geom_xmat[ids], self.scene_data.geom_xmat[ids], rtol=0, atol=1e-10)):
            raise ValueError("static environment geometry changed after ray scene construction")
        rays = self.pattern.next_frame(n_rays)
        times = np.uint64(stamp_ns) + rays.offsets_ns.astype(np.uint64)
        origins, quaternions = history.poses(times)
        directions = rotate_vectors(quaternions, rays.directions)
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)
        distances = self._ranges(origins, directions)
        valid = np.isfinite(distances) & (distances >= self.min_range_m) & (distances < self.max_range_m)
        metadata = {**self.metadata, "history_coverage_ns": list(history.bounds_ns),
                    "first_scheduled_point_ns": stamp_ns, "last_scheduled_point_ns": last_stamp,
                    "maximum_pose_knot_interval_ns": history.max_interval_ns(stamp_ns, last_stamp)}
        return LidarScan(stamp_ns, (rays.directions[valid] * distances[valid, None]).astype(np.float32),
                         rays.offsets_ns[valid], rays.lines[valid], rays.ray_indices[valid], int(n_rays), metadata)
