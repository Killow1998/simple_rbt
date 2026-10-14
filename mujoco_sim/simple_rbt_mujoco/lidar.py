"""MID360 CSV rays and an indexed MuJoCo-LiDAR adapter, independent of ROS.

The bundled CSV's ``Time/s`` column actually contains consecutive sample IDs.
It is replayed at 200,000 rays/s: 20,000 rays span one 10 Hz scan. ``CsvLidar``
is an instantaneous snapshot: its timestamp is the snapshot time and offsets
are zero. Use ``RollingCsvLidar`` for acquisition times and motion distortion.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ScanRays:
    theta: np.ndarray  # azimuth, radians
    phi: np.ndarray  # elevation, radians (not CSV zenith)
    directions: np.ndarray  # sensor-local unit vectors, N x 3
    offsets_ns: np.ndarray
    lines: np.ndarray
    ray_indices: np.ndarray  # original zero-based CSV row indices, before filtering
    csv_sample_ids: np.ndarray  # unmodified first-column IDs


@dataclass(frozen=True)
class LidarScan:
    stamp_ns: int  # scan start for rolling acquisition; snapshot time otherwise
    xyz: np.ndarray  # float32, sensor-local, valid returns only
    offsets_ns: np.ndarray  # uint32, acquisition offsets of those SAME returns
    lines: np.ndarray  # uint8, original CSV row modulo laser count
    ray_indices: np.ndarray
    rays_cast: int
    metadata: dict = field(default_factory=dict)

    @property
    def point_num(self) -> int:
        return len(self.xyz)

    @property
    def valid_count(self) -> int:
        return self.point_num

    @property
    def missed_count(self) -> int:
        return self.rays_cast - self.point_num

    @property
    def timebase(self) -> int:
        return self.stamp_ns


class CsvScanPattern:
    """Continuous replay of consecutive CSV sample IDs and zenith directions."""

    def __init__(self, sample_ids: np.ndarray, azimuth: np.ndarray,
                 zenith: np.ndarray, sample_rate_hz: float = 200000,
                 lines: int = 4):
        if not np.isfinite(sample_rate_hz) or sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be finite and positive")
        if not isinstance(lines, (int, np.integer)) or not 1 <= lines <= 256:
            raise ValueError("lines must be an integer in [1, 256]")
        columns = tuple(np.asarray(value) for value in (sample_ids, azimuth, zenith))
        columns = tuple(value[:, 0] if value.ndim == 2 and value.shape[1] == 1 else value
                        for value in columns)
        if any(value.ndim != 1 for value in columns) or len({len(value) for value in columns}) != 1:
            raise ValueError("CSV must contain equal-length sample ID, azimuth and zenith columns")
        # Preserve column_stack's common numeric type without copying all
        # 800,000 rows into a second three-column table at startup.
        dtype = np.result_type(*(value.dtype for value in columns))
        ids, azimuth, zenith = (value.astype(dtype, copy=False) for value in columns)
        if len(ids) == 0 or not all(np.isfinite(value).all() for value in (ids, azimuth, zenith)):
            raise ValueError("CSV must contain finite sample IDs and angles")
        if not np.equal(ids, np.floor(ids)).all() or not np.equal(np.diff(ids), 1).all():
            raise ValueError("CSV first column must contain consecutive integer sample IDs, not seconds")
        if np.any((zenith < 0) | (zenith > 180)):
            raise ValueError("CSV zenith angles must be in [0, 180] degrees")
        self.sample_ids = ids.astype(np.int64)
        self.theta = np.deg2rad(azimuth).astype(np.float32, copy=False)
        self.phi = np.deg2rad(90.0 - zenith).astype(np.float32, copy=False)
        self.directions = np.empty((len(ids), 3), dtype=np.float32)
        cos_phi = np.cos(self.phi)
        np.multiply(cos_phi, np.cos(self.theta), out=self.directions[:, 0])
        np.multiply(cos_phi, np.sin(self.theta), out=self.directions[:, 1])
        np.sin(self.phi, out=self.directions[:, 2])
        self.sample_rate_hz = float(sample_rate_hz)
        self.lines = int(lines)
        self.cursor = 0
        self.rays_emitted = 0

    @classmethod
    def from_csv(cls, path: str | Path, sample_rate_hz: float = 200000,
                 lines: int = 4) -> CsvScanPattern:
        values = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.float64, ndmin=2)
        if values.shape[1] != 3:
            raise ValueError("CSV must have sample ID, azimuth/deg, zenith/deg columns")
        return cls(values[:, 0], values[:, 1], values[:, 2], sample_rate_hz, lines)

    def __len__(self) -> int:
        return len(self.sample_ids)

    def reset(self) -> None:
        self.cursor = 0
        self.rays_emitted = 0

    def next_frame(self, n_rays: int = 20000) -> ScanRays:
        if not isinstance(n_rays, (int, np.integer)) or n_rays <= 0:
            raise ValueError("n_rays must be a positive integer")
        last_offset = round((int(n_rays) - 1) * 1e9 / self.sample_rate_hz)
        if last_offset > np.iinfo(np.uint32).max:
            raise ValueError("scan duration exceeds Livox uint32 offset_time capacity")
        indices = (self.cursor + np.arange(n_rays, dtype=np.int64)) % len(self)
        offsets = np.rint(np.arange(n_rays) * (1e9 / self.sample_rate_hz)).astype(np.uint32)
        frame = ScanRays(
            theta=self.theta[indices], phi=self.phi[indices],
            directions=self.directions[indices], offsets_ns=offsets,
            lines=(indices % self.lines).astype(np.uint8),
            ray_indices=indices, csv_sample_ids=self.sample_ids[indices],
        )
        self.cursor = int((self.cursor + n_rays) % len(self))
        self.rays_emitted += int(n_rays)
        return frame


class CsvLidar:
    """Raycast every requested ray, then filter returns with the same index mask.

    ``stamp_ns`` is the instantaneous snapshot time (normally scan end). Every
    returned offset is zero: no earlier acquisition times are invented.
    ``bodyexclude=None`` excludes the site's body, not its articulated children.
    """

    def __init__(self, model: Any, pattern: CsvScanPattern, *, backend: str = "cpu",
                 site_name: str = "lidar", min_range_m: float = 0.1,
                 max_range_m: float = 40.0, bodyexclude: int | None = None,
                 device: str | None = None, geomgroup: np.ndarray | None = None):
        if backend not in ("cpu", "warp"):
            raise ValueError("backend must be 'cpu' or 'warp'")
        if not np.isfinite([min_range_m, max_range_m]).all() or not 0 <= min_range_m < max_range_m:
            raise ValueError("require finite 0 <= min_range_m < max_range_m")
        # Lazy import keeps pattern parsing usable without MuJoCo or ROS.
        from mujoco_lidar import MjLidarWrapper

        if bodyexclude is None:
            bodyexclude = int(model.site(site_name).bodyid[0])
            # A site attached to world must not suppress all world geometry.
            if bodyexclude == 0:
                bodyexclude = -1
        args: dict[str, Any] = {"bodyexclude": int(bodyexclude)}
        if device is not None:
            args["device"] = device
        if geomgroup is not None:
            args["geomgroup"] = np.asarray(geomgroup, dtype=np.uint8)
        self.raycaster = MjLidarWrapper(model, site_name, backend=backend,
                                     cutoff_dist=float(max_range_m), args=args)
        self.pattern = pattern
        self.min_range_m = float(min_range_m)
        self.max_range_m = float(max_range_m)
        self.metadata = {"acquisition_model": "snapshot_end",
                         "point_time_model": "all_returns_at_snapshot_stamp",
                         "sample_rate_hz": pattern.sample_rate_hz,
                         "pose_interpolation": None,
                         "robot_self_occlusion": "only_selected_body_excluded",
                         "pattern_timing_source": "synthetic CSV sample order; not recorded hardware timestamps"}

    def scan(self, data: Any, stamp_ns: int, n_rays: int = 20000) -> LidarScan:
        if isinstance(stamp_ns, (bool, np.bool_)) or not isinstance(stamp_ns, (int, np.integer)) or not 0 <= stamp_ns <= np.iinfo(np.uint64).max:
            raise ValueError("stamp_ns must be a nonnegative uint64 simulation timestamp")
        rays = self.pattern.next_frame(n_rays)
        distances = np.asarray(self.raycaster.trace_rays(data, rays.theta, rays.phi)).reshape(-1)
        if len(distances) != n_rays:
            raise RuntimeError("raycaster did not return one range per input ray")
        valid = np.isfinite(distances) & (distances >= self.min_range_m) & (distances < self.max_range_m)
        return LidarScan(
            stamp_ns=int(stamp_ns),
            xyz=(rays.directions[valid] * distances[valid, None]).astype(np.float32),
            offsets_ns=np.zeros(int(valid.sum()), dtype=np.uint32), lines=rays.lines[valid],
            ray_indices=rays.ray_indices[valid], rays_cast=int(n_rays),
            metadata=dict(self.metadata),
        )
