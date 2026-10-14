"""MID-360 IMU geometry and ideal 200 Hz sampling, independent of ROS.

The manual places the IMU at (+11, +23.29, -44.12) mm in the LiDAR
point-cloud frame, with parallel axes. FAST-LIO's opposite translation is
LiDAR-in-IMU, not the transform used to attach this IMU to the LiDAR.
Internal samples use SI specific force (including gravity at rest). The
hardware/driver2 acceleration unit is g; a transport must convert explicitly.
No orientation estimate, noise, bias, quantization or chip filtering is modeled.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from numbers import Real
import xml.etree.ElementTree as ET

import numpy as np

IMU_IN_LIDAR_POSITION = (.011, .02329, -.04412)
IMU_PERIOD_NS = 5_000_000
G0 = 9.80665
GYRO_LIMIT_RAD_S = float(np.deg2rad(2000.))
ACCEL_LIMIT_M_S2 = 4. * G0


def _vector(value, size, name):
    raw = np.asarray(value, dtype=object)
    if (raw.shape != (size,) or any(isinstance(v, (bool, np.bool_))
            or not isinstance(v, Real) for v in raw.flat)):
        raise ValueError(f"{name} must contain {size} real numbers, not booleans")
    result = np.asarray(value, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must be finite")
    return result.copy()


def _xml_vector(value, size, name):
    try:
        return _vector([float(v) for v in value.split()], size, name)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {name}") from exc


def _effective_site_defaults(root, parent, site):
    """Resolve only the class selected by this site and its default ancestors."""
    classes = {"main": {}}

    def visit(default, inherited, top=False):
        name = default.get("class", "main" if top else "")
        if not name or (top and name != "main") or (not top and name in classes):
            raise ValueError("Invalid or duplicate MJCF default class")
        attributes = dict(inherited)
        template = default.find("site")
        if template is not None:
            attributes.update(template.attrib)
        classes[name] = attributes
        for child in default.findall("default"):
            visit(child, attributes)

    defaults = root.findall("default")
    if len(defaults) > 1:
        raise ValueError("Expected at most one top-level MJCF default")
    if defaults:
        visit(defaults[0], {}, top=True)
    selected = site.get("class")
    ancestors = {child: node for node in root.iter() for child in node}
    ancestor = parent
    while selected is None and ancestor is not None:
        selected = ancestor.get("childclass")
        ancestor = ancestors.get(ancestor)
    selected = selected or "main"
    if selected not in classes:
        raise ValueError(f"Unknown lidar default class: {selected}")
    return selected, classes[selected]


def append_mid360_imu(root):
    """Append a separate IMU on the body's explicit LiDAR site pose.

    The input LiDAR site must be directly under a body/worldbody and use a
    local ``pos`` plus ``quat`` (including inherited defaults). Only the
    selected default class and its ancestors affect this pose. Effective
    alternate orientation encodings are rejected: MuJoCo does not clear a
    default euler/axisangle merely because a site supplies an explicit quat.
    Existing policy sensors and sites are never changed.
    """
    matches = [(parent, site) for parent in root.iter()
               for site in parent.findall("site") if site.get("name") == "lidar"]
    if len(matches) != 1 or matches[0][0].tag not in ("body", "worldbody"):
        raise ValueError("Expected exactly one lidar site directly under a body/worldbody")
    parent, lidar = matches[0]
    reserved = {"mid360_imu", "mid360_gyro", "mid360_accelerometer"}
    if any(element.get("name") in reserved for element in root.iter()):
        raise ValueError("MID-360 IMU names already exist")
    default_class, inherited = _effective_site_defaults(root, parent, lidar)
    attributes = {**inherited, **lidar.attrib}
    if any(key in attributes for key in ("euler", "axisangle", "xyaxes", "zaxis", "fromto")):
        raise ValueError("Effective lidar site orientation must use a local quaternion")
    position = _xml_vector(attributes.get("pos", "0 0 0"), 3, "lidar position")
    quaternion = _xml_vector(attributes.get("quat", "1 0 0 0"), 4, "lidar quaternion")
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < np.finfo(float).tiny:
        raise ValueError("lidar quaternion must have a nonzero finite norm")
    quaternion /= norm
    import mujoco
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, quaternion)
    position += rotation.reshape(3, 3) @ np.asarray(IMU_IN_LIDAR_POSITION)
    # Explicitly select the same safe defaults as the LiDAR. Its parent may
    # have a different childclass with an alternate orientation or fromto.
    ET.SubElement(parent, "site", {"name": "mid360_imu", "size": "0.002",
                  "class": default_class,
                  "pos": " ".join(format(v, ".17g") for v in position),
                  "quat": " ".join(format(v, ".17g") for v in quaternion)})
    sensors = root.find("sensor")
    if sensors is None:
        sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="mid360_gyro", site="mid360_imu")
    ET.SubElement(sensors, "accelerometer", name="mid360_accelerometer", site="mid360_imu")
    calibration = {"imu_in_lidar_position_m": list(IMU_IN_LIDAR_POSITION),
                   "imu_in_lidar_quaternion_wxyz": [1., 0., 0., 0.]}
    calibration_hash = hashlib.sha256(json.dumps(calibration, sort_keys=True,
                                                separators=(",", ":")).encode()).hexdigest()
    return {"site": "mid360_imu", "parent_frame": "lidar",
            "imu_in_lidar_position_m": list(IMU_IN_LIDAR_POSITION),
            "imu_in_lidar_quaternion_wxyz": [1., 0., 0., 0.],
            "point_transform": "p_lidar = p_imu + imu_in_lidar_position_m; axes aligned",
            "lidar_in_imu_position_m": [-v for v in IMU_IN_LIDAR_POSITION],
            "calibration_sha256": calibration_hash,
            "geometry_source": "Livox MID-360 User Manual EN, printed page 15",
            "calibration_scope": "nominal device geometry; not a per-unit measured calibration",
            "gyro_sensor": "mid360_gyro", "accelerometer_sensor": "mid360_accelerometer"}


@dataclass(frozen=True)
class ImuSample:
    stamp_ns: int
    angular_velocity_rad_s: np.ndarray
    specific_force_m_s2: np.ndarray
    sampling_method: str
    source_stamps_ns: tuple[int, int]
    saturation_enabled: bool
    gyro_saturated: tuple[bool, bool, bool]
    accel_saturated: tuple[bool, bool, bool]


class Mid360ImuSampler:
    """Sample adjacent physics observations on a zero-anchored 5 ms grid.

    Inputs must be finite SI measurements in the MID-360 IMU frame, at
    strictly increasing integer nanosecond timestamps. At a grid-aligned first
    observation (including zero), emit immediately. Otherwise wait for the
    next grid tick. Never extrapolate: an off-grid tick is emitted only after
    its right-hand physics observation arrives, using componentwise linear
    interpolation in the moving sensor frame. This is a temporal approximation,
    not an exact reconstruction or a model of the hardware anti-alias filter.

    Optional saturation clips each emitted axis to the default hardware range
    after interpolation. It is a range approximation only, not the chip pipeline.
    Default behavior is ideal/unclipped, and metadata makes this explicit.
    """
    def __init__(self, *, saturation=False):
        if not isinstance(saturation, (bool, np.bool_)):
            raise ValueError("saturation must be a boolean")
        self.saturation = bool(saturation)
        self._previous = None
        self._next_tick = None
        self.samples_emitted = 0

    @property
    def metadata(self):
        return {"frame_id": "mid360_imu", "period_ns": IMU_PERIOD_NS,
                "rate_hz": 200, "angular_velocity_unit": "rad/s",
                "specific_force_unit": "m/s^2", "hardware_acceleration_unit": "g",
                "standard_gravity_m_s2": G0, "orientation_available": False,
                "sampling": "exact on grid; otherwise adjacent-sample linear interpolation",
                "interpolation_frame": "moving sensor axes; temporal approximation",
                "extrapolation": False, "grid_origin_ns": 0,
                "saturation_enabled": self.saturation,
                "saturation_policy": "per-axis output clip after interpolation" if self.saturation else "ideal unclipped",
                "gyro_range_rad_s": GYRO_LIMIT_RAD_S, "specific_force_range_m_s2": ACCEL_LIMIT_M_S2,
                "noise_bias_quantization_chip_filter": "not modeled",
                "samples_emitted": self.samples_emitted}

    def _sample(self, stamp, gyro, accel, sources):
        gyro = gyro.copy()
        accel = accel.copy()
        gyro_flags = np.abs(gyro) > GYRO_LIMIT_RAD_S if self.saturation else np.zeros(3, dtype=bool)
        accel_flags = np.abs(accel) > ACCEL_LIMIT_M_S2 if self.saturation else np.zeros(3, dtype=bool)
        if self.saturation:
            np.clip(gyro, -GYRO_LIMIT_RAD_S, GYRO_LIMIT_RAD_S, out=gyro)
            np.clip(accel, -ACCEL_LIMIT_M_S2, ACCEL_LIMIT_M_S2, out=accel)
        gyro.setflags(write=False)
        accel.setflags(write=False)
        return ImuSample(stamp, gyro, accel,
                         "exact" if sources[0] == sources[1] else "linear_interpolation",
                         sources, self.saturation, tuple(bool(v) for v in gyro_flags),
                         tuple(bool(v) for v in accel_flags))

    def observe(self, stamp_ns, gyro, accel_si):
        if (isinstance(stamp_ns, (bool, np.bool_))
                or not isinstance(stamp_ns, (int, np.integer)) or stamp_ns < 0):
            raise ValueError("stamp_ns must be a nonnegative integer, not a boolean")
        stamp = int(stamp_ns)
        if self._previous is not None and stamp <= self._previous[0]:
            raise ValueError("IMU observation timestamps must strictly increase")
        gyro = _vector(gyro, 3, "gyro")
        accel = _vector(accel_si, 3, "accel_si")
        output = []
        if self._previous is None:
            self._next_tick = ((stamp + IMU_PERIOD_NS - 1) // IMU_PERIOD_NS) * IMU_PERIOD_NS
        while self._next_tick <= stamp:
            tick = self._next_tick
            if tick == stamp:
                output.append(self._sample(tick, gyro, accel, (stamp, stamp)))
            else:
                before, previous_gyro, previous_accel = self._previous
                weight = (tick - before) / (stamp - before)
                # Convex interpolation avoids overflow from subtracting opposing finite values.
                output.append(self._sample(tick, (1. - weight) * previous_gyro + weight * gyro,
                                           (1. - weight) * previous_accel + weight * accel,
                                           (before, stamp)))
            self._next_tick += IMU_PERIOD_NS
        self._previous = (stamp, gyro, accel)
        self.samples_emitted += len(output)
        return output
