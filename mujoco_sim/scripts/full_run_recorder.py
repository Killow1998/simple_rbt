"""Independent contracts and digests of messages actually received over DDS."""
from collections import Counter
import hashlib
import json
import math
import numpy as np
from full_run_evidence import imu_digest_update, scan_digest


def stamp_ns(value):
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def xyz(value):
    return [value.x, value.y, value.z]


def xyzw(value):
    return [value.x, value.y, value.z, value.w]


class RosRecorder:
    """Independent checks on messages received from DDS, never worker arrays."""

    def __init__(self):
        self.counts, self.errors = Counter(), Counter()
        self.last_stamp = {}
        self.first_clock = None
        self.scans, self.points, self.images, self.odometry = [], [], {}, []
        self.dynamic_tf, self.static_tf = {}, {}
        self.imu_hash = hashlib.sha256()
        self.imu_si_hash = hashlib.sha256()
        self.imu_stamps, self.imu_si_stamps = [], []
        self.image_stats = {}
        self.uniform_rgb_frames = Counter()
        self.maximum_rgb_std = {}
        self.status = None

    def stamped(self, key, stamp):
        self.counts[key] += 1
        if key in self.last_stamp and stamp <= self.last_stamp[key]:
            self.errors[key + "_nonmonotonic_stamp"] += 1
        self.last_stamp[key] = stamp

    def clock(self, message):
        stamp = stamp_ns(message.clock)
        self.stamped("clock", stamp)
        if self.first_clock is None:
            self.first_clock = stamp

    def scan(self, message):
        stamp = stamp_ns(message.header.stamp)
        self.stamped("lidar", stamp)
        if message.header.frame_id != "livox_frame" or stamp != message.timebase or message.point_num != len(message.points):
            self.errors["lidar_header"] += 1
        if not message.points:
            self.errors["empty_lidar"] += 1
            return
        values = np.asarray([(p.x, p.y, p.z, p.offset_time, p.line, p.tag, p.reflectivity) for p in message.points])
        offsets, lines = values[:, 3], values[:, 4]
        if (not np.isfinite(values).all() or np.any(offsets < 0) or np.any(offsets >= 100_000_000)
                or np.any(np.diff(offsets) < 0) or np.any(lines < 0) or np.any(lines > 3)
                or np.any(lines != (offsets // 5000) % 4)
                or np.any(values[:, 5] != 0) or np.any(values[:, 6] != 100)):
            self.errors["lidar_values"] += 1
        self.scans.append({"stamp_ns": stamp, "point_count": message.point_num,
                           "scan_sha256": scan_digest(values[:, :3], offsets, lines),
                           "last_point_ns": stamp + int(offsets[-1])})

    def pointcloud(self, message):
        stamp = stamp_ns(message.header.stamp)
        self.stamped("pointcloud2", stamp)
        layout = [(f.name, f.offset, f.datatype, f.count) for f in message.fields]
        expected = [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1), ("intensity", 12, 7, 1),
                    ("tag", 16, 2, 1), ("line", 17, 2, 1), ("timestamp", 18, 8, 1)]
        if (message.header.frame_id != "livox_frame" or message.point_step != 26 or message.height != 1
                or message.row_step != message.width * 26 or len(message.data) != message.row_step
                or message.is_bigendian or layout != expected):
            self.errors["pointcloud2_layout"] += 1
        self.points.append({"stamp_ns": stamp, "pointcloud2_sha256": hashlib.sha256(message.data).hexdigest()})

    def imu(self, message, si=False):
        key = "imu_si" if si else "imu"
        stamp = stamp_ns(message.header.stamp)
        self.stamped(key, stamp)
        (self.imu_si_stamps if si else self.imu_stamps).append(stamp)
        angular, acceleration, orientation = xyz(message.angular_velocity), xyz(message.linear_acceleration), xyzw(message.orientation)
        if (message.header.frame_id != ("imu_frame" if si else "livox_frame")
                or not np.isfinite(angular + acceleration).all()
                or orientation != ([0.] * 4 if si else [0., 0., 0., 1.])
                or list(message.orientation_covariance) != ([-1.] + [0.] * 8 if si else [0.] * 9)
                or list(message.angular_velocity_covariance) != [0.] * 9
                or list(message.linear_acceleration_covariance) != [0.] * 9):
            self.errors[key + "_values"] += 1
        if not si and np.asarray(angular + acceleration).tolist() != np.asarray(angular + acceleration, dtype=np.float32).astype(float).tolist():
            self.errors["imu_not_float32_wire_values"] += 1
        imu_digest_update(self.imu_si_hash if si else self.imu_hash, stamp, angular, acceleration, orientation)

    def odom(self, message):
        stamp = stamp_ns(message.header.stamp)
        self.stamped("odom", stamp)
        position, quaternion = xyz(message.pose.pose.position), xyzw(message.pose.pose.orientation)
        linear, angular = xyz(message.twist.twist.linear), xyz(message.twist.twist.angular)
        if (message.header.frame_id != "odom" or message.child_frame_id != "base_link"
                or not np.isfinite(position + quaternion + linear + angular).all()
                or abs(np.linalg.norm(quaternion) - 1) > 1e-5):
            self.errors["odom_values"] += 1
        self.odometry.append({"stamp_ns": stamp, "position": position, "quaternion": quaternion,
                              "linear": linear, "angular": angular})

    def tf(self, message, static=False):
        self.counts["tf_static" if static else "tf"] += 1
        for transform in message.transforms:
            row = {"parent": transform.header.frame_id, "child": transform.child_frame_id,
                   "stamp_ns": stamp_ns(transform.header.stamp), "position": xyz(transform.transform.translation),
                   "quaternion": xyzw(transform.transform.rotation)}
            if not np.isfinite(row["position"] + row["quaternion"]).all():
                self.errors["tf_values"] += 1
            if static:
                self.static_tf[row["child"]] = row
            else:
                if row["parent"] != "odom" or row["child"] != "base_link":
                    self.errors["tf_frames"] += 1
                self.dynamic_tf[row["stamp_ns"]] = row

    def image(self, message, key):
        stamp = stamp_ns(message.header.stamp)
        self.stamped(key, stamp)
        depth = key.endswith("depth")
        width, height = (720, 360) if key.startswith("panorama") else (320, 240)
        if (message.header.frame_id != "camera_optical_frame" or message.encoding != ("32FC1" if depth else "rgb8")
                or message.width != width or message.height != height or message.is_bigendian
                or message.step != width * (4 if depth else 3) or len(message.data) != message.height * message.step):
            self.errors[key + "_layout"] += 1
            return
        values = np.frombuffer(message.data, dtype="<f4" if depth else "u1")
        finite = np.isfinite(values)
        if depth:
            if not finite.any() or np.isnan(values).any() or np.isneginf(values).any() or np.any(values[finite] <= 0):
                self.errors[key + "_values"] += 1
        else:
            variation = float(np.std(values))
            self.maximum_rgb_std[key] = max(self.maximum_rgb_std.get(key, 0.), variation)
            # A valid camera can temporarily see only a uniform floor/sky.
            self.uniform_rgb_frames[key] += variation < 1.
        self.images.setdefault(key, []).append({"stamp_ns": stamp, "sha256": hashlib.sha256(message.data).hexdigest()})
        self.image_stats[key] = {"width": width, "height": height, "encoding": message.encoding,
                                 "no_hit_fraction": float(np.isposinf(values).mean()) if depth else 0.,
                                 "no_hit_count": int(np.isposinf(values).sum()) if depth else 0}

    def camera_info(self, message):
        self.stamped("camera_info", stamp_ns(message.header.stamp))
        focal = 240 / (2 * math.tan(math.radians(60) / 2))
        expected_k = [focal, 0., 159.5, 0., focal, 119.5, 0., 0., 1.]
        if (message.width != 320 or message.height != 240 or message.header.frame_id != "camera_optical_frame"
                or not np.allclose(message.k, expected_k, rtol=0, atol=1e-5)):
            self.errors["camera_info"] += 1

    def on_status(self, message):
        self.status = json.loads(message.data)
