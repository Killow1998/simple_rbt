"""Optional ROS 2 transport for the MuJoCo simulation.

Importing this module does not import ROS. Source Humble and livox_ros_driver2
before constructing RosBridge. All public timestamps are *simulation* nanoseconds;
command expiry alone uses monotonic wall time so pausing the simulation is safe.
"""
from __future__ import annotations

from array import array
import math
import operator
import os
import re
import time
import threading
from typing import Callable, Sequence

import numpy as np

from .mid360 import G0


CUSTOM_POINT_CACHE_LIMIT = 20_000


POINT_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
    "offsets": [0, 4, 8, 12, 16, 17, 18],
    "itemsize": 26,
})


def split_stamp_ns(stamp_ns: int) -> tuple[int, int]:
    """Split a nonnegative integral simulation timestamp for builtin_interfaces/Time."""
    stamp_ns = operator.index(stamp_ns)
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    if not 0 <= seconds <= 2_147_483_647:
        raise ValueError("simulation timestamp is outside ROS Time range")
    return seconds, nanoseconds


def _finite_vector(value: Sequence[float], length: int, name: str) -> tuple:
    result = tuple(float(v) for v in value)
    if len(result) != length or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {length} finite values")
    return result


def _quaternion(value: Sequence[float]) -> tuple:
    result = _finite_vector(value, 4, "quaternion_xyzw")
    norm = math.sqrt(sum(v * v for v in result))
    if not math.isclose(norm, 1.0, abs_tol=1e-5):
        raise ValueError("quaternion_xyzw must have unit norm")
    return result


def _scan_arrays(scan):
    xyz = np.asarray(scan.xyz, dtype=np.float32)
    offsets = np.asarray(scan.offsets_ns)
    lines = np.asarray(scan.lines)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("scan xyz must have shape (N, 3)")
    count = len(xyz)
    if offsets.shape != (count,) or lines.shape != (count,):
        raise ValueError("scan offsets_ns and lines must each contain N values")
    if not np.isfinite(xyz).all():
        raise ValueError("scan xyz contains a nonfinite point")
    for values, maximum, name in ((offsets, 2**32 - 1, "offsets_ns"), (lines, 255, "lines")):
        if values.dtype.kind not in "iu" or np.any(values < 0) or np.any(values > maximum):
            raise ValueError(f"scan {name} contains values outside its integer message type")
    return xyz, offsets, lines


def _pointcloud_records(stamp_ns, xyz, offsets, lines):
    """Pack already-validated arrays; public entry points validate exactly once."""
    records = np.zeros(len(xyz), dtype=POINT_DTYPE)
    records["x"], records["y"], records["z"] = xyz.T
    records["intensity"] = 100.0
    # LivoxPointXyzrtlt is packed (26 bytes), with absolute ns stored as double.
    records["timestamp"] = np.asarray(offsets, dtype=np.uint64) + operator.index(stamp_ns)
    records["tag"] = 0
    records["line"] = lines
    return records


def pointcloud_data(scan) -> bytes:
    """Pack explicit little-endian PointCloud2 records, preserving each ray's time."""
    return _pointcloud_records(scan.stamp_ns, *_scan_arrays(scan)).tobytes()


def _typed_byte_array(values):
    """One C-buffer copy into ROS's public typed-array fast path.

    Noncontiguous input first gets a contiguous copy, preserving the existing
    C-order behavior. The array owns its data before the caller can reuse input.
    """
    contiguous = np.ascontiguousarray(values)
    result = array("B")
    if contiguous.nbytes:
        result.frombytes(memoryview(contiguous).cast("B"))
    return result


def imu_wire_values(angular_velocity, specific_force_m_s2):
    """MID360 UDP float32 values, promoted to ROS float64 without adding precision."""
    angular = np.asarray(_finite_vector(angular_velocity, 3, "angular_velocity"), dtype=np.float32)
    acceleration = np.asarray(_finite_vector(specific_force_m_s2, 3, "specific_force_m_s2")) / G0
    acceleration = acceleration.astype(np.float32)
    if not np.isfinite(angular).all() or not np.isfinite(acceleration).all():
        raise ValueError("IMU values exceed the finite float32 wire range")
    return angular.astype(float), acceleration.astype(float)


def robot_command_topic(robot_name: str) -> str:
    """One explicit robot name owns its velocity command namespace."""
    if not isinstance(robot_name, str) or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", robot_name) is None:
        raise ValueError("robot_name must be one ROS name: letters, digits or underscores; cannot start with a digit")
    return f"/{robot_name}/cmd_vel"


class VelocityCommand:
    """Latest supported planar command; invalid inputs clear motion immediately."""

    def __init__(self, timeout: float = 0.5, *, clock: Callable[[], float] = time.monotonic,
                 allow_lateral: bool = True,
                 command_validator: Callable[[float, float, float], bool] | None = None):
        self.timeout = float(timeout)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("command timeout must be positive and finite")
        self.allow_lateral = bool(allow_lateral)
        self._validator = command_validator
        self._clock = clock
        self._received_at = None
        self._value = (0.0, 0.0, 0.0)

    def update(self, vx: float, vy: float, wz: float) -> None:
        try:
            value = _finite_vector((vx, vy, wz), 3, "command")
            if not self.allow_lateral and value[1] != 0.:
                raise ValueError("This robot cannot command lateral velocity")
            if self._validator is not None and not self._validator(*value):
                raise ValueError("This robot cannot execute the requested planar motion")
        except (ValueError, TypeError):
            # Reject the entire command: do not keep old motion or silently
            # drop lateral velocity while accepting its forward/yaw components.
            self._received_at = None
            self._value = (0.0, 0.0, 0.0)
            return
        self._value = value
        self._received_at = self._clock()

    @property
    def value(self) -> tuple[float, float, float]:
        if self._received_at is None or self._clock() - self._received_at >= self.timeout:
            return (0.0, 0.0, 0.0)
        return self._value


class RosBridge:
    """Thin publishers and cmd_vel receiver; physics is owned by the caller.

    Required extrinsics are ``(translation_xyz, quaternion_xyzw)`` poses of each
    sensor in base_link. Never infer these from a guessed mounting height.
    Odometry position/orientation are in odom, while its twist is in base_link.
    IMU vectors are in imu_frame. Lidar XYZ is already in livox_frame.

    A new ROS context uses domain 83 unless ROS_DOMAIN_ID or domain_id is supplied.
    ROS_LOCALHOST_ONLY defaults to 1, preventing discovery on the physical robot's
    network. Explicit environment settings remain available for deliberate use.
    """

    def __init__(
        self,
        *,
        robot_name: str,
        lidar_extrinsics,
        imu_extrinsics,
        camera_extrinsics=None,
        command_timeout: float = 0.5,
        publish_pointcloud2: bool = True,
        domain_id: int | None = None,
        node_name: str = "simple_rbt_mujoco",
        odom_frame: str = "odom",
        base_frame: str = "base_link",
        lidar_frame: str = "livox_frame",
        imu_frame: str = "imu_frame",
        camera_frame: str = "camera_optical_frame",
    ):
        self.command_topic = robot_command_topic(robot_name)
        self.robot_name = robot_name
        self.node_namespace = f"/{robot_name}"
        from .robots import NO_LATERAL_ROBOTS, command_is_supported
        self._commands = VelocityCommand(command_timeout,
            allow_lateral=robot_name not in NO_LATERAL_ROBOTS,
            command_validator=lambda vx, vy, wz: command_is_supported(robot_name, vx, vy, wz))
        extrinsics = []
        for child, pose in ((lidar_frame, lidar_extrinsics), (imu_frame, imu_extrinsics)):
            translation, quaternion = pose
            extrinsics.append((child, _finite_vector(translation, 3, "translation"), _quaternion(quaternion)))
        if camera_extrinsics is not None:
            position, quaternion = camera_extrinsics
            extrinsics.append((camera_frame, _finite_vector(position, 3, "translation"), _quaternion(quaternion)))
        frames = (odom_frame, base_frame, lidar_frame, imu_frame) + ((camera_frame,) if camera_extrinsics is not None else ())
        if len(set(frames)) != len(frames):
            raise ValueError("odom, base, lidar and imu frames must be distinct")
        if not all(isinstance(frame, str) and frame and not frame.startswith("/")
                   for frame in frames):
            raise ValueError("TF frame names must be nonempty and must not begin with a slash")
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from geometry_msgs.msg import TransformStamped, Twist
            from livox_ros_driver2.msg import CustomMsg, CustomPoint
            from nav_msgs.msg import Odometry
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
            from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        except ImportError as error:
            raise RuntimeError(
                "ROS 2 bridge requires the Python 3.10 Humble environment and built "
                "livox_ros_driver2 messages. Source /opt/ros/humble/setup.bash and "
                "your Livox workspace install/local_setup.bash. The simulator core "
                "can run without --ros2."
            ) from error
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
        self._rclpy = rclpy
        self._types = dict(TransformStamped=TransformStamped, CustomMsg=CustomMsg,
                           CustomPoint=CustomPoint, Odometry=Odometry, Clock=Clock,
                           Imu=Imu, PointCloud2=PointCloud2, PointField=PointField, Image=Image, CameraInfo=CameraInfo)
        self.odom_frame, self.base_frame = odom_frame, base_frame
        self.lidar_frame, self.imu_frame = lidar_frame, imu_frame
        self.camera_frame = camera_frame
        self._custom_point_pool = []
        self._scan_lock = threading.RLock()
        self._closed = False
        self._context = Context()
        self._context.init(args=[], domain_id=int(os.environ.get("ROS_DOMAIN_ID", "83"))
                           if domain_id is None else domain_id)
        self.node = None
        self._executor = None
        try:
            self.node = rclpy.create_node(node_name, namespace=self.node_namespace,
                                         context=self._context, use_global_arguments=False)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self.node)
            self._clock_pub = self.node.create_publisher(
                Clock, "/clock", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self._scan_pub = self.node.create_publisher(CustomMsg, "/livox/lidar", 10)
            self._points_pub = self.node.create_publisher(PointCloud2, "/livox/points", 10) if publish_pointcloud2 else None
            self._imu_pub = self.node.create_publisher(Imu, "/livox/imu", 10)
            self._imu_si_pub = self.node.create_publisher(Imu, "/livox/imu_si", 10)
            self._odom_pub = self.node.create_publisher(Odometry, "/odom", 10)
            self._camera_pubs = {}
            if camera_extrinsics is not None:
                for kind, prefix in (("perspective", "/camera"), ("panorama", "/camera/panorama")):
                    self._camera_pubs[kind] = (
                        self.node.create_publisher(Image, prefix + "/image_raw", 2),
                        self.node.create_publisher(Image, prefix + "/depth/image_raw", 2),
                    )
                self._camera_info_pub = self.node.create_publisher(CameraInfo, "/camera/camera_info", 2)
            self._cmd_sub = self.node.create_subscription(Twist, "cmd_vel", self._on_command, 10)
            self._tf = TransformBroadcaster(self.node)
            self._static_tf = StaticTransformBroadcaster(self.node)
            self._static_tf.sendTransform([
                self._transform(0, base_frame, child, position, quaternion)
                for child, position, quaternion in extrinsics
            ])
        except Exception:
            self.close()
            raise

    @property
    def command(self) -> tuple[float, float, float]:
        return self._commands.value

    @property
    def metadata(self):
        return {
            "robot_name": self.robot_name, "node_namespace": self.node_namespace,
            "command_topic": self.command_topic, "command_timeout_wall_s": self._commands.timeout,
            "command_allows_lateral": self._commands.allow_lateral,
            "command_allows_in_place_turn": self.robot_name != "fr_mini",
            "invalid_command_policy": "clear all velocity components immediately; valid subsequent commands resume control",
            "driver_contract": "livox_ros_driver2 6b9356cadf77084619ba406e6a0eb41163b08039",
            "imu_input": "MID360 internal IMU gyro rad/s and specific force m/s^2; sampling owned by caller",
            "imu_rate_hz": 200,
            "imu_raw": {"topic": "/livox/imu", "frame_id": self.lidar_frame,
                        "angular_velocity_unit": "rad/s", "linear_acceleration_unit": "g",
                        "g0_m_s2": G0, "numeric_precision": "float32 UDP values promoted to ROS float64",
                        "orientation": "unavailable; fixed identity placeholder and zero covariances match ROS2 driver defaults, not an estimate",
                        "frame_convention": "driver uses LiDAR housing axes/name; measurement origin is internal IMU, see imu_frame TF"},
            "imu_si": {"topic": "/livox/imu_si", "frame_id": self.imu_frame,
                       "angular_velocity_unit": "rad/s", "linear_acceleration_unit": "m/s^2",
                       "orientation": "unavailable; zero quaternion, orientation_covariance[0]=-1",
                       "source": "same sampled physical SI values before UDP float32 conversion"},
            "orientation_argument": "accepted for API compatibility but never published; simulator truth is not an IMU measurement",
            "pointcloud2": {"point_step": 26, "timestamp": "float64 absolute simulation nanoseconds at byte18",
                            "tag": "0: MID360 normal high-confidence synthetic return"},
        }

    def _on_command(self, message) -> None:
        self._commands.update(message.linear.x, message.linear.y, message.angular.z)

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        if not self._closed and self._context.ok():
            self._executor.spin_once(timeout_sec=timeout_sec)

    @staticmethod
    def _stamp(target, stamp_ns: int) -> None:
        target.sec, target.nanosec = split_stamp_ns(stamp_ns)

    def _header(self, target, stamp_ns: int, frame: str) -> None:
        self._stamp(target.stamp, stamp_ns)
        target.frame_id = frame

    @staticmethod
    def _xyz(target, vector) -> None:
        target.x, target.y, target.z = _finite_vector(vector, 3, "vector")

    @staticmethod
    def _xyzw(target, quaternion) -> None:
        target.x, target.y, target.z, target.w = _quaternion(quaternion)

    def _transform(self, stamp_ns, parent, child, translation, quaternion):
        message = self._types["TransformStamped"]()
        self._header(message.header, stamp_ns, parent)
        message.child_frame_id = child
        self._xyz(message.transform.translation, translation)
        self._xyzw(message.transform.rotation, quaternion)
        return message

    def publish_clock(self, stamp_ns: int) -> None:
        message = self._types["Clock"]()
        self._stamp(message.clock, stamp_ns)
        self._clock_pub.publish(message)

    def _custom_points(self, xyz, offsets, lines):
        """Reuse at most 20k generated point objects; never truncate larger scans.

        Only public ROS setters are used. publish() synchronously converts the
        Python message before returning; the next scan can then reuse this pool.
        publish_scan holds the same lock through CustomMsg and PointCloud2.
        """
        count = len(xyz)
        factory = self._types["CustomPoint"]
        if count <= CUSTOM_POINT_CACHE_LIMIT:
            missing = count - len(self._custom_point_pool)
            if missing > 0:
                self._custom_point_pool.extend(factory(tag=0, reflectivity=100) for _ in range(missing))
            points = self._custom_point_pool[:count]
        else:
            # Preserve the previously unrestricted public scan size. Oversized
            # messages allocate their own objects without growing retained state.
            points = [factory(tag=0, reflectivity=100) for _ in range(count)]
        for point, point_xyz, offset, line in zip(points, xyz, offsets, lines):
            point.x, point.y, point.z = map(float, point_xyz)
            point.offset_time, point.line = int(offset), int(line)
        return points

    def publish_scan(self, scan) -> None:
        with self._scan_lock:
            xyz, offsets, lines = _scan_arrays(scan)
            message = self._types["CustomMsg"]()
            self._header(message.header, scan.stamp_ns, self.lidar_frame)
            message.timebase = operator.index(scan.stamp_ns)
            message.point_num = len(xyz)
            message.lidar_id = 0
            message.points = self._custom_points(xyz, offsets, lines)
            self._scan_pub.publish(message)
            if self._points_pub is not None:
                cloud = self._types["PointCloud2"]()
                self._header(cloud.header, scan.stamp_ns, self.lidar_frame)
                cloud.height, cloud.width = 1, len(xyz)
                field = self._types["PointField"]
                cloud.fields = [field(name=name, offset=offset, datatype=datatype, count=1)
                                for name, offset, datatype in (
                                    ("x", 0, field.FLOAT32), ("y", 4, field.FLOAT32),
                                    ("z", 8, field.FLOAT32), ("intensity", 12, field.FLOAT32),
                                    ("tag", 16, field.UINT8), ("line", 17, field.UINT8),
                                    ("timestamp", 18, field.FLOAT64))]
                cloud.is_bigendian = False
                cloud.point_step, cloud.row_step = 26, len(xyz) * 26
                cloud.is_dense = True
                # Humble validates a bytes sequence byte-by-byte in Python. A typed
                # byte array takes its supported fast path, preserving all bytes.
                cloud.data = _typed_byte_array(_pointcloud_records(scan.stamp_ns, xyz, offsets, lines))
                self._points_pub.publish(cloud)

    def publish_imu(self, stamp_ns: int, angular_velocity, linear_acceleration,
                    orientation_xyzw=None) -> None:
        """Publish MID360 raw driver values and a separate SI companion.

        Inputs are SI values at the internal IMU origin. The optional orientation
        argument is deliberately ignored: MID360 provides no orientation estimate.
        Raw /livox/imu mirrors the driver's non-SI g units and frame-name convention.
        """
        angular, acceleration_g = imu_wire_values(angular_velocity, linear_acceleration)
        message = self._types["Imu"]()
        self._header(message.header, stamp_ns, self.lidar_frame)
        self._xyz(message.angular_velocity, angular)
        self._xyz(message.linear_acceleration, acceleration_g)
        # The C++ ROS2 Imu default constructs Quaternion with w=1. This fixed
        # placeholder matches this local driver; it is not measured orientation.
        message.orientation.w = 1.0
        self._imu_pub.publish(message)
        si = self._types["Imu"]()
        self._header(si.header, stamp_ns, self.imu_frame)
        self._xyz(si.angular_velocity, angular_velocity)
        self._xyz(si.linear_acceleration, linear_acceleration)
        si.orientation.w = 0.0
        si.orientation_covariance[0] = -1.0
        self._imu_si_pub.publish(si)

    def publish_odometry(self, stamp_ns: int, position, quaternion_xyzw,
                         linear_velocity, angular_velocity) -> None:
        message = self._types["Odometry"]()
        self._header(message.header, stamp_ns, self.odom_frame)
        message.child_frame_id = self.base_frame
        self._xyz(message.pose.pose.position, position)
        self._xyzw(message.pose.pose.orientation, quaternion_xyzw)
        self._xyz(message.twist.twist.linear, linear_velocity)
        self._xyz(message.twist.twist.angular, angular_velocity)
        self._odom_pub.publish(message)
        self._tf.sendTransform(self._transform(stamp_ns, self.odom_frame, self.base_frame,
                                               position, quaternion_xyzw))

    def publish_camera(self, stamp_ns: int, rgb, depth, *, kind: str = "perspective",
                       fov_y_deg: float = 60.0) -> None:
        """Publish RGB and metric depth; panoramic images have no pinhole CameraInfo.

        Perspective depth is optical-axis Z; panoramic depth is radial distance.
        Optical frame extrinsics must have been provided at construction.
        """
        if kind not in ("perspective", "panorama"):
            raise ValueError("camera kind must be perspective or panorama")
        if kind not in self._camera_pubs:
            raise ValueError("camera_extrinsics are required to publish camera images")
        rgb = np.asarray(rgb)
        depth = np.asarray(depth, dtype="<f4")
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError("RGB image must be uint8 with shape (H, W, 3)")
        height, width = rgb.shape[:2]
        if height == 0 or width == 0 or depth.shape != (height, width):
            raise ValueError("depth and RGB must have matching nonzero height and width")
        if kind == "perspective" and not (math.isfinite(fov_y_deg) and 0 < fov_y_deg < 180):
            raise ValueError("vertical field of view must be between 0 and 180 degrees")
        for values, encoding, channels, publisher in (
            (rgb, "rgb8", 3, self._camera_pubs[kind][0]),
            (depth, "32FC1", 4, self._camera_pubs[kind][1]),
        ):
            message = self._types["Image"]()
            self._header(message.header, stamp_ns, self.camera_frame)
            message.height, message.width = height, width
            message.encoding, message.is_bigendian = encoding, False
            message.step = width * channels
            # Keep ROS assertions enabled; array('B') passes the generated
            # typed-buffer check without millions of per-byte Python checks.
            message.data = _typed_byte_array(values)
            publisher.publish(message)
        if kind == "perspective":
            info = self._types["CameraInfo"]()
            self._header(info.header, stamp_ns, self.camera_frame)
            info.height, info.width = height, width
            focal = height / (2.0 * math.tan(math.radians(fov_y_deg) / 2.0))
            cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
            info.distortion_model, info.d = "plumb_bob", [0.0] * 5
            info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self._camera_info_pub.publish(info)

    def close(self) -> None:
        with self._scan_lock:
            if self._closed:
                return
            self._closed = True
            self._custom_point_pool.clear()
            if self._executor is not None:
                self._executor.shutdown()
            if self.node is not None:
                self.node.destroy_node()
            self._context.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
