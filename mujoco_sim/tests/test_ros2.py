"""Transport contracts; live DDS test is opt-in and confined to localhost domain 83.

SIMPLE_RBT_ROS_TESTS=1 python -m unittest discover -s mujoco_sim/tests -p test_ros2.py
requires Humble and the local livox_ros_driver2 install to have been sourced.
"""
import os
import struct
import subprocess
import sys
import time
import threading
import unittest
from types import SimpleNamespace

import numpy as np

from simple_rbt_mujoco.ros2 import (
    VelocityCommand, pointcloud_data, split_stamp_ns, imu_wire_values, robot_command_topic,
    RosBridge, POINT_DTYPE, CUSTOM_POINT_CACHE_LIMIT, _typed_byte_array,
)
from simple_rbt_mujoco.mid360 import G0


def example_scan():
    return SimpleNamespace(
        stamp_ns=7_999_999_500,
        xyz=np.array([[1.25, -2.5, 3.75], [-4.0, 5.0, -6.0]], dtype=np.float32),
        offsets_ns=np.array([0, 900], dtype=np.uint32),
        lines=np.array([0, 3], dtype=np.uint8),
    )


class TransportContractTests(unittest.TestCase):
    def test_import_does_not_require_ros(self):
        result = subprocess.run(
            [sys.executable, "-c", "import simple_rbt_mujoco.ros2; import sys; "
             "assert simple_rbt_mujoco.ros2.robot_command_topic('go2') == '/go2/cmd_vel'; "
             "assert 'rclpy' not in sys.modules; assert 'livox_ros_driver2' not in sys.modules"],
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_robot_command_topic_requires_a_single_valid_name(self):
        for name in ("go2", "go2w", "Xian_Robot_2", "_robot"):
            with self.subTest(name=name):
                self.assertEqual(robot_command_topic(name), f"/{name}/cmd_vel")
        for name in ("", "/go2", "go2/left", "2go", "go-2", "go 2", "機器人", True, None):
            with self.subTest(name=name):
                with self.assertRaises((TypeError, ValueError)):
                    robot_command_topic(name)

    def test_bridge_cannot_silently_fall_back_to_a_global_command_topic(self):
        from simple_rbt_mujoco.ros2 import RosBridge
        with self.assertRaises(TypeError):
            RosBridge(lidar_extrinsics=((0., 0., 0.), (0., 0., 0., 1.)),
                      imu_extrinsics=((0., 0., 0.), (0., 0., 0., 1.)))

    def test_stamp_preserves_nanoseconds_across_second_boundary(self):
        self.assertEqual(split_stamp_ns(7_999_999_500), (7, 999_999_500))
        self.assertEqual(split_stamp_ns(8_000_000_400), (8, 400))
        self.assertEqual(split_stamp_ns(np.int64(0)), (0, 0))
        with self.assertRaises(ValueError):
            split_stamp_ns(-1)
        with self.assertRaises(TypeError):
            split_stamp_ns(1.2)

    def test_pointcloud_binary_layout_and_per_point_time(self):
        packed = pointcloud_data(example_scan())
        self.assertEqual(len(packed), 52)
        first = struct.unpack_from("<ffffBBd", packed, 0)
        second = struct.unpack_from("<ffffBBd", packed, 26)
        self.assertEqual(first, (1.25, -2.5, 3.75, 100.0, 0, 0, 7_999_999_500.))
        self.assertEqual(second, (-4.0, 5.0, -6.0, 100.0, 0, 3, 8_000_000_400.))

    def test_typed_payload_preserves_bytes_and_owns_storage(self):
        cases = {
            "rgb": np.arange(36, dtype=np.uint8).reshape(2, 6, 3),
            "strided_rgb": np.arange(36, dtype=np.uint8).reshape(2, 6, 3)[:, ::2],
            "reversed_depth": np.array([[np.inf, 2., 3.], [4., 5., 6.]], dtype="<f4")[:, ::-1],
            "empty_cloud": np.zeros(0, dtype=POINT_DTYPE),
            "structured_cloud": np.zeros(3, dtype=POINT_DTYPE),
        }
        for name, values in cases.items():
            with self.subTest(layout=name):
                expected = values.tobytes(order="C")
                payload = _typed_byte_array(values)
                self.assertEqual(payload.typecode, "B")
                self.assertEqual(bytes(payload), expected)
                values[...] = 0
                self.assertEqual(bytes(payload), expected, "ROS payload must own its storage")

    def test_point_pool_is_bounded_and_never_truncates_oversized_scans(self):
        # Lightweight stand-in avoids loading ROS in the default CPU test suite;
        # real generated setters and queued publication are checked over DDS below.
        bridge = object.__new__(RosBridge)
        bridge._types = {"CustomPoint": SimpleNamespace}
        bridge._custom_point_pool = []

        def scan_arrays(count, value):
            return (np.full((count, 3), value, dtype=np.float32),
                    np.arange(count, dtype=np.uint32) * 5000,
                    (np.arange(count, dtype=np.uint32) % 4).astype(np.uint8))

        original = bridge._custom_points(*scan_arrays(CUSTOM_POINT_CACHE_LIMIT, 1.))
        original_ids = [id(point) for point in original]
        for count in (1, 0, 7, CUSTOM_POINT_CACHE_LIMIT):
            points = bridge._custom_points(*scan_arrays(count, -2.))
            self.assertEqual(len(points), count)
            self.assertEqual([id(point) for point in points], original_ids[:count])
            self.assertEqual(len(bridge._custom_point_pool), CUSTOM_POINT_CACHE_LIMIT)
            for index, point in enumerate(points):
                self.assertEqual((point.x, point.y, point.z, point.offset_time,
                                  point.line, point.tag, point.reflectivity),
                                 (-2., -2., -2., index * 5000, index % 4, 0, 100))
        oversized = bridge._custom_points(*scan_arrays(CUSTOM_POINT_CACHE_LIMIT + 1, 3.))
        self.assertEqual(len(oversized), CUSTOM_POINT_CACHE_LIMIT + 1)
        self.assertEqual(len(bridge._custom_point_pool), CUSTOM_POINT_CACHE_LIMIT)
        self.assertTrue(set(original_ids).isdisjoint(id(point) for point in oversized))
        self.assertEqual(oversized[-1].offset_time, CUSTOM_POINT_CACHE_LIMIT * 5000)
        self.assertEqual(oversized[-1].x, 3.)

    def test_concurrent_scan_calls_serialize_the_entire_message_pair(self):
        # Block the first publisher with its message still pointing at the pool.
        # A simultaneous call must wait through BOTH publications before reuse.
        bridge = object.__new__(RosBridge)
        bridge._scan_lock = threading.RLock()
        bridge._custom_point_pool = []
        bridge.lidar_frame = "livox_frame"
        def message():
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace()))
        class Field(SimpleNamespace):
            FLOAT32, UINT8, FLOAT64 = 7, 2, 8
        bridge._types = {"CustomPoint": SimpleNamespace, "CustomMsg": message,
                         "PointCloud2": message, "PointField": Field}
        entered, release, second_attempted, second_published = (threading.Event() for _ in range(4))
        delivered, errors = [], []
        class Publisher:
            def __init__(self, kind):
                self.kind = kind
            def publish(self, msg):
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                if self.kind == "scan" and stamp == 1:
                    entered.set()
                    if not release.wait(2.):
                        raise AssertionError("test did not release first publisher")
                if stamp == 2:
                    second_published.set()
                payload = ([point.x for point in msg.points] if self.kind == "scan" else bytes(msg.data))
                delivered.append((stamp, self.kind, payload))
        bridge._scan_pub, bridge._points_pub = Publisher("scan"), Publisher("cloud")
        scans = [SimpleNamespace(stamp_ns=stamp, xyz=np.full((2, 3), stamp, dtype=np.float32),
                    offsets_ns=np.array([0, 5000], dtype=np.uint32),
                    lines=np.array([0, 1], dtype=np.uint8)) for stamp in (1, 2)]
        def send(scan):
            try:
                if scan.stamp_ns == 2:
                    second_attempted.set()
                bridge.publish_scan(scan)
            except BaseException as error:
                errors.append(error)
        threads = [threading.Thread(target=send, args=(scan,)) for scan in scans]
        try:
            threads[0].start()
            self.assertTrue(entered.wait(2.))
            threads[1].start()
            self.assertTrue(second_attempted.wait(2.))
            self.assertFalse(second_published.wait(.03), "second scan bypassed first scan's publication lock")
        finally:
            release.set()
            for thread in threads:
                if thread.ident is not None:
                    thread.join(2.)
        self.assertFalse(any(thread.is_alive() for thread in threads))
        self.assertEqual(errors, [])
        self.assertEqual(delivered, [(1, "scan", [1., 1.]), (1, "cloud", pointcloud_data(scans[0])),
                                     (2, "scan", [2., 2.]), (2, "cloud", pointcloud_data(scans[1]))])

    def test_mid360_raw_imu_uses_g_and_udp_float_precision(self):
        gyro, force = imu_wire_values((.1, .2, .3), (0., 0., G0))
        np.testing.assert_array_equal(gyro, np.array([.1, .2, .3], dtype=np.float32).astype(float))
        np.testing.assert_array_equal(force, [0., 0., 1.])
        with self.assertRaises(ValueError):
            imu_wire_values([0., float("nan"), 0.], [0., 0., G0])

    def test_malformed_scan_cannot_silently_wrap_offsets(self):
        scan = example_scan()
        scan.offsets_ns = np.array([0, 2**32], dtype=np.uint64)
        with self.assertRaises(ValueError):
            pointcloud_data(scan)
        scan.offsets_ns = np.array([0, -1], dtype=np.int64)
        with self.assertRaises(ValueError):
            pointcloud_data(scan)

    def test_deadman_expires_independently_of_simulation_time(self):
        now = [100.0]
        command = VelocityCommand(0.5, clock=lambda: now[0])
        self.assertEqual(command.value, (0.0, 0.0, 0.0))
        command.update(1, 2, 3)
        now[0] = 100.499
        self.assertEqual(command.value, (1.0, 2.0, 3.0))
        now[0] = 100.5
        self.assertEqual(command.value, (0.0, 0.0, 0.0))
        command.update(1, 2, 3)
        command.update(float("nan"), 0, 0)
        self.assertEqual(command.value, (0.0, 0.0, 0.0))

    def test_unsupported_lateral_command_stops_all_axes_and_allows_recovery(self):
        for lateral in (.2, -.2, 1e-12):
            with self.subTest(lateral=lateral):
                now = [100.]
                command = VelocityCommand(.5, clock=lambda: now[0], allow_lateral=False)
                command.update(.6, 0., .3)
                self.assertEqual(command.value, (.6, 0., .3))
                now[0] = 100.1
                command.update(.8, lateral, -.4)
                self.assertEqual(command.value, (0., 0., 0.))
                now[0] = 100.2
                command.update(.8, lateral, -.4)
                self.assertEqual(command.value, (0., 0., 0.))
                now[0] = 100.3
                command.update(-.2, -0., .1)
                self.assertEqual(command.value, (-.2, 0., .1))
                now[0] = 100.799
                self.assertEqual(command.value, (-.2, 0., .1))
                now[0] = 100.801
                self.assertEqual(command.value, (0., 0., 0.))


    def test_fr_validator_rejects_complete_command_before_deadman_and_recovers(self):
        from simple_rbt_mujoco.robots import command_is_supported
        now = [100.]
        command = VelocityCommand(.5, clock=lambda: now[0], allow_lateral=False,
            command_validator=lambda vx, vy, wz: command_is_supported("fr_mini", vx, vy, wz))
        invalid = ((.3, .1, .2), (.3, -1e-12, .2), (0., 0., .2),
                   (1e-12, 0., .2), (-1e-12, 0., -.2),
                   (1e-9, 0., .2), (-1e-9, 0., -.2))
        for values in invalid:
            with self.subTest(command=values):
                command.update(.3, 0., .2)
                self.assertEqual(command.value, (.3, 0., .2))
                now[0] += .1
                command.update(*values)
                self.assertEqual(command.value, (0., 0., 0.))
                command.update(-.3, 0., .2)
                self.assertEqual(command.value, (-.3, 0., .2))
        command.update(1e-9, 0., 0.)  # Tiny straight motion is not a spin request.
        self.assertEqual(command.value, (1e-9, 0., 0.))
        now[0] += .501
        self.assertEqual(command.value, (0., 0., 0.))


    def test_two_robot_deadmen_expire_independently_at_half_a_second(self):
        now = [100.0]
        commands = {name: VelocityCommand(.5, clock=lambda: now[0]) for name in ("go2", "go2w")}
        commands["go2"].update(.3, 0., .2)
        now[0] = 100.1
        commands["go2w"].update(-.2, .1, -.3)
        now[0] = 100.499
        self.assertEqual(commands["go2"].value, (.3, 0., .2))
        self.assertEqual(commands["go2w"].value, (-.2, .1, -.3))
        now[0] = 100.5
        self.assertEqual(commands["go2"].value, (0., 0., 0.))
        self.assertEqual(commands["go2w"].value, (-.2, .1, -.3))
        now[0] = 100.601
        self.assertEqual(commands["go2w"].value, (0., 0., 0.))



@unittest.skipUnless(os.environ.get("SIMPLE_RBT_ROS_TESTS") == "1", "live ROS test is opt-in")
class LiveRosTransportTests(unittest.TestCase):
    def test_scout_and_diffdrive_lateral_message_brakes_then_valid_command_recovers(self):
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        import rclpy
        from rclpy.context import Context
        from geometry_msgs.msg import Twist
        from simple_rbt_mujoco.ros2 import RosBridge

        context = Context()
        context.init(args=[], domain_id=83)
        peer = rclpy.create_node("simple_rbt_nonholonomic_command_test", context=context)
        try:
            for robot in ("scout_mini", "diffdrive"):
                with self.subTest(robot=robot), RosBridge(
                    robot_name=robot, domain_id=83, command_timeout=2.,
                    lidar_extrinsics=((0., 0., .2), (0., 0., 0., 1.)),
                    imu_extrinsics=((.011, .02329, .15588), (0., 0., 0., 1.))) as bridge:
                    publisher = peer.create_publisher(Twist, robot_command_topic(robot), 10)
                    try:
                        deadline = time.monotonic() + 5.
                        while not publisher.get_subscription_count() and time.monotonic() < deadline:
                            bridge.spin_once(.005)
                        self.assertEqual(publisher.get_subscription_count(), 1)
                        self.assertFalse(bridge.metadata["command_allows_lateral"])

                        def send_and_expect(values, expected):
                            message = Twist()
                            message.linear.x, message.linear.y, message.angular.z = values
                            publisher.publish(message)
                            deadline = time.monotonic() + .5
                            while bridge.command != expected and time.monotonic() < deadline:
                                bridge.spin_once(.005)
                            self.assertEqual(bridge.command, expected)

                        send_and_expect((.6, 0., .2), (.6, 0., .2))
                        # This must clear the whole command before the 2s timeout,
                        # without raising out of spin_once or continuing vx/wz.
                        send_and_expect((.8, .1, -.4), (0., 0., 0.))
                        send_and_expect((-.3, 0., .1), (-.3, 0., .1))
                    finally:
                        peer.destroy_publisher(publisher)
        finally:
            peer.destroy_node()
            context.try_shutdown()

    def test_local_ros_messages_commands_and_frames(self):
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["ROS_DOMAIN_ID"] = "83"
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from geometry_msgs.msg import Twist
        from livox_ros_driver2.msg import CustomMsg
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
        from tf2_msgs.msg import TFMessage
        from simple_rbt_mujoco.ros2 import RosBridge

        bridge = RosBridge(robot_name="go2w", lidar_extrinsics=((0.1, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0)),
                           imu_extrinsics=((0.03, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0)),
                           camera_extrinsics=((0.2, 0.0, 0.3), (-0.5, 0.5, -0.5, 0.5)),
                           domain_id=83, command_timeout=0.2)
        context = Context()
        context.init(args=[], domain_id=83)
        peer = rclpy.create_node("simple_rbt_transport_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(peer)
        received = {}
        subscriptions = []
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name, message_type, topic, qos in (
            ("clock", Clock, "/clock", best_effort),
            ("scan", CustomMsg, "/livox/lidar", 10),
            ("points", PointCloud2, "/livox/points", 10),
            ("imu", Imu, "/livox/imu", 10),
            ("imu_si", Imu, "/livox/imu_si", 10),
            ("odom", Odometry, "/odom", 10),
            ("tf", TFMessage, "/tf", 10),
            ("static", TFMessage, "/tf_static", durable),
            ("rgb", Image, "/camera/image_raw", 2),
            ("depth", Image, "/camera/depth/image_raw", 2),
            ("camera_info", CameraInfo, "/camera/camera_info", 2),
            ("panorama", Image, "/camera/panorama/image_raw", 2),
        ):
            subscriptions.append(peer.create_subscription(
                message_type, topic, lambda msg, key=name: received.__setitem__(key, msg), qos))
        command_pub = peer.create_publisher(Twist, bridge.command_topic, 10)
        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                bridge.spin_once()
                executor.spin_once(timeout_sec=0.01)
                if command_pub.get_subscription_count() and all(peer.count_publishers(s.topic_name) for s in subscriptions):
                    break
            self.assertTrue(command_pub.get_subscription_count(), "cmd_vel discovery timed out")
            scan = example_scan()
            # Strided input and infinite sky depth must survive serialization
            # byte-for-byte without changes to row order or floating-point bits.
            rgb = np.arange(36, dtype=np.uint8).reshape(2, 6, 3)[:, ::2]
            depth = np.array([[np.inf, 2.5, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)[:, ::-1]
            bridge.publish_clock(scan.stamp_ns)
            bridge.publish_scan(scan)
            bridge.publish_imu(scan.stamp_ns, (0.1, 0.2, 0.3), (0.0, 0.0, G0),
                               (0., 0., 1., 0.))  # Nonidentity truth must never leak.
            bridge.publish_odometry(scan.stamp_ns, (1.0, 2.0, 0.3), (0.0, 0.0, 0.0, 1.0),
                                    (0.4, 0.0, 0.0), (0.0, 0.0, 0.2))
            bridge.publish_camera(scan.stamp_ns, rgb, depth)
            bridge.publish_camera(scan.stamp_ns, rgb, depth, kind="panorama")
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = 0.4, -0.2, 0.3
            command_pub.publish(command)
            deadline = time.monotonic() + 5.0
            command_received = False
            while time.monotonic() < deadline:
                bridge.spin_once()
                executor.spin_once(timeout_sec=0.01)
                command_received |= bridge.command == (0.4, -0.2, 0.3)
                if len(received) == 12 and command_received:
                    break
            self.assertTrue(command_received, "cmd_vel never reached bridge")
            self.assertEqual(set(received), {"clock", "scan", "points", "imu", "imu_si", "odom", "tf", "static",
                                             "rgb", "depth", "camera_info", "panorama"})
            self.assertEqual(received["clock"].clock.sec, 7)
            self.assertEqual(received["clock"].clock.nanosec, 999_999_500)
            msg = received["scan"]
            self.assertEqual(msg.timebase, scan.stamp_ns)
            self.assertEqual(msg.header.frame_id, "livox_frame")
            self.assertEqual(msg.point_num, 2)
            self.assertEqual(msg.points[1].offset_time, 900)
            self.assertEqual(msg.points[1].line, 3)
            self.assertEqual(msg.points[1].tag, 0)
            cloud = received["points"]
            self.assertEqual(cloud.point_step, 26)
            self.assertEqual(cloud.row_step, 52)
            self.assertEqual([(f.name, f.offset, f.datatype) for f in cloud.fields][-3:],
                             [("tag", 16, 2), ("line", 17, 2), ("timestamp", 18, 8)])
            self.assertEqual(bytes(cloud.data), pointcloud_data(scan))
            self.assertEqual(received["imu"].header.frame_id, "livox_frame")
            self.assertEqual(received["imu"].linear_acceleration.z, 1.)
            self.assertEqual(received["imu_si"].header.frame_id, "imu_frame")
            self.assertEqual(received["imu_si"].linear_acceleration.z, G0)
            for name in ("imu", "imu_si"):
                imu = received[name]
                self.assertEqual([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w],
                                 [0.] * 4 if name == "imu_si" else [0., 0., 0., 1.])
                self.assertEqual(list(imu.orientation_covariance),
                                 ([-1.] + [0.] * 8) if name == "imu_si" else [0.] * 9)
                self.assertEqual(list(imu.angular_velocity_covariance), [0.] * 9)
                self.assertEqual(list(imu.linear_acceleration_covariance), [0.] * 9)
            self.assertEqual(bridge.metadata["imu_raw"]["linear_acceleration_unit"], "g")
            self.assertEqual(received["odom"].header.frame_id, "odom")
            self.assertEqual(received["odom"].child_frame_id, "base_link")
            transform = received["tf"].transforms[0]
            self.assertEqual(transform.child_frame_id, "base_link")
            self.assertEqual(transform.header.stamp.nanosec, 999_999_500)
            static = {t.child_frame_id: t for t in received["static"].transforms}
            self.assertAlmostEqual(static["livox_frame"].transform.translation.z, 0.2)
            self.assertAlmostEqual(static["imu_frame"].transform.translation.x, 0.03)
            self.assertAlmostEqual(static["camera_optical_frame"].transform.translation.x, 0.2)
            self.assertEqual(received["rgb"].encoding, "rgb8")
            self.assertEqual(bytes(received["rgb"].data), rgb.tobytes())
            self.assertEqual(received["depth"].encoding, "32FC1")
            self.assertEqual(bytes(received["depth"].data), depth.tobytes())
            self.assertEqual(received["camera_info"].width, 3)
            self.assertAlmostEqual(received["camera_info"].k[0], 1.7320508075688774)
            self.assertEqual(received["panorama"].encoding, "rgb8")
            time.sleep(0.21)
            self.assertEqual(bridge.command, (0.0, 0.0, 0.0))
        finally:
            executor.shutdown()
            peer.destroy_node()
            context.try_shutdown()
            bridge.close()
            bridge.close()  # Cleanup is safe after partial start or repeated shutdown.


    def test_queued_scans_keep_their_values_after_point_pool_reuse(self):
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from livox_ros_driver2.msg import CustomMsg
        from sensor_msgs.msg import PointCloud2

        received = {"scan": [], "cloud": []}
        context = Context()
        context.init(args=[], domain_id=83)
        peer = rclpy.create_node("simple_rbt_pool_lifetime_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(peer)
        bridge = None
        try:
            bridge = RosBridge(robot_name="go2w", domain_id=83,
                lidar_extrinsics=((.1, 0., .2), (0., 0., 0., 1.)),
                imu_extrinsics=((.111, .02329, .15588), (0., 0., 0., 1.)))
            subscriptions = [
                peer.create_subscription(CustomMsg, "/livox/lidar", received["scan"].append, 10),
                peer.create_subscription(PointCloud2, "/livox/points", received["cloud"].append, 10),
            ]
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                if all(peer.count_publishers(sub.topic_name) for sub in subscriptions) and (
                    bridge._scan_pub.get_subscription_count() and bridge._points_pub.get_subscription_count()):
                    break
                executor.spin_once(timeout_sec=.005)
            self.assertEqual(bridge._scan_pub.get_subscription_count(), 1)
            self.assertEqual(bridge._points_pub.get_subscription_count(), 1)
            scans = []
            expected = {}
            for index, count in enumerate((4, 1, 0, 3)):
                scan = SimpleNamespace(
                    stamp_ns=999_999_500 + index * 100_000_000,
                    xyz=(np.arange(count * 3, dtype=np.float32).reshape(count, 3) + 10 * index),
                    offsets_ns=np.arange(count, dtype=np.uint32) * 5000,
                    lines=((np.arange(count, dtype=np.uint32) + index) % 4).astype(np.uint8),
                )
                scans.append(scan)
                expected[scan.stamp_ns] = (scan.xyz.copy(), scan.offsets_ns.copy(),
                                          scan.lines.copy(), pointcloud_data(scan))
            # Queue all frames before any receiver callback. Reusing point objects
            # must not overwrite a preceding published frame's DDS-owned data.
            for scan in scans:
                bridge.publish_scan(scan)
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline and any(len(v) < 4 for v in received.values()):
                executor.spin_once(timeout_sec=.005)
            self.assertEqual({key: len(value) for key, value in received.items()}, {"scan": 4, "cloud": 4})
            self.assertEqual([msg.timebase for msg in received["scan"]], list(expected))
            for msg in received["scan"]:
                xyz, offsets, lines, _ = expected[msg.timebase]
                self.assertEqual(msg.point_num, len(xyz))
                self.assertEqual(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec, msg.timebase)
                self.assertEqual(msg.header.frame_id, "livox_frame")
                self.assertEqual([(point.x, point.y, point.z) for point in msg.points],
                                 [tuple(row) for row in xyz])
                self.assertEqual([point.offset_time for point in msg.points], offsets.tolist())
                self.assertEqual([point.line for point in msg.points], lines.tolist())
                self.assertTrue(all(point.tag == 0 and point.reflectivity == 100 for point in msg.points))
            for msg in received["cloud"]:
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                self.assertEqual(bytes(msg.data), expected[stamp][3])
                self.assertEqual(msg.width, len(expected[stamp][0]))
            bridge.close()
            self.assertEqual(bridge._custom_point_pool, [], "Close must release retained point objects")
        finally:
            executor.shutdown()
            peer.destroy_node()
            context.try_shutdown()
            if bridge is not None:
                bridge.close()

    def test_robot_namespaces_isolate_new_wheels_and_ignore_global_cmd_vel(self):
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["ROS_DOMAIN_ID"] = "83"
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import Twist
        from simple_rbt_mujoco.ros2 import RosBridge

        bridges = {}
        context = peer = executor = None
        try:
            for name in ("go2", "go2w", "fr_mini", "ridgeback", "ranger_mini"):
                bridges[name] = RosBridge(
                    robot_name=name, command_timeout=5., domain_id=83,
                    lidar_extrinsics=((.02, 0., .18), (0., 0., 0., 1.)),
                    imu_extrinsics=((.031, .02329, .13588), (0., 0., 0., 1.)))
                self.assertEqual(bridges[name].node.get_namespace(), f"/{name}")
                self.assertEqual(bridges[name].command_topic, robot_command_topic(name))
            context = Context()
            context.init(args=[], domain_id=83)
            peer = rclpy.create_node("simple_rbt_namespace_test", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(peer)
            publishers = {name: peer.create_publisher(Twist, robot_command_topic(name), 10)
                          for name in bridges}
            global_publisher = peer.create_publisher(Twist, "/cmd_vel", 10)

            def spin():
                for bridge in bridges.values():
                    bridge.spin_once()
                executor.spin_once(timeout_sec=.005)

            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                spin()
                if all(p.get_subscription_count() == 1 for p in publishers.values()):
                    break
            for name, publisher in publishers.items():
                self.assertEqual(publisher.get_subscription_count(), 1, f"{name} discovery/isolation failed")
            self.assertEqual(global_publisher.get_subscription_count(), 0)
            self.assertEqual(peer.get_subscriptions_info_by_topic("/cmd_vel"), [])

            expected = {"go2": (.3, .1, .2), "go2w": (-.2, -.1, -.3),
                        "fr_mini": (.3, 0., .2), "ridgeback": (.3, .2, .4),
                        "ranger_mini": (-.2, -.3, .5)}
            for name, bridge in bridges.items():
                self.assertEqual(bridge.metadata["command_allows_lateral"], name != "fr_mini")
            for name, values in expected.items():
                command = Twist()
                command.linear.x, command.linear.y, command.angular.z = values
                publishers[name].publish(command)
            deadline = time.monotonic() + 1.
            while time.monotonic() < deadline:
                spin()
                if all(bridges[name].command == values for name, values in expected.items()):
                    break
            for name, values in expected.items():
                self.assertEqual(bridges[name].command, values, f"Wrong command delivered to {name}")

            # Observe every reject through DDS, while unrelated robots keep
            # their independent compound commands. The .5s observation bound is
            # far shorter than the 5s timeout: zero must be an active rejection.
            invalid_fr = ((.4, .1, -.3), (0., 0., .2), (1e-12, 0., .2),
                          (-1e-12, 0., -.2), (1e-9, 0., .2), (-1e-9, 0., -.2))

            def send_fr_and_expect(values, result):
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = values
                publishers["fr_mini"].publish(message)
                deadline = time.monotonic() + .5
                while time.monotonic() < deadline:
                    spin()
                    if bridges["fr_mini"].command == result:
                        break
                self.assertEqual(bridges["fr_mini"].command, result)
                expected["fr_mini"] = result
                for robot, other_values in expected.items():
                    self.assertEqual(bridges[robot].command, other_values,
                                     f"FR message changed unrelated robot {robot}")

            send_fr_and_expect((-.3, 0., .2), (-.3, 0., .2))
            for invalid in invalid_fr:
                with self.subTest(fr_invalid=invalid):
                    send_fr_and_expect(invalid, (0., 0., 0.))
                    send_fr_and_expect((.3, 0., .2), (.3, 0., .2))
            send_fr_and_expect((-.3, 0., .2), (-.3, 0., .2))

            # An absolute, unnamespaced command must have no receiver and cannot
            # overwrite any robot's last command during the live timeout.
            global_command = Twist()
            global_command.linear.x, global_command.linear.y, global_command.angular.z = .9, .8, .7
            deadline = time.monotonic() + .25
            while time.monotonic() < deadline:
                global_publisher.publish(global_command)
                spin()
                for name, values in expected.items():
                    self.assertEqual(bridges[name].command, values, f"Global cmd_vel overwrote {name}")
            self.assertEqual(global_publisher.get_subscription_count(), 0)
            self.assertEqual(peer.get_subscriptions_info_by_topic("/cmd_vel"), [])
        finally:
            if executor is not None:
                executor.shutdown()
            if peer is not None:
                peer.destroy_node()
            if context is not None:
                context.try_shutdown()
            for bridge in bridges.values():
                bridge.close()



if __name__ == "__main__":
    unittest.main()
