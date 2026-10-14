"""Formal full-course gates and joins between physics and independent ROS evidence."""
import copy
import hashlib
import importlib.util
import os
import subprocess
from pathlib import Path
from simple_rbt_mujoco.paths import ROOT
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

SCRIPT = ROOT / "tools/validate_full_run.py"
spec = importlib.util.spec_from_file_location("validate_full_run", SCRIPT)
full = importlib.util.module_from_spec(spec)
spec.loader.exec_module(full)
from simple_rbt_mujoco.terrains import append_terrain


def test_source_fingerprints_cover_relocated_runtime_and_sensor_inputs():
    hashes = full.source_fingerprints()
    required = {
        "tools/project.py", "tools/ros_env.sh", "tools/validate_full_run.py", "tools/full_run_gates.py",
        "reference/python/simple_rbt_mujoco/paths.py",
        "reference/python/simple_rbt_mujoco/core.py",
        "reference/python/tests/test_full_run.py",
        "assets/catalog.json", "assets/models/diffdrive-none.xml",
        "assets/manifests/go2_manifest.json", "assets/policies/go2_robot_lab.onnx",
        "assets/lidar/mid360-real-centr.csv", "src/core.cpp", "src/lidar_cuda.cu",
        "include/simple_rbt/core.hpp", "tests/full_run_oracle.cpp",
        "third_party/manifest.json", "CMakeLists.txt",
        "ros2_interfaces/livox_ros_driver2/msg/CustomMsg.msg",
    }
    assert required <= hashes.keys()
    assert not any(path.startswith("build/") for path in hashes)


def test_default_native_worker_is_the_built_repository_entrypoint(tmp_path):
    executable = Path(full.worker_program("cpp", tmp_path / "config.json")[0])
    assert executable == ROOT / "build/native/simple-rbt-native"
    result = subprocess.run([str(executable), "--help"], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert "--robot" in result.stdout and "C++17" in result.stdout


def profile(name="flat", level="standard"):
    return append_terrain(ET.fromstring("<mujoco><worldbody/></mujoco>"), name, level=level)


def monitor(terrain=None):
    return full.CourseMonitor(terrain or profile(),
        {"settle_s": 2., "speed_m_s": .6, "commanded_distance_budget_m": 8.}, [0., 0., .4], .01)


def update(watcher, seconds, x, y=0., clearance=.3, tilt=0., torso=(), velocity=(.6, 0., 0.)):
    angle = np.radians(tilt)
    rotation = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
    position = np.array([x, y, watcher.profile.height_at(x, y) + clearance])
    watcher.update(round(seconds * 1e9), position, np.array([0., 0., 0., 1.]), rotation,
                   np.asarray(velocity), np.zeros(3), list(torso))


class FormalGateTests(unittest.TestCase):
    def test_long_course_requires_ascent_and_descent_finish(self):
        watcher = monitor(profile("ramp", "extreme"))
        update(watcher, 12, 6.)
        self.assertFalse(watcher.success)
        self.assertEqual(watcher.phase, "descent")
        self.assertEqual(watcher.thresholds["finish_x_m"], 10.6)
        update(watcher, 22, 10.6)
        self.assertTrue(watcher.success)
        self.assertEqual(watcher.finish_stamp_ns, 22_000_000_000)

    def test_clearing_finish_does_not_override_a_safety_failure(self):
        for kwargs, reason in [({"tilt": 51}, "tilt_limit"),
                               ({"clearance": .099}, "base_clearance_limit"),
                               ({"torso": ("base",)}, "torso_environment_contact")]:
            with self.subTest(reason=reason):
                watcher = monitor()
                update(watcher, 12, 6.1, **kwargs)
                self.assertFalse(watcher.success)
                self.assertEqual(watcher.failure, reason)

    def test_backtracking_does_not_disable_corridor_gate(self):
        watcher = monitor()
        update(watcher, 3, 1.1)
        update(watcher, 4, .5, y=1.)
        self.assertEqual(watcher.failure, "left_course_corridor")

    def test_timeout_is_not_misclassified_as_contact_or_fall(self):
        watcher = monitor()
        update(watcher, 16, .5)
        self.assertEqual(watcher.failure, "time_limit_before_finish")
        self.assertEqual(watcher.contact_steps, 0)
        self.assertEqual(watcher.report()["final"]["tilt_deg"], 0.)

    def test_nonfinite_state_cannot_complete(self):
        watcher = monitor()
        watcher.update(10_000_000_000, np.array([np.nan, 0, .4]), np.array([0, 0, 0, 1]),
                       np.eye(3), np.zeros(3), np.zeros(3), [])
        self.assertEqual(watcher.failure, "nonfinite_state")
        self.assertFalse(watcher.success)

    def test_tracking_window_excludes_settle_and_post_finish_tail(self):
        watcher = monitor()
        update(watcher, 1, .1, velocity=(10, 10, 10))
        self.assertEqual(len(watcher.velocities), 0)
        update(watcher, 12, 6.)
        update(watcher, 13, 6.1, velocity=(10, 10, 10))
        self.assertEqual(watcher.velocities, [[.6, 0., 0., 0., 0., 0.]])
        self.assertEqual(watcher.finish_stamp_ns, 12_000_000_000)

    def test_wheel_gate_uses_actual_body_contact_and_stricter_tilt_with_height_diagnostic(self):
        config = {"settle_s": 2., "speed_m_s": .6, "commanded_distance_budget_m": 8.,
                  "robot_half_width_m": .23}
        watcher = full.CourseMonitor(profile(), config, [0., 0., .08], .01)
        update(watcher, 12., 6.1, clearance=.08)
        self.assertTrue(watcher.success)
        self.assertIsNone(watcher.thresholds["minimum_base_clearance_m"])
        for kwargs, reason in [({"tilt": 46.}, "tilt_limit"),
                               ({"torso": ["chassis_deck / floor"]}, "torso_environment_contact")]:
            watcher = full.CourseMonitor(profile(), config, [0., 0., .2], .01)
            update(watcher, 3., 1.1, **kwargs)
            self.assertEqual(watcher.failure, reason)

    def test_wheel_corridor_follows_current_collision_width(self):
        config = {"settle_s": 2., "speed_m_s": .6, "commanded_distance_budget_m": 8.,
                  "robot_half_width_m": .23}
        watcher = full.CourseMonitor(profile(), config, [0., 0., .2], .01)
        watcher.update(3_000_000_000, np.array([1.1, .78, .2]), np.array([0., 0., 0., 1.]),
                       np.eye(3), np.array([.6, 0., 0.]), np.zeros(3), [], robot_half_width_m=.35)
        self.assertEqual(watcher.failure, "left_course_corridor")
        self.assertAlmostEqual(watcher.thresholds["corridor_half_width_m"], .75)

    def test_original_velocity_thresholds_are_not_relaxed(self):
        good = [[.6, 0, 0, 0, 0, 0]] * 100
        self.assertTrue(full.tracking_result(good, .6, .01)["passed"])
        for values in ([.7, 0, 0, 0, 0, 0], [.6, .101, 0, 0, 0, 0], [.6, 0, 0, 0, 0, -.101]):
            with self.subTest(values=values):
                self.assertFalse(full.tracking_result([values] * 100, .6, .01)["passed"])
        self.assertFalse(full.tracking_result(good[:99], .6, .01)["passed"])

    def test_actual_default_contact_mix_is_recorded_per_pair(self):
        friction = full.FrictionMonitor(None)
        friction.observe([1, 1, .01, .001, .001], "floor|body")
        friction.observe([.8, .8, .02, .01, .01], "floor|wheel")
        report = friction.report()
        self.assertTrue(report["passed"])
        self.assertEqual(report["contact_pairs"]["floor|body"]["effective_friction_min"][0], 1.)
        self.assertEqual(report["contact_pairs"]["floor|wheel"]["effective_friction_min"][0], .8)

    def test_explicit_contact_override_must_actually_reach_contacts(self):
        friction = full.FrictionMonitor(.4)
        self.assertFalse(friction.report()["passed"])
        friction.observe([.4, .4, .02, .01, .01], "floor|wheel")
        self.assertTrue(friction.report()["passed"])
        friction.observe([.8, .8, .02, .01, .01], "terrain|wheel")
        self.assertFalse(friction.report()["passed"])
        self.assertEqual(friction.report()["mismatched_samples"], 1)


def matching_evidence():
    """A consistent recorded-message fixture for testing the acceptance join."""
    config = {"run_id": "test", "robot": "go2w", "controller": "rl_sar", "camera_hz": 10,
              "settle_s": 2., "min_real_time_factor": .95, "command_topic": "/go2w/cmd_vel"}
    received = full.RosRecorder()
    received.status = {"run_id": "test", "pid": 123, "config_sha256": full.digest_json(config),
                       "finish_x_m": 6., "command_topic": "/go2w/cmd_vel"}
    received.first_clock = 2_000_000
    received.last_stamp["clock"] = 24_000_000_000
    received.counts.update({key: 240 for key in ("lidar", "pointcloud2", "odom", "tf", "rgb", "depth", "panorama_rgb", "panorama_depth", "camera_info")})
    received.counts["imu"] = received.counts["imu_si"] = 4801
    received.counts["clock"] = 1000
    received.imu_hash.update(b"fixture contains all expected IMU values")
    received.imu_si_hash.update(b"fixture contains all expected SI IMU values")
    received.imu_stamps = list(range(0, 24_000_000_001, 5_000_000))
    received.imu_si_stamps = received.imu_stamps.copy()
    for i in range(1, 241):
        stamp = i * 100_000_000
        row = {"stamp_ns": stamp, "position": [max(0., min(6.1, (i / 10 - 2) * .3)), 0., .4],
               "quaternion": [0., 0., 0., 1.], "linear": [.3 if i < 226 else 0., 0., 0.], "angular": [0., 0., 0.]}
        received.odometry.append(row)
        received.dynamic_tf[stamp] = {"position": row["position"], "quaternion": row["quaternion"]}
        received.scans.append({"stamp_ns": stamp - 100_000_000, "point_count": 2,
                               "scan_sha256": str(i), "last_point_ns": stamp - 1})
        received.points.append({"stamp_ns": stamp - 100_000_000, "pointcloud2_sha256": str(i)})
    for key in ("rgb", "depth", "panorama_rgb", "panorama_depth"):
        received.images[key] = [{"stamp_ns": i * 100_000_000, "sha256": str(i)} for i in range(1, 241)]
    received.image_stats["panorama_depth"] = {"no_hit_count": 100}
    model = {frame: {"translation": [0., 0., 0.], "quaternion": [0., 0., 0., 1.]}
             for frame in ("livox_frame", "imu_frame", "camera_optical_frame")}
    model["imu_frame"]["translation"] = [.011, .02329, -.04412]
    received.static_tf = {frame: {"parent": "base_link", "position": value["translation"], "quaternion": value["quaternion"]}
                          for frame, value in model.items()}
    evidence = {"run_id": "test", "pid": 123, "controller": "rl_sar", "config_sha256": full.digest_json(config),
                "command_topic": "/go2w/cmd_vel",
                "ros_transport": {"robot_name": "go2w", "node_namespace": "/go2w", "command_topic": "/go2w/cmd_vel"},
                "sources": {"source": "unchanged"}, "friction": {"passed": True}, "error": None,
                "formal_course": {"success": True, "failure_reason": None, "finish_stamp_ns": 22_000_000_000,
                                  "thresholds": {"finish_x_m": 6.}, "speed_tracking": {"passed": True}},
                "emitted": dict(received.counts), "simulation_end_ns": 24_000_000_000, "real_time_factor": .99,
                "imu_sha256": received.imu_hash.hexdigest(), "cameras": copy.deepcopy(received.images),
                "imu_si_sha256": received.imu_si_hash.hexdigest(), "physics_timestep_s": .002,
                "imu_samples": [{"stamp_ns": t,
                    "sampling_method": "exact" if t % 2_000_000 == 0 else "linear_interpolation",
                    "source_stamps_ns": [(t // 2_000_000) * 2_000_000, ((t + 1_999_999) // 2_000_000) * 2_000_000]}
                    for t in received.imu_stamps],
                "lidar_model": {"acquisition_model": "rolling_static_environment"},
                "odometry": copy.deepcopy(received.odometry),
                "scans": [{**scan, "pointcloud2_sha256": point["pointcloud2_sha256"],
                           "metadata": {"acquisition_model": "rolling_static_environment",
                                        "history_coverage_ns": [0, 24_000_000_000],
                                        "first_scheduled_point_ns": scan["stamp_ns"],
                                        "last_scheduled_point_ns": scan["stamp_ns"] + 99_995_000}}
                          for scan, point in zip(received.scans, received.points)],
                "command_transitions": [{"stamp_ns": 22_500_000_000, "monotonic_s": 10.5, "command": [0., 0., 0.]}],
                "final_command": [0., 0., 0.]}
    commands = [{"sim_stamp_ns": 2_000_000_000, "monotonic_s": 1.},
                {"sim_stamp_ns": 22_000_000_000, "monotonic_s": 10.}]
    return config, evidence, received, commands, model


class AcceptanceJoinTests(unittest.TestCase):
    def check_fixture(self, change=None):
        config, evidence, received, commands, model = matching_evidence()
        if change:
            change(evidence, received)
        with patch.object(full, "source_fingerprints", return_value={"source": "unchanged"}), \
                patch("check_ros2.model_sensor_transforms", return_value=model):
            return full.full_run_checks(config, evidence, received, commands, 0, {"source": "unchanged"}, 123)

    def test_same_run_evidence_can_pass_all_gates(self):
        checks, summary = self.check_fixture()
        self.assertTrue(summary["overall_acceptance"], checks)

    def test_route_transport_pass_cannot_hide_bad_speed_tracking(self):
        checks, summary = self.check_fixture(lambda evidence, _: evidence["formal_course"]["speed_tracking"].update(passed=False))
        self.assertFalse(summary["overall_acceptance"])
        self.assertTrue(summary["route_ros_realtime_pass"])
        self.assertFalse(checks["original_speed_thresholds_pass"])

    def test_actual_sensor_payload_or_count_mismatch_fails(self):
        for change, gate in [
            (lambda evidence, received: received.points[0].update(pointcloud2_sha256="corrupt"), "pointcloud_payloads_match_worker"),
            (lambda evidence, received: received.counts.update(imu=-1), "all_reliable_messages_received"),
            (lambda evidence, received: received.odometry[0]["position"].__setitem__(0, 42.), "odometry_matches_same_physics_run"),
        ]:
            with self.subTest(gate=gate):
                checks, summary = self.check_fixture(change)
                self.assertFalse(summary["overall_acceptance"])
                self.assertFalse(checks[gate])

    def test_source_or_controller_or_pid_change_fails(self):
        for change, gate in [
            (lambda evidence, received: evidence.update(sources={"source": "changed"}), "source_unchanged_during_run"),
            (lambda evidence, received: evidence.update(controller="another_policy"), "native_controller_unchanged"),
            (lambda evidence, received: received.status.update(pid=456), "same_run_identity"),
        ]:
            with self.subTest(gate=gate):
                checks, summary = self.check_fixture(change)
                self.assertFalse(summary["overall_acceptance"])
                self.assertFalse(checks[gate])

    def test_wrong_robot_or_global_command_topic_cannot_pass(self):
        for change in (
            lambda e, r: e.update(command_topic="/cmd_vel"),
            lambda e, r: r.status.update(command_topic="/go2/cmd_vel"),
            lambda e, r: e["ros_transport"].update(command_topic="/cmd_vel"),
            lambda e, r: e["ros_transport"].update(robot_name="go2"),
            lambda e, r: e["ros_transport"].update(node_namespace="/"),
        ):
            with self.subTest(change=change):
                checks, summary = self.check_fixture(change)
                self.assertFalse(summary["overall_acceptance"])
                self.assertFalse(checks["robot_command_namespace_matches"])

    def test_device_sampling_and_internal_extrinsics_are_required(self):
        for change, gate in [
            (lambda e, r: r.imu_stamps.pop(20), "mid360_imu_exact_200hz_grid"),
            (lambda e, r: r.imu_si_stamps.pop(20), "imu_covers_every_complete_scan"),
            (lambda e, r: e["imu_samples"][1].update(source_stamps_ns=[6_000_000, 8_000_000]), "imu_sampling_brackets_valid"),
            (lambda e, r: e["lidar_model"].update(acquisition_model="snapshot_end"), "rolling_lidar_acquisition"),
            (lambda e, r: e["scans"][0]["metadata"].update(history_coverage_ns=[0, 50_000_000]), "rolling_lidar_pose_history_covers_acquisition"),
            (lambda e, r: r.static_tf["imu_frame"].update(position=[0., 0., 0.]), "mid360_internal_rigid_transform"),
            (lambda e, r: e.update(imu_si_sha256="corrupted"), "imu_si_payloads_match_worker"),
        ]:
            with self.subTest(gate=gate):
                checks, summary = self.check_fixture(change)
                self.assertFalse(summary["overall_acceptance"])
                self.assertFalse(checks[gate])

    def test_realtime_deadman_and_physical_contact_remain_required(self):
        for change, gate in [
            (lambda evidence, received: evidence.update(real_time_factor=.94), "rtf_at_least_requested_minimum"),
            (lambda evidence, received: evidence.update(final_command=[.3, 0, 0]), "deadman_after_finish"),
            (lambda evidence, received: evidence["friction"].update(passed=False), "effective_contact_friction_verified"),
        ]:
            with self.subTest(gate=gate):
                checks, summary = self.check_fixture(change)
                self.assertFalse(summary["overall_acceptance"])
                self.assertFalse(checks[gate])


@unittest.skipUnless(os.environ.get("SIMPLE_RBT_ROS_TESTS") == "1", "live DDS interface test is opt-in")
class RecorderDdsTests(unittest.TestCase):
    def test_recorder_checks_actual_typed_messages_and_payload_hashes(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from livox_ros_driver2.msg import CustomMsg
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2
        from tf2_msgs.msg import TFMessage
        from simple_rbt_mujoco.ros2 import RosBridge, pointcloud_data, imu_wire_values
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        context = Context()
        context.init(args=[], domain_id=86)
        node = rclpy.create_node("full_run_recorder_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        recorder = full.RosRecorder()
        qos_static = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [node.create_subscription(kind, topic, callback, qos) for kind, topic, callback, qos in (
            (CustomMsg, "/livox/lidar", recorder.scan, 10), (PointCloud2, "/livox/points", recorder.pointcloud, 10),
            (Imu, "/livox/imu", recorder.imu, 10), (Odometry, "/odom", recorder.odom, 10),
            (Imu, "/livox/imu_si", lambda msg: recorder.imu(msg, True), 10),
            (TFMessage, "/tf", recorder.tf, 10), (TFMessage, "/tf_static", lambda msg: recorder.tf(msg, True), qos_static),
            (Image, "/camera/image_raw", lambda msg: recorder.image(msg, "rgb"), 10),
            (Image, "/camera/depth/image_raw", lambda msg: recorder.image(msg, "depth"), 10),
            (Image, "/camera/panorama/image_raw", lambda msg: recorder.image(msg, "panorama_rgb"), 10),
            (Image, "/camera/panorama/depth/image_raw", lambda msg: recorder.image(msg, "panorama_depth"), 10),
            (CameraInfo, "/camera/camera_info", recorder.camera_info, 10),
        )]
        pose = ((.02, 0., .18), (0., 0., 0., 1.))
        try:
            with RosBridge(robot_name="go2w", lidar_extrinsics=pose, imu_extrinsics=pose,
                           camera_extrinsics=pose, domain_id=86) as bridge:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline and any(bridge.node.count_subscribers(s.topic_name) < 1 for s in subscriptions):
                    bridge.spin_once(.002)
                    executor.spin_once(timeout_sec=.002)
                scan = SimpleNamespace(stamp_ns=1_900_000_000,
                    xyz=np.array([[1., 2., 3.], [4., 5., 6.]], dtype=np.float32),
                    offsets_ns=np.array([0, 99_995_000], dtype=np.uint32), lines=np.array([0, 3], dtype=np.uint8))
                bridge.publish_scan(scan)
                bridge.publish_imu(2_000_000_000, [.1, .2, .3], [0., 0., 9.81], [0., 0., 1., 0.])
                bridge.publish_odometry(2_000_000_000, [1., 2., .4], [0., 0., 0., 1.], [.3, 0., 0.], [0., 0., 0.])
                expected_images = {}
                for kind, shape in (("perspective", (240, 320)), ("panorama", (360, 720))):
                    rgb = np.arange(shape[0] * shape[1] * 3, dtype=np.uint8).reshape(*shape, 3)
                    depth = np.full(shape, 2.5, dtype=np.float32)
                    depth[:shape[0] // 2] = np.inf
                    bridge.publish_camera(2_000_000_000, rgb, depth, kind=kind)
                    for suffix, values in (("rgb", rgb), ("depth", depth)):
                        key = suffix if kind == "perspective" else "panorama_" + suffix
                        expected_images[key] = hashlib.sha256(values.tobytes()).hexdigest()
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline and sum(recorder.counts.values()) < 12:
                    bridge.spin_once(.001)
                    executor.spin_once(timeout_sec=.002)
                self.assertFalse(recorder.errors, recorder.errors)
                self.assertEqual(sum(recorder.counts.values()), 12)
                self.assertEqual(recorder.scans[0]["scan_sha256"], full.scan_digest(scan.xyz, scan.offsets_ns, scan.lines))
                self.assertEqual(recorder.points[0]["pointcloud2_sha256"], hashlib.sha256(pointcloud_data(scan)).hexdigest())
                self.assertEqual({key: rows[0]["sha256"] for key, rows in recorder.images.items()}, expected_images)
                imu_hash = hashlib.sha256()
                raw_gyro, raw_force = imu_wire_values([.1, .2, .3], [0., 0., 9.81])
                full.imu_digest_update(imu_hash, 2_000_000_000, raw_gyro, raw_force, [0., 0., 0., 1.])
                self.assertEqual(recorder.imu_hash.hexdigest(), imu_hash.hexdigest())
                si_hash = hashlib.sha256()
                full.imu_digest_update(si_hash, 2_000_000_000, [.1, .2, .3], [0., 0., 9.81], [0.] * 4)
                self.assertEqual(recorder.imu_si_hash.hexdigest(), si_hash.hexdigest())
        finally:
            executor.shutdown()
            node.destroy_node()
            context.try_shutdown()


if __name__ == "__main__":
    unittest.main()
