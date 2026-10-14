#!/usr/bin/env python3
"""Exercise the actual CLI over localhost DDS and save an auditable pass/fail report.

Source Humble and livox_ros_driver2, then run using the project's Python 3.10 venv.
The simulator is a separate process (C++ by default; Python is an explicit oracle).
No fabricated sensor messages are injected.
Only velocity commands are published by this checker. The default domain is 84.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def vector(value):
    return [value.x, value.y, value.z]


def mid360_extrinsics_match(lidar_position, lidar_quaternion, imu_position, imu_quaternion):
    """Independent official nominal internal offset, expressed in housing axes."""
    q = np.asarray(lidar_quaternion)
    offset = np.array([.011, .02329, -.04412])
    rotated = offset + 2 * np.cross(q[:3], np.cross(q[:3], offset) + q[3] * offset)
    return bool(np.allclose(np.asarray(imu_position) - lidar_position, rotated, rtol=0, atol=1e-7)
                and abs(abs(np.dot(q, imu_quaternion)) - 1) < 1e-7)


def model_sensor_transforms(robot, controller):
    """Read generated sensor-site XML without constructing physics or a policy."""
    from simple_rbt_mujoco.robots import WHEEL_MODELS, wheel_model
    if robot == "go2w":
        from simple_rbt_mujoco.go2w import build_go2w_xml
        xml = build_go2w_xml()
    elif robot in WHEEL_MODELS:
        xml = wheel_model(robot)[0]()
    elif robot == "diffdrive":
        from simple_rbt_mujoco.core import _diffdrive_xml
        xml = _diffdrive_xml()
    elif controller in ("himloco", "robot_lab"):
        from simple_rbt_mujoco.go2 import build_go2_xml
        xml = build_go2_xml(controller)
    else:
        from simple_rbt_mujoco.core import UPSTREAM, _go2_xml
        xml = _go2_xml(UPSTREAM)
    root = ET.fromstring(xml)
    from simple_rbt_mujoco.mid360 import append_mid360_imu
    append_mid360_imu(root)
    expected = {}
    for frame, name in (("livox_frame", "lidar"), ("imu_frame", "mid360_imu"),
                        ("camera_optical_frame", "camera_mount")):
        site = root.find(f".//site[@name='{name}']")
        position = [float(v) for v in site.get("pos", "0 0 0").split()]
        w, x, y, z = map(float, site.get("quat", "1 0 0 0").split())
        quaternion = np.array([x, y, z, w])
        if name == "camera_mount":
            # Optical right/down/forward -> body forward/left/up.
            optical = np.array([-.5, .5, -.5, .5])
            a, b = quaternion[:3], optical[:3]
            quaternion = np.r_[w * b + optical[3] * a + np.cross(a, b),
                               w * optical[3] - np.dot(a, b)]
        expected[frame] = {"translation": position, "quaternion": quaternion.tolist()}
    return expected


def cli_program(args, artifacts):
    """Launch the selected real entrypoint with the same sensor/control options."""
    launchers = {"cpp": "run.sh", "python": "run-python.sh"}
    if args.runtime not in launchers:
        raise ValueError("runtime must be cpp or python")
    return [str(ROOT / "mujoco_sim" / launchers[args.runtime]),
            "--robot", args.robot, "--backend", args.backend,
            "--terrain", args.terrain, "--controller", args.controller,
            "--control-mode", args.control_mode, "--camera-hz", str(args.camera_hz),
            "--duration", str(args.duration), "--ros2", "--settle", "1", "--cameras",
            "--output", str(artifacts)]


def main():
    from simple_rbt_mujoco.robots import ROBOT_NAMES, CONTROLLER_NAMES, DEFAULT_CONTROLLERS, validate_terrain
    from simple_rbt_mujoco.terrains import TERRAIN_NAMES
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", choices=("cpp", "python"), default="cpp",
                        help="Simulator implementation (default: cpp); Python is the retained oracle")
    parser.add_argument("--output", type=Path, default=ROOT / "output/validation-20261013/ros2-end-to-end.json")
    parser.add_argument("--domain", type=int, default=84)
    parser.add_argument("--backend", choices=("cpu", "warp"), default="cpu")
    parser.add_argument("--robot", choices=ROBOT_NAMES, default="go2")
    parser.add_argument("--terrain", choices=TERRAIN_NAMES, default="flat")
    parser.add_argument("--controller", choices=CONTROLLER_NAMES, default="auto")
    parser.add_argument("--camera-hz", type=int, choices=(1, 10), default=1)
    parser.add_argument("--control-mode", choices=("native", "tracked"), default="tracked")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--min-real-time-factor", type=float,
                        help="Optional minimum simulation/wall-clock ratio required to pass")
    parser.add_argument("--wall-timeout", type=float, default=120.0)
    args = parser.parse_args()
    try:
        validate_terrain(args.robot, args.terrain)
    except ValueError as error:
        parser.error(str(error))
    if args.duration < 8.0:
        parser.error("duration must be at least 8 seconds for command and timeout acceptance")
    if args.min_real_time_factor is not None and (not math.isfinite(args.min_real_time_factor) or args.min_real_time_factor <= 0):
        parser.error("min-real-time-factor must be positive and finite")
    if args.output.exists():
        parser.error("output report already exists; use a new path to preserve evidence")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ.setdefault("MUJOCO_GL", "egl")
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
    from simple_rbt_mujoco.ros2 import robot_command_topic

    command_topic = robot_command_topic(args.robot)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S-%fZ")
    artifacts = args.output.parent / ("ros2-cli-" + run_id)
    log_path = args.output.parent / ("ros2-cli-" + run_id + ".log")
    context = Context()
    context.init(args=[], domain_id=args.domain)
    node = rclpy.create_node("simple_rbt_end_to_end_check", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    counts = Counter()
    errors = Counter()
    clock_samples = []
    odometry = []
    scan_stats = []
    camera_stats = {}
    camera_stamps = {}
    imu_stamps = []
    imu_si_stamps = []
    imu_values, imu_si_values = {}, {}
    imu_max_abs = np.zeros(6)
    transforms = {}
    static_transforms = {}

    def on_clock(msg):
        stamp = nanoseconds(msg.clock)
        if clock_samples and stamp <= clock_samples[-1]:
            errors["clock_not_strictly_increasing"] += 1
        clock_samples.append(stamp)
        counts["clock"] += 1

    def on_scan(msg):
        counts["lidar"] += 1
        stamp = nanoseconds(msg.header.stamp)
        if stamp != msg.timebase:
            errors["lidar_header_timebase_mismatch"] += 1
        if msg.header.frame_id != "livox_frame" or len(msg.points) != msg.point_num:
            errors["lidar_frame_or_count_invalid"] += 1
        points = np.asarray([(p.x, p.y, p.z, p.offset_time, p.line, p.tag) for p in msg.points])
        if not len(points):
            errors["empty_lidar_scan"] += 1
            return
        if not np.isfinite(points[:, :3]).all():
            errors["nonfinite_lidar_xyz"] += 1
        offsets = points[:, 3].astype(np.int64)
        if np.any(offsets < 0) or np.any(offsets >= 100_000_000) or np.any(np.diff(offsets) < 0):
            errors["lidar_offsets_invalid"] += 1
        if np.any(points[:, 4] != (offsets // 5000) % 4) or np.any(points[:, 5] != 0):
            errors["lidar_lines_or_tags_invalid"] += 1
        scan_stats.append({"stamp_ns": stamp, "points": int(msg.point_num),
                           "first_offset_ns": int(offsets[0]), "last_offset_ns": int(offsets[-1])})

    def on_points(msg):
        counts["pointcloud2"] += 1
        expected = [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7),
                    ("tag", 16, 2), ("line", 17, 2), ("timestamp", 18, 8)]
        if (msg.header.frame_id != "livox_frame" or msg.point_step != 26 or msg.row_step != msg.width * 26
                or [(f.name, f.offset, f.datatype) for f in msg.fields] != expected):
            errors["pointcloud2_layout_invalid"] += 1
        if len(msg.data) != msg.height * msg.row_step:
            errors["pointcloud2_payload_size_invalid"] += 1

    def on_imu(msg, si=False):
        key = "imu_si" if si else "imu"
        counts[key] += 1
        stamp = nanoseconds(msg.header.stamp)
        stamps = imu_si_stamps if si else imu_stamps
        if stamps and stamp <= stamps[-1]:
            errors[key + "_time_not_strictly_increasing"] += 1
        stamps.append(stamp)
        values = np.asarray(vector(msg.angular_velocity) + vector(msg.linear_acceleration))
        q = msg.orientation
        if (not np.isfinite(values).all() or msg.header.frame_id != ("imu_frame" if si else "livox_frame")
                or [q.x, q.y, q.z, q.w] != ([0.] * 4 if si else [0., 0., 0., 1.])
                or list(msg.orientation_covariance) != ([-1.] + [0.] * 8 if si else [0.] * 9)
                or list(msg.angular_velocity_covariance) != [0.] * 9
                or list(msg.linear_acceleration_covariance) != [0.] * 9):
            errors[key + "_invalid"] += 1
        (imu_si_values if si else imu_values)[stamp] = values
        if not si:
            imu_max_abs[:] = np.maximum(imu_max_abs, np.abs(values))

    def on_odom(msg):
        counts["odom"] += 1
        if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
            errors["odometry_frames_invalid"] += 1
        q = msg.pose.pose.orientation
        values = vector(msg.pose.pose.position) + [q.x, q.y, q.z, q.w] + vector(msg.twist.twist.linear)
        angular = vector(msg.twist.twist.angular)
        if not np.isfinite(values + angular).all() or abs(np.linalg.norm(values[3:7]) - 1) > 1e-5:
            errors["odometry_values_invalid"] += 1
        odometry.append({"stamp_ns": nanoseconds(msg.header.stamp), "position": values[:3],
                         "linear_velocity": values[7:], "angular_velocity": angular})

    def on_tf(msg, static=False):
        counts["tf_static" if static else "tf"] += 1
        target = static_transforms if static else transforms
        for item in msg.transforms:
            q = item.transform.rotation
            target[item.child_frame_id] = {
                "parent": item.header.frame_id, "stamp_ns": nanoseconds(item.header.stamp),
                "translation": vector(item.transform.translation), "quaternion": [q.x, q.y, q.z, q.w],
            }

    def on_image(msg, key):
        counts[key] += 1
        stamp = nanoseconds(msg.header.stamp)
        stamps = camera_stamps.setdefault(key, [])
        if stamps and stamp <= stamps[-1]:
            errors[key + "_time_not_strictly_increasing"] += 1
        stamps.append(stamp)
        depth = key.endswith("depth")
        expected_encoding = "32FC1" if depth else "rgb8"
        expected_step = msg.width * (4 if depth else 3)
        if (msg.encoding != expected_encoding or msg.step != expected_step or
                len(msg.data) != msg.height * msg.step or msg.header.frame_id != "camera_optical_frame"):
            errors[key + "_layout_invalid"] += 1
            return
        values = np.frombuffer(msg.data, dtype="<f4" if depth else np.uint8)
        finite = values[np.isfinite(values)]
        if not len(finite) or (depth and (np.any(finite <= 0) or np.isnan(values).any() or np.isneginf(values).any())):
            errors[key + "_values_invalid"] += 1
        if not depth and np.std(values) < 1.0:
            errors[key + "_uniform_image"] += 1
        camera_stats[key] = {"width": msg.width, "height": msg.height,
                             "encoding": msg.encoding, "stamp_ns": nanoseconds(msg.header.stamp),
                             "finite_fraction": float(len(finite) / len(values)),
                             "no_hit_count": int(np.isposinf(values).sum()) if depth else 0,
                             "no_hit_fraction": float(np.isposinf(values).mean()) if depth else 0.0,
                             "min_finite": float(np.min(finite)) if len(finite) else None,
                             "max_finite": float(np.max(finite)) if len(finite) else None}

    def on_camera_info(msg):
        counts["camera_info"] += 1
        expected_focal = msg.height / (2 * math.tan(math.radians(60) / 2))
        if (msg.width != 320 or msg.height != 240 or abs(msg.k[0] - expected_focal) > 1e-5
                or msg.header.frame_id != "camera_optical_frame"):
            errors["perspective_calibration_invalid"] += 1

    subscriptions = []
    qos_clock = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    qos_static = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for msg_type, topic, callback, qos in (
        (Clock, "/clock", on_clock, qos_clock),
        (CustomMsg, "/livox/lidar", on_scan, 10),
        (PointCloud2, "/livox/points", on_points, 10),
        (Imu, "/livox/imu", on_imu, 100),
        (Imu, "/livox/imu_si", lambda msg: on_imu(msg, True), 100),
        (Odometry, "/odom", on_odom, 20),
        (TFMessage, "/tf", on_tf, 10),
        (TFMessage, "/tf_static", lambda msg: on_tf(msg, True), qos_static),
        (Image, "/camera/image_raw", lambda msg: on_image(msg, "rgb"), 2),
        (Image, "/camera/depth/image_raw", lambda msg: on_image(msg, "depth"), 2),
        (Image, "/camera/panorama/image_raw", lambda msg: on_image(msg, "panorama_rgb"), 2),
        (Image, "/camera/panorama/depth/image_raw", lambda msg: on_image(msg, "panorama_depth"), 2),
        (CameraInfo, "/camera/camera_info", on_camera_info, 2),
    ):
        subscriptions.append(node.create_subscription(msg_type, topic, callback, qos))
    command_pub = node.create_publisher(Twist, command_topic, 10)
    command = Twist()
    command.linear.x = 0.3
    program = cli_program(args, artifacts)
    child_env = os.environ.copy()
    child_env["PYTHONPATH"] = str(ROOT / "mujoco_sim") + os.pathsep + child_env.get("PYTHONPATH", "")
    started = time.monotonic()
    process = None
    first_command_sim_ns = last_command_sim_ns = None
    command_count = 0
    try:
        with log_path.open("x") as log:
            process = subprocess.Popen(program, cwd=ROOT, env=child_env, stdout=log, stderr=subprocess.STDOUT)
            next_command = started
            finished_at = None
            while time.monotonic() - started < args.wall_timeout:
                executor.spin_once(timeout_sec=0.005)
                now = time.monotonic()
                sim_ns = clock_samples[-1] if clock_samples else 0
                if now >= next_command and command_pub.get_subscription_count():
                    # Publish from sim t=1 through t=5 (four seconds), then silence.
                    # No explicit zero is sent: the bridge dead-man must stop motion.
                    if 1_000_000_000 <= sim_ns < 5_000_000_000:
                        command_pub.publish(command)
                        first_command_sim_ns = sim_ns if first_command_sim_ns is None else first_command_sim_ns
                        last_command_sim_ns = sim_ns
                        command_count += 1
                    next_command = now + 0.05
                if process.poll() is not None:
                    if finished_at is None:
                        finished_at = now
                    if now - finished_at >= 0.4:
                        break
            else:
                errors["simulator_wall_timeout"] += 1
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
    except Exception as error:
        errors["checker_exception"] += 1
        exception = repr(error)
    else:
        exception = None
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()

    summary_path = artifacts / "summary.json"
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else None
    checks = {}
    checks["simulator_exited_successfully"] = process is not None and process.returncode == 0
    checks["requested_runtime_profile_applied"] = bool(summary and
        summary.get("robot") == args.robot and summary.get("backend") == args.backend and
        summary.get("control_mode") == args.control_mode and summary.get("camera_hz") == args.camera_hz and
        summary.get("terrain", {}).get("name") == args.terrain and
        summary.get("controller") == (DEFAULT_CONTROLLERS[args.robot] if args.controller == "auto" else args.controller))
    contract = (summary or {}).get("ros_sensor_contract") or {}
    checks["robot_command_namespace_matches"] = bool(
        contract.get("robot_name") == args.robot and
        contract.get("node_namespace") == "/" + args.robot and
        contract.get("command_topic") == command_topic)
    checks["rolling_mid360_acquisition_profile"] = bool(summary and
        isinstance(summary.get("scan_model"), dict) and
        summary["scan_model"].get("acquisition_model") == "rolling_static_environment" and
        isinstance(summary.get("imu_model"), dict) and summary["imu_model"].get("rate_hz") == 200)
    if args.min_real_time_factor is not None:
        checks["wall_clock_real_time_target_met"] = bool(summary and
            summary.get("real_time_factor", 0) >= args.min_real_time_factor)
    required_topics = ("clock", "lidar", "pointcloud2", "imu", "imu_si", "odom", "tf", "tf_static", "rgb", "depth",
                       "panorama_rgb", "panorama_depth", "camera_info")
    checks["all_required_topics_received"] = all(counts[key] > 0 for key in required_topics)
    checks["timestamps_and_message_contracts_valid"] = not errors
    # CLI starts immediately after bridge creation; unlike full_run's discovery
    # handshake, this probe may miss a prefix. Require the remaining exact grid.
    checks["mid360_imu_200hz_grid"] = all(stamps and stamps[0] <= 100_000_000
        and stamps[-1] >= round(args.duration * 1e9) - 5_000_000
        and stamps[0] % 5_000_000 == 0 and np.all(np.diff(stamps) == 5_000_000)
        for stamps in (imu_stamps, imu_si_stamps))
    from simple_rbt_mujoco.mid360 import G0
    overlap = sorted(set(imu_values) & set(imu_si_values))
    checks["raw_and_si_imu_units_match"] = bool(overlap and all(np.array_equal(imu_values[t],
        (imu_si_values[t] / np.array([1., 1., 1., G0, G0, G0])).astype(np.float32).astype(float)) for t in overlap))
    common = set(overlap)
    covered_scans = [s for s in scan_stats if s["stamp_ns"] >= (overlap[0] if overlap else math.inf)]
    checks["imu_covers_complete_scan_intervals"] = bool(covered_scans and all(
        all(t in common for t in range(s["stamp_ns"], s["stamp_ns"] + 100_000_001, 5_000_000)) for s in covered_scans))
    checks["clock_crossed_seconds_and_completed"] = bool(clock_samples and clock_samples[-1] >= int(args.duration * 1e9) - 10_000_000
                                                          and clock_samples[0] < 1_000_000_000)
    checks["lidar_point_times_no_later_than_final_clock"] = bool(scan_stats and clock_samples and
        max(s["stamp_ns"] + s["last_offset_ns"] for s in scan_stats) <= clock_samples[-1])
    checks["published_four_seconds_of_commands"] = bool(command_count >= 20 and first_command_sim_ns is not None
                                                         and last_command_sim_ns - first_command_sim_ns >= 3_500_000_000)
    checks["correct_transform_tree"] = (transforms.get("base_link", {}).get("parent") == "odom" and
        all(static_transforms.get(child, {}).get("parent") == "base_link"
            for child in ("livox_frame", "imu_frame", "camera_optical_frame")))
    expected_transforms = model_sensor_transforms(args.robot, (summary or {}).get("controller", args.controller))
    checks["static_transforms_match_actual_model_sites"] = all(
        frame in static_transforms and
        np.allclose(static_transforms[frame]["translation"], expected["translation"], rtol=0, atol=1e-7) and
        abs(abs(np.dot(static_transforms[frame]["quaternion"], expected["quaternion"])) - 1) < 1e-7
        for frame, expected in expected_transforms.items())
    lidar_tf, imu_tf = static_transforms.get("livox_frame"), static_transforms.get("imu_frame")
    checks["mid360_internal_rigid_transform"] = bool(lidar_tf and imu_tf and mid360_extrinsics_match(
        lidar_tf["translation"], lidar_tf["quaternion"], imu_tf["translation"], imu_tf["quaternion"]))
    displacement = None
    terminal_speed = None
    steady_command_statistics = None
    if odometry:
        start = min(odometry, key=lambda row: abs(row["stamp_ns"] - 1_000_000_000))
        end = odometry[-1]
        displacement = float(np.linalg.norm(np.asarray(end["position"][:2]) - start["position"][:2]))
        terminal = [np.linalg.norm(row["linear_velocity"][:2]) for row in odometry if row["stamp_ns"] >= 7_000_000_000]
        terminal_speed = float(np.mean(terminal)) if terminal else None
        # Last third of the commanded interval [1,5) s avoids startup transients.
        steady_rows = [row for row in odometry if 3_666_666_667 <= row["stamp_ns"] < 5_000_000_000]
        if steady_rows:
            actual = np.asarray([[*row["linear_velocity"][:2], row["angular_velocity"][2]] for row in steady_rows])
            target = np.array([.3, 0., 0.])
            steady_command_statistics = {
                "simulation_window_s": [11 / 3, 5], "samples": len(steady_rows),
                "target_vx_vy_wz": target.tolist(), "mean_actual_vx_vy_wz": actual.mean(axis=0).tolist(),
                "mean_absolute_error_vx_vy_wz": np.abs(actual - target).mean(axis=0).tolist(),
                "scope": "Last third of command interval; ideal simulated body velocity measured from received odometry.",
            }
    checks["robot_moved_under_ros_command"] = displacement is not None and displacement > 0.5
    checks["deadman_stopped_motion"] = terminal_speed is not None and terminal_speed < 0.08
    expected_camera_frames = int(args.duration * args.camera_hz)
    camera_period_ns = round(1_000_000_000 / args.camera_hz)
    checks["both_camera_streams_at_requested_rate"] = all(
        expected_camera_frames - 1 <= counts[key] <= expected_camera_frames and
        camera_stamps[key][-1] >= int(args.duration * 1e9) - camera_period_ns and
        np.all(np.diff(camera_stamps[key]) >= camera_period_ns - 1)
        for key in ("rgb", "depth", "panorama_rgb", "panorama_depth")
        if key in camera_stamps) and len(camera_stamps) == 4
    # This fixture has an open sky: full-sphere panorama must distinguish no-hit
    # pixels from distant surfaces. Huge finite far-plane values are a regression.
    checks["panorama_open_sky_has_infinite_no_hit_depth"] = camera_stats.get("panorama_depth", {}).get("no_hit_count", 0) > 0
    passed = all(checks.values())
    report = {
        "passed": passed, "checks": checks, "errors": dict(errors), "exception": exception,
        "created_utc": datetime.now(timezone.utc).isoformat(), "domain_id": args.domain,
        "runtime": args.runtime,
        "localhost_only": True, "cli_argv": program, "simulator_exit_code": process.returncode if process else None,
        "configuration": {"runtime": args.runtime, "robot": args.robot, "command_topic": command_topic,
                          "terrain": args.terrain, "controller": args.controller,
                          "control_mode": args.control_mode, "camera_hz": args.camera_hz, "backend": args.backend,
                          "min_real_time_factor": args.min_real_time_factor},
        "acceptance_scope": "Sensor/ROS transport and command timeout on the selected terrain; this short run does not establish complete terrain traversal.",
        "wall_seconds": time.monotonic() - started, "counts": dict(counts),
        "first_clock_ns": clock_samples[0] if clock_samples else None,
        "last_clock_ns": clock_samples[-1] if clock_samples else None,
        "commands": {"topic": command_topic, "velocity_m_s": 0.3, "first_sim_ns": first_command_sim_ns,
                     "last_sim_ns": last_command_sim_ns, "messages": command_count,
                     "stop_method": "publisher silence after simulation t=5; bridge monotonic timeout"},
        "planar_displacement_m": displacement, "terminal_mean_planar_speed_m_s": terminal_speed,
        "steady_command_statistics": steady_command_statistics,
        "imu_max_abs_gyro_accel": imu_max_abs.tolist(), "camera_stats": camera_stats,
        "imu_raw_units": ["rad/s", "g"], "imu_si_units": ["rad/s", "m/s^2"],
        "imu_stamps_ns": imu_stamps, "imu_si_stamps_ns": imu_si_stamps,
        "imu_received_per_sim_second": counts["imu"] / args.duration,
        "camera_expected_frames_per_stream": expected_camera_frames,
        "camera_requested_rate_timebase": "simulation time",
        "camera_effective_wall_hz": {key: counts[key] / summary["wall_seconds"] for key in camera_stamps} if summary else None,
        "camera_stamps_ns": camera_stamps,
        "last_dynamic_transforms": transforms, "static_transforms": static_transforms,
        "expected_model_sensor_transforms": expected_transforms,
        "scan_stats": scan_stats, "odometry": odometry, "simulator_summary": summary,
        "fresh_simulator_process_resources": {"loop": (summary or {}).get("resources_loop"),
                                               "run": (summary or {}).get("resources_run")},
        "simulator_log": str(log_path), "simulator_artifacts": str(artifacts),
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": passed, "checks": checks, "errors": dict(errors),
                      "report": str(args.output), "log": str(log_path)}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
