#!/usr/bin/env python3
"""Validate one complete terrain course, ROS transport and resources in one run.

The parent sends only /<robot>/cmd_vel. A fresh worker performs actual MuJoCo physics,
LiDAR raycasts, camera rendering and ROS publication. No sensor is fabricated.
Source Humble and Livox messages, then use the project Python 3.10 environment.
The independent checker supports either the Python oracle or native C++ worker.
"""
from __future__ import annotations

import argparse
import hashlib
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback
import uuid

from project import ROOT
from full_run_evidence import (CPP_WORKER, digest_json, imu_digest_update,
                               scan_digest, source_fingerprints, write_json)
from full_run_course import CourseMonitor, FrictionMonitor, tracking_result
from full_run_recorder import RosRecorder, stamp_ns, xyz, xyzw
from full_run_worker import worker
from full_run_gates import full_run_checks as _full_run_checks


def full_run_checks(config, evidence, received, commands, child_returncode, initial_sources, worker_pid=None):
    """Snapshot current sources once, then evaluate the unchanged acceptance join."""
    return _full_run_checks(config, evidence, received, commands, child_returncode,
                            initial_sources, worker_pid, current_sources=source_fingerprints())


def worker_program(runtime, config_path, native_binary=None):
    """Choose the worker without importing either simulator or altering its gates."""
    if runtime == "python":
        return [sys.executable, str(Path(__file__).resolve()), "--worker-config", str(config_path)]
    if runtime == "cpp":
        return [str(native_binary or CPP_WORKER), "--worker-config", str(config_path)]
    raise ValueError("runtime must be python or cpp")


def run_parent(args):
    from simple_rbt_mujoco.resources import ResourceMeter
    from simple_rbt_mujoco.ros2 import robot_command_topic
    checker_meter = ResourceMeter()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from profile_resources import process_tree, resident_kib, gpu_memory_mib

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = {"run_id": uuid.uuid4().hex, "output": str(output), "robot": args.robot,
              "runtime": args.runtime,
              "command_topic": robot_command_topic(args.robot),
              "terrain": args.terrain, "terrain_level": args.terrain_level,
              "controller": args.controller, "control_mode": args.control_mode,
              "seed": args.seed, "speed_m_s": args.speed, "friction": args.friction,
              "backend": args.backend, "camera_hz": args.camera_hz, "settle_s": args.settle,
              "commanded_distance_budget_m": 8., "min_real_time_factor": args.min_real_time_factor,
              "domain": args.domain, "wall_timeout_s": args.wall_timeout,
              "sensor_profile": {"lidar_rays_per_frame": 20000, "lidar_hz": 10,
                  "perspective_rgb_depth_size": [320, 240], "panorama_rgb_depth_size": [720, 360],
                  "camera_hz": args.camera_hz, "imu_hz": 200, "imu_initial_sample_ns": 0,
                  "imu_topics": {"/livox/imu": "g, rad/s, driver convention", "/livox/imu_si": "m/s^2, rad/s, physical IMU frame"},
                  "lidar_acquisition": "rolling_static_environment"}}
    config_path = output / "config.json"
    write_json(config_path, config)
    sources = source_fingerprints()
    # Native workers can rehash this exact path set when writing worker.json;
    # acceptance still compares worker hashes with fresh parent hashes at end.
    write_json(output / "sources.json", sources)
    received = RosRecorder()
    context = Context()
    context.init(args=[], domain_id=args.domain)
    node = rclpy.create_node("full_run_checker_" + config["run_id"][:8], context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    qos_clock = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    qos_static = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []
    for kind, topic, callback, qos in (
        (Clock, "/clock", received.clock, qos_clock), (CustomMsg, "/livox/lidar", received.scan, 100),
        (PointCloud2, "/livox/points", received.pointcloud, 100), (Imu, "/livox/imu", received.imu, 1000),
        (Imu, "/livox/imu_si", lambda msg: received.imu(msg, True), 1000),
        (Odometry, "/odom", received.odom, 100), (TFMessage, "/tf", received.tf, 100),
        (TFMessage, "/tf_static", lambda msg: received.tf(msg, True), qos_static),
        (Image, "/camera/image_raw", lambda msg: received.image(msg, "rgb"), 10),
        (Image, "/camera/depth/image_raw", lambda msg: received.image(msg, "depth"), 10),
        (Image, "/camera/panorama/image_raw", lambda msg: received.image(msg, "panorama_rgb"), 10),
        (Image, "/camera/panorama/depth/image_raw", lambda msg: received.image(msg, "panorama_depth"), 10),
        (CameraInfo, "/camera/camera_info", received.camera_info, 10),
        (String, "/full_run/status", received.on_status, qos_static),
    ):
        subscriptions.append(node.create_subscription(kind, topic, callback, qos))
    command_pub = node.create_publisher(Twist, config["command_topic"], 10)
    command = Twist()
    command.linear.x = args.speed
    # A frozen baseline executable can be selected for ablation experiments;
    # the product launcher and acceptance gates remain unchanged.
    native_binary = Path(os.environ.get("SIMPLE_RBT_NATIVE_BINARY", str(CPP_WORKER))).resolve()
    program = worker_program(args.runtime, config_path, native_binary)
    commands, resource_samples = [], []
    stop_sampling = threading.Event()
    process = None
    sampler = None
    exception = None
    started = time.monotonic()

    def sample_resources():
        while not stop_sampling.is_set():
            try:
                pids = process_tree(process.pid)
                resource_samples.append({"monotonic_s": time.monotonic(), "pids": pids,
                    "resident_mib": resident_kib(pids) / 1024., "compute_vram_mib": gpu_memory_mib(pids)})
            except Exception as error:
                resource_samples.append({"monotonic_s": time.monotonic(), "sampling_error": repr(error)})
            stop_sampling.wait(1.)

    try:
        with (output / "worker.log").open("x") as log:
            process = subprocess.Popen(program, cwd=ROOT, env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT)
            sampler = threading.Thread(target=sample_resources, daemon=True)
            sampler.start()
            next_command = started
            exited_at = None
            observed_finish = False
            while time.monotonic() - started < args.wall_timeout:
                executor.spin_once(timeout_sec=.002)
                now = time.monotonic()
                sim_stamp = received.last_stamp.get("clock", 0)
                finish_x = received.status.get("finish_x_m", math.inf) if received.status else math.inf
                observed_finish = bool(observed_finish or (received.odometry and received.odometry[-1]["position"][0] >= finish_x))
                if now >= next_command and command_pub.get_subscription_count():
                    if sim_stamp >= round(args.settle * 1e9) and not observed_finish and process.poll() is None:
                        command_pub.publish(command)
                        commands.append({"sim_stamp_ns": sim_stamp, "monotonic_s": now, "command": [args.speed, 0., 0.]})
                    next_command = now + .05
                if process.poll() is not None:
                    exited_at = exited_at or now
                    if now - exited_at >= .6:
                        break
            else:
                received.errors["worker_wall_timeout"] += 1
    except Exception:
        exception = traceback.format_exc()
        received.errors["checker_exception"] += 1
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        stop_sampling.set()
        if sampler:
            sampler.join(timeout=6)
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()
    worker_path = output / "worker.json"
    evidence = json.loads(worker_path.read_text()) if worker_path.exists() else {}
    checks, acceptance = full_run_checks(config, evidence, received, commands,
        process.returncode if process else None, sources, process.pid if process else None)
    resources = evidence.get("resources_loop", {})
    loop_samples = [row for row in resource_samples if resources.get("interval_start_monotonic_s", math.inf)
                    <= row["monotonic_s"] <= resources.get("interval_end_monotonic_s", -math.inf)]
    vram = [row["compute_vram_mib"] for row in loop_samples if row.get("compute_vram_mib") is not None]
    report = {**acceptance, "checks": checks, "errors": dict(received.errors), "exception": exception,
              "created_utc": datetime.now(timezone.utc).isoformat(), "config": config,
              "runtime": config["runtime"],
              "native_binary_sha256": (hashlib.sha256(native_binary.read_bytes()).hexdigest()
                                       if args.runtime == "cpp" else None),
              "worker_pid": evidence.get("pid"), "worker_program": program, "sources": sources,
              "worker_result": "worker.json", "worker_log": "worker.log", "compiled_model": "compiled-model.xml",
              "formal_course": evidence.get("formal_course"), "effective_friction": evidence.get("friction"),
              "deployment": evidence.get("deployment"),
              "control_mode": evidence.get("control_mode"),
              "control_feedback_scope": evidence.get("control_feedback_scope"),
              "control_feedback_source": evidence.get("control_feedback_source"),
              "received_counts": dict(received.counts), "emitted_counts": evidence.get("emitted"),
              "clock_first_last_ns": [received.first_clock, received.last_stamp.get("clock")],
              "received_imu_sha256": received.imu_hash.hexdigest(), "received_scans": received.scans,
              "received_imu_si_sha256": received.imu_si_hash.hexdigest(),
              "received_imu_stamps_ns": received.imu_stamps, "received_imu_si_stamps_ns": received.imu_si_stamps,
              "mid360": evidence.get("mid360"), "imu_sampler": evidence.get("imu_sampler"),
              "lidar_model": evidence.get("lidar_model"), "ros_transport": evidence.get("ros_transport"),
              "received_image_hashes": received.images, "image_stats": received.image_stats,
              "uniform_rgb_frame_counts": dict(received.uniform_rgb_frames),
              "maximum_rgb_std": received.maximum_rgb_std,
              "received_odometry": received.odometry, "static_tf": received.static_tf,
              "commands_published": commands, "commands_stop_method": "publisher silence after received odometry crosses finish; no explicit zero",
              "real_time_factor": evidence.get("real_time_factor"),
              "camera_effective_wall_hz": {key: received.counts[key] / evidence["wall_seconds"] for key in received.images} if evidence.get("wall_seconds") else {},
              "worker_resources_loop": resources, "worker_resources_run": evidence.get("resources_run"),
              "checker_resources": checker_meter.report(), "pid_resource_samples": resource_samples,
              "worker_loop_sampled_compute_vram_peak_mib": max(vram) if vram else None,
              "gpu_measurement_scope": "1 Hz nvidia-smi compute-process samples for worker PID tree during exact loop interval; not a guaranteed peak or complete graphics/desktop VRAM",
              "limitations": ["Outer ideal simulator velocity/heading feedback is enabled only in tracked mode; controller feedback is described in deployment. Neither is a verified real-robot estimator/firmware",
                              "LiDAR uses per-ray acquisition poses against static environment geometry; robot self-occlusion excluded",
                              "IMU is ideal without noise/bias; 5ms samples interpolate adjacent physical samples when grids do not align",
                              "Raw IMU mirrors Livox driver g units and housing-frame header; SI companion identifies the actual internal IMU origin"]}
    write_json(output / "report.json", report)
    print(json.dumps({**acceptance, "checks": checks, "report": str(output / "report.json")}, indent=2))
    return 0 if acceptance["overall_acceptance"] else 1


def main():
    from simple_rbt_mujoco.robots import ROBOT_NAMES, CONTROLLER_NAMES, COMMAND_LIMITS, WHEELED_ROBOTS, validate_terrain
    from simple_rbt_mujoco.terrains import TERRAIN_NAMES
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker-config", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--runtime", choices=("python", "cpp"), default="cpp",
                        help="Worker implementation; the Python checker and all acceptance gates are unchanged")
    parser.add_argument("--robot", choices=ROBOT_NAMES, default="go2w")
    parser.add_argument("--controller", choices=CONTROLLER_NAMES, default="auto")
    parser.add_argument("--terrain", choices=TERRAIN_NAMES, default="ramp")
    parser.add_argument("--terrain-level", choices=("standard", "moderate", "hard", "extreme"),
                        help="Default: standard for wheeled chassis, extreme for legged models")
    parser.add_argument("--control-mode", choices=("native", "tracked"), default="tracked")
    parser.add_argument("--speed", type=float, default=.6)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--friction", type=float, help="Explicit contact mu; omit to preserve source mixing")
    parser.add_argument("--backend", choices=("cpu", "warp"), default="warp")
    parser.add_argument("--camera-hz", type=int, choices=(1, 10), default=10,
                        help="Supported rates divide both 200 Hz Go2 and 500 Hz Go2W physics clocks")
    parser.add_argument("--settle", type=float, default=2.)
    parser.add_argument("--min-real-time-factor", type=float, default=.95)
    parser.add_argument("--domain", type=int, default=85)
    parser.add_argument("--wall-timeout", type=float, default=180.)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.terrain_level is None:
        args.terrain_level = "standard" if args.robot in WHEELED_ROBOTS else "extreme"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ.setdefault("MUJOCO_GL", "egl")
    for variable in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[variable] = "1"
    if args.worker_config:
        try:
            return worker(args.worker_config)
        except Exception:
            config = json.loads(args.worker_config.read_text())
            write_json(Path(config["output"]) / "worker.json", {"run_id": config["run_id"],
                "pid": os.getpid(), "config_sha256": digest_json(config), "error": traceback.format_exc()})
            raise
    try:
        validate_terrain(args.robot, args.terrain)
    except ValueError as error:
        parser.error(str(error))
    max_speed = COMMAND_LIMITS[args.robot][0]
    if (not math.isfinite(args.speed) or not 0 < args.speed <= max_speed or args.seed < 0
            or not math.isfinite(args.settle) or args.settle < 0 or not 0 <= args.domain <= 232
            or not math.isfinite(args.min_real_time_factor) or args.min_real_time_factor < .95
            or not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0
            or (args.friction is not None and (not math.isfinite(args.friction) or not 0 < args.friction <= 2))):
        parser.error("Require valid speed/seed/settle/domain/friction, positive timeout, and RTF threshold >=0.95")
    if args.output is None:
        parser.error("--output NEW_DIR is required")
    native_binary = Path(os.environ.get("SIMPLE_RBT_NATIVE_BINARY", str(CPP_WORKER))).resolve()
    if args.runtime == "cpp" and (not native_binary.is_file() or not os.access(native_binary, os.X_OK)):
        parser.error(f"Native worker is not built or executable: {native_binary}; run ./setup.sh first")
    return run_parent(args)


if __name__ == "__main__":
    raise SystemExit(main())
