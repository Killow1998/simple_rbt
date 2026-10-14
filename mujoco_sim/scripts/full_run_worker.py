"""Python reference worker: actual physics, sensors and DDS publication."""
from collections import Counter
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import time
import traceback
import numpy as np
from full_run_course import CourseMonitor, FrictionMonitor
from full_run_evidence import (digest_json, imu_digest_update, scan_digest,
                               source_fingerprints, write_json)


def worker(config_path):
    config = json.loads(Path(config_path).read_text())
    os.environ["ROS_DOMAIN_ID"] = str(config["domain"])
    output = Path(config["output"])
    from simple_rbt_mujoco.resources import ResourceMeter
    run_meter = ResourceMeter()
    import mujoco
    from simple_rbt_mujoco.core import Simulation, SCAN_CSV, BODY_FROM_OPTICAL
    from simple_rbt_mujoco.lidar import CsvScanPattern
    from simple_rbt_mujoco.rolling_lidar import RollingCsvLidar, PoseHistory
    from simple_rbt_mujoco.mid360 import Mid360ImuSampler
    from simple_rbt_mujoco.cameras import PerspectiveCamera, PanoramaCamera
    from simple_rbt_mujoco.ros2 import RosBridge, pointcloud_data, imu_wire_values, robot_command_topic
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy

    command_topic = robot_command_topic(config["robot"])
    if config.get("command_topic") != command_topic:
        raise ValueError("Worker command_topic must match its robot namespace")

    sim = Simulation(config["robot"], terrain=config["terrain"], terrain_level=config["terrain_level"],
                     seed=config["seed"], controller=config["controller"],
                     velocity_feedback=config["control_mode"] == "tracked",
                     terrain_friction=config["friction"])
    profile = sim.terrain
    mujoco.mj_saveLastXML(str(output / "compiled-model.xml"), sim.model)
    from simple_rbt_mujoco.robots import WHEEL_MODELS, WHEELED_ROBOTS
    monitor_config = config
    if sim.robot in WHEEL_MODELS:
        from simple_rbt_mujoco.wheel_geometry import collision_envelope
        monitor_config = {**config, "robot_half_width_m": collision_envelope(sim.model, sim.data,
                            reference_frame="world")["outer_half_width_m"]}
    monitor = CourseMonitor(profile, monitor_config, sim.pose()[0], sim.model.opt.timestep)
    friction = FrictionMonitor(config["friction"])
    pattern = CsvScanPattern.from_csv(SCAN_CSV)
    lidar = RollingCsvLidar(sim.model, pattern, backend=config["backend"])
    initial_lidar_pos, initial_lidar_q = sim.pose("lidar")
    warmup_history = PoseHistory()
    warmup_history.add(0, initial_lidar_pos, initial_lidar_q[[3, 0, 1, 2]])
    warmup_history.add(100_000_000, initial_lidar_pos, initial_lidar_q[[3, 0, 1, 2]])
    lidar.scan(sim.data, stamp_ns=0, n_rays=20000, history=warmup_history)
    pattern.reset()
    history = PoseHistory()
    history.add(0, initial_lidar_pos, initial_lidar_q[[3, 0, 1, 2]])
    imu_sampler = Mid360ImuSampler()
    emitted = Counter()
    imu_hash = hashlib.sha256()
    imu_si_hash = hashlib.sha256()
    imu_samples = []
    scans, cameras, odometry, command_transitions = [], {}, [], []
    lidar_timings, camera_timings = [], []
    base_id = int(sim.model.site_bodyid[sim.model.site("base_origin").id])
    rolling_contacts = sim.robot in WHEELED_ROBOTS
    if rolling_contacts:
        from simple_rbt_mujoco.wheel_geometry import wheel_contact_geom_ids, body_environment_contacts
        wheel_geom_ids = wheel_contact_geom_ids(sim.model)
    error = None
    loop_usage = None
    with ExitStack() as stack:
        optical = np.empty(4)
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, sim.model.site_quat[sim.model.site("camera_mount").id])
        mujoco.mju_mat2Quat(optical, (rotation.reshape(3, 3) @ BODY_FROM_OPTICAL).ravel())
        bridge = stack.enter_context(RosBridge(
            robot_name=config["robot"],
            lidar_extrinsics=sim.extrinsics("lidar"), imu_extrinsics=sim.extrinsics("mid360_imu"),
            camera_extrinsics=(sim.extrinsics("camera_mount")[0], optical[[1, 2, 3, 0]]),
            domain_id=config["domain"], node_name="full_run_worker_" + config["run_id"][:8]))
        status_pub = bridge.node.create_publisher(String, "/full_run/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        camera_objects = (PerspectiveCamera(sim.model), PanoramaCamera(sim.model))
        for camera in camera_objects:
            stack.callback(camera.close)
            camera.render(sim.data, *sim.camera_pose())
        required = ["/clock", "/livox/lidar", "/livox/points", "/livox/imu", "/livox/imu_si", "/odom", "/tf", "/tf_static",
                    "/camera/image_raw", "/camera/depth/image_raw", "/camera/panorama/image_raw",
                    "/camera/panorama/depth/image_raw", "/camera/camera_info", "/full_run/status"]
        deadline = time.monotonic() + 15
        while not (all(bridge.node.count_subscribers(topic) >= 1 for topic in required)
                   and bridge.node.count_publishers(command_topic) >= 1):
            if time.monotonic() >= deadline:
                raise RuntimeError("DDS discovery did not complete for every required topic")
            bridge.spin_once(.01)
        status_pub.publish(String(data=json.dumps({"run_id": config["run_id"], "pid": os.getpid(),
            "command_topic": bridge.command_topic,
            "config_sha256": digest_json(config), "finish_x_m": profile.finish_x,
            "physics_timestep_s": sim.model.opt.timestep, "ready": True})))
        lidar_every = round(.1 / sim.model.opt.timestep)
        camera_period_ns = round(1e9 / config["camera_hz"])
        next_camera_ns = camera_period_ns
        previous_command = None
        def publish_imu_samples(samples):
            for sample in samples:
                gyro, force = sample.angular_velocity_rad_s, sample.specific_force_m_s2
                bridge.publish_imu(sample.stamp_ns, gyro, force)
                raw_gyro, raw_force = imu_wire_values(gyro, force)
                imu_digest_update(imu_hash, sample.stamp_ns, raw_gyro, raw_force, [0., 0., 0., 1.])
                imu_digest_update(imu_si_hash, sample.stamp_ns, gyro, force, [0.] * 4)
                emitted["imu"] += 1
                emitted["imu_si"] += 1
                imu_samples.append({"stamp_ns": sample.stamp_ns, "sampling_method": sample.sampling_method,
                                    "source_stamps_ns": list(sample.source_stamps_ns)})
        start_wall = time.monotonic()
        loop_meter = ResourceMeter()
        try:
            publish_imu_samples(imu_sampler.observe(0, *sim.imu()[:2]))
            while True:
                bridge.spin_once()
                command = bridge.command if sim.data.time >= config["settle_s"] else (0., 0., 0.)
                if command != previous_command:
                    command_transitions.append({"stamp_ns": sim.stamp_ns, "monotonic_s": time.monotonic(), "command": list(command)})
                    previous_command = command
                sim.set_command(*command)
                sim.step()
                stamp = sim.stamp_ns
                lidar_position, lidar_quaternion = sim.pose("lidar")
                history.add(stamp, lidar_position, lidar_quaternion[[3, 0, 1, 2]])
                pos, quat = sim.pose()
                linear, angular = sim.velocity()
                rotation = sim.data.site_xmat[sim.model.site("base_origin").id].reshape(3, 3)
                torso_names = []
                for contact in sim.data.contact:
                    a, b = int(contact.geom1), int(contact.geom2)
                    ba, bb = int(sim.model.geom_bodyid[a]), int(sim.model.geom_bodyid[b])
                    if {ba, bb} == {0, base_id}:
                        torso_names.append(sim.model.geom(a if ba == base_id else b).name or "unnamed")
                    if (ba == 0) != (bb == 0):
                        environment, robot_geom = (a, b) if ba == 0 else (b, a)
                        pair = f"{sim.model.geom(environment).name}|{sim.model.geom(robot_geom).name or robot_geom}"
                        friction.observe(contact.friction, pair)
                if rolling_contacts:
                    torso_names = body_environment_contacts(sim.model, sim.data, wheel_geom_ids)
                if sim.robot in WHEEL_MODELS:
                    current_half_width = collision_envelope(sim.model, sim.data,
                            reference_frame="world")["outer_half_width_m"]
                else:
                    current_half_width = None
                monitor.update(stamp, pos, quat, rotation, linear, angular, sorted(set(torso_names)),
                               robot_half_width_m=current_half_width)
                bridge.publish_clock(stamp)
                emitted["clock"] += 1
                publish_imu_samples(imu_sampler.observe(stamp, *sim.imu()[:2]))
                if sim.steps % lidar_every == 0:
                    before = time.perf_counter()
                    scan = lidar.scan(sim.data, stamp_ns=stamp - 100_000_000, n_rays=20000, history=history)
                    lidar_timings.append(time.perf_counter() - before)
                    bridge.publish_scan(scan)
                    emitted["lidar"] += 1
                    emitted["pointcloud2"] += 1
                    scans.append({"stamp_ns": scan.stamp_ns, "point_count": scan.point_num,
                                  "scan_sha256": scan_digest(scan.xyz, scan.offsets_ns, scan.lines),
                                  "pointcloud2_sha256": hashlib.sha256(pointcloud_data(scan)).hexdigest(),
                                  "metadata": scan.metadata})
                    bridge.publish_odometry(stamp, pos, quat, linear, angular)
                    emitted["odom"] += 1
                    emitted["tf"] += 1
                    odometry.append({"stamp_ns": stamp, "position": pos.tolist(), "quaternion": quat.tolist(),
                                     "linear": linear.tolist(), "angular": angular.tolist(),
                                     "command": sim.command.tolist(), "policy_command": sim.policy_command.tolist()})
                if stamp >= next_camera_ns:
                    before = time.perf_counter()
                    for kind, camera in zip(("perspective", "panorama"), camera_objects):
                        frame = camera.render(sim.data, *sim.camera_pose())
                        bridge.publish_camera(stamp, frame.rgb, frame.depth, kind=kind)
                        for suffix, values, dtype in (("rgb", frame.rgb, "u1"), ("depth", frame.depth, "<f4")):
                            key = suffix if kind == "perspective" else "panorama_" + suffix
                            cameras.setdefault(key, []).append({"stamp_ns": stamp,
                                "sha256": hashlib.sha256(np.asarray(values, dtype=dtype).tobytes(order="C")).hexdigest()})
                            emitted[key] += 1
                        if kind == "perspective":
                            emitted["camera_info"] += 1
                    camera_timings.append(time.perf_counter() - before)
                    next_camera_ns += camera_period_ns
                remaining = stamp * 1e-9 - (time.monotonic() - start_wall)
                if remaining > 0:
                    time.sleep(remaining)
                if monitor.failure or (monitor.success and stamp >= monitor.finish_stamp_ns + 2_000_000_000):
                    break
        except Exception:
            error = traceback.format_exc()
            monitor.failure = monitor.failure or "simulation_or_sensor_error"
            monitor.success = False
        wall = time.monotonic() - start_wall
        loop_usage = loop_meter.report()
        final_command = list(bridge.command)
        # Drain outstanding reliable DDS data; this time is outside loop RTF.
        deadline = time.monotonic() + .3
        while time.monotonic() < deadline:
            bridge.spin_once(.005)
    report = {"run_id": config["run_id"], "pid": os.getpid(), "config_sha256": digest_json(config),
              "command_topic": bridge.command_topic,
              "config": config, "sources": source_fingerprints(), "terrain": profile.metadata,
              "controller": sim.controller_name, "physics_timestep_s": sim.model.opt.timestep,
              "deployment": sim.deployment_metadata,
              "control_mode": config["control_mode"],
              "control_feedback_scope": "Outer body velocity/heading tracker; controller feedback is recorded in deployment",
              "control_feedback_source": "ideal simulated body velocity and heading" if sim.command_tracker else None,
              "formal_course": monitor.report(), "friction": friction.report(),
              "simulation_end_ns": sim.stamp_ns, "wall_seconds": wall,
              "real_time_factor": sim.stamp_ns * 1e-9 / wall,
              "timing_scope": "paced physics + every formal gate + all sensors + DDS publication; excludes setup/JIT/renderer warmup and final DDS drain",
              "resources_loop": loop_usage, "resources_run": run_meter.report(),
              "resource_scope": "fresh simulator PID only; peak RSS includes imports/setup; checker overhead separately reported",
              "emitted": dict(emitted), "imu_sha256": imu_hash.hexdigest(),
              "imu_si_sha256": imu_si_hash.hexdigest(), "imu_samples": imu_samples, "scans": scans,
              "mid360": sim.mid360_metadata, "imu_sampler": imu_sampler.metadata,
              "lidar_model": lidar.metadata, "ros_transport": bridge.metadata,
              "cameras": cameras, "odometry": odometry, "command_transitions": command_transitions,
              "final_command": final_command, "error": error,
              "lidar_median_ms": float(np.median(lidar_timings) * 1000) if lidar_timings else None,
              "camera_pair_median_ms": float(np.median(camera_timings) * 1000) if camera_timings else None}
    write_json(output / "worker.json", report)
    return 0 if error is None else 2
