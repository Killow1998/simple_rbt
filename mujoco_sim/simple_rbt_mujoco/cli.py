"""Run the standalone simulator, optionally connecting its outputs to ROS 2."""
import argparse
import csv
from contextlib import ExitStack
from datetime import datetime
import json
from pathlib import Path
import time

import numpy as np

from .core import ROOT, SCAN_CSV, Simulation
from .resources import ResourceMeter, directory_size
from .robots import ROBOT_NAMES, CONTROLLER_NAMES, command_is_supported, validate_terrain
from .terrains import TERRAIN_NAMES


def main(argv=None):
    run_resources = ResourceMeter()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=ROBOT_NAMES, default="go2")
    parser.add_argument("--terrain", choices=TERRAIN_NAMES, default="flat")
    parser.add_argument("--terrain-level", choices=["standard", "moderate", "hard", "extreme"], default="standard",
                        help="Named course dimensions; higher levels increase height and route length")
    parser.add_argument("--difficulty", type=float, default=1., help="Scale standard-level terrain slope/height; other levels require 1")
    parser.add_argument("--seed", type=int, default=0, help="Deterministic rough-terrain seed")
    parser.add_argument("--friction", type=float, help="Explicit course/floor sliding coefficient (0,2]; takes contact priority over feet/wheels")
    parser.add_argument("--controller", choices=CONTROLLER_NAMES, default="auto")
    parser.add_argument("--go2-torque-mode", choices=["configured", "upstream"], default="configured",
                        help="Go2 policy PD torque: config clipping or upstream MuJoCo path with MJCF motor limits")
    parser.add_argument("--go2-startup", choices=["crouched", "upstream_getup"], default="crouched",
                        help="Go2 policy startup; GetUp completes before the stationary settle interval")
    parser.add_argument("--control-mode", choices=["native", "tracked"], default="tracked",
                        help="tracked adds ideal simulator velocity/heading feedback to the native policy")
    parser.add_argument("--backend", choices=["cpu", "warp"], default="cpu")
    parser.add_argument("--lidar-motion", choices=["rolling", "snapshot"], default="rolling",
                        help="rolling uses each point's capture pose; snapshot is a frame-end debug capture with zero offsets")
    parser.add_argument("--duration", type=float, default=10., help="Total simulation seconds, including Go2 GetUp and settle")
    parser.add_argument("--cmd", type=float, nargs=3, default=[0., 0., 0.], metavar=("VX", "VY", "WZ"))
    parser.add_argument("--settle", type=float, default=1., help="Stationary seconds after Go2 GetUp, if selected")
    parser.add_argument("--rays", type=int, default=20000,
                        help="Keep first N rays of each 20,000-sample frame (debug); <=20000")
    parser.add_argument("--ros2", action="store_true", help="ROS /<robot>/cmd_vel overrides --cmd; localhost/domain83 by default")
    parser.add_argument("--viewer", action="store_true", help="Open a MuJoCo desktop viewer")
    parser.add_argument("--realtime", action="store_true", help="Pace physics to wall clock")
    parser.add_argument("--cameras", action="store_true", help="Alias for --sensors full; frame rate comes from --camera-hz")
    parser.add_argument("--sensors", choices=["physics", "lidar", "full"], default="lidar",
                        help="physics avoids loading LiDAR/graphics; full adds both cameras")
    parser.add_argument("--camera-hz", type=float, default=1., help="Both cameras' render rate in simulation time")
    parser.add_argument("--summary-only", action="store_true", help="Measure normally; save only summary, without sensor/trajectory files")
    parser.add_argument("--output", type=Path, help="New artifact directory (must not already exist)")
    args = parser.parse_args(argv)
    if args.cameras:
        args.sensors = "full"
    args.cameras = args.sensors == "full"
    if not np.isfinite(args.difficulty) or not 0 < args.difficulty <= 3:
        parser.error("difficulty must be in (0, 3]")
    if args.terrain_level != "standard" and args.difficulty != 1:
        parser.error("named terrain levels require --difficulty 1; use --terrain-level to select dimensions")
    if args.friction is not None and (not np.isfinite(args.friction) or not 0 < args.friction <= 2):
        parser.error("friction must be finite and in (0, 2]")
    if (args.robot != "go2" or args.controller == "legacy") and (
            args.go2_torque_mode != "configured" or args.go2_startup != "crouched"):
        parser.error("deployment alternatives require the Go2 HIMLoco or RobotLab controller")
    if not np.isfinite(args.camera_hz) or not 0 < args.camera_hz <= 30:
        parser.error("camera-hz must be in (0, 30]")
    if not np.isfinite(args.duration) or args.duration <= 0 or not np.isfinite(args.settle) or args.settle < 0:
        parser.error("duration must be positive and settle nonnegative")
    if not 1 <= args.rays <= 20000:
        parser.error("rays must be within 1..20000 for the 100 ms frame")
    if not np.isfinite(args.cmd).all():
        parser.error("cmd must be finite")
    if not command_is_supported(args.robot, *args.cmd):
        parser.error(f"{args.robot} cannot execute this planar command; check lateral or in-place-turn capability")
    try:
        validate_terrain(args.robot, args.terrain)
    except ValueError as error:
        parser.error(str(error))
    output = args.output or ROOT / "output/mujoco" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    output.mkdir(parents=True, exist_ok=False)

    from .lidar import CsvLidar, CsvScanPattern
    from .mid360 import Mid360ImuSampler
    from .rolling_lidar import PoseHistory, RollingCsvLidar
    sim = Simulation(args.robot, terrain=args.terrain, difficulty=args.difficulty,
                     seed=args.seed, controller=args.controller, velocity_feedback=args.control_mode == "tracked",
                     terrain_level=args.terrain_level, terrain_friction=args.friction,
                     go2_torque_mode=args.go2_torque_mode, go2_startup=args.go2_startup)
    lidar = None
    history = None
    if args.sensors != "physics":
        pattern = CsvScanPattern.from_csv(SCAN_CSV)
        if args.lidar_motion == "rolling":
            lidar = RollingCsvLidar(sim.model, pattern, backend=args.backend)
            history = PoseHistory()
            position, quaternion = sim.pose("lidar")
            history.add(0, position, quaternion[[3, 0, 1, 2]])
            # A stationary scratch history compiles ray kernels without advancing
            # physics or publishing synthetic warmup samples.
            warmup_history = PoseHistory()
            warmup_history.add(0, position, quaternion[[3, 0, 1, 2]])
            warmup_history.add(100_000_000, position, quaternion[[3, 0, 1, 2]])
            lidar.scan(sim.data, stamp_ns=0, n_rays=args.rays, history=warmup_history)
        else:
            lidar = CsvLidar(sim.model, pattern, backend=args.backend)
            lidar.scan(sim.data, stamp_ns=0, n_rays=args.rays)
        # GPU warmup is excluded from loop timing but included in process resource totals.
        pattern.reset()
    imu_sampler = Mid360ImuSampler()
    imu_samples = 0
    imu_interpolated_samples = 0
    last_imu_stamp_ns = None
    start_position = sim.pose()[0]
    timings, counts, camera_timings = [], [], []
    last_scan = None
    last_scan_pose = None
    last_scan_end_ns = None
    camera_metadata = None
    min_height = float(start_position[2])
    max_tilt_deg = 0.
    with ExitStack() as stack:
        trajectory = None
        imu_output = None
        if not args.summary_only:
            trajectory = csv.writer(stack.enter_context((output / "trajectory.csv").open("w")))
            trajectory.writerow(["time_s", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz", "wx", "wy", "wz"])
            imu_output = csv.writer(stack.enter_context((output / "mid360-imu-si.csv").open("w")))
            imu_output.writerow(["stamp_ns", "wx_rad_s", "wy_rad_s", "wz_rad_s",
                                 "specific_force_x_m_s2", "specific_force_y_m_s2", "specific_force_z_m_s2"])
        bridge = None
        if args.ros2:
            from .ros2 import RosBridge
            import mujoco
            from .core import BODY_FROM_OPTICAL
            optical_quat = np.empty(4)
            mount_rotation = np.empty(9)
            mujoco.mju_quat2Mat(mount_rotation, sim.model.site_quat[sim.model.site("camera_mount").id])
            mujoco.mju_mat2Quat(optical_quat, (mount_rotation.reshape(3, 3) @ BODY_FROM_OPTICAL).ravel())
            bridge = RosBridge(robot_name=args.robot, lidar_extrinsics=sim.extrinsics("lidar"),
                               imu_extrinsics=sim.extrinsics("mid360_imu"),
                               camera_extrinsics=(sim.extrinsics("camera_mount")[0], optical_quat[[1, 2, 3, 0]]))
            stack.callback(bridge.close)
        viewer = None
        if args.viewer:
            import mujoco.viewer
            viewer = stack.enter_context(mujoco.viewer.launch_passive(sim.model, sim.data))
        cameras = None
        if args.cameras:
            from .cameras import PerspectiveCamera, PanoramaCamera
            cameras = (PerspectiveCamera(sim.model), PanoramaCamera(sim.model))
            for cam in cameras:
                stack.callback(cam.close)
        steps = int(np.ceil(args.duration / sim.model.opt.timestep))
        lidar_every = round(.1 / sim.model.opt.timestep)
        camera_period_ns = round(1e9 / args.camera_hz)
        next_camera_ns = camera_period_ns
        start_wall = time.perf_counter()
        loop_resources = ResourceMeter()
        policy_clock_origin = None
        first_command_interval_s = None
        def emit_imu_samples(samples):
            nonlocal imu_samples, imu_interpolated_samples, last_imu_stamp_ns
            for sample in samples:
                imu_samples += 1
                imu_interpolated_samples += sample.sampling_method == "linear_interpolation"
                last_imu_stamp_ns = sample.stamp_ns
                if imu_output:
                    imu_output.writerow([sample.stamp_ns, *sample.angular_velocity_rad_s,
                                         *sample.specific_force_m_s2])
                if bridge:
                    bridge.publish_imu(sample.stamp_ns, sample.angular_velocity_rad_s,
                                       sample.specific_force_m_s2)
        if bridge:
            bridge.publish_clock(0)
        emit_imu_samples(imu_sampler.observe(0, *sim.imu()[:2]))
        for _ in range(steps):
            if viewer and not viewer.is_running():
                break
            if bridge:
                bridge.spin_once()
            if sim.startup_complete and policy_clock_origin is None:
                policy_clock_origin = float(sim.data.time)
            command = bridge.command if bridge else args.cmd
            ready_to_move = policy_clock_origin is not None and sim.data.time >= policy_clock_origin + args.settle
            if ready_to_move and first_command_interval_s is None:
                first_command_interval_s = float(sim.data.time)
            sim.set_command(*(command if ready_to_move else (0., 0., 0.)))
            sim.step()
            if history is not None:
                position, quaternion = sim.pose("lidar")
                history.add(sim.stamp_ns, position, quaternion[[3, 0, 1, 2]])
            min_height = min(min_height, float(sim.pose()[0][2]))
            upright = sim.data.site_xmat[sim.model.site("base_origin").id].reshape(3, 3)[2, 2]
            max_tilt_deg = max(max_tilt_deg, float(np.degrees(np.arccos(np.clip(upright, -1., 1.)))))
            if bridge:
                bridge.publish_clock(sim.stamp_ns)
            emit_imu_samples(imu_sampler.observe(sim.stamp_ns, *sim.imu()[:2]))
            if sim.steps % lidar_every == 0:
                if lidar:
                    before = time.perf_counter()
                    if history is not None:
                        last_scan = lidar.scan(sim.data, stamp_ns=sim.stamp_ns - 100_000_000,
                                               n_rays=args.rays, history=history)
                    else:
                        last_scan = lidar.scan(sim.data, stamp_ns=sim.stamp_ns, n_rays=args.rays)
                    if args.rays < 20000:
                        pattern.next_frame(20000 - args.rays)
                    last_scan_pose = sim.pose("lidar")
                    last_scan_end_ns = sim.stamp_ns
                    timings.append(time.perf_counter() - before)
                    counts.append(last_scan.point_num)
                pos, quat = sim.pose()
                lin, angular = sim.velocity()
                if trajectory:
                    trajectory.writerow([sim.stamp_ns * 1e-9, *pos, *quat, *lin, *angular])
                if bridge:
                    if lidar:
                        bridge.publish_scan(last_scan)
                    bridge.publish_odometry(sim.stamp_ns, pos, quat, lin, angular)
            if cameras and sim.stamp_ns >= next_camera_ns:
                before = time.perf_counter()
                for kind, cam in zip(("perspective", "panorama"), cameras):
                    frame = cam.render(sim.data, *sim.camera_pose())
                    if bridge:
                        bridge.publish_camera(sim.stamp_ns, frame.rgb, frame.depth, kind=kind)
                camera_timings.append(time.perf_counter() - before)
                next_camera_ns += camera_period_ns
            if viewer and sim.steps % 5 == 0:
                viewer.sync()
            if args.realtime or args.viewer or args.ros2:
                remaining = sim.data.time - (time.perf_counter() - start_wall)
                if remaining > 0:
                    time.sleep(remaining)
        wall = time.perf_counter() - start_wall
        loop_usage = loop_resources.report()
        if cameras and not args.summary_only:
            from PIL import Image
            position, rotation = sim.camera_pose()
            camera_metadata = {
                "stamp_ns": sim.stamp_ns, "position_world_m": position.tolist(),
                "rotation_world_from_optical": rotation.tolist(),
                "optical_axes": "+X right, +Y down, +Z forward", "depth_unit": "metre",
                "no_surface_depth": "positive infinity", "cameras": {},
            }
            for kind, cam in zip(("perspective", "panorama"), cameras):
                frame = cam.render(sim.data, position, rotation)
                Image.fromarray(frame.rgb).save(output / f"{kind}-rgb.png")
                np.save(output / f"{kind}-depth-{frame.depth_kind}.npy", frame.depth)
                detail = {"width": cam.width, "height": cam.height, "depth_kind": frame.depth_kind}
                if kind == "perspective":
                    detail.update(K=cam.intrinsics.tolist(), vertical_fov_deg=cam.fov_y_deg)
                else:
                    detail.update(projection="equirectangular", horizontal_fov_deg=360,
                                  vertical_fov_deg=180, cubemap_face_size=cam.face_size,
                                  longitude="2*pi*((column+0.5)/width-0.5)",
                                  latitude="pi*(0.5-(row+0.5)/height)")
                camera_metadata["cameras"][kind] = detail
            (output / "camera-metadata.json").write_text(json.dumps(camera_metadata, indent=2) + "\n")

    if last_scan is not None and not args.summary_only:
        per_point_pose = {}
        if history is not None:
            positions, quaternions = history.poses(last_scan.stamp_ns + last_scan.offsets_ns.astype(np.int64))
            per_point_pose = {"capture_position_world_m": positions,
                              "capture_quaternion_world_xyzw": quaternions[:, [1, 2, 3, 0]]}
        np.savez_compressed(output / "last-scan.npz", xyz=last_scan.xyz,
                            offsets_ns=last_scan.offsets_ns, lines=last_scan.lines,
                            ray_indices=last_scan.ray_indices,
                            stamp_ns=np.uint64(last_scan.stamp_ns),
                            reference_stamp_ns=np.uint64(last_scan_end_ns),
                            acquisition_mode=args.lidar_motion,
                            position_world_m=last_scan_pose[0], quaternion_world_xyzw=last_scan_pose[1],
                            **per_point_pose)
    summary = {
        "robot": args.robot, "controller": sim.controller_name, "terrain": sim.terrain.metadata,
        "deployment": sim.deployment_metadata,
        "policy_clock_origin_s": policy_clock_origin,
        "requested_settle_s": args.settle,
        "planned_movement_command_start_s": policy_clock_origin + args.settle if policy_clock_origin is not None else None,
        "movement_command_start_s": first_command_interval_s,
        "command_time_definition": "First post-settle command interval before its physics step; null when duration ends before it begins",
        "control_mode": "tracked" if sim.command_tracker else "native",
        "control_feedback_source": "ideal simulated body velocity and heading" if sim.command_tracker else None,
        "control_feedback_scope": "Outer body velocity/heading tracker; controller feedback is recorded in deployment",
        "tracker_command_saturation_fraction": sim.command_tracker.saturation_fraction if sim.command_tracker else None,
        "final_policy_command_m_s_rad_s": sim.policy_command.tolist(),
        "backend": args.backend if lidar else None, "simulation_seconds": sim.data.time,
        "wall_seconds": wall, "real_time_factor": sim.data.time / wall,
        "requested_command_m_s_rad_s": args.cmd if not bridge else f"ROS {bridge.command_topic} + wall-clock timeout",
        "ros_command_topic": bridge.command_topic if bridge else None,
        "applied_command_limits": {name: [-float(limit), float(limit)] for name, limit
                                   in zip(("vx_m_s", "vy_m_s", "wz_rad_s"), sim.command_limits)},
        "final_applied_command_m_s_rad_s": sim.command.tolist(),
        "displacement_m": (sim.pose()[0] - start_position).tolist(),
        "min_base_height_m": min_height, "max_tilt_deg": max_tilt_deg,
        "lidar_frames": len(timings), "rays_per_frame": args.rays,
        "lidar_median_ms": float(np.median(timings) * 1000) if timings else None,
        "lidar_p95_ms": float(np.percentile(timings, 95) * 1000) if timings else None,
        "min_valid_points": min(counts) if counts else 0,
        "max_valid_points": max(counts) if counts else 0,
        "scan_model": lidar.metadata if lidar else None,
        "mid360_geometry": sim.mid360_metadata,
        "imu_model": imu_sampler.metadata,
        "imu_samples": imu_samples, "imu_interpolated_samples": imu_interpolated_samples,
        "last_imu_stamp_ns": last_imu_stamp_ns,
        "ros_sensor_contract": bridge.metadata if bridge else None,
        "timing_scope": "simulation loop incl. lidar, transport/pacing when enabled; excludes setup, JIT warmup and final image saving",
        "last_lidar_frame_end_ns": last_scan_end_ns,
        "saved_camera_stamp_ns": camera_metadata["stamp_ns"] if camera_metadata is not None else None,
        "cameras": args.cameras, "ros2": args.ros2,
        "sensor_profile": args.sensors, "camera_hz": args.camera_hz if cameras else 0,
        "camera_frames_per_sensor": len(camera_timings),
        "camera_pair_median_ms": float(np.median(camera_timings) * 1000) if camera_timings else None,
        "camera_pair_p95_ms": float(np.percentile(camera_timings, 95) * 1000) if camera_timings else None,
        "resources_loop": loop_usage, "resources_run": run_resources.report(),
        "resources_run_scope": "main() through output preparation; excludes top-level imports; peak RSS still covers whole process",
        "artifacts_before_summary": directory_size(output),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"output": str(output), **summary}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
