#!/usr/bin/env python3
"""Observe planar commands and stopping in fresh, tracked flat-terrain cases.

No sensor/rendering workload is included. Measurements establish only these
commands, durations and flat scene; they do not certify hardware or all terrains.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
COMMANDS = {"yaw": (0., 0., .5), "lateral": (0., .3, 0.),
            "reverse": (-.3, 0., 0.), "combined": (.3, 0., .3),
            "park": (0., 0., 0.)}
THRESHOLDS = {
    "mean_body_velocity_error_vx_vy_wz": [.10, .10, .15],
    "maximum_tilt_deg": 35., "minimum_base_height_m": .15,
    "park_maximum_planar_drift_m": .10, "park_maximum_yaw_drift_rad": .10,
    "stop_final_second_mean_planar_speed_m_s": .08,
    "stop_final_second_mean_abs_yaw_rate_rad_s": .10,
}


def run_case(robot, name, go2_controller="himloco", *, go2_torque_mode="configured", go2_startup="crouched"):
    import numpy as np
    from simple_rbt_mujoco.core import Simulation
    deployment = {"go2_torque_mode": go2_torque_mode, "go2_startup": go2_startup} if robot == "go2" else {}
    sim = Simulation(robot, terrain="flat", controller=go2_controller if robot == "go2" else "auto",
                     velocity_feedback=True, **deployment)
    dt = float(sim.model.opt.timestep)
    tracker = sim.command_tracker
    yaw_previous = 0.
    yaw_unwrapped = 0.
    samples = []
    max_tilt, min_height = 0., math.inf
    started = time.perf_counter()
    while not sim.startup_complete:
        if sim.data.time >= 10.:
            raise RuntimeError("Go2 startup watchdog expired")
        sim.set_command(0., 0., 0.)
        sim.step()
    policy_origin = float(sim.data.time)
    if policy_origin:
        rotation = sim.data.site_xmat[sim.model.site("base_origin").id].reshape(3, 3)
        yaw_previous = math.atan2(rotation[1, 0], rotation[0, 0])

    def phase(label, command, seconds):
        nonlocal yaw_previous, yaw_unwrapped, max_tilt, min_height
        start_position = sim.pose()[0]
        start_yaw = yaw_unwrapped
        before = (tracker.update_count, tracker.saturation_count, tracker.correction_saturation_count)
        velocities, positions, yaw_values = [], [], []
        for _ in range(round(seconds / dt)):
            sim.set_command(*command)
            sim.step()
            position, quaternion = sim.pose()
            linear, angular = sim.velocity()
            rotation = sim.data.site_xmat[sim.model.site("base_origin").id].reshape(3, 3)
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            yaw_unwrapped += math.atan2(math.sin(yaw - yaw_previous), math.cos(yaw - yaw_previous))
            yaw_previous = yaw
            tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1., 1.))))
            max_tilt, min_height = max(max_tilt, tilt), min(min_height, float(position[2]))
            if not np.isfinite(np.r_[position, quaternion, linear, angular]).all():
                raise RuntimeError("Nonfinite physical state")
            positions.append(position.copy())
            velocities.append([linear[0], linear[1], angular[2]])
            yaw_values.append(yaw_unwrapped)
            if sim.steps % round(.1 / dt) == 0:
                samples.append({"stamp_ns": sim.stamp_ns, "phase": label,
                                "position_world_m": position.tolist(),
                                "body_vx_vy_wz": velocities[-1],
                                "yaw_unwrapped_rad": yaw_unwrapped,
                                "policy_command": sim.policy_command.tolist()})
        positions = np.asarray(positions)
        velocity = np.asarray(velocities)
        steady = velocity[len(velocity) // 2:]
        final = velocity[-round(1. / dt):]
        target = np.asarray(command)
        update_count = tracker.update_count - before[0]
        return {
            "duration_s": seconds, "command_vx_vy_wz": list(command),
            "start_position_world_m": start_position.tolist(),
            "final_position_world_m": positions[-1].tolist(),
            "displacement_world_m": (positions[-1] - start_position).tolist(),
            "maximum_planar_displacement_m": float(np.linalg.norm(positions[:, :2] - start_position[:2], axis=1).max()),
            "yaw_change_rad": yaw_unwrapped - start_yaw,
            "maximum_abs_yaw_change_rad": float(np.max(np.abs(np.asarray(yaw_values) - start_yaw))),
            "mean_body_vx_vy_wz": velocity.mean(axis=0).tolist(),
            "steady_window": "last half of this phase",
            "steady_mean_body_vx_vy_wz": steady.mean(axis=0).tolist(),
            "steady_mean_error_vx_vy_wz": (steady.mean(axis=0) - target).tolist(),
            "steady_mean_absolute_error_vx_vy_wz": np.abs(steady - target).mean(axis=0).tolist(),
            "steady_rmse_vx_vy_wz": np.sqrt(((steady - target) ** 2).mean(axis=0)).tolist(),
            "final_second_mean_planar_speed_m_s": float(np.linalg.norm(final[:, :2], axis=1).mean()),
            "final_second_mean_abs_yaw_rate_rad_s": float(np.abs(final[:, 2]).mean()),
            "command_saturation_fraction": (tracker.saturation_count - before[1]) / max(1, update_count),
            "correction_saturation_fraction": (tracker.correction_saturation_count - before[2]) / max(1, update_count),
        }

    settle = phase("settle", (0., 0., 0.), 2.)
    command = phase("command", COMMANDS[name], 8.)
    stop = phase("stop", (0., 0., 0.), 3.)
    checks = {
        "upright": max_tilt <= THRESHOLDS["maximum_tilt_deg"] and min_height >= THRESHOLDS["minimum_base_height_m"],
        "steady_body_velocity_tracks_command": bool(np.all(np.abs(command["steady_mean_error_vx_vy_wz"]) <= THRESHOLDS["mean_body_velocity_error_vx_vy_wz"])),
        "stopped_planar_motion": stop["final_second_mean_planar_speed_m_s"] <= THRESHOLDS["stop_final_second_mean_planar_speed_m_s"],
        "stopped_yaw_motion": stop["final_second_mean_abs_yaw_rate_rad_s"] <= THRESHOLDS["stop_final_second_mean_abs_yaw_rate_rad_s"],
    }
    if name == "park":
        checks["park_position_drift"] = command["maximum_planar_displacement_m"] <= THRESHOLDS["park_maximum_planar_drift_m"]
        checks["park_heading_drift"] = command["maximum_abs_yaw_change_rad"] <= THRESHOLDS["park_maximum_yaw_drift_rad"]
    return {"robot": robot, "case": name, "controller": sim.controller_name,
            "deployment": sim.deployment_metadata, "policy_clock_origin_s": policy_origin,
            "control_mode": "tracked", "terrain": sim.terrain.metadata,
            "physics_timestep_s": dt, "passed": all(checks.values()), "checks": checks,
            "maximum_tilt_deg": max_tilt, "minimum_base_height_m": min_height,
            "settle": settle, "command": command, "stop": stop, "samples": samples,
            "simulation_seconds": sim.data.time, "wall_seconds": time.perf_counter() - started}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", nargs="+", choices=("go2", "go2w"), default=["go2", "go2w"])
    parser.add_argument("--go2-controller", choices=("himloco", "robot_lab", "legacy"), default="himloco")
    parser.add_argument("--go2-torque-mode", choices=("configured", "upstream"), default="configured")
    parser.add_argument("--go2-startup", choices=("crouched", "upstream_getup"), default="crouched")
    parser.add_argument("--cases", nargs="+", choices=tuple(COMMANDS), default=list(COMMANDS))
    parser.add_argument("--output", type=Path, required=True, help="New JSON file; existing evidence is preserved")
    parser.add_argument("--require-pass", action="store_true", help="Exit 1 when any declared acceptance check fails")
    args = parser.parse_args()
    if args.go2_controller == "legacy" and (args.go2_torque_mode != "configured" or args.go2_startup != "crouched"):
        parser.error("Go2 deployment alternatives require himloco or robot_lab")
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[name] = os.environ.get("SIMPLE_RBT_CPU_THREADS", "1")
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "thresholds": THRESHOLDS,
        "scope": "Fresh model/data/controller per case; optional Go2 GetUp followed by full 2 s settle + 8 s command + 3 s stop on flat terrain. No rendering or sensor workload.",
        "control_aid": "Ideal simulated body velocity and heading adjust only native policy commands; no scene forces, teleporting, weight or native-gain changes.",
        "limitations": "Tests only listed commands/durations; average-error acceptance does not imply small instantaneous gait oscillations, hardware validity or arbitrary terrain success.",
        "argv": [sys.executable, *sys.argv],
        "requested_go2_controller": args.go2_controller,
        "requested_go2_torque_mode": args.go2_torque_mode, "requested_go2_startup": args.go2_startup,
        "source_sha256": {name: hashlib.sha256((PACKAGE_ROOT / "simple_rbt_mujoco" / name).read_bytes()).hexdigest()
                          for name in ("core.py", "command_tracking.py", "go2.py", "go2w.py", "go2_deployment.py")},
        "cases": [],
    }
    for robot in args.robots:
        for name in args.cases:
            try:
                result = run_case(robot, name, args.go2_controller,
                                  go2_torque_mode=args.go2_torque_mode, go2_startup=args.go2_startup)
            except Exception as error:
                result = {"robot": robot, "case": name, "passed": False,
                          "exception": f"{type(error).__name__}: {error}"}
            report["cases"].append(result)
            report["passed"] = all(row["passed"] for row in report["cases"])
            args.output.write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps({key: result.get(key) for key in ("robot", "case", "passed", "checks", "exception")}), flush=True)
    return 1 if args.require_pass and not report["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
