#!/usr/bin/env python3
"""Fresh-process CPU acceptance for four physical wheeled chassis.

Scope is flat motion, continuous ramps and hills. Fixed gates operate on actual
body/sensor/contact states; a failed route is not a reason to change the model.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import traceback

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROOT = PACKAGE_ROOT.parent
sys.path.insert(0, str(PACKAGE_ROOT))
ROBOTS = ("scout_mini", "fr_mini", "ridgeback", "ranger_mini")
LEVELS = ("standard", "moderate", "hard", "extreme")
GATE_SOURCE = Path(__file__).with_name("evaluate_scout_mini.py")
_spec = importlib.util.spec_from_file_location("_wheel_shared_gates", GATE_SOURCE)
_gates = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_gates)
parking_metrics = _gates.parking_metrics


def tracking_metrics(stamps, velocities, headings, target, transient_s=1.):
    """Existing speed/yaw gates, with commanded lateral motion treated as a target."""
    import numpy as np
    result = _gates.tracking_metrics(stamps, velocities, headings, target, transient_s)
    if "checks" not in result:
        return result
    # Scout's zero-sideways gate would wrongly fail an intentional strafe. Keep
    # that exact gate for vy=0; apply the same 15%/0.05 rule for a nonzero target.
    if target[1] != 0.:
        tolerance = max(.05, .15 * abs(float(target[1])))
        result["checks"].pop("mean_absolute_vy")
        result["checks"]["mean_vy_error"] = bool(abs(
            result["steady_mean_body_vx_vy_vz_wx_wy_wz"][1] - target[1]) <= tolerance)
        result["thresholds"].pop("mean_absolute_vy_m_s")
        result["thresholds"]["mean_vy_error_m_s"] = tolerance
    result["passed"] = all(result["checks"].values())
    result["target_vx_vy_wz"] = np.asarray(target, dtype=float).tolist()
    return result


def make_cases(robots=ROBOTS, control_modes=("native",), speeds=(.3, .6, 1.),
               levels=("standard", "moderate")):
    from simple_rbt_mujoco.robots import COMMAND_LIMITS
    robots, control_modes, speeds, levels = map(list, (robots, control_modes, speeds, levels))
    if not robots or any(name not in ROBOTS for name in robots):
        raise ValueError(f"robots must be chosen from {ROBOTS}")
    if not control_modes or any(mode not in ("native", "tracked") for mode in control_modes):
        raise ValueError("control modes must be native or tracked")
    if not levels or any(level not in LEVELS for level in levels):
        raise ValueError(f"levels must be chosen from {LEVELS}")
    if not speeds or any(not math.isfinite(float(v)) or v <= 0 for v in speeds):
        raise ValueError("speeds must be finite and positive")
    if any(speed > COMMAND_LIMITS[robot][0] for robot in robots for speed in speeds):
        raise ValueError("requested speed exceeds at least one selected robot's command limit")
    for values in (robots, control_modes, speeds, levels):
        if len(values) != len(set(values)):
            raise ValueError("duplicate sweep values would produce duplicate cases")
    cases = []
    for robot in robots:
        flat = [(f"forward_{float(speed)}", [float(speed), 0., 0.]) for speed in speeds]
        flat += [("reverse_0.3", [-.3, 0., 0.])]
        if robot == "fr_mini":
            flat += [("arc_left", [.3, 0., .2]), ("reverse_arc_left", [-.3, 0., .2])]
        else:
            flat += [("turn_left", [0., 0., .5]), ("turn_right", [0., 0., -.5]),
                     ("arc_left", [.3, 0., .3])]
        if robot in ("ridgeback", "ranger_mini"):
            flat += [("strafe_left", [0., .3, 0.]), ("strafe_right", [0., -.3, 0.]),
                     ("compound", [.3, .2, .3])]
        for mode in control_modes:
            def append(name, terrain, level, command):
                case_id = f"{robot}__{mode}__{terrain}__{level}__{name}"
                cases.append({"case": case_id, "motion": name, "robot": robot,
                              "controller": "auto", "control_mode": mode, "terrain": terrain,
                              "terrain_level": level, "difficulty": 1., "seed": 0,
                              "command_vx_vy_wz": command, "settle_s": 2., "flat_motion_s": 6.,
                              "stop_s": 2., "steady_exclude_first_s": 1.,
                              "distance_budget_m": 8., "max_samples": 240})
            for name, command in flat:
                append(name, "flat", "standard", command)
            for terrain in ("ramp", "hills"):
                for level in levels:
                    for speed in speeds:
                        append(f"forward_{float(speed)}", terrain, level, [float(speed), 0., 0.])
    return cases


def motion_budget(config, profile):
    if config["terrain"] == "flat":
        return float(config["flat_motion_s"])
    speed = float(config["command_vx_vy_wz"][0])
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("A course needs a finite positive forward command")
    return max(float(config["distance_budget_m"]), float(profile.finish_x) + 2.) / speed


def source_provenance():
    paths = [Path(__file__).resolve(), GATE_SOURCE,
             *sorted((PACKAGE_ROOT / "simple_rbt_mujoco").glob("*.py")),
             *sorted((PACKAGE_ROOT / "assets").glob("*manifest.json"))]
    head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True, text=True, check=False)
    return {"worktree_base_commit": head.stdout.strip() if head.returncode == 0 else None,
            "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
            "thread_environment": {name: os.environ.get(name) for name in
                                   ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
            "logical_source_bytes": sum(path.stat().st_size for path in paths),
            "scope": "All Python package modules (including wheel geometry helper), all asset manifests, this evaluator and unchanged shared Scout gates."}


def controller_state(controller, model, data):
    """Read current joint state; do not label passive fronts or rollers as motors."""
    position, velocity = controller.read_wheel_state(data)
    indices = controller.actuator_indices
    state = {"wheel_order": list(controller.wheel_names), "wheel_position_rad": position.tolist(),
             "wheel_velocity_rad_s": velocity.tolist(), "wheel_target_rad_s": controller.target_velocity.tolist(),
             "wheel_requested_target_rad_s": controller.requested_target_velocity.tolist(),
             "drive_actuator_names": [model.actuator(int(index)).name for index in indices],
             "drive_actuator_torque_nm": data.actuator_force[indices].tolist(),
             "saturation_scale": float(controller.saturation_scale)}
    if hasattr(controller, "read_steering_state"):
        q, dq = controller.read_steering_state(data)
        indices = controller.steering_actuator_indices
        state.update(steering_position_rad=q.tolist(), steering_velocity_rad_s=dq.tolist(),
                     steering_target_rad=controller.steering_targets.tolist(),
                     steering_actuator_names=[model.actuator(int(index)).name for index in indices],
                     steering_actuator_torque_nm=data.actuator_force[indices].tolist())
    for name in ("mode", "requested_mode", "alignment_scale", "curvature_saturated",
                 "requested_curvature", "applied_curvature", "effective_yaw_command", "yaw_correction"):
        if hasattr(controller, name):
            value = getattr(controller, name)
            state[name] = value.item() if hasattr(value, "item") else value
    if hasattr(controller, "applied_command"):
        state["applied_vx_vy_wz"] = controller.applied_command.tolist()
    if hasattr(controller, "drive_actuator_indices") and len(controller.drive_actuator_indices) == 2:
        state["front_target_note"] = "First two target rates predict passive front-wheel rolling; only rear wheels are driven."
    return state


def chassis_bottom_gap(model, data, profile):
    """Nine actual chassis underside points, diagnostic only; no base-origin shortcut."""
    import numpy as np
    geom = model.geom("chassis").id
    sx, sy, sz = model.geom_size[geom]
    points = np.array([[x * sx, y * sy, -sz] for x in (-1., 0., 1.) for y in (-1., 0., 1.)])
    points = points @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
    return float(np.min(points[:, 2] - profile.height_at(points[:, 0], points[:, 1])))


def evaluate_case(config):
    from simple_rbt_mujoco.resources import ResourceMeter
    meter = ResourceMeter()
    import numpy as np
    from simple_rbt_mujoco.core import Simulation
    from simple_rbt_mujoco.wheel_geometry import (
        collision_envelope, wheel_contact_geom_ids, body_environment_contacts)
    if config["robot"] not in ROBOTS or config["terrain"] not in ("flat", "ramp", "hills"):
        raise ValueError("Wheel evaluation supports selected wheeled models on flat/ramp/hills only")
    sim = Simulation(config["robot"], controller="auto", terrain=config["terrain"],
                     terrain_level=config["terrain_level"], difficulty=config["difficulty"], seed=config["seed"],
                     velocity_feedback=config["control_mode"] == "tracked")
    model, data, profile, controller = sim.model, sim.data, sim.terrain, sim.controller
    target = np.asarray(config["command_vx_vy_wz"], dtype=float)
    sim.set_command(*target)
    if not np.array_equal(sim.command, target):
        raise ValueError("Evaluation target was clipped; do not silently test a different command")
    sim.set_command(0., 0., 0.)
    initial_geometry = collision_envelope(model, data)
    initial_world_geometry = collision_envelope(model, data, reference_frame="world")
    wheel_ids = wheel_contact_geom_ids(model)
    terrain_case = config["terrain"] != "flat"
    duration = motion_budget(config, profile)
    thresholds = {"max_tilt_deg": 45., "non_wheel_environment_contact_is_failure": True,
                  "corridor_margin_m": .1, "corridor_formula": "course width/2 - current world-Y collision outer_half_width relative to base_origin - .1m",
                  "corridor_enforcement": "terrain motion, from first base entry through route completion",
                  "finish_x_m": profile.finish_x if terrain_case else None,
                  "motion_time_budget_s": duration, "parking_s": config["stop_s"],
                  "clearance_height_is_diagnostic_only": True}
    dt = float(model.opt.timestep)
    initial_position = sim.pose()[0].copy()
    last_position, last_stamp_ns, last_time = initial_position.copy(), sim.stamp_ns, float(data.time)
    initialization, provenance = meter.report(), source_provenance()
    t_motion, v_motion, yaw_motion, t_stop, v_stop, history = [], [], [], [], [], []
    failure = failure_phase = None
    route_complete = motion_complete = entered = False
    max_tilt = max_lateral = 0.
    max_x = float(initial_position[0])
    min_gap = math.inf
    min_corridor = profile.width / 2 - initial_world_geometry["outer_half_width_m"] - .1
    contacts = set()
    previous_yaw = math.atan2(data.site("base_origin").xmat[3], data.site("base_origin").xmat[0])
    unwrapped_yaw = previous_yaw
    start_position = stop_position = stop_end_position = None
    sample_every = max(1, round(.05 / dt))
    mode_counts = {}
    motion_saturated_steps = 0
    max_drive_torque = np.zeros(len(controller.actuator_indices))
    max_steering_torque = np.zeros(len(getattr(controller, "steering_actuator_indices", [])))
    loop_meter = ResourceMeter()
    for phase, seconds, command in (("settle", config["settle_s"], [0., 0., 0.]),
                                     ("motion", duration, target), ("stop", config["stop_s"], [0., 0., 0.])):
        if failure:
            break
        if phase == "motion":
            start_position = sim.pose()[0].copy()
        elif phase == "stop":
            stop_position = sim.pose()[0].copy()
        start_step = sim.steps
        for _ in range(math.ceil(seconds / dt)):
            try:
                sim.set_command(*command)
                sim.step()
                position, quaternion = sim.pose()
                linear, angular = sim.velocity()
                state = np.r_[position, quaternion, linear, angular, data.qpos, data.qvel, data.ctrl, data.actuator_force, data.time]
                if not np.isfinite(state).all():
                    failure, failure_phase = "nonfinite_state", phase
                    break
            except Exception as error:
                failure, failure_phase = f"simulation_error: {type(error).__name__}: {error}", phase
                break
            last_position, last_stamp_ns, last_time = position.copy(), sim.stamp_ns, float(data.time)
            elapsed = (sim.steps - start_step) * dt
            rotation = data.site("base_origin").xmat.reshape(3, 3)
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            unwrapped_yaw += math.atan2(math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw))
            previous_yaw = yaw
            tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1., 1.))))
            max_tilt, max_lateral, max_x = max(max_tilt, tilt), max(max_lateral, abs(float(position[1]))), max(max_x, float(position[0]))
            current_contacts = body_environment_contacts(model, data, wheel_ids)
            contacts.update(current_contacts)
            entered = entered or (terrain_case and position[0] >= profile.entry_x)
            corridor = min_corridor
            if terrain_case and phase == "motion":
                corridor = profile.width / 2. - collision_envelope(model, data, reference_frame="world")["outer_half_width_m"] - .1
                min_corridor = min(min_corridor, corridor)
            if current_contacts:
                failure = "non_wheel_environment_contact"
            elif tilt > thresholds["max_tilt_deg"]:
                failure = "tilt_limit"
            elif terrain_case and phase == "motion" and entered and abs(position[1]) > corridor:
                failure = "course_corridor_exit"
            if failure:
                failure_phase = _gates.course_phase(profile, float(position[0])) if terrain_case and phase == "motion" else phase
            measured = np.r_[linear, angular]
            if phase == "motion":
                t_motion.append(elapsed); v_motion.append(measured.tolist()); yaw_motion.append(unwrapped_yaw)
                mode = getattr(controller, "mode", sim.controller_name)
                mode_counts[mode] = mode_counts.get(mode, 0) + 1
                motion_saturated_steps += int(controller.saturation_scale < 1. - 1e-12 or getattr(controller, "curvature_saturated", False))
                max_drive_torque = np.maximum(max_drive_torque, np.abs(data.actuator_force[controller.actuator_indices]))
                if len(max_steering_torque):
                    max_steering_torque = np.maximum(max_steering_torque, np.abs(data.actuator_force[controller.steering_actuator_indices]))
            elif phase == "stop":
                t_stop.append(elapsed); v_stop.append(measured.tolist()); stop_end_position = position.copy()
            if sim.steps % sample_every == 0 or failure:
                gap = chassis_bottom_gap(model, data, profile)
                min_gap = min(min_gap, gap)
                history.append({"stamp_ns": last_stamp_ns, "phase": phase,
                                "course_phase": _gates.course_phase(profile, float(position[0])) if terrain_case else None,
                                "position_m": position.tolist(), "world_heading_unwrapped_rad": unwrapped_yaw,
                                "body_vx_vy_vz_wx_wy_wz": measured.tolist(), "tilt_deg": tilt,
                                "chassis_bottom_sampled_gap_m": gap, "corridor_half_width_m": corridor if terrain_case else None,
                                "user_command_vx_vy_wz": sim.command.tolist(), "controller_command_vx_vy_wz": sim.policy_command.tolist(),
                                "actuation": controller_state(controller, model, data), "body_contacts": current_contacts})
            if failure:
                break
            if phase == "motion" and terrain_case and position[0] >= profile.finish_x:
                route_complete = motion_complete = True
                break
        if phase == "motion" and not failure:
            motion_complete = route_complete if terrain_case else True
            if not motion_complete:
                failure = "route_timeout"
                failure_phase = _gates.course_phase(profile, float(last_position[0]))
    loop_resources = loop_meter.report()
    loop_resources["simulated_seconds"] = last_time
    loop_resources["unpaced_realtime_factor_with_acceptance_instrumentation"] = last_time / loop_resources["wall_seconds"]
    tracking = tracking_metrics(t_motion, v_motion, yaw_motion, target, config["steady_exclude_first_s"])
    parking = parking_metrics(t_stop, v_stop, config["stop_s"])
    if len(history) > config["max_samples"]:
        history = [history[index] for index in np.linspace(0, len(history) - 1, config["max_samples"], dtype=int)]
    tracker = sim.command_tracker
    tracker_metadata = ({"source": "ideal simulation body velocity and heading", "updates": tracker.update_count,
                         "saturation_count": tracker.saturation_count,
                         "correction_saturation_count": tracker.correction_saturation_count} if tracker else None)
    failures = ([failure] if failure else []) + ([] if tracking["passed"] else ["velocity_tracking"]) + (
        [] if parking["passed"] else ["parking" if t_stop else "parking_not_evaluated"])
    return {"config": config, "config_sha256": hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
            "actual_controller": sim.controller_name, "model_metadata": controller.metadata,
            "control_contract": {"native": "declared model controller, including its documented internal joint/gyro feedback",
                                 "additional_body_tracker": tracker is not None,
                                 "scope": "Simulator engineering model/control; not manufacturer firmware or calibrated hardware"},
            "terrain": profile.metadata, "compiled_geometry_initial": initial_geometry,
            "compiled_geometry_initial_world": initial_world_geometry,
            "wheel_contact_geom_ids": wheel_ids, "thresholds": thresholds, "physics_period_s": dt,
            "safety_passed": failure is None or failure == "route_timeout", "motion_completed": motion_complete,
            "route_completed": route_complete if terrain_case else None, "tracking": tracking, "parking": parking,
            "parking_evaluated": bool(t_stop), "passed": failure is None and motion_complete and tracking["passed"] and parking["passed"],
            "failure_reason": failure, "failure_phase": failure_phase, "failed_acceptance_checks": failures,
            "initial_position_m": initial_position.tolist(), "motion_start_position_m": start_position.tolist() if start_position is not None else None,
            "final_position_m": last_position.tolist(), "last_finite_stamp_ns": last_stamp_ns, "last_finite_simulation_time_s": last_time,
            "stop_displacement_m": (stop_end_position - stop_position).tolist() if stop_end_position is not None else None,
            "max_base_x_m": max_x, "max_absolute_world_y_m": max_lateral, "max_tilt_deg": max_tilt,
            "minimum_corridor_half_width_m": min_corridor, "minimum_sampled_chassis_bottom_gap_m": min_gap if math.isfinite(min_gap) else None,
            "clearance_sampling": "9 actual chassis underside points every50ms; diagnostic only; body/deck collisions are evaluated separately every physics step",
            "non_wheel_contact_names": sorted(contacts),
            "actuation": {"drive_actuator_names": [model.actuator(int(i)).name for i in controller.actuator_indices],
                          "maximum_absolute_drive_torque_nm": max_drive_torque.tolist(),
                          "maximum_absolute_steering_torque_nm": max_steering_torque.tolist(),
                          "motion_mode_counts": mode_counts,
                          "motion_target_saturation_fraction": motion_saturated_steps / max(1, len(t_motion)),
                          "controller_whole_case_saturation_fraction": controller.saturation_fraction,
                          "joint_telemetry": "Actual four wheel joint angles/rates; FR fronts are passive. No artificial mecanum tire-slip metric."},
            "velocity_feedback": tracker_metadata,
            "resources": {"initialization": initialization, "physics_and_acceptance_loop": loop_resources,
                          "whole_case": meter.report(), "scope": "Fresh CPU worker, unpaced physics plus evaluation instrumentation; no LiDAR/camera/ROS/GPU. Peak RSS is whole-process, not steady RSS."},
            "provenance": provenance, "trajectory": history}


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", nargs="+", choices=ROBOTS, default=list(ROBOTS))
    parser.add_argument("--control-modes", nargs="+", choices=("native", "tracked"), default=["native"])
    parser.add_argument("--speeds", nargs="+", type=float, default=[.3, .6, 1.])
    parser.add_argument("--levels", nargs="+", choices=LEVELS, default=["standard", "moderate"])
    parser.add_argument("--output", type=Path, default=ROOT / "output/wheel-chassis-20261013/native-matrix")
    parser.add_argument("--case-json", help=argparse.SUPPRESS)
    parser.add_argument("--case-output", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.case_json:
        try:
            write_json(args.case_output, evaluate_case(json.loads(args.case_json)))
        except Exception:
            traceback.print_exc()
            return 2
        return 0
    try:
        cases = make_cases(args.robots, args.control_modes, args.speeds, args.levels)
    except ValueError as error:
        parser.error(str(error))
    args.output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, str(Path(__file__).resolve()), *argv]
    provenance = source_provenance()
    summaries = []
    for config in cases:
        path = args.output / (config["case"] + ".json")
        worker = [sys.executable, str(Path(__file__).resolve()), "--case-json", json.dumps(config), "--case-output", str(path)]
        try:
            completed = subprocess.run(worker, capture_output=True, text=True, timeout=180)
            (args.output / (config["case"] + ".log")).write_text(completed.stdout + completed.stderr)
            if completed.returncode or not path.exists():
                raise RuntimeError(f"Worker exit {completed.returncode}; see {config['case']}.log")
            full = json.loads(path.read_text())
            summary = {k: v for k, v in full.items() if k not in ("trajectory", "model_metadata", "provenance")}
            summary["worker_source_sha256"] = full["provenance"]["source_sha256"]
            summary["sources_match_batch_start"] = full["provenance"]["source_sha256"] == provenance["source_sha256"]
        except (subprocess.TimeoutExpired, RuntimeError) as error:
            summary = {"config": config, "passed": False, "runtime_error": str(error)}
        summaries.append(summary)
        write_json(args.output / "summary.json", {"command": command, "planned_cases": len(cases),
                                                  "completed_cases": len(summaries), "cases": summaries, "provenance": provenance})
        print(config["case"], "PASS" if summary["passed"] else "FAIL", flush=True)
    from simple_rbt_mujoco.resources import directory_size
    write_json(args.output / "disk-usage.json", {"output_before_this_file": directory_size(args.output)})
    lines = ["# Wheeled chassis CPU motion acceptance", "", "No friction, model, gain or acceptance gate tuning between cases.", "",
             "| Robot | Mode | Motion / terrain / level | Target vx/vy/wz | Measured vx/vy/wz | Route or motion | Speed | Stop | Cause |",
             "|---|---|---|---|---|---|---|---|---|"]
    for row in summaries:
        config = row["config"]; mean = row.get("tracking", {}).get("steady_mean_body_vx_vy_vz_wx_wy_wz")
        actual = "/".join(f"{mean[i]:.3f}" for i in (0, 1, 5)) if mean else "unavailable"
        flag = lambda value: "pass" if value else "fail"
        lines.append("| " + " | ".join([config["robot"], config["control_mode"],
            "/".join(config[k] for k in ("motion", "terrain", "terrain_level")),
            "/".join(f"{v:.3f}" for v in config["command_vx_vy_wz"]), actual,
            flag(row.get("motion_completed")), flag(row.get("tracking", {}).get("passed")),
            flag(row.get("parking", {}).get("passed")) if row.get("parking_evaluated") else "not run",
            row.get("runtime_error") or ", ".join(row.get("failed_acceptance_checks", [])) or "—"]) + " |")
    lines += ["", "Command: `" + shlex.join(command) + "`", "",
              "Measured values exclude the first motion second; full transient errors remain in JSON. Native includes each declared internal controller; tracked adds ideal body feedback.",
              "FR-mini velocity refers to the rear axle and its front rolling joints are passive. Mecanum roller contacts are wheel contacts. Deck impacts remain body contacts.",
              "No stairs or step-rough terrain are included. A timeout is failure within this route/time budget, not proof of categorical hardware inability.",
              "Resources cover CPU physics, native IMU/control and acceptance instrumentation; LiDAR/camera/ROS workloads are excluded. Whole-process peak RSS is not steady-state RSS."]
    (args.output / "RESULTS.md").write_text("\n".join(lines) + "\n")
    print(args.output)
    return 2 if any("runtime_error" in row for row in summaries) else 0


if __name__ == "__main__":
    raise SystemExit(main())
