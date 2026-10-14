#!/usr/bin/env python3
"""Fresh-process Scout Mini motion and complete-course acceptance, CPU physics only.

Fixed gates are declared before execution. Failures are measurements, not reasons
for retuning friction/gains. Resource costs exclude cameras, LiDAR and ROS.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import shlex
from pathlib import Path
import subprocess
import sys
import traceback

from project import ROOT, REFERENCE_ROOT
FLAT_COMMANDS = {
    "forward_0.3": (.3, 0., 0.), "forward_0.6": (.6, 0., 0.),
    "forward_1.0": (1., 0., 0.), "reverse_0.3": (-.3, 0., 0.),
    "turn_left": (0., 0., .5), "turn_right": (0., 0., -.5),
    "arc_left": (.3, 0., .3),
}
OPTIONAL_FLAT_COMMANDS = {"forward_1.5": (1.5, 0., 0.)}
TERRAIN_CASES = ("ramp", "hills")


def make_cases(selected=("all",), control_mode="native"):
    if control_mode not in ("native", "tracked"):
        raise ValueError("control_mode must be native or tracked")
    names = []
    original_cases = list(FLAT_COMMANDS) + list(TERRAIN_CASES)
    available = original_cases + list(OPTIONAL_FLAT_COMMANDS)
    for item in selected:
        expansion = (original_cases if item == "all" else list(FLAT_COMMANDS) if item == "flat"
                     else list(TERRAIN_CASES) if item == "terrain" else [item])
        for name in expansion:
            if name not in available:
                raise ValueError(f"Unknown case {name}; choose all, flat, terrain or {available}")
            if name not in names:
                names.append(name)
    if not names:
        raise ValueError("At least one case is required")
    return [{"case": name, "robot": "scout_mini", "controller": "skid_steer",
             "control_mode": control_mode, "terrain": name if name in TERRAIN_CASES else "flat",
             "terrain_level": "standard", "difficulty": 1., "seed": 0,
             "command_vx_vy_wz": list(FLAT_COMMANDS.get(name, OPTIONAL_FLAT_COMMANDS.get(name, (.3, 0., 0.)))),
             "settle_s": 2., "flat_motion_s": 6., "stop_s": 2.,
             "steady_exclude_first_s": 1., "distance_budget_m": 8., "max_samples": 200}
            for name in names]


def tracking_metrics(stamps, velocities, headings, target, transient_s=1.):
    """Body velocities and unwrapped WORLD heading have distinct contracts."""
    import numpy as np
    t, v, yaw, command = map(np.asarray, (stamps, velocities, headings, target))
    if len(t) == 0:
        return {"passed": False, "reason": "no commanded-motion samples"}
    mask = t >= transient_s - 1e-9
    # At least one second of post-transient measurement is required.
    valid = bool(mask.any() and t[-1] >= transient_s + 1. - 1e-9)
    observed = v[mask] if mask.any() else v
    errors = v[:, [0, 1, 5]] - command
    mean = observed.mean(axis=0)
    vx_tolerance = max(.05, .15 * abs(float(command[0])))
    wz_tolerance = max(.05, .15 * abs(float(command[2])))
    checks = {"sufficient_steady_window": valid,
              "mean_vx_error": bool(abs(mean[0] - command[0]) <= vx_tolerance),
              "mean_wz_error": bool(abs(mean[5] - command[2]) <= wz_tolerance),
              "mean_absolute_vy": bool(np.abs(observed[:, 1]).mean() <= .1),
              "zero_yaw_oscillation": bool(command[2] != 0. or np.abs(observed[:, 5]).mean() <= .1)}
    steady_t, steady_yaw = (t[mask], yaw[mask]) if mask.any() else (t, yaw)
    heading_rate = (float((steady_yaw[-1] - steady_yaw[0]) / (steady_t[-1] - steady_t[0]))
                    if len(steady_t) > 1 and steady_t[-1] > steady_t[0] else None)
    return {"passed": all(checks.values()), "checks": checks,
            "thresholds": {"mean_vx_error_m_s": vx_tolerance, "mean_wz_error_rad_s": wz_tolerance,
                           "mean_absolute_vy_m_s": .1, "zero_yaw_mean_absolute_wz_rad_s": .1,
                           "steady_exclude_first_s": transient_s, "minimum_steady_duration_s": 1.},
            "steady_mean_body_vx_vy_vz_wx_wy_wz": mean.tolist(),
            "steady_std_body_vx_vy_vz_wx_wy_wz": observed.std(axis=0).tolist(),
            "steady_mean_absolute_body_vy_m_s": float(np.abs(observed[:, 1]).mean()),
            "steady_mean_absolute_body_wz_rad_s": float(np.abs(observed[:, 5]).mean()),
            "full_motion_rmse_vx_vy_wz": np.sqrt(np.mean(errors ** 2, axis=0)).tolist(),
            "steady_world_heading_rate_rad_s": heading_rate,
            "world_heading_change_rad": float(yaw[-1] - yaw[0]),
            "motion_sample_duration_s": float(t[-1]),
            "body_vs_world_note": "Body angular Z and world-heading derivative differ when tilted; gates use measured body velocity."}


def parking_metrics(stamps, velocities, duration=2.):
    import numpy as np
    t, v = np.asarray(stamps), np.asarray(velocities)
    if len(t) == 0:
        return {"passed": False, "reason": "stop phase not executed"}
    last = v[t >= duration - 1. - 1e-9]
    if len(last) == 0:
        return {"passed": False, "reason": "no final-second stop samples"}
    planar = float(np.linalg.norm(last[:, :2], axis=1).mean())
    yaw = float(np.abs(last[:, 5]).mean())
    return {"passed": bool(t[-1] >= duration - 1e-9 and planar <= .05 and yaw <= .05),
            "mean_planar_speed_last_second_m_s": planar,
            "mean_absolute_wz_last_second_rad_s": yaw,
            "thresholds": {"mean_planar_speed_m_s": .05, "mean_absolute_wz_rad_s": .05},
            "sample_duration_s": float(t[-1])}


def geometry_contract(model, data):
    """Derive body clearance and tire envelope from the compiled model."""
    import mujoco
    import numpy as np
    site = model.site("base_origin").id
    origin = data.site_xpos[site]
    base_R = data.site_xmat[site].reshape(3, 3)
    chassis = model.geom("chassis").id
    if model.geom_type[chassis] != mujoco.mjtGeom.mjGEOM_BOX:
        raise ValueError("Scout evaluation requires the declared box chassis")
    wheel_names = ("front_left", "front_right", "rear_left", "rear_right")
    wheel_ids = [model.geom(name + "_tire").id for name in wheel_names]
    lower, upper = np.full(3, np.inf), np.full(3, -np.inf)
    for gid in [chassis, *wheel_ids]:
        local_R = base_R.T @ data.geom_xmat[gid].reshape(3, 3)
        size = model.geom_size[gid]
        if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX:
            extent = np.abs(local_R) @ size
        elif model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            # Exact axis-aligned envelope of the oriented solid cylinder.
            axis = local_R[:, 2]
            extent = size[0] * np.sqrt(np.maximum(0., 1. - axis ** 2)) + size[1] * np.abs(axis)
        elif model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            # Ellipsoid support along each base-frame axis, not a rotated box.
            extent = np.sqrt(np.sum((local_R * size) ** 2, axis=1))
        else:
            raise ValueError("Unexpected Scout collision geometry")
        centre = base_R.T @ (data.geom_xpos[gid] - origin)
        lower, upper = np.minimum(lower, centre - extent), np.maximum(upper, centre + extent)
    chassis_bottom = float(data.geom_xpos[chassis, 2] - model.geom_size[chassis, 2])
    return {"chassis_full_size_m": (2. * model.geom_size[chassis]).tolist(),
            "nominal_base_world_z_m": float(origin[2]),
            "nominal_chassis_bottom_world_z_m": chassis_bottom,
            "collision_envelope_base_lower_m": lower.tolist(),
            "collision_envelope_base_upper_m": upper.tolist(),
            "outer_half_width_m": float(max(abs(lower[1]), abs(upper[1]))),
            "wheel_names": list(wheel_names),
            "wheel_radius_m": [float(model.geom_size[i, 0]) for i in wheel_ids],
            "wheel_positive_joint_direction": "all +Y; positive joint velocity drives +X",
            "clearance_gate": "No base-height threshold; actual non-wheel environment contact and tilt are safety gates."}


def body_environment_contacts(model, data, wheel_ids):
    """Actual non-wheel robot/environment contacts; wheel contacts are expected."""
    names = set()
    for contact in data.contact:
        a, b = int(contact.geom1), int(contact.geom2)
        if a < 0 or b < 0 or contact.dist > 1e-6:
            continue
        if (model.geom_bodyid[a] == 0) == (model.geom_bodyid[b] == 0):
            continue
        robot_geom = b if model.geom_bodyid[a] == 0 else a
        if robot_geom not in wheel_ids:
            names.add((model.geom(a).name or str(a)) + " / " + (model.geom(b).name or str(b)))
    return sorted(names)


def course_phase(profile, x):
    if x >= profile.finish_x:
        return "complete"
    if x < profile.entry_x:
        return "approach"
    if profile.name in ("ramp", "stairs"):
        for key, phase in (("ascent_end_x_m", "ascent"), ("landing_end_x_m", "top"),
                           ("descent_end_x_m", "descent")):
            if x < profile.parameters[key]:
                return phase
        return "exit"
    return "exit" if x >= profile.exit_x else "traversal"


def source_provenance():
    paths = [Path(__file__).resolve(), *sorted((REFERENCE_ROOT / "simple_rbt_mujoco").glob("*.py")),
             ROOT / "assets/manifests/scout_mini_manifest.json"]
    paths = [path for path in paths if path.is_file()]
    head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"],
                          text=True, capture_output=True, check=False)
    return {"worktree_base_commit": head.stdout.strip() if head.returncode == 0 else None,
            "source_hash_scope": "Package-wide audit hashes plus this evaluator and Scout manifest; camera/LiDAR/ROS/CLI modules are included for provenance but their workloads are not executed. Other scripts are not hashed.",
            "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
            "source_and_manifest_disk": {"files": len(paths), "logical_bytes": sum(p.stat().st_size for p in paths),
                                         "allocated_bytes": sum(p.stat().st_blocks * 512 for p in paths),
                                         "scope": "Python package source, this evaluator and Scout manifest; excludes Python/MuJoCo dependencies and output."},
            "thread_environment": {key: os.environ.get(key) for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")}}


def build_case_simulation(config):
    from simple_rbt_mujoco.core import Simulation
    sim = Simulation("scout_mini", controller="skid_steer", terrain=config["terrain"],
                     terrain_level=config["terrain_level"], difficulty=config["difficulty"], seed=config["seed"],
                     velocity_feedback=config["control_mode"] == "tracked")
    return sim


def evaluate_case(config):
    from simple_rbt_mujoco.resources import ResourceMeter
    total_meter = ResourceMeter()
    import numpy as np
    sim = build_case_simulation(config)
    model, data, profile = sim.model, sim.data, sim.terrain
    geometry = geometry_contract(model, data)
    target = np.asarray(config["command_vx_vy_wz"], dtype=float)
    sim.set_command(*target)
    if not np.array_equal(sim.command, target):
        raise ValueError("Requested evaluation command was clipped; do not silently test another target")
    sim.set_command(0., 0., 0.)
    wheel_names = geometry["wheel_names"]
    wheel_ids = [model.geom(name + "_tire").id for name in wheel_names]
    wheel_body_ids = [model.body(name + "_wheel").id for name in wheel_names]
    wheel_dof_ids = [int(model.joint(name + "_wheel_joint").dofadr[0]) for name in wheel_names]
    radii = np.array(geometry["wheel_radius_m"])
    base_site, chassis_id = model.site("base_origin").id, model.geom("chassis").id
    corridor = profile.width / 2. - geometry["outer_half_width_m"] - .1
    terrain_case = config["terrain"] != "flat"
    motion_limit = (max(config["distance_budget_m"], profile.finish_x + 2.) / target[0]
                    if terrain_case else config["flat_motion_s"])
    thresholds = {"max_tilt_deg": 45., "non_wheel_environment_contact_is_failure": True,
                  "corridor_half_width_m": corridor, "corridor_margin_m": .1,
                  "corridor_enforcement": "terrain cases, from first entry through command completion",
                  "finish_x_m": profile.finish_x if terrain_case else None,
                  "motion_time_budget_s": motion_limit, "parking_s": config["stop_s"],
                  "clearance_height_is_diagnostic_only": True}
    initial_position = sim.pose()[0].copy()
    last_finite_position = initial_position.copy()
    initialization = total_meter.report()
    provenance = source_provenance()
    dt = model.opt.timestep
    command_t, command_v, command_yaw, command_slip, command_actuation = [], [], [], [], []
    parking_t, parking_v, history = [], [], []
    contacts, failure, failure_phase = set(), None, None
    last_yaw, unwrapped_yaw = 0., 0.
    max_tilt = max_lateral = max_x = 0.
    min_bottom_gap = math.inf
    entered = route_complete = motion_complete = False
    motion_start_position = stop_start_position = None
    stop_end_position = None
    sample_every = max(1, round(.05 / dt))
    bottom_xy = [(x, y) for x in (-1., 0., 1.) for y in (-1., 0., 1.)]
    loop_meter = ResourceMeter()
    for phase, duration, command in (("settle", config["settle_s"], [0., 0., 0.]),
                                     ("motion", motion_limit, target),
                                     ("stop", config["stop_s"], [0., 0., 0.])):
        if failure:
            break
        if phase == "motion":
            motion_start_position = sim.pose()[0].copy()
        elif phase == "stop":
            stop_start_position = sim.pose()[0].copy()
        start_step = sim.steps
        for local_step in range(math.ceil(duration / dt)):
            sim.set_command(*command)
            try:
                sim.step()
            except Exception as exc:
                failure, failure_phase = f"simulation_error: {type(exc).__name__}: {exc}", phase
                break
            pos, quat = sim.pose()
            linear, angular = sim.velocity()
            R = data.site_xmat[base_site].reshape(3, 3)
            values = np.r_[pos, quat, linear, angular, data.qvel, data.ctrl]
            if not np.isfinite(values).all():
                failure, failure_phase = "nonfinite_state", phase
                break
            last_finite_position = pos.copy()
            t = (sim.steps - start_step) * dt
            yaw = math.atan2(R[1, 0], R[0, 0])
            unwrapped_yaw += math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
            last_yaw = yaw
            tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1., 1.))))
            max_tilt, max_lateral, max_x = max(max_tilt, tilt), max(max_lateral, abs(float(pos[1]))), max(max_x, float(pos[0]))
            current_contacts = body_environment_contacts(model, data, wheel_ids)
            contacts.update(current_contacts)
            entered = entered or (terrain_case and pos[0] >= profile.entry_x)
            if current_contacts:
                failure = "non_wheel_environment_contact"
            elif tilt > thresholds["max_tilt_deg"]:
                failure = "tilt_limit"
            elif terrain_case and phase == "motion" and entered and abs(pos[1]) > corridor:
                failure = "course_corridor_exit"
            if failure:
                failure_phase = course_phase(profile, float(pos[0])) if terrain_case and phase == "motion" else phase
            velocity = np.r_[linear, angular]
            centre_base = (data.xpos[wheel_body_ids] - pos) @ R
            wheel_centre_vx = linear[0] + np.cross(np.broadcast_to(angular, (4, 3)), centre_base)[:, 0]
            rim_speed = data.qvel[wheel_dof_ids] * radii
            slip = rim_speed - wheel_centre_vx
            if phase == "motion":
                command_t.append(t); command_v.append(velocity.tolist()); command_yaw.append(unwrapped_yaw)
                command_slip.append(slip.tolist())
                command_actuation.append([sim.controller.effective_yaw_command, sim.controller.yaw_correction,
                                          *data.actuator_force[sim.controller.actuator_indices]])
            elif phase == "stop":
                parking_t.append(t); parking_v.append(velocity.tolist())
                stop_end_position = pos.copy()
            if sim.steps % sample_every == 0 or failure:
                chassis_R = data.geom_xmat[chassis_id].reshape(3, 3)
                size = model.geom_size[chassis_id]
                points = np.array([[x * size[0], y * size[1], -size[2]] for x, y in bottom_xy]) @ chassis_R.T + data.geom_xpos[chassis_id]
                gap = float(np.min(points[:, 2] - profile.height_at(points[:, 0], points[:, 1])))
                min_bottom_gap = min(min_bottom_gap, gap)
                history.append({"stamp_ns": sim.stamp_ns, "phase": phase,
                                "course_phase": course_phase(profile, float(pos[0])) if terrain_case else None,
                                "position_m": pos.tolist(), "body_vx_vy_vz_wx_wy_wz": velocity.tolist(),
                                "world_heading_unwrapped_rad": unwrapped_yaw, "tilt_deg": tilt,
                                "chassis_bottom_sampled_gap_m": gap, "wheel_rim_speed_m_s": rim_speed.tolist(),
                                "wheel_longitudinal_slip_m_s": slip.tolist(), "command_to_wheel_controller": sim.policy_command.tolist(),
                                "wheel_target_rad_s": sim.controller.target_velocity.tolist(),
                                "effective_yaw_command_rad_s": sim.controller.effective_yaw_command,
                                "yaw_feedback_correction_rad_s": sim.controller.yaw_correction,
                                "wheel_actuator_torque_nm": data.actuator_force[sim.controller.actuator_indices].tolist(),
                                "body_contacts": current_contacts})
            if failure:
                break
            if phase == "motion" and terrain_case and pos[0] >= profile.finish_x:
                route_complete = motion_complete = True
                break
        if phase == "motion" and not failure:
            motion_complete = route_complete if terrain_case else True
            if not motion_complete:
                failure, failure_phase = "route_timeout", course_phase(profile, float(sim.pose()[0][0]))
    loop_resources = loop_meter.report()
    loop_resources["simulated_seconds"] = float(data.time)
    loop_resources["unpaced_realtime_factor"] = float(data.time) / loop_resources["wall_seconds"]
    tracking = tracking_metrics(command_t, command_v, command_yaw, target, config["steady_exclude_first_s"])
    parking = parking_metrics(parking_t, parking_v, config["stop_s"])
    slip_array = np.asarray(command_slip)
    slip = ({"mean_signed_m_s": slip_array.mean(axis=0).tolist(),
             "mean_absolute_m_s": np.abs(slip_array).mean(axis=0).tolist(),
             "max_absolute_m_s": np.abs(slip_array).max(axis=0).tolist()}
            if len(slip_array) else {})
    slip.update({"wheel_order": wheel_names,
                 "definition": "wheel angular speed * tire radius minus local chassis rigid-body longitudinal velocity at that axle",
                 "interpretation": "diagnostic only; skid steering requires lateral scrub, and this longitudinal residual is not a calibrated tire slip ratio"})
    actuation = np.asarray(command_actuation)
    actuator_metrics = ({"mean_effective_yaw_command_rad_s": float(actuation[:, 0].mean()),
                         "max_absolute_yaw_correction_rad_s": float(np.abs(actuation[:, 1]).max()),
                         "max_absolute_wheel_actuator_torque_nm": np.abs(actuation[:, 2:]).max(axis=0).tolist()}
                        if len(actuation) else {})
    tracker = sim.command_tracker
    tracker_metadata = ({"source": "simulation truth body velocity and heading", "updates": tracker.update_count,
                         "saturation_count": tracker.saturation_count,
                         "correction_saturation_count": tracker.correction_saturation_count} if tracker else None)
    if len(history) > config["max_samples"]:
        history = [history[i] for i in np.linspace(0, len(history) - 1, config["max_samples"], dtype=int)]
    result = {"config": config, "config_sha256": hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
              "actual_controller": sim.controller_name, "model_metadata": sim.controller.metadata,
              "control_contract": {"wheel_controller": "skid steer with bounded wheel servos",
                                   "internal_gyro_yaw_pi": sim.controller.yaw_feedback,
                                   "additional_body_tracker": tracker is not None,
                                   "scope": "Simulator integration control, not manufacturer firmware or calibrated hardware"},
              "actuation": actuator_metrics,
              "deployment_metadata": sim.deployment_metadata, "terrain": profile.metadata,
              "compiled_geometry": geometry, "thresholds": thresholds, "physics_period_s": dt,
              "safety_passed": failure is None or failure == "route_timeout",
              "motion_completed": motion_complete, "route_completed": route_complete if terrain_case else None,
              "tracking": tracking, "parking": parking, "parking_evaluated": bool(parking_t),
              "passed": failure is None and motion_complete and tracking["passed"] and parking["passed"],
              "failure_reason": failure, "failure_phase": failure_phase,
              "failed_acceptance_checks": ([failure] if failure else []) + ([] if tracking["passed"] else ["velocity_tracking"]) + ([] if parking["passed"] else ["parking" if parking_t else "parking_not_evaluated"]),
              "initial_position_m": initial_position.tolist(), "final_position_m": last_finite_position.tolist(),
              "motion_start_position_m": motion_start_position.tolist() if motion_start_position is not None else None,
              "stop_displacement_m": (stop_end_position - stop_start_position).tolist() if stop_end_position is not None else None,
              "max_base_x_m": max_x, "max_absolute_world_y_m": max_lateral, "max_tilt_deg": max_tilt,
              "minimum_sampled_chassis_bottom_gap_m": min_bottom_gap if math.isfinite(min_bottom_gap) else None,
              "clearance_sampling": "9 underside points every 50 ms; vertical gap to upper terrain surface, diagnostic only at risers",
              "non_wheel_contact_names": sorted(contacts), "wheel_slip": slip,
              "wheel_target_saturation_fraction": sim.controller.saturation_fraction,
              "velocity_feedback": tracker_metadata, "resources": {"initialization": initialization,
              "physics_loop": loop_resources, "whole_case": total_meter.report(),
              "scope": "Fresh CPU-only worker, unpaced physics; no ROS, LiDAR, camera, GPU or realtime scheduling claims."},
              "provenance": provenance, "trajectory": history}
    return result


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", default=["all"])
    parser.add_argument("--control-mode", choices=("native", "tracked"), default="native")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--case-json", help=argparse.SUPPRESS)
    parser.add_argument("--case-output", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.case_json:
        try:
            result = evaluate_case(json.loads(args.case_json))
            write_json(args.case_output, result)
        except Exception:
            traceback.print_exc()
            return 2
        return 0
    try:
        cases = make_cases(args.cases, args.control_mode)
    except ValueError as error:
        parser.error(str(error))
    output = args.output or ROOT / "output" / ("scout-mini-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    output.mkdir(parents=True, exist_ok=False)
    summaries = []
    command = [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]]
    for config in cases:
        path = output / (config["case"] + ".json")
        worker = [sys.executable, str(Path(__file__).resolve()), "--case-json", json.dumps(config), "--case-output", str(path)]
        try:
            completed = subprocess.run(worker, capture_output=True, text=True, timeout=180)
            (output / (config["case"] + ".log")).write_text(completed.stdout + completed.stderr)
            if completed.returncode or not path.exists():
                raise RuntimeError(f"Worker exit {completed.returncode}; see {config['case']}.log")
            full = json.loads(path.read_text())
            summary = {key: value for key, value in full.items() if key not in ("trajectory", "provenance", "model_metadata")}
        except (subprocess.TimeoutExpired, RuntimeError) as error:
            summary = {"config": config, "passed": False, "runtime_error": str(error)}
        summaries.append(summary)
        write_json(output / "summary.json", {"command": command, "cases": summaries,
                                             "provenance": source_provenance()})
        print(config["case"], "PASS" if summary["passed"] else "FAIL", flush=True)
    from simple_rbt_mujoco.resources import directory_size
    write_json(output / "disk-usage.json", {"output_before_this_file": directory_size(output)})
    lines = ["# Scout Mini staged physics acceptance", "", "No gains, friction or gates changed between cases.", "",
             "| Case | Target vx / wz | Measured vx / wz | Motion/route | Speed | Stop | Cause |",
             "|---|---|---|---|---|---|---|"]
    for row in summaries:
        flag = lambda value: "pass" if value else "fail"
        target = row["config"]["command_vx_vy_wz"]
        measured = row.get("tracking", {}).get("steady_mean_body_vx_vy_vz_wx_wy_wz")
        actual_text = f"{measured[0]:.3f} / {measured[5]:.3f}" if measured else "unavailable"
        lines.append("| " + " | ".join([row["config"]["case"], f"{target[0]:.3f} / {target[2]:.3f}", actual_text,
             flag(row.get("motion_completed")), flag(row.get("tracking", {}).get("passed")),
             flag(row.get("parking", {}).get("passed")) if row.get("parking_evaluated") else "not run",
             row.get("runtime_error") or ", ".join(row.get("failed_acceptance_checks", [])) or "—"]) + " |")
    lines.extend(["", "Velocity columns are m/s and rad/s; measured values exclude the first second of motion.",
                  "", "Command: `" + shlex.join(command) + "`", "", "Native means the declared skid-steer controller, including its internal gyro yaw PI; it does not mean open loop.",
                  "Tracked additionally enables simulation body-velocity feedback. Neither is manufacturer firmware.",
                  "Model geometry/assumptions and fixed numeric gates are in each JSON. Actual body velocity,",
                  "world heading and longitudinal wheel-slip residuals are measured independently.",
                  "Wheel chassis acceptance uses flat ground, ramps and continuous hills; stairs and step-cell roughness are excluded.",
                  "CPU/RAM costs are fresh-process unpaced physics with native IMU/control. LiDAR/camera/ROS workloads and dependency installation disk usage are excluded."])
    (output / "RESULTS.md").write_text("\n".join(lines) + "\n")
    print(output)
    return 2 if any("runtime_error" in row for row in summaries) else 0


if __name__ == "__main__":
    raise SystemExit(main())
