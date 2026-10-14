#!/usr/bin/env python3
"""Measure complete terrain traversals in fresh processes, without rendering/sensors.

Every result names the requested and actual controller. A short run near the
start never passes: the robot must reach the selected geometry's finish line.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

from project import ROOT, REFERENCE_ROOT
from simple_rbt_mujoco.robots import ROBOT_NAMES, WHEEL_MODELS, WHEELED_ROBOTS, COMMAND_LIMITS, validate_terrain
from simple_rbt_mujoco.terrains import TERRAIN_NAMES
TERRAINS = TERRAIN_NAMES
LEVELS = ("standard", "moderate", "hard", "extreme")
TORQUE_MODES = ("configured", "upstream")
STARTUPS = ("crouched", "upstream_getup")


def source_provenance():
    """Hash the small executable/configuration sources, without walking assets."""
    sources = [Path(__file__).resolve(), *sorted((REFERENCE_ROOT / "simple_rbt_mujoco").glob("*.py")),
               *sorted((ROOT / "assets/manifests").glob("*_manifest.json")),
               *sorted((ROOT / "assets/policies").glob("*.onnx"))]
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sources if path.is_file()}
    head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"],
                          capture_output=True, text=True, check=False)
    return {"worktree_base_commit": head.stdout.strip() if head.returncode == 0 else None,
            "source_sha256": hashes,
            "thread_environment": {name: os.environ.get(name) for name in
                                   ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")}}


def route_budget(config, profile):
    """Keep the standard 8 m allowance and cover longer routes plus 2 m margin."""
    budget = max(config["commanded_distance_budget_m"], profile.finish_x + 2.)
    return budget, config["settle_s"] + budget / config["speed_m_s"]


def course_phase(profile, x):
    """Locate the base along the actual geometry, never a fixed X=6 deadline."""
    if x >= profile.finish_x:
        return "complete"
    if x < profile.entry_x:
        return "approach"
    parameters = profile.parameters
    if profile.name in ("ramp", "stairs"):
        if x < parameters["ascent_end_x_m"]:
            return "ascent"
        if x < parameters["landing_end_x_m"]:
            return "top"
        if x < parameters["descent_end_x_m"]:
            return "descent"
        return "exit"
    if profile.name in ("rough", "hills") and x >= profile.exit_x:
        return "exit"
    return "traversal"


def clearance_observation(profile, position, previous_position, torso_contact):
    """Report pointwise clearance, retaining discontinuities at vertical risers.

    The 0.10 m gate is a conservative base-origin-to-surface distance, not a
    contact/support-polygon test. A robot can fail it without touching terrain.
    At a riser, height_at intentionally chooses the higher adjacent surface.
    """
    x, y, z = map(float, position)
    height = profile.height_at(x, y)
    epsilon = 1e-5
    before = profile.height_at(x - epsilon, y)
    after = profile.height_at(x + epsilon, y)
    previous_height = profile.height_at(float(previous_position[0]), float(previous_position[1]))
    risers = []
    if profile.name == "stairs":
        p = profile.parameters
        count, tread = p["ascending_steps"], p["tread_length_m"]
        risers = ([p["ascent_start_x_m"] + i * tread for i in range(count)] +
                  [p["descent_start_x_m"] + i * tread for i in range(p["descending_risers"])])
    low, high = sorted((float(previous_position[0]), x))
    crossed = [edge for edge in risers if low < edge <= high]
    return {"terrain_height_at_base_xy_m": height, "base_origin_height_m": z,
            "base_clearance_m": z - height, "torso_contact": bool(torso_contact),
            "previous_terrain_height_at_base_xy_m": previous_height,
            "terrain_height_change_since_previous_step_m": height - previous_height,
            "height_before_after_10um_m": [before, after],
            "at_vertical_height_discontinuity": profile.name in ("stairs", "rough") and abs(after - before) > 1e-8,
            "nearest_stair_riser_distance_m": min((abs(x - edge) for edge in risers), default=None),
            "crossed_stair_riser_x_m": crossed,
            "definition": "Vertical gap at base-origin XY; higher surface at an edge. Low gap alone is not proof of falling or torso contact."}


def evaluate_case(config):
    from simple_rbt_mujoco.resources import ResourceMeter
    total_meter = ResourceMeter()
    import mujoco
    import numpy as np
    from simple_rbt_mujoco.core import Simulation

    deployment_options = {}
    if config["robot"] == "go2":
        deployment_options = {"go2_torque_mode": config.get("go2_torque_mode", "configured"),
                              "go2_startup": config.get("go2_startup", "crouched")}
    sim = Simulation(config["robot"], terrain=config["terrain"],
                     difficulty=config["difficulty"], seed=config["seed"],
                     terrain_level=config.get("terrain_level", "standard"),
                     terrain_friction=config.get("terrain_friction"),
                     controller=config["controller"],
                     velocity_feedback=config.get("control_mode", "native") == "tracked",
                     **deployment_options)
    initialization = total_meter.report()
    profile = sim.terrain
    speed, settle = config["speed_m_s"], config["settle_s"]
    effective_budget, maximum_duration = route_budget(config, profile)
    thresholds = {"finish_x_m": profile.finish_x, "max_tilt_deg": 50.,
                  "minimum_base_clearance_m": .10,
                  "corridor_half_width_m": profile.width / 2. - .25,
                  "corridor_active_x_m": [profile.entry_x, profile.finish_x],
                  "corridor_enforcement": "from first entry through completion, including any backtracking",
                  "torso_environment_contact_is_failure": True,
                  "maximum_simulation_duration_s": maximum_duration,
                  "requested_distance_budget_m": config["commanded_distance_budget_m"],
                  "effective_distance_budget_m": effective_budget,
                  "distance_budget_rule": "max(requested budget, selected finish_x + 2 m)",
                  "settle_s": settle}
    base_id = int(sim.model.site_bodyid[sim.model.site("base_origin").id])
    support_geom_ids = set()
    if config["robot"] in WHEELED_ROBOTS:
        from simple_rbt_mujoco.wheel_geometry import caster_contact_geom_ids
        support_geom_ids = set(caster_contact_geom_ids(sim.model))
    initial_position, initial_quaternion = sim.pose()
    R = sim.data.site_xmat[sim.model.site("base_origin").id].reshape(3, 3)
    last_yaw = math.atan2(R[1, 0], R[0, 0])
    yaw_drift, max_yaw, max_lateral = 0., 0., abs(float(initial_position[1]))
    max_roll, max_pitch, max_tilt = 0., 0., 0.
    maximum_x = float(initial_position[0])
    entered_course = maximum_x >= profile.entry_x
    min_clearance = math.inf
    torso_contact_steps, torso_contact_names = 0, set()
    velocities, velocity_stamps, history = [], [], []
    tracker_start = None
    failure, error = None, None
    success = False
    last_position, last_quaternion = initial_position.copy(), initial_quaternion.copy()
    last_finite_simulation_seconds = float(sim.data.time)
    initially_ready = bool(getattr(sim, "startup_complete", True))
    evaluation_origin = float(sim.data.time) if initially_ready else None
    evaluation_start_step = sim.steps if initially_ready else None
    startup_start_seconds = float(sim.data.time)
    startup_end_position = initial_position.copy() if initially_ready else None
    startup_end_quaternion = initial_quaternion.copy() if initially_ready else None
    startup_timeout = config.get("startup_timeout_s", 10.)
    startup_contacts, startup_contact_names = 0, set()
    startup_min_clearance, startup_max_tilt = math.inf, 0.
    startup_yaw_drift = 0.
    first_command_sim_time = None
    last_control_phase = "settle" if initially_ready else "startup"
    last_phase = course_phase(profile, float(initial_position[0]))
    phase_order = {"approach": 0, "ascent": 1, "traversal": 1, "top": 2,
                   "descent": 3, "exit": 4, "complete": 5}
    furthest_phase = last_phase
    phase_transitions = [{"phase": last_phase, "stamp_ns": sim.stamp_ns,
                          "base_x_m": float(initial_position[0])}]
    final_clearance_observation = None
    sample_every = max(1, round(.1 / sim.model.opt.timestep))
    loop_meter = ResourceMeter()
    startup_meter = ResourceMeter() if not initially_ready else None
    startup_resources = None
    poststartup_meter = ResourceMeter() if initially_ready else None
    settle_meter = ResourceMeter() if initially_ready else None
    settle_resources = None
    budget_steps = math.ceil(maximum_duration / sim.model.opt.timestep)
    startup_max_steps = math.ceil(startup_timeout / sim.model.opt.timestep) if not initially_ready else 0
    for _ in range(budget_steps + startup_max_steps):
        if evaluation_start_step is not None and sim.steps - evaluation_start_step >= budget_steps:
            break
        was_startup = evaluation_origin is None
        evaluation_elapsed = None if was_startup else float(sim.data.time) - evaluation_origin
        commanding = not was_startup and evaluation_elapsed >= settle
        last_control_phase = "startup" if was_startup else "command" if commanding else "settle"
        if commanding and first_command_sim_time is None:
            first_command_sim_time = float(sim.data.time)
            settle_resources = settle_meter.report()
        if sim.command_tracker and tracker_start is None and commanding:
            tracker_start = (sim.command_tracker.update_count,
                             sim.command_tracker.saturation_count,
                             sim.command_tracker.correction_saturation_count,
                             sim.command_tracker.saturated_axes.copy())
        sim.set_command(speed if commanding else 0., 0., 0.)
        try:
            sim.step()
        except Exception as caught:
            failure, error = ("startup_simulation_error" if was_startup else "simulation_error"), f"{type(caught).__name__}: {caught}"
            break
        pos, quat = sim.pose()
        linear, angular = sim.velocity()
        R = sim.data.site_xmat[sim.model.site("base_origin").id].reshape(3, 3)
        if not np.isfinite(np.r_[pos, quat, linear, angular, R.ravel(), sim.data.time]).all():
            failure = "startup_nonfinite_state" if was_startup else "nonfinite_state"
            error = "Nonfinite pose, velocity, rotation or simulation time; final pose below is the last finite state"
            break
        previous_position = last_position.copy()
        last_position, last_quaternion = pos.copy(), quat.copy()
        last_finite_simulation_seconds = float(sim.data.time)
        current_phase = course_phase(profile, float(pos[0]))
        if current_phase != last_phase:
            phase_transitions.append({"phase": current_phase, "stamp_ns": sim.stamp_ns,
                                      "base_x_m": float(pos[0])})
        last_phase = current_phase
        if phase_order[current_phase] > phase_order[furthest_phase]:
            furthest_phase = current_phase
        maximum_x = max(maximum_x, float(pos[0]))
        entered_course = entered_course or pos[0] >= profile.entry_x
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.asin(float(np.clip(-R[2, 0], -1., 1.)))
        tilt = math.acos(float(np.clip(R[2, 2], -1., 1.)))
        yaw = math.atan2(R[1, 0], R[0, 0])
        yaw_change = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
        if was_startup:
            startup_yaw_drift += yaw_change
        else:
            yaw_drift += yaw_change
        last_yaw = yaw
        terrain_height = profile.height_at(float(pos[0]), float(pos[1]))
        if not math.isfinite(terrain_height):
            failure, error = "nonfinite_terrain_height", "Terrain returned a nonfinite surface height"
            break
        clearance = float(pos[2]) - terrain_height
        if was_startup:
            startup_min_clearance = min(startup_min_clearance, clearance)
            startup_max_tilt = max(startup_max_tilt, math.degrees(tilt))
        else:
            min_clearance = min(min_clearance, clearance)
            max_roll, max_pitch = max(max_roll, abs(roll)), max(max_pitch, abs(pitch))
            max_tilt, max_yaw = max(max_tilt, tilt), max(max_yaw, abs(yaw_drift))
            max_lateral = max(max_lateral, abs(float(pos[1])))
        torso_touching = False
        for contact in sim.data.contact:
            a, b = int(contact.geom1), int(contact.geom2)
            body_a, body_b = int(sim.model.geom_bodyid[a]), int(sim.model.geom_bodyid[b])
            robot_geom = a if body_a == base_id else b
            if {body_a, body_b} == {0, base_id} and robot_geom not in support_geom_ids:
                torso_touching = True
                names = startup_contact_names if was_startup else torso_contact_names
                names.add(sim.model.geom(a if body_a == base_id else b).name or "unnamed")
        if was_startup:
            startup_contacts += int(torso_touching)
        else:
            torso_contact_steps += int(torso_touching)
        if not was_startup and sim.data.time - evaluation_origin >= settle:
            velocities.append([*linear, *angular])
            velocity_stamps.append(sim.stamp_ns)
        if was_startup:
            if math.degrees(tilt) > thresholds["max_tilt_deg"]:
                failure = "startup_tilt_limit"
            elif bool(getattr(sim, "startup_complete", True)):
                evaluation_origin, evaluation_start_step = float(sim.data.time), sim.steps
                startup_end_position, startup_end_quaternion = pos.copy(), quat.copy()
                startup_resources = startup_meter.report()
                poststartup_meter, settle_meter = ResourceMeter(), ResourceMeter()
            elif sim.data.time - startup_start_seconds >= startup_timeout:
                failure = "startup_timeout"
        elif math.degrees(tilt) > thresholds["max_tilt_deg"]:
            failure = "tilt_limit"
        elif clearance < thresholds["minimum_base_clearance_m"]:
            failure = "base_clearance_limit"
        elif torso_touching:
            failure = "torso_environment_contact"
        elif entered_course and abs(pos[1]) > thresholds["corridor_half_width_m"]:
            failure = "left_course_corridor"
        elif pos[0] >= profile.finish_x:
            success = True
        if failure or success:
            final_clearance_observation = clearance_observation(profile, pos, previous_position, torso_touching)
        if sim.steps % sample_every == 0 or failure or success:
            history.append({"stamp_ns": sim.stamp_ns, "position_m": pos.tolist(),
                            "quaternion_xyzw": quat.tolist(),
                            "linear_velocity_base_m_s": linear.tolist(),
                            "angular_velocity_base_rad_s": angular.tolist(),
                            "command_m_s_rad_s": sim.command.tolist(),
                            "policy_command_m_s_rad_s": sim.policy_command.tolist(),
                            "terrain_height_m": terrain_height,
                            "base_clearance_m": clearance, "tilt_deg": math.degrees(tilt),
                            "torso_contact": torso_touching, "course_phase": current_phase,
                            "control_phase": last_control_phase,
                            "startup_phase": getattr(sim, "startup_phase", "complete"),
                            "evaluation_elapsed_s": float(sim.data.time) - evaluation_origin if evaluation_origin is not None else None})
        if failure or success:
            break
    loop_resources = loop_meter.report()
    if startup_meter and startup_resources is None:
        startup_resources = startup_meter.report()
    poststartup_resources = poststartup_meter.report() if poststartup_meter else None
    if settle_meter and settle_resources is None:
        settle_resources = settle_meter.report()
    if not success and failure is None:
        failure = "startup_timeout" if evaluation_origin is None else "time_limit_before_finish"
    if len(history) > config["max_samples"]:
        indices = np.linspace(0, len(history) - 1, config["max_samples"], dtype=int)
        history = [history[index] for index in indices]
    tracking = {"scope": "every physics step after post-startup settle through finish/failure",
                "samples": len(velocities), "command_base_m_s_rad_s": [speed, 0., 0.],
                "velocity_fields": ["vx_m_s", "vy_m_s", "vz_m_s", "wx_rad_s", "wy_rad_s", "wz_rad_s"]}
    tracking_thresholds = {"mean_forward_error_m_s": max(.05, .15 * speed),
                           "mean_absolute_lateral_velocity_m_s": .1,
                           "mean_absolute_yaw_rate_rad_s": .1,
                           "minimum_tracking_window_s": 1.,
                           "rmse_is_a_pass_criterion": False}
    speed_tracking_passed = None
    if velocities:
        measured = np.array(velocities)
        target = np.array([speed, 0., 0.])
        mean_forward_error = abs(float(measured[:, 0].mean()) - speed)
        mean_abs_lateral = float(np.abs(measured[:, 1]).mean())
        mean_abs_yaw_rate = float(np.abs(measured[:, 5]).mean())
        tracking_seconds = len(measured) * sim.model.opt.timestep
        if tracking_seconds >= tracking_thresholds["minimum_tracking_window_s"]:
            speed_tracking_passed = bool(
                mean_forward_error <= tracking_thresholds["mean_forward_error_m_s"]
                and mean_abs_lateral <= tracking_thresholds["mean_absolute_lateral_velocity_m_s"]
                and mean_abs_yaw_rate <= tracking_thresholds["mean_absolute_yaw_rate_rad_s"])
        tracking.update({
            "first_stamp_ns": velocity_stamps[0], "last_stamp_ns": velocity_stamps[-1],
            "tracking_seconds": tracking_seconds, "mean_forward_error_m_s": mean_forward_error,
            "mean_absolute_lateral_velocity_m_s": mean_abs_lateral,
            "mean_absolute_yaw_rate_rad_s": mean_abs_yaw_rate,
            "mean_velocity_base_m_s_rad_s": measured.mean(axis=0).tolist(),
            "std_velocity_base_m_s_rad_s": measured.std(axis=0).tolist(),
            "linear_rmse_by_axis_m_s": np.sqrt(np.mean((measured[:, :3] - target) ** 2, axis=0)).tolist(),
            "forward_rmse_m_s": float(np.sqrt(np.mean((measured[:, 0] - speed) ** 2))),
            "yaw_rate_rmse_rad_s": float(np.sqrt(np.mean(measured[:, 5] ** 2))),
        })
    else:
        tracking["note"] = "Case ended before the command/tracking interval"
    tracker_stats = None
    if sim.command_tracker:
        tracker = sim.command_tracker
        tracker_stats = {"scope": "entire simulation including settle",
                         "updates": tracker.update_count,
                         "command_saturation_fraction": tracker.saturation_fraction,
                         "command_saturated_steps": tracker.saturation_count,
                         "command_saturated_axes_vx_vy_wz": tracker.saturated_axes.tolist(),
                         "correction_saturated_steps": tracker.correction_saturation_count,
                         "command_limits": tracker.command_limits.tolist(),
                         "correction_limits": tracker.correction_limits.tolist()}
        if tracker_start is not None:
            updates = tracker.update_count - tracker_start[0]
            tracker_stats["post_settle"] = {
                "updates": updates,
                "command_saturation_fraction": (tracker.saturation_count - tracker_start[1]) / max(1, updates),
                "command_saturated_axes_vx_vy_wz": (tracker.saturated_axes - tracker_start[3]).tolist(),
                "correction_saturation_fraction": (tracker.correction_saturation_count - tracker_start[2]) / max(1, updates)}
    whole_process = total_meter.report()
    simulation_seconds = float(sim.data.time) if math.isfinite(sim.data.time) else None
    startup_last_position = startup_end_position if startup_end_position is not None else last_position
    return {
        "case": config, "actual_controller": sim.controller_name,
        "deployment": getattr(sim, "deployment_metadata", {}),
        "source_provenance": source_provenance(),
        "control_mode": "tracked" if sim.command_tracker else "native",
        "control_feedback_source": "ideal simulated body velocity and world heading" if sim.command_tracker else None,
        "control_feedback_scope": "Outer body velocity/heading tracker; controller feedback is recorded in deployment",
        "feedback_limitation": "Simulator state feedback is not a verified real-robot estimator" if sim.command_tracker else None,
        "tracker_saturation": tracker_stats,
        "mujoco_version": mujoco.__version__, "terrain": profile.metadata,
        "terrain_level": config.get("terrain_level", "standard"),
        "terrain_friction_requested": config.get("terrain_friction"),
        "frames": {"position": "world", "velocity": "base_origin", "quaternion_order": "xyzw"},
        "thresholds": thresholds, "success": success, "failure_reason": failure,
        "speed_tracking_passed": speed_tracking_passed,
        "speed_tracking_thresholds": tracking_thresholds,
        "error": error, "simulation_seconds": simulation_seconds,
        "last_finite_simulation_seconds": last_finite_simulation_seconds,
        "timing": {"evaluation_clock_origin_sim_s": evaluation_origin,
                   "evaluation_elapsed_s": last_finite_simulation_seconds - evaluation_origin if evaluation_origin is not None else None,
                   "absolute_deadline_sim_s": evaluation_origin + maximum_duration if evaluation_origin is not None else None,
                   "first_command_sim_time_s": first_command_sim_time,
                   "actual_settle_elapsed_s": first_command_sim_time - evaluation_origin if first_command_sim_time is not None else None,
                   "route_threshold_clock": "elapsed simulation time after startup completion; GetUp consumes neither settle nor route budget",
                   "clock_policy": "absolute MuJoCo time and pose are never reset; command transition is quantized to a physics step"},
        "startup": {"requested": config.get("go2_startup", "crouched") if config["robot"] == "go2" else None,
                    "completed": evaluation_origin is not None,
                    "controller_reported_complete": bool(getattr(sim, "startup_complete", True)),
                    "phase": getattr(sim, "startup_phase", "complete"),
                    "steps": int(getattr(sim, "startup_steps", 0)),
                    "elapsed_s": float(getattr(sim, "startup_elapsed_s", 0.)),
                    "absolute_start_sim_s": startup_start_seconds,
                    "absolute_end_sim_s": evaluation_origin,
                    "watchdog_s": startup_timeout,
                    "failure_reason": failure if last_control_phase == "startup" else None,
                    "displacement_world_m": (startup_last_position - initial_position).tolist(),
                    "end_position_m": startup_end_position.tolist() if startup_end_position is not None else None,
                    "end_quaternion_xyzw": startup_end_quaternion.tolist() if startup_end_quaternion is not None else None,
                    "yaw_drift_deg": math.degrees(startup_yaw_drift),
                    "max_tilt_deg": startup_max_tilt,
                    "minimum_base_clearance_m": startup_min_clearance if math.isfinite(startup_min_clearance) else None,
                    "torso_contact_steps": startup_contacts,
                    "torso_contact_geom_names": sorted(startup_contact_names),
                    "gates": "During GetUp: finite state, tilt <=50 degrees, and startup watchdog. Contact/clearance are recorded; original route gates start on the first subsequent policy step."},
        "initial_position_m": initial_position.tolist(),
        "final_position_m": last_position.tolist(), "final_quaternion_xyzw": last_quaternion.tolist(),
        "progress_x_m": float(last_position[0] - initial_position[0]),
        "maximum_x_m": maximum_x, "reached_terrain_entry": bool(entered_course),
        "progress": {"final_phase": last_phase, "furthest_phase": furthest_phase,
                     "failure_phase": last_phase if failure else None,
                     "failure_control_phase": last_control_phase if failure else None,
                     "reason": failure or "reached_geometry_finish",
                     "course_fraction_from_entry": float(np.clip((maximum_x - profile.entry_x) /
                                                                  (profile.finish_x - profile.entry_x), 0., 1.)),
                     "phase_transitions": phase_transitions,
                     "geometry_boundaries": {key: value for key, value in profile.parameters.items()
                                             if key.endswith("_x_m")}},
        "max_abs_lateral_position_m": max_lateral, "final_yaw_drift_deg": math.degrees(yaw_drift),
        "max_abs_yaw_drift_deg": math.degrees(max_yaw), "max_abs_roll_deg": math.degrees(max_roll),
        "max_abs_pitch_deg": math.degrees(max_pitch), "max_tilt_deg": math.degrees(max_tilt),
        "minimum_base_clearance_m": min_clearance if math.isfinite(min_clearance) else None,
        "fall_detected": failure in ("tilt_limit", "base_clearance_limit", "torso_environment_contact"),
        "fall_detected_definition": "Legacy safety-gate classification; clearance alone does not establish a physical fall",
        "actual_tilt_or_contact_failure": math.degrees(max_tilt) > thresholds["max_tilt_deg"] or torso_contact_steps > 0,
        "route_metric_scope": "after startup completion; separate startup metrics retain GetUp contact, clearance and tilt",
        "clearance_gate_triggered": failure == "base_clearance_limit",
        "safety_gate_triggered": failure in ("tilt_limit", "base_clearance_limit", "torso_environment_contact", "left_course_corridor"),
        "clearance_observation": final_clearance_observation,
        "clearance_observation_scope": "At a safety gate or successful finish; null for timeout or an invalid/error state",
        "torso_contact_steps": torso_contact_steps, "torso_contact_geom_names": sorted(torso_contact_names),
        "velocity_tracking": tracking, "initialization_resources": initialization,
        "loop_resources": loop_resources, "whole_case_resources": whole_process,
        "startup_resources": startup_resources, "post_startup_resources": poststartup_resources,
        "settle_resources": settle_resources,
        "loop_real_time_factor": simulation_seconds / loop_resources["wall_seconds"] if simulation_seconds is not None else None,
        "resource_scope": "fresh process; physics, policy and evaluator checks; no LiDAR, ROS or rendering",
        "trajectory_sampling": {"retained": len(history), "max_retained": config["max_samples"],
                                "original_sample_interval_s": sample_every * sim.model.opt.timestep,
                                "downsampling": "fixed cadence plus final failure/completion; uniform subsampling when over cap"},
        "trajectory": history,
    }


def _write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", nargs="+", choices=ROBOT_NAMES, default=["go2", "go2w"])
    parser.add_argument("--go2-controllers", nargs="+", choices=("legacy", "himloco", "robot_lab"), default=["robot_lab"])
    parser.add_argument("--go2-torque-modes", nargs="+", choices=TORQUE_MODES, default=["configured"])
    parser.add_argument("--go2-startups", nargs="+", choices=STARTUPS, default=["crouched"])
    parser.add_argument("--control-modes", nargs="+", choices=("native", "tracked"), default=["native"])
    parser.add_argument("--terrains", nargs="+", choices=TERRAINS,
                        help="Default: flat/ramp/hills for wheel batches; legacy four terrains for legged-only batches")
    parser.add_argument("--levels", nargs="+", choices=LEVELS, default=["standard"])
    parser.add_argument("--speeds", nargs="+", type=float, default=[.3, .6, 1.])
    parser.add_argument("--rough-seeds", nargs="+", type=int, default=[0, 1])
    parser.add_argument("--frictions", nargs="+", type=float, default=[None],
                        help="Terrain sliding friction values; omitted preserves the baseline materials")
    parser.add_argument("--difficulty", type=float, default=1.)
    parser.add_argument("--settle", type=float, default=2.)
    parser.add_argument("--startup-timeout", type=float, default=10., help="Separate GetUp watchdog in simulation seconds")
    parser.add_argument("--distance-budget", type=float, default=8., help="Timeout budget in commanded metres; effective budget is max(requested, finish_x+2 m)")
    parser.add_argument("--max-samples", type=int, default=200)
    parser.add_argument("--wall-timeout", type=float, default=180., help="Maximum wall seconds for one isolated case")
    parser.add_argument("--output", type=Path, help="New output directory, must not already exist")
    parser.add_argument("--case-json", help=argparse.SUPPRESS)
    parser.add_argument("--case-output", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.case_json is not None:
        if args.case_output is None or args.case_output.exists():
            parser.error("Internal worker requires a new case output path")
        config = json.loads(args.case_json)
        try:
            result = evaluate_case(config)
        except Exception as error:
            result = {"case": config, "actual_controller": None, "success": False,
                      "failure_reason": "initialization_or_evaluation_error",
                      "error": f"{type(error).__name__}: {error}", "traceback": traceback.format_exc()}
        _write_json(args.case_output, result)
        return 0
    if args.terrains is None:
        args.terrains = ["flat", "ramp", "hills"] if any(robot in WHEELED_ROBOTS for robot in args.robots) else ["flat", "ramp", "stairs", "rough"]
    try:
        for robot in args.robots:
            for terrain in args.terrains:
                validate_terrain(robot, terrain)
    except ValueError as error:
        parser.error(str(error))
    max_speed = min(COMMAND_LIMITS[robot][0] for robot in args.robots)
    if (not math.isfinite(args.difficulty) or not 0 < args.difficulty <= 3
            or not all(math.isfinite(v) and 0 < v <= max_speed for v in args.speeds)
            or not math.isfinite(args.settle) or args.settle < 0
            or not math.isfinite(args.distance_budget) or args.distance_budget <= 0
            or args.max_samples < 2 or min(args.rough_seeds) < 0
            or not all(v is None or math.isfinite(v) and 0 < v <= 2 for v in args.frictions)
            or not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0
            or not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0):
        parser.error(f"Require difficulty in (0,3], speeds in (0,{max_speed:g}], friction in (0,2], nonnegative settle/seeds and positive budgets")
    if any(level != "standard" for level in args.levels) and args.difficulty != 1.:
        parser.error("Named terrain levels require --difficulty 1")
    if ("go2" in args.robots and "legacy" in args.go2_controllers and
            (args.go2_torque_modes != ["configured"] or args.go2_startups != ["crouched"])):
        parser.error("Go2 deployment factors support himloco/robot_lab; legacy accepts baseline configured/crouched only")
    output = args.output or ROOT / "output" / ("terrain-evaluation-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    output.mkdir(parents=True, exist_ok=False)
    cases_dir = output / "cases"
    cases_dir.mkdir()
    configs = []
    for robot in dict.fromkeys(args.robots):
        controllers = (list(dict.fromkeys(args.go2_controllers)) if robot == "go2" else
                       [WHEEL_MODELS[robot][3]] if robot in WHEEL_MODELS else
                       ["auto"] if robot == "diffdrive" else ["rl_sar"])
        deployments = list(itertools.product(dict.fromkeys(args.go2_torque_modes), dict.fromkeys(args.go2_startups))) if robot == "go2" else [(None, None)]
        for controller, mode, terrain, level, speed, friction, deployment in itertools.product(
                controllers, dict.fromkeys(args.control_modes), dict.fromkeys(args.terrains),
                dict.fromkeys(args.levels), dict.fromkeys(args.speeds), dict.fromkeys(args.frictions), deployments):
            for seed in (dict.fromkeys(args.rough_seeds) if terrain == "rough" else [0]):
                configs.append({"robot": robot, "controller": controller, "terrain": terrain, "terrain_level": level,
                                "speed_m_s": speed, "difficulty": args.difficulty, "seed": seed, "control_mode": mode,
                                "terrain_friction": friction,
                                "go2_torque_mode": deployment[0], "go2_startup": deployment[1],
                                "startup_timeout_s": args.startup_timeout,
                                "settle_s": args.settle, "commanded_distance_budget_m": args.distance_budget,
                                "max_samples": args.max_samples})
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "purpose": "Full-course locomotion and per-case fresh-process CPU/RAM evaluation",
              "controller_selection": "Explicit candidates; no automatic winner or controller substitution",
              "source_provenance": source_provenance(),
              "argv": [sys.executable, str(Path(__file__).resolve()), *(sys.argv[1:] if argv is None else argv)],
              "planned_cases": len(configs), "cases": []}
    for index, config in enumerate(configs):
        stem = f"{index:03d}-{config['robot']}-{config['controller']}-{config['control_mode']}-{config['terrain']}-{config['terrain_level']}-v{config['speed_m_s']:g}-seed{config['seed']}"
        if config["robot"] == "go2":
            stem += f"-torque-{config['go2_torque_mode']}-start-{config['go2_startup']}"
        if config["terrain_friction"] is not None:
            stem += f"-friction-{config['terrain_friction']:g}"
        path, log = cases_dir / (stem + ".json"), cases_dir / (stem + ".log")
        command = [sys.executable, str(Path(__file__).resolve()), "--case-json", json.dumps(config), "--case-output", str(path)]
        started = time.perf_counter()
        try:
            completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, timeout=args.wall_timeout, check=False)
            log.write_text(completed.stdout)
            result = json.loads(path.read_text()) if path.exists() else {
                "case": config, "success": False, "failure_reason": "worker_exit_without_result",
                "worker_returncode": completed.returncode}
        except subprocess.TimeoutExpired as error:
            text = error.stdout or ""
            log.write_text(text.decode(errors="replace") if isinstance(text, bytes) else text)
            result = {"case": config, "success": False, "failure_reason": "worker_wall_timeout",
                      "timeout_wall_seconds": args.wall_timeout}
        if not path.exists():
            _write_json(path, result)
        entry = {key: value for key, value in result.items() if key not in ("trajectory", "traceback")}
        entry.update({"result_file": str(path.relative_to(output)), "log_file": str(log.relative_to(output)),
                      "worker_total_wall_seconds": time.perf_counter() - started})
        report["cases"].append(entry)
        report["completed_cases"] = len(report["cases"])
        report["successful_cases"] = sum(bool(case["success"]) for case in report["cases"])
        report["speed_tracking_evaluated_cases"] = sum(case.get("speed_tracking_passed") is not None for case in report["cases"])
        report["speed_tracking_passed_cases"] = sum(case.get("speed_tracking_passed") is True for case in report["cases"])
        _write_json(output / "summary.json", report)
        print(f"[{index + 1}/{len(configs)}] {stem}: {'PASS' if result['success'] else result.get('failure_reason', 'FAIL')}", flush=True)
    print(json.dumps({"report": str(output / "summary.json"),
                      "completed": report["completed_cases"], "passed": report["successful_cases"]}, indent=2))
    runtime_failures = {"initialization_or_evaluation_error", "worker_exit_without_result", "worker_wall_timeout"}
    return 2 if any(case.get("failure_reason") in runtime_failures for case in report["cases"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
