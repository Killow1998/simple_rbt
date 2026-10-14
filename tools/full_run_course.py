"""Python oracle for physical route, speed and effective-contact gates."""
from collections import Counter
import math
import numpy as np
from evaluate_terrains import clearance_observation, course_phase, route_budget


def tracking_result(velocities, speed, timestep):
    thresholds = {"mean_forward_error_m_s": max(.05, .15 * speed),
                  "mean_absolute_lateral_velocity_m_s": .1,
                  "mean_absolute_yaw_rate_rad_s": .1,
                  "minimum_tracking_window_s": 1., "rmse_is_a_pass_criterion": False}
    result = {"thresholds": thresholds, "samples": len(velocities),
              "window": "after settle through formal finish/failure; excludes deadman tail",
              "passed": False}
    if not velocities:
        return result
    values = np.asarray(velocities, dtype=float)
    error = abs(float(values[:, 0].mean()) - speed)
    lateral = float(np.abs(values[:, 1]).mean())
    yaw = float(np.abs(values[:, 5]).mean())
    seconds = len(values) * timestep
    result.update(tracking_seconds=seconds, mean_forward_error_m_s=error,
                  mean_absolute_lateral_velocity_m_s=lateral,
                  mean_absolute_yaw_rate_rad_s=yaw,
                  mean_velocity_base_m_s_rad_s=values.mean(axis=0).tolist(),
                  linear_rmse_by_axis_m_s=np.sqrt(np.mean((values[:, :3] - [speed, 0, 0]) ** 2, axis=0)).tolist(),
                  yaw_rate_rmse_rad_s=float(np.sqrt(np.mean(values[:, 5] ** 2))))
    result["passed"] = bool(seconds >= 1. and error <= thresholds["mean_forward_error_m_s"]
                            and lateral <= .1 and yaw <= .1)
    return result


class CourseMonitor:
    """The evaluator's unchanged gates, checked at every physics step."""

    def __init__(self, profile, config, initial_position, timestep):
        self.profile, self.config, self.timestep = profile, config, timestep
        budget, duration = route_budget(config, profile)
        self.wheel_safety = "robot_half_width_m" in config
        self.thresholds = {"finish_x_m": profile.finish_x, "max_tilt_deg": 45. if self.wheel_safety else 50.,
                           "minimum_base_clearance_m": None if self.wheel_safety else .1,
                           "base_clearance_is_diagnostic_only": self.wheel_safety,
                           "corridor_half_width_m": (profile.width / 2. - config["robot_half_width_m"] - .1
                                if "robot_half_width_m" in config else profile.width / 2. - .25),
                           "corridor_enforcement": "after first entry, including backtracking",
                           "torso_environment_contact_is_failure": True,
                           "maximum_simulation_duration_s": duration,
                           "effective_distance_budget_m": budget}
        self.success, self.failure = False, None
        self.min_corridor_half_width = self.thresholds["corridor_half_width_m"]
        self.finish_stamp_ns = None
        self.previous_position = np.array(initial_position, dtype=float)
        self.entered = initial_position[0] >= profile.entry_x
        self.phase = course_phase(profile, float(initial_position[0]))
        self.transitions = [{"phase": self.phase, "stamp_ns": 0, "base_x_m": float(initial_position[0])}]
        self.velocities = []
        self.min_clearance = math.inf
        self.max_tilt = 0.
        self.contact_steps = 0
        self.last = None
        self.final_clearance = None

    def update(self, stamp_ns, position, quaternion, rotation, linear, angular, torso_names,
               robot_half_width_m=None):
        if self.success or self.failure:
            return
        values = np.r_[position, quaternion, rotation.ravel(), linear, angular]
        if not np.isfinite(values).all():
            self.failure = "nonfinite_state"
            return
        t = stamp_ns * 1e-9
        if self.wheel_safety and robot_half_width_m is not None:
            self.thresholds["corridor_half_width_m"] = self.profile.width / 2. - robot_half_width_m - .1
            self.min_corridor_half_width = min(self.min_corridor_half_width,
                                              self.thresholds["corridor_half_width_m"])
        height = self.profile.height_at(position[0], position[1])
        if not math.isfinite(height):
            self.failure = "nonfinite_terrain_height"
            return
        clearance = float(position[2] - height)
        tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1., 1.))))
        self.min_clearance = min(self.min_clearance, clearance)
        self.max_tilt = max(self.max_tilt, tilt)
        self.entered = bool(self.entered or position[0] >= self.profile.entry_x)
        self.contact_steps += bool(torso_names)
        phase = course_phase(self.profile, float(position[0]))
        if phase != self.phase:
            self.transitions.append({"phase": phase, "stamp_ns": stamp_ns, "base_x_m": float(position[0])})
        self.phase = phase
        if t >= self.config["settle_s"]:
            self.velocities.append([*linear, *angular])
        if tilt > self.thresholds["max_tilt_deg"]:
            self.failure = "tilt_limit"
        elif not self.wheel_safety and clearance < .1:
            self.failure = "base_clearance_limit"
        elif torso_names:
            self.failure = "torso_environment_contact"
        elif self.entered and abs(position[1]) > self.thresholds["corridor_half_width_m"]:
            self.failure = "left_course_corridor"
        elif position[0] >= self.profile.finish_x:
            self.success, self.finish_stamp_ns = True, stamp_ns
        elif t + 1e-12 >= self.thresholds["maximum_simulation_duration_s"]:
            self.failure = "time_limit_before_finish"
        self.last = {"stamp_ns": stamp_ns, "position_m": np.asarray(position).tolist(),
                     "tilt_deg": tilt, "terrain_height_m": float(height),
                     "base_clearance_m": clearance, "torso_contact_geoms": torso_names,
                     "course_phase": phase}
        if self.success or self.failure:
            self.final_clearance = clearance_observation(self.profile, position, self.previous_position, bool(torso_names))
        self.previous_position = np.array(position, copy=True)

    def report(self):
        return {"success": self.success, "failure_reason": self.failure,
                "finish_stamp_ns": self.finish_stamp_ns, "thresholds": self.thresholds,
                "phase_transitions": self.transitions, "final": self.last,
                "minimum_base_clearance_m": self.min_clearance if math.isfinite(self.min_clearance) else None,
                "max_tilt_deg": self.max_tilt, "torso_contact_steps": self.contact_steps,
                "minimum_corridor_half_width_m": self.min_corridor_half_width,
                "final_clearance_observation": self.final_clearance,
                "failure_interpretation": "Safety-gate failures do not all mean a physical fall; the reason and actual contact are recorded.",
                "speed_tracking": tracking_result(self.velocities, self.config["speed_m_s"], self.timestep)}


class FrictionMonitor:
    def __init__(self, requested):
        self.requested = requested
        self.count = 0
        self.minimum = np.full(5, np.inf)
        self.maximum = np.full(5, -np.inf)
        self.pairs = Counter()
        self.by_pair = {}
        self.mismatches = 0

    def observe(self, coefficients, pair):
        values = np.asarray(coefficients, dtype=float)
        self.count += 1
        self.minimum = np.minimum(self.minimum, values)
        self.maximum = np.maximum(self.maximum, values)
        self.pairs[pair] += 1
        pair_stats = self.by_pair.setdefault(pair, {"minimum": values.copy(), "maximum": values.copy()})
        pair_stats["minimum"] = np.minimum(pair_stats["minimum"], values)
        pair_stats["maximum"] = np.maximum(pair_stats["maximum"], values)
        if self.requested is not None and not np.allclose(values, [self.requested, self.requested, .02, .01, .01], rtol=0, atol=1e-10):
            self.mismatches += 1

    def report(self):
        return {"requested_mu": self.requested, "mixing": "source defaults" if self.requested is None else "environment priority2, condim6, (mu,.02,.01)",
                "observed_contact_samples": self.count, "mismatched_samples": self.mismatches,
                "effective_friction_min": self.minimum.tolist() if self.count else None,
                "effective_friction_max": self.maximum.tolist() if self.count else None,
                "contact_pairs": {name: {"samples": count,
                    "effective_friction_min": self.by_pair[name]["minimum"].tolist(),
                    "effective_friction_max": self.by_pair[name]["maximum"].tolist()}
                    for name, count in self.pairs.items()},
                "passed": self.count > 0 and not self.mismatches}
