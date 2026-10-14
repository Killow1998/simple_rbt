"""Optional simulator velocity feedback, separate from each frozen native policy.

This aid uses ideal simulated body velocity and heading. It is NOT a faithful
hardware estimator, native-policy score, or change to the learned weights. Only
the three ordinary policy commands are adjusted: no scene force, pose override,
contact edit or teleport is used. Saturation is observable instead of hidden.
"""
from __future__ import annotations

import math
import numpy as np


def _angle_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class CommandTracker:
    """Bounded PI velocity correction plus heading hold for zero-yaw commands.

    Defaults are simulator integration gains, not robot calibration. Measured
    vx/vy/yaw-rate are low-pass filtered (0.10 s), and the correction integrator
    uses conditional anti-windup. While turning, heading hold disengages; when
    a turn stops, the current heading becomes the next straight-line reference.
    """

    def __init__(self, *, hold_heading=True, filter_time_s=.10,
                 kp=(.8, .8, 1.), ki=(.8, .8, .5), heading_kp=1.5,
                 command_limits=(1., .6, 1.), correction_limits=(.4, .3, .6)):
        self.hold_heading = bool(hold_heading)
        self.filter_time_s = float(filter_time_s)
        self.kp = np.asarray(kp, dtype=float)
        self.ki = np.asarray(ki, dtype=float)
        self.heading_kp = float(heading_kp)
        self.command_limits = np.asarray(command_limits, dtype=float)
        self.correction_limits = np.asarray(correction_limits, dtype=float)
        for values in (self.kp, self.ki, self.command_limits, self.correction_limits):
            if values.shape != (3,) or not np.isfinite(values).all() or np.any(values < 0):
                raise ValueError("Tracker gains and bounds require three finite nonnegative values")
        if not np.isfinite([self.filter_time_s, self.heading_kp]).all() or min(self.filter_time_s, self.heading_kp) < 0:
            raise ValueError("Tracker time constant and heading gain must be finite/nonnegative")
        if np.any(self.command_limits <= 0) or np.any(self.correction_limits <= 0):
            raise ValueError("Tracker command and correction limits must be positive")
        self.reset()

    def reset(self, heading_yaw=None):
        if heading_yaw is not None and not np.isfinite(heading_yaw):
            raise ValueError("Heading must be finite")
        self.heading_reference = heading_yaw
        self.integral = np.zeros(3)
        self.filtered_velocity = None
        self.last_error = np.zeros(3)
        self.correction = np.zeros(3)
        self.last_output = np.zeros(3)
        self.requested_target = np.zeros(3)
        self.update_count = 0
        self.saturation_count = 0
        self.saturated_axes = np.zeros(3, dtype=np.int64)
        self.correction_saturation_count = 0
        self._turning = False

    @property
    def saturation_fraction(self):
        return self.saturation_count / max(1, self.update_count)

    def update(self, target, measured_body_linear, gyro, heading_yaw, dt):
        target = np.asarray(target, dtype=float)
        linear = np.asarray(measured_body_linear, dtype=float)
        gyro = np.asarray(gyro, dtype=float)
        if any(x.shape != (3,) for x in (target, linear, gyro)):
            raise ValueError("Target, body linear velocity and gyro must be length three")
        if not np.isfinite(np.concatenate((target, linear, gyro, [heading_yaw, dt]))).all() or dt <= 0:
            raise ValueError("Tracker inputs must be finite and dt positive")
        velocity = np.array([linear[0], linear[1], gyro[2]])
        if self.filtered_velocity is None:
            self.filtered_velocity = velocity.copy()
        alpha = 1. if self.filter_time_s == 0 else -math.expm1(-dt / self.filter_time_s)
        self.filtered_velocity += alpha * (velocity - self.filtered_velocity)
        if self.heading_reference is None:
            self.heading_reference = float(heading_yaw)
        turning = abs(target[2]) > 1e-6
        if turning or self._turning:
            self.heading_reference = float(heading_yaw)
        self._turning = turning
        velocity_target = target.copy()
        if self.hold_heading and not turning:
            velocity_target[2] += np.clip(self.heading_kp * _angle_error(self.heading_reference, heading_yaw), -.35, .35)
        error = velocity_target - self.filtered_velocity
        candidate_integral = np.clip(self.integral + self.ki * error * dt,
                                     -self.correction_limits, self.correction_limits)
        raw_correction = self.kp * error + candidate_integral
        limited_correction = np.clip(raw_correction, -self.correction_limits, self.correction_limits)
        unbounded = velocity_target + limited_correction
        output = np.clip(unbounded, -self.command_limits, self.command_limits)
        # Freeze only integrator components that push further into either bound.
        blocked = ((raw_correction - limited_correction) * error > 0) | ((unbounded - output) * error > 0)
        self.integral = np.where(blocked, self.integral, candidate_integral)
        raw_correction = self.kp * error + self.integral
        correction = np.clip(raw_correction, -self.correction_limits, self.correction_limits)
        unbounded = velocity_target + correction
        output = np.clip(unbounded, -self.command_limits, self.command_limits)
        saturated = np.abs(unbounded - output) > 1e-12
        self.update_count += 1
        self.saturation_count += int(saturated.any())
        self.saturated_axes += saturated
        self.correction_saturation_count += int(np.any(np.abs(raw_correction - correction) > 1e-12))
        self.last_error = error
        self.correction = output - target
        self.last_output = output
        self.requested_target = target.copy()
        return output.copy()
