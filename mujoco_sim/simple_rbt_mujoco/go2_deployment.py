"""Deterministic replay of pinned rl_sar Go2's first Passive -> GetUp path.

Source: fan-ziqi/rl_sar commit 36bac3930bca95239de9bcfade283832e6ee4409,
policy/go2/base.yaml, fsm_go2.hpp and RLFSMState::Interpolate in rl_sdk.cpp.
The caller steps physics and starts its fresh policy on the step AFTER completion.
This reproduces the GetUp command schedule, not the asynchronous FSM/keyboard:
there is no passive dwell, and RL entry is automatic instead of user-triggered.
No joint pose, physics parameter, external force or policy state is changed.
"""
from __future__ import annotations

import json
import math

import numpy as np

from .go2 import MANIFEST

UPSTREAM_COMMIT = "36bac3930bca95239de9bcfade283832e6ee4409"
_CONTROL_DT = .005
_PRE_POSE = np.tile(np.array([0., 1.36, -2.65], dtype=np.float32), 4)
_STAND_POSE = np.tile(np.array([0., .8, -1.5], dtype=np.float32), 4)
_KP, _KD = 80., 3.  # Initial base.yaml gains, before a policy config is loaded.


class Go2GetUp:
    """Write upstream GetUp PD commands, without stepping MuJoCo.

    ``update`` returns completion after writing this tick's command. The caller
    must still perform that physics step before exposing startup completion.
    Repeated updates after completion hold the standing PD target without
    advancing startup elapsed time. Input model must retain the native 5 ms
    control/physics step. All arrays use the SDK joint order, independent of
    which locomotion policy is selected later.
    """

    def __init__(self, model, data):
        if not math.isclose(float(model.opt.timestep), _CONTROL_DT, rel_tol=0., abs_tol=1e-12):
            raise ValueError("Pinned Go2 GetUp requires the native 5 ms timestep")
        manifest = json.loads(MANIFEST.read_text())
        self.joint_names = list(manifest["sdk_joint_names"])
        joints = [model.joint(name).id for name in self.joint_names]
        self.qpos_indices = np.asarray([model.jnt_qposadr[j] for j in joints])
        self.qvel_indices = np.asarray([model.jnt_dofadr[j] for j in joints])
        actuators = [np.flatnonzero(model.actuator_trnid[:, 0] == joint) for joint in joints]
        if any(len(matches) != 1 for matches in actuators):
            raise ValueError("GetUp requires one existing torque actuator per SDK joint")
        self.actuator_indices = np.asarray([matches[0] for matches in actuators])
        self.start_position = data.qpos[self.qpos_indices].astype(np.float32).copy()
        self.target_position = self.start_position.copy()
        self._percent_pre = np.float32(0.)
        self._percent_stand = np.float32(0.)
        self.steps = 0
        self.phase = "pre_getup"
        self.complete = False

    @property
    def elapsed_s(self):
        return self.steps * _CONTROL_DT

    @property
    def metadata(self):
        return {
            "source_repository": "https://github.com/fan-ziqi/rl_sar",
            "source_commit": UPSTREAM_COMMIT,
            "source_files": ["policy/go2/base.yaml", "src/rl_sar/fsm_robot/fsm_go2.hpp",
                             "src/rl_sar/library/core/rl_sdk/rl_sdk.cpp"],
            "scope": "first Passive -> GetUp with no passive dwell; automatic RL entry on next step",
            "limitations": "Deterministic GetUp command replay; no keyboard delay or asynchronous scheduler phase",
            "control_period_s": _CONTROL_DT,
            "nominal_phase_durations_s": [1., 2.],
            "interpolation": "float32 percent, increment before output; skip only if max joint delta < 0.1 rad",
            "phase_boundary": "first-stage completion immediately advances second stage in the same tick",
            "joint_order": list(self.joint_names),
            "start_position_rad": self.start_position.tolist(),
            "pre_getup_position_rad": _PRE_POSE.tolist(),
            "standing_position_rad": _STAND_POSE.tolist(),
            "kp": _KP, "kd": _KD,
            "torque_limit": "native MJCF actuator limits; no policy config clamp during GetUp",
            "complete": self.complete, "phase": self.phase,
            "steps": self.steps, "elapsed_s": self.elapsed_s,
        }

    def _interpolate(self, percent, start, target, duration):
        if percent >= 1.:
            return percent
        if percent == 0. and np.max(np.abs(start - target)) < np.float32(.1):
            percent = np.float32(1.)
        # Match C++ float duration / dt, float increment and float accumulation.
        frames = max(1, math.ceil(float(np.float32(duration) / np.float32(_CONTROL_DT))))
        increment = np.float32(1.) / np.float32(frames)
        percent = np.minimum(np.float32(1.), np.float32(percent + increment))
        self.target_position = (np.float32(1.) - percent) * start + percent * target
        return percent

    def update(self, model, data):
        if not self.complete:
            self.steps += 1
            self._percent_pre = self._interpolate(self._percent_pre, self.start_position,
                                                  _PRE_POSE, 1.)
            if self._percent_pre >= 1.:
                self.phase = "getup"
                self._percent_stand = self._interpolate(self._percent_stand, _PRE_POSE,
                                                        _STAND_POSE, 2.)
                if self._percent_stand >= 1.:
                    self.complete = True
                    self.phase = "complete"
        torque = (_KP * (self.target_position.astype(float) - data.qpos[self.qpos_indices])
                  - _KD * data.qvel[self.qvel_indices])
        data.ctrl[self.actuator_indices] = torque
        return self.complete
