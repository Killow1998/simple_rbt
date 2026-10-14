"""Contact-driven Ranger Mini 3.0 approximation with four real steering joints.

Current product mass/axle spacing are sourced; shell, tire and actuator details
are explicitly estimated in the manifest. This is a general swerve controller,
not a reproduction of AgileX's discrete-mode CAN firmware. No body-state writes.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .paths import MANIFESTS

MANIFEST_PATH = MANIFESTS / "ranger_mini_manifest.json"
MODEL_VERSION = "ranger-mini-3-current-spec-swerve-rigid-v1"
WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_NAMES = tuple(name + "_wheel_joint" for name in WHEEL_NAMES)
STEERING_JOINT_NAMES = tuple(name + "_steering_joint" for name in WHEEL_NAMES)
WHEELBASE_M, TRACK_M = .494, .370
WHEEL_RADIUS_M, WHEEL_WIDTH_M = .110, .070
MASS_KG, WHEEL_MASS_KG, MODULE_MASS_KG = 73., 2.5, 1.
CHASSIS_SIZE_M = (.700, .140, .215)
DECK_SIZE_M = (.700, .460, .080)
GROUND_CLEARANCE_M = .105
BASE_HEIGHT_M = GROUND_CLEARANCE_M + CHASSIS_SIZE_M[2] / 2
WHEEL_Z_M = WHEEL_RADIUS_M - BASE_HEIGHT_M
CHASSIS_COLLISION_NAMES = ("chassis", "chassis_deck")
PHYSICS_PERIOD_S = .005
COMMAND_LIMITS = (1., .6, 1.)
MAX_WHEEL_SPEED_RAD_S = 1.5 / WHEEL_RADIUS_M
WHEEL_TORQUE_LIMIT_NM, WHEEL_VELOCITY_GAIN = 20., 6.
STEERING_LIMIT_RAD = math.pi
STEERING_RATE_LIMIT_RAD_S = 2.
STEERING_TORQUE_LIMIT_NM = 20.
STEERING_POSITION_GAIN, STEERING_VELOCITY_GAIN = 8., 12.
ALIGNMENT_RELEASE_RAD = .10
TIRE_FRICTION = (.8, .005, .0001)
MODULE_XY = np.array([[sx * WHEELBASE_M / 2, sy * TRACK_M / 2]
                      for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))])


def model_metadata():
    return json.loads(MANIFEST_PATH.read_text())


def build_ranger_mini_xml() -> str:
    """Robot-only MJCF; caller adds floor/terrain and the separate MID360 IMU.

    Base is the keel center, at nominal z=.2125 m. The wider upper deck clears
    the swept tire envelope. Both named chassis geoms are physical collisions.
    Steering pivots cross wheel hubs (zero scrub offset assumption), about +Z;
    drive hinges use module +Y, so positive speed rolls along module +X.
    """
    root = ET.Element("mujoco", model=MODEL_VERSION)
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep=str(PHYSICS_PERIOD_S),
                  integrator="implicitfast", cone="elliptic", iterations="50")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", damping=".02")
    ET.SubElement(defaults, "geom", friction=".8 .005 .0001", condim="6",
                  solref=".02 1", solimp=".9 .95 .001")
    world = ET.SubElement(root, "worldbody")
    base = ET.SubElement(world, "body", name="base", pos=f"0 0 {BASE_HEIGHT_M}")
    ET.SubElement(base, "freejoint", name="root")
    ET.SubElement(base, "geom", name="chassis", type="box",
                  size=" ".join(str(s / 2) for s in CHASSIS_SIZE_M),
                  mass="44", rgba=".22 .25 .28 1", group="2")
    ET.SubElement(base, "geom", name="chassis_deck", type="box", pos="0 0 .0675",
                  size=" ".join(str(s / 2) for s in DECK_SIZE_M),
                  mass="15", rgba=".91 .66 .12 1", group="2")
    for name, pos in (("base_origin", "0 0 0"), ("imu", "0 0 .04"),
                      ("lidar", "0 0 .1275"), ("camera_mount", ".36 0 .0675")):
        ET.SubElement(base, "site", name=name, pos=pos, size=".002")
    for name, (x, y) in zip(WHEEL_NAMES, MODULE_XY):
        module = ET.SubElement(base, "body", name=name + "_steering",
                               pos=f"{x} {y} {WHEEL_Z_M}")
        ET.SubElement(module, "joint", name=name + "_steering_joint", type="hinge",
                      axis="0 0 1", limited="true", range=f"{-STEERING_LIMIT_RAD} {STEERING_LIMIT_RAD}")
        ET.SubElement(module, "inertial", pos="0 0 .04", mass=str(MODULE_MASS_KG),
                      diaginertia=".0015 .0015 .001")
        ET.SubElement(module, "geom", name=name + "_steering_visual", type="cylinder",
                      pos="0 0 .075", size=".025 .035", mass="0", contype="0",
                      conaffinity="0", rgba=".75 .4 .08 1", group="2")
        wheel = ET.SubElement(module, "body", name=name + "_wheel")
        ET.SubElement(wheel, "joint", name=name + "_wheel_joint", type="hinge", axis="0 1 0")
        ET.SubElement(wheel, "geom", name=name + "_tire", type="ellipsoid",
                      size=f"{WHEEL_RADIUS_M} {WHEEL_RADIUS_M} {WHEEL_WIDTH_M / 2}",
                      quat=".7071067811865476 -.7071067811865476 0 0",
                      mass=str(WHEEL_MASS_KG), rgba=".10 .11 .12 1", group="2", priority="1")
    actuators = ET.SubElement(root, "actuator")
    # Drive indices remain a separate four-element array even with eight motors.
    for name in WHEEL_NAMES:
        ET.SubElement(actuators, "velocity", name=name + "_wheel", joint=name + "_wheel_joint",
                      kv=str(WHEEL_VELOCITY_GAIN), ctrllimited="true",
                      ctrlrange=f"{-MAX_WHEEL_SPEED_RAD_S} {MAX_WHEEL_SPEED_RAD_S}",
                      forcelimited="true", forcerange=f"{-WHEEL_TORQUE_LIMIT_NM} {WHEEL_TORQUE_LIMIT_NM}")
    for name in WHEEL_NAMES:
        ET.SubElement(actuators, "velocity", name=name + "_steering", joint=name + "_steering_joint",
                      kv=str(STEERING_VELOCITY_GAIN), ctrllimited="true",
                      ctrlrange=f"{-STEERING_RATE_LIMIT_RAD_S} {STEERING_RATE_LIMIT_RAD_S}",
                      forcelimited="true", forcerange=f"{-STEERING_TORQUE_LIMIT_NM} {STEERING_TORQUE_LIMIT_NM}")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="gyro", site="imu")
    ET.SubElement(sensors, "accelerometer", name="accelerometer", site="imu")
    ET.SubElement(sensors, "velocimeter", name="base_linvel", site="base_origin")
    ET.SubElement(sensors, "gyro", name="base_angvel", site="base_origin")
    return ET.tostring(root, encoding="unicode")


def _nearest_heading(heading, current):
    """Equivalent bounded azimuth requiring least physical joint rotation."""
    candidates = [(heading + k * math.pi, -1. if k % 2 else 1.)
                  for k in (0, -1, 1, -2, 2)
                  if -STEERING_LIMIT_RAD <= heading + k * math.pi <= STEERING_LIMIT_RAD]
    return min(candidates, key=lambda item: abs(item[0] - current))


class RangerMiniController:
    """Four-module swerve mapping and bounded physical joint servos.

    Native mode has joint feedback only: chassis velocity is not fed back here.
    All module states use FL, FR, RL, RR. Angles/speeds are radians/rad s^-1.
    Drive is held at zero while any active module differs by >.1 rad from its
    desired direction. At a full zero command, latch CURRENT azimuths and brake;
    repeated zero updates do not keep chasing a previous steering target.
    """
    wheel_names = WHEEL_NAMES

    def __init__(self, model):
        self.qpos_indices = np.array([model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self.qvel_indices = np.array([model.joint(n).dofadr[0] for n in JOINT_NAMES])
        self.steering_qpos_indices = np.array([model.joint(n).qposadr[0] for n in STEERING_JOINT_NAMES])
        self.steering_qvel_indices = np.array([model.joint(n).dofadr[0] for n in STEERING_JOINT_NAMES])
        self.actuator_indices = np.array([model.actuator(n + "_wheel").id for n in WHEEL_NAMES])
        self.steering_actuator_indices = np.array([model.actuator(n + "_steering").id for n in WHEEL_NAMES])
        self.target_velocity = np.zeros(4)
        self.requested_target_velocity = np.zeros(4)
        self.steering_targets = np.zeros(4)
        self.steering_velocity_targets = np.zeros(4)
        self.steering_errors = np.zeros(4)
        self.steering_positions = np.zeros(4)
        self.steering_velocities = np.zeros(4)
        self.wheel_positions = np.zeros(4)
        self.wheel_velocities = np.zeros(4)
        self.last_command = np.zeros(3)
        self.mode = self.requested_mode = "brake"
        self.steps = self.saturation_count = self.alignment_count = 0
        self.saturation_scale = self.alignment_scale = 1.
        self._braking = False

    @property
    def metadata(self):
        return model_metadata()

    @property
    def saturation_fraction(self):
        return self.saturation_count / max(1, self.steps)

    @property
    def alignment_fraction(self):
        return self.alignment_count / max(1, self.steps)

    def read_wheel_state(self, data):
        return data.qpos[self.qpos_indices].copy(), data.qvel[self.qvel_indices].copy()

    def read_steering_state(self, data):
        return data.qpos[self.steering_qpos_indices].copy(), data.qvel[self.steering_qvel_indices].copy()

    def update(self, model, data, command):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("Ranger Mini command must contain three finite values [vx, vy, wz]")
        vx, vy, wz = command
        self.last_command[:] = command
        self.steering_positions[:], self.steering_velocities[:] = self.read_steering_state(data)
        self.wheel_positions[:], self.wheel_velocities[:] = self.read_wheel_state(data)
        braking = bool(np.max(np.abs(command)) < 1e-12)
        active = np.zeros(4, dtype=bool)
        self.requested_target_velocity.fill(0.)
        if braking:
            if not self._braking:
                self.steering_targets[:] = self.steering_positions
            self.requested_mode = "brake"
        else:
            self.requested_mode = ("translate" if abs(wz) < 1e-12 else
                                   "spin" if math.hypot(vx, vy) < 1e-12 else "compound")
            for i, (x, y) in enumerate(MODULE_XY):
                ux, uy = vx - wz * y, vy + wz * x
                speed = math.hypot(ux, uy)
                if speed < 1e-10:
                    self.steering_targets[i] = self.steering_positions[i]
                    continue
                active[i] = True
                angle, sign = _nearest_heading(math.atan2(uy, ux), self.steering_positions[i])
                self.steering_targets[i] = angle
                self.requested_target_velocity[i] = sign * speed / WHEEL_RADIUS_M
        self._braking = braking
        self.steering_errors[:] = self.steering_targets - self.steering_positions
        self.steering_velocity_targets[:] = np.clip(STEERING_POSITION_GAIN * self.steering_errors,
                                                   -STEERING_RATE_LIMIT_RAD_S, STEERING_RATE_LIMIT_RAD_S)
        # MuJoCo integrates the velocity-feedback derivative implicitly; an
        # external explicit high-gain torque loop is unstable for a light hub.
        data.ctrl[self.steering_actuator_indices] = self.steering_velocity_targets
        maximum = float(np.max(np.abs(self.requested_target_velocity)))
        self.saturation_scale = min(1., MAX_WHEEL_SPEED_RAD_S / maximum) if maximum else 1.
        aligned = bool(not active.any() or np.max(np.abs(self.steering_errors[active])) <= ALIGNMENT_RELEASE_RAD)
        self.alignment_scale = 1. if aligned else 0.
        self.target_velocity[:] = self.requested_target_velocity * self.saturation_scale * self.alignment_scale
        data.ctrl[self.actuator_indices] = self.target_velocity
        self.mode = self.requested_mode if aligned else "aligning"
        self.steps += 1
        self.saturation_count += int(self.saturation_scale < 1.)
        self.alignment_count += int(not aligned)
