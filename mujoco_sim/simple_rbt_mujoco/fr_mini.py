"""Contact-driven FR-mini Ackermann approximation with a rear-axle base frame.

Manufacturer dimensions/topology are pinned in the adjacent manifest. Procedural
body/tire shapes, rigid suspension, mass distribution and servos are engineering
approximations, not a reconstruction of the manufacturer's vehicle controller.
Only two rear wheel motors and two front steering hinges are actuated. Front
rolling joints are passive; no base pose/velocity/external wrench is commanded.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

MODEL_VERSION = "fr-mini-ackermann-rigid-v1"
MANIFEST_PATH = Path(__file__).resolve().parents[1] / "assets/fr_mini_manifest.json"
WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_NAMES = tuple(name + "_wheel_joint" for name in WHEEL_NAMES)
STEERING_NAMES = ("front_left", "front_right")
COMMAND_LIMITS = (1., 0., 1.)
WHEELBASE_M, TRACK_M, WHEEL_RADIUS_M = .400, .312, .070
WHEEL_WIDTH_M = .045  # unspecified by manufacturer; estimated tire section width
MASS_KG, WHEEL_MASS_KG, KNUCKLE_MASS_KG = 18., .65, .20
BASE_HEIGHT_M, WHEEL_Z_M = .1255, -.0555
PHYSICS_PERIOD_S = .005
MIN_REAR_AXLE_TURN_RADIUS_M = 1.1  # interpretation of manual's unspecified radius reference
MAX_CURVATURE = 1. / MIN_REAR_AXLE_TURN_RADIUS_M
MAX_STEERING_ANGLE_RAD = math.atan(WHEELBASE_M / (MIN_REAR_AXLE_TURN_RADIUS_M - TRACK_M / 2))
PUBLISHED_MAX_SPEED_M_S = 6.5 / 3.6
MAX_WHEEL_SPEED_RAD_S = PUBLISHED_MAX_SPEED_M_S / WHEEL_RADIUS_M
WHEEL_TORQUE_LIMIT_NM, WHEEL_VELOCITY_GAIN = 4., 4.
STEERING_TORQUE_LIMIT_NM, STEERING_POSITION_GAIN, STEERING_VELOCITY_GAIN = 3., 40., 2.


def validate_command(command):
    """Validate rear-axle Twist semantics; global speed clipping is the caller's.

    Lateral velocity and an in-place rotation are physically unsupported. Finite
    moving commands with excessive curvature remain valid but are limited by the
    front steering geometry in the controller, with that limit exposed in logs.
    """
    value = np.asarray(command, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("FR-mini command must be finite [vx, vy, wz]")
    if value[1] != 0.:
        raise ValueError("FR-mini cannot command lateral velocity")
    if abs(value[0]) <= 1e-9 and value[2] != 0.:
        raise ValueError("FR-mini cannot rotate in place; nonzero yaw needs forward or reverse motion")
    return value.copy()


def model_metadata():
    return json.loads(MANIFEST_PATH.read_text())


def build_fr_mini_xml() -> str:
    """Robot-only MJCF. Base is over the rear axle, NOT the chassis center.

    Chassis center X=+.2 m. On flat ground base_origin Z=.1255 m, underside
    Z=.061 m, roof Z=.190 m. A narrow belly and full-width upper deck leave
    room for steering tires; all shapes are approximate, contact-enabled boxes.
    """
    root = ET.Element("mujoco", model=MODEL_VERSION)
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep=str(PHYSICS_PERIOD_S), integrator="implicitfast",
                  cone="elliptic", iterations="50")
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", damping=".01")
    ET.SubElement(default, "geom", friction=".8 .005 .0001", condim="6",
                  solref=".02 1", solimp=".9 .95 .001")
    world = ET.SubElement(root, "worldbody")
    base = ET.SubElement(world, "body", name="base", pos=f"0 0 {BASE_HEIGHT_M}")
    ET.SubElement(base, "freejoint", name="root")
    ET.SubElement(base, "geom", name="chassis", type="box", pos=".2 0 -.015",
                  size=".3075 .10 .0495", mass="13", rgba=".25 .3 .33 1", group="2")
    ET.SubElement(base, "geom", name="upper_deck", type="box", pos=".2 0 .0495",
                  size=".3075 .2025 .015", mass="2", rgba=".78 .80 .82 1", group="2")
    for name, pos in (("base_origin", "0 0 0"), ("imu", ".2 0 .04"),
                      ("lidar", ".2 0 .09"), ("camera_mount", ".515 0 .045")):
        ET.SubElement(base, "site", name=name, pos=pos, size=".002")
    wheel_inertia = (WHEEL_MASS_KG * (3 * WHEEL_RADIUS_M ** 2 + WHEEL_WIDTH_M ** 2) / 12,
                     WHEEL_MASS_KG * WHEEL_RADIUS_M ** 2 / 2)
    for name, front, side in zip(WHEEL_NAMES, (True, True, False, False), (1, -1, 1, -1)):
        x, y = (WHEELBASE_M if front else 0.), side * TRACK_M / 2
        parent = base
        position = f"{x} {y} {WHEEL_Z_M}"
        if front:
            parent = ET.SubElement(base, "body", name=name + "_steering", pos=position)
            ET.SubElement(parent, "inertial", mass=str(KNUCKLE_MASS_KG), pos="0 0 0",
                          diaginertia=".0002 .0002 .0002")
            ET.SubElement(parent, "joint", name=name + "_steering_joint", type="hinge", axis="0 0 1",
                          limited="true", range=f"{-MAX_STEERING_ANGLE_RAD} {MAX_STEERING_ANGLE_RAD}", damping=".05")
            position = "0 0 0"
        wheel = ET.SubElement(parent, "body", name=name + "_wheel", pos=position)
        ET.SubElement(wheel, "joint", name=name + "_wheel_joint", type="hinge", axis="0 1 0",
                      damping=".002" if front else ".01")
        # Rounded contact surface, cylinder-like inertia; neither is measured CAD.
        ET.SubElement(wheel, "inertial", pos="0 0 0", mass=str(WHEEL_MASS_KG),
                      quat=".7071067811865476 -.7071067811865476 0 0",
                      diaginertia=f"{wheel_inertia[0]} {wheel_inertia[0]} {wheel_inertia[1]}")
        ET.SubElement(wheel, "geom", name=name + "_tire", type="ellipsoid",
                      size=f"{WHEEL_RADIUS_M} {WHEEL_RADIUS_M} {WHEEL_WIDTH_M/2}",
                      quat=".7071067811865476 -.7071067811865476 0 0", mass=str(WHEEL_MASS_KG),
                      rgba=".12 .12 .13 1", group="2", priority="1")
    actuator = ET.SubElement(root, "actuator")
    for name in STEERING_NAMES:
        ET.SubElement(actuator, "position", name=name + "_steering", joint=name + "_steering_joint",
                      kp=str(STEERING_POSITION_GAIN), kv=str(STEERING_VELOCITY_GAIN), ctrllimited="true",
                      ctrlrange=f"{-MAX_STEERING_ANGLE_RAD} {MAX_STEERING_ANGLE_RAD}", forcelimited="true",
                      forcerange=f"{-STEERING_TORQUE_LIMIT_NM} {STEERING_TORQUE_LIMIT_NM}")
    for name in WHEEL_NAMES[2:]:
        ET.SubElement(actuator, "velocity", name=name + "_wheel", joint=name + "_wheel_joint",
                      kv=str(WHEEL_VELOCITY_GAIN), ctrllimited="true",
                      ctrlrange=f"{-MAX_WHEEL_SPEED_RAD_S} {MAX_WHEEL_SPEED_RAD_S}", forcelimited="true",
                      forcerange=f"{-WHEEL_TORQUE_LIMIT_NM} {WHEEL_TORQUE_LIMIT_NM}")
    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "gyro", name="gyro", site="imu")
    ET.SubElement(sensor, "accelerometer", name="accelerometer", site="imu")
    ET.SubElement(sensor, "velocimeter", name="base_linvel", site="base_origin")
    ET.SubElement(sensor, "gyro", name="base_angvel", site="base_origin")
    return ET.tostring(root, encoding="unicode")


class FrMiniController:
    """Bounded Ackermann steering and rear wheel velocity servos, without body PI.

    Front wheel target rates are predictions, not commands to hidden front motors.
    The two rear motors are in actuator_indices/drive_actuator_indices; steering
    has separate steering_actuator_indices. read_wheel_state returns FL/FR/RL/RR
    joint angles/rates. Raw state snapshots are taken before the next physics step.
    """
    wheel_names = WHEEL_NAMES
    command_limits = COMMAND_LIMITS
    max_curvature = MAX_CURVATURE
    validate_command = staticmethod(validate_command)

    def __init__(self, model):
        self.qpos_indices = np.array([model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self.qvel_indices = np.array([model.joint(n).dofadr[0] for n in JOINT_NAMES])
        self.steering_qpos_indices = np.array([model.joint(n + "_steering_joint").qposadr[0] for n in STEERING_NAMES])
        self.steering_qvel_indices = np.array([model.joint(n + "_steering_joint").dofadr[0] for n in STEERING_NAMES])
        self.drive_actuator_indices = np.array([model.actuator(n + "_wheel").id for n in WHEEL_NAMES[2:]])
        self.actuator_indices = self.drive_actuator_indices
        self.steering_actuator_indices = np.array([model.actuator(n + "_steering").id for n in STEERING_NAMES])
        self.last_command = np.zeros(3)
        self.applied_command = np.zeros(3)
        self.target_velocity = np.zeros(4)
        self.requested_target_velocity = np.zeros(4)
        self.steering_targets = np.zeros(2)
        self.wheel_positions = np.zeros(4)
        self.wheel_velocities = np.zeros(4)
        self.steering_positions = np.zeros(2)
        self.requested_curvature = self.applied_curvature = 0.
        self.saturation_scale = 1.
        self.curvature_saturated = False
        self.steps = self.saturation_count = self.curvature_saturation_count = 0

    @property
    def metadata(self):
        return model_metadata()

    @property
    def saturation_fraction(self):
        return self.saturation_count / max(1, self.steps)

    def read_wheel_state(self, data):
        return data.qpos[self.qpos_indices].copy(), data.qvel[self.qvel_indices].copy()

    def read_steering_state(self, data):
        return data.qpos[self.steering_qpos_indices].copy(), data.qvel[self.steering_qvel_indices].copy()

    @staticmethod
    def _rolling_rates(vx, wz):
        left, right = vx - wz * TRACK_M / 2, vx + wz * TRACK_M / 2
        sign = -1. if vx < 0. else 1.
        return np.array([sign * math.hypot(left, wz * WHEELBASE_M),
                         sign * math.hypot(right, wz * WHEELBASE_M), left, right]) / WHEEL_RADIUS_M

    def update(self, model, data, command):
        command = validate_command(command)
        self.last_command[:] = command
        vx, _, wz = command
        self.requested_curvature = float(wz / vx) if abs(vx) > 1e-9 else 0.
        self.applied_curvature = float(np.clip(self.requested_curvature, -self.max_curvature, self.max_curvature))
        self.curvature_saturated = not math.isclose(self.requested_curvature, self.applied_curvature, rel_tol=0., abs_tol=1e-12)
        applied_wz = float(vx * self.applied_curvature)
        self.requested_target_velocity[:] = self._rolling_rates(vx, wz)
        targets = self._rolling_rates(vx, applied_wz)
        maximum_rear = float(np.max(np.abs(targets[2:])))
        self.saturation_scale = min(1., MAX_WHEEL_SPEED_RAD_S / maximum_rear) if maximum_rear else 1.
        self.target_velocity[:] = targets * self.saturation_scale
        self.applied_command[:] = [vx * self.saturation_scale, 0., applied_wz * self.saturation_scale]
        k = self.applied_curvature
        self.steering_targets[:] = [math.atan(WHEELBASE_M * k / (1 - TRACK_M * k / 2)),
                                    math.atan(WHEELBASE_M * k / (1 + TRACK_M * k / 2))]
        data.ctrl[self.drive_actuator_indices] = self.target_velocity[2:]
        data.ctrl[self.steering_actuator_indices] = self.steering_targets
        self.wheel_positions[:], self.wheel_velocities[:] = self.read_wheel_state(data)
        self.steering_positions[:] = data.qpos[self.steering_qpos_indices]
        self.steps += 1
        self.curvature_saturation_count += int(self.curvature_saturated)
        self.saturation_count += int(self.curvature_saturated or self.saturation_scale < 1.)
