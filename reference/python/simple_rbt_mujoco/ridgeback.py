"""Procedural Ridgeback R100 with four driven hubs and forty passive rollers.

Contact drives every base motion. Each +/-45 degree barrel roller is a separate
free hinge with isotropic friction; no anisotropic slip shortcut or body wrench
is used. Dimensions follow pinned official R100 sources. Roller profile, motor
limits, mass distribution and lack of rocker articulation remain approximations.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .paths import MANIFESTS

MANIFEST_PATH = MANIFESTS / "ridgeback_manifest.json"
MODEL_VERSION = "ridgeback-r100-passive-rollers-v1"
WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")
WHEEL_RADIUS_M = .0759
WHEEL_WIDTH_M = .079
HALF_WHEELBASE_M = .319
HALF_TRACK_M = (.472 + WHEEL_WIDTH_M) / 2.
MASS_KG = 135.
BASE_HEIGHT_M = WHEEL_RADIUS_M - .050 + .216 / 2.
CHASSIS_SIZE_M = (.960, .420, .216)
PHYSICS_PERIOD_S = .005
ROLLER_COUNT = 10
ROLLER_CENTRE_RADIUS_M = .060
ROLLER_RADIUS_M = WHEEL_RADIUS_M - ROLLER_CENTRE_RADIUS_M
ROLLER_HALF_LENGTH_M = math.sqrt(2. * (WHEEL_WIDTH_M / 2.) ** 2 - ROLLER_RADIUS_M ** 2)
WHEEL_TORQUE_LIMIT_NM = 20.
WHEEL_VELOCITY_GAIN = 8.
MAX_WHEEL_SPEED_RAD_S = 1.1 / WHEEL_RADIUS_M
COMMAND_LIMITS = (1., .6, 1.)
# Roller axes at the bottom contact are (1, slope, 0)/sqrt(2).
ROLLER_SLOPES = (-1., 1., 1., -1.)


def model_metadata():
    return json.loads(MANIFEST_PATH.read_text())


def _values(values):
    return " ".join(format(float(value), ".17g") for value in values)


def build_ridgeback_xml() -> str:
    root = ET.Element("mujoco", model=MODEL_VERSION)
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep=str(PHYSICS_PERIOD_S), integrator="implicitfast",
                  cone="elliptic", iterations="80")
    defaults = ET.SubElement(root, "default")
    robot_defaults = ET.SubElement(defaults, "default", attrib={"class": "ridgeback_robot"})
    ET.SubElement(robot_defaults, "geom", friction=".8 .002 .0001", condim="3", priority="1",
                  contype="2", conaffinity="1", solref=".015 1", solimp=".9 .95 .001")
    world = ET.SubElement(root, "worldbody")
    base = ET.SubElement(world, "body", name="base", pos=f"0 0 {BASE_HEIGHT_M:.17g}", childclass="ridgeback_robot")
    ET.SubElement(base, "freejoint", name="root")
    # Central lower collision box leaves real wheel wells. The full-width deck
    # is above the wheel tops; do not fill the wheels with a rectangular chassis.
    ET.SubElement(base, "geom", name="chassis", type="box", size=_values(np.array(CHASSIS_SIZE_M) / 2.),
                  mass="123", rgba=".14 .14 .16 1", group="2")
    deck_bottom = BASE_HEIGHT_M + CHASSIS_SIZE_M[2] / 2.
    deck_height = .311 - deck_bottom
    ET.SubElement(base, "geom", name="deck", type="box", size=_values((.480, .3965, deck_height / 2.)),
                  pos=_values((0., 0., deck_bottom + deck_height / 2. - BASE_HEIGHT_M)),
                  mass="2", rgba=".85 .66 .13 1", group="2")
    for name, pos in (("base_origin", (0., 0., 0.)),
                      ("imu", (.2085, -.2902, .1681 - .108)),
                      ("lidar", (0., 0., .20)), ("camera_mount", (.485, 0., .12))):
        ET.SubElement(base, "site", name=name, pos=_values(pos), size=".002")
    for name, (sx, sy), slope in zip(WHEEL_NAMES, ((1, 1), (1, -1), (-1, 1), (-1, -1)), ROLLER_SLOPES):
        wheel = ET.SubElement(base, "body", name=name + "_wheel",
                              pos=_values((sx * HALF_WHEELBASE_M, sy * HALF_TRACK_M,
                                           WHEEL_RADIUS_M - BASE_HEIGHT_M)))
        ET.SubElement(wheel, "joint", name=name + "_wheel_joint", type="hinge", axis="0 1 0", damping=".02")
        ET.SubElement(wheel, "geom", name=name + "_hub", type="cylinder", size=".038 .025",
                      quat=".7071067811865476 -.7071067811865476 0 0", mass="1.5",
                      rgba=".28 .28 .30 1", group="2")
        for index in range(ROLLER_COUNT):
            theta = index * 2. * math.pi / ROLLER_COUNT
            radial = np.array((math.sin(theta), 0., -math.cos(theta)))
            tangent = np.array((math.cos(theta), 0., math.sin(theta)))
            axis = (tangent + np.array((0., slope, 0.))) / math.sqrt(2.)
            roller_name = f"{name}_roller_{index}"
            roller = ET.SubElement(wheel, "body", name=roller_name,
                                   pos=_values(ROLLER_CENTRE_RADIUS_M * radial))
            ET.SubElement(roller, "joint", name=roller_name + "_joint", type="hinge",
                          axis=_values(axis), damping=".00001")
            # An axisymmetric ellipsoid stays geometrically unchanged as this
            # passive joint spins, but its material contact velocity changes.
            ET.SubElement(roller, "geom", name=roller_name + "_geom", type="ellipsoid",
                          zaxis=_values(axis), size=_values((ROLLER_RADIUS_M, ROLLER_RADIUS_M, ROLLER_HALF_LENGTH_M)),
                          mass=".1", rgba=".08 .08 .09 1", group="2")
    actuators = ET.SubElement(root, "actuator")
    for name in WHEEL_NAMES:
        ET.SubElement(actuators, "velocity", name=name + "_wheel", joint=name + "_wheel_joint",
                      kv=str(WHEEL_VELOCITY_GAIN), ctrllimited="true",
                      ctrlrange=_values((-MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S)),
                      forcelimited="true", forcerange=_values((-WHEEL_TORQUE_LIMIT_NM, WHEEL_TORQUE_LIMIT_NM)))
    sensors = ET.SubElement(root, "sensor")
    for tag, name, site in (("gyro", "gyro", "imu"), ("accelerometer", "accelerometer", "imu"),
                            ("velocimeter", "base_linvel", "base_origin"), ("gyro", "base_angvel", "base_origin")):
        ET.SubElement(sensors, tag, name=name, site=site)
    return ET.tostring(root, encoding="unicode")


class RidgebackController:
    """Four velocity-controlled hubs; passive roller joints are never actuated.

    Order is FL, FR, RL, RR. All hub axes are +Y, so positive rotation drives
    forward. The map follows the actual axle geometry, not the rounded .59 m
    controller parameter upstream. There is no native body-velocity feedback.
    """
    wheel_names = WHEEL_NAMES

    def __init__(self, model):
        self.qpos_indices = np.array([model.joint(name + "_wheel_joint").qposadr[0] for name in WHEEL_NAMES])
        self.qvel_indices = np.array([model.joint(name + "_wheel_joint").dofadr[0] for name in WHEEL_NAMES])
        self.actuator_indices = np.array([model.actuator(name + "_wheel").id for name in WHEEL_NAMES])
        self.target_velocity = np.zeros(4)
        self.requested_target_velocity = np.zeros(4)
        self.wheel_positions = np.zeros(4)
        self.wheel_velocities = np.zeros(4)
        self.last_command = np.zeros(3)
        self.steps = self.saturation_count = 0
        self.saturation_scale = 1.

    @property
    def metadata(self):
        return model_metadata()

    @property
    def saturation_fraction(self):
        return self.saturation_count / max(1, self.steps)

    def read_wheel_state(self, data):
        return data.qpos[self.qpos_indices].copy(), data.qvel[self.qvel_indices].copy()

    def update(self, model, data, command):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("Ridgeback command requires finite [vx, vy, wz]")
        self.last_command[:] = command
        vx, vy, wz = command
        yaw = (HALF_WHEELBASE_M + HALF_TRACK_M) * wz
        targets = np.array((vx - vy - yaw, vx + vy + yaw, vx + vy - yaw, vx - vy + yaw)) / WHEEL_RADIUS_M
        self.requested_target_velocity[:] = targets
        maximum = float(np.max(np.abs(targets)))
        self.saturation_scale = min(1., MAX_WHEEL_SPEED_RAD_S / maximum) if maximum else 1.
        self.target_velocity[:] = targets * self.saturation_scale
        data.ctrl[self.actuator_indices] = self.target_velocity
        self.wheel_positions[:], self.wheel_velocities[:] = self.read_wheel_state(data)
        self.steps += 1
        self.saturation_count += int(self.saturation_scale < 1. - 1e-12)
