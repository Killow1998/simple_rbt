"""Scout Mini four-wheel skid steer, with explicit first-stage approximations.

Geometry/mass follow the current AgileX SCOUT MINI product specification, not
its inconsistent legacy ROS URDF. Tires and chassis are procedural primitives:
no third-party mesh is copied. Suspension is rigid, tire friction is isotropic,
and wheel mass distribution, actuator gains and torque are engineering estimates.
Rounded tire contact surfaces retain the v1 cylinder inertia. This improves
turning/contact behavior but has not passed the 4 cm or 5 cm stair route.
This is a contact-driven functional model, not a calibrated hardware replica.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

MANIFEST_PATH = Path(__file__).resolve().parents[1] / "assets/scout_mini_manifest.json"
MODEL_VERSION = "scout-mini-current-spec-rounded-rigid-v2"
WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_NAMES = tuple(name + "_wheel_joint" for name in WHEEL_NAMES)
WHEELBASE_M = .451
TRACK_M = .490
WHEEL_RADIUS_M = .080
WHEEL_WIDTH_M = .090
MASS_KG = 23.
WHEEL_MASS_KG = 1.
CHASSIS_MASS_KG = MASS_KG - 4 * WHEEL_MASS_KG
CHASSIS_SIZE_M = (.612, .400, .130)
GROUND_CLEARANCE_M = .115
BASE_HEIGHT_M = GROUND_CLEARANCE_M + CHASSIS_SIZE_M[2] / 2
WHEEL_Z_M = WHEEL_RADIUS_M - BASE_HEIGHT_M
PHYSICS_PERIOD_S = .005
MAX_LINEAR_SPEED_M_S = 1.5
MAX_WHEEL_SPEED_RAD_S = MAX_LINEAR_SPEED_M_S / WHEEL_RADIUS_M
WHEEL_TORQUE_LIMIT_NM = 8.
WHEEL_VELOCITY_GAIN = 4.
TIRE_FRICTION = (.8, .005, .0001)
# Keep v1 solid-cylinder inertia while replacing only its sharp contact surface.
# Principal axes use the same local frame as the tire (local Z -> body +Y).
WHEEL_INERTIA_LOCAL_KG_M2 = (.0022749999999999997, .0022749999999999997, .0032000000000000002)


def model_metadata():
    """Fresh serializable provenance and explicit assumptions."""
    return json.loads(MANIFEST_PATH.read_text())


def build_scout_mini_xml() -> str:
    """Return robot-only MJCF; the caller adds floor, terrain and device IMU.

    Base origin is the chassis-box center, nominally 0.180 m above flat ground.
    Four hinge axes are +Y; positive angular speed drives the robot along +X.
    The chassis underside is at 0.115 m and the roof at 0.245 m. Wheel rotation
    is the only actuation: no body velocity servo, external force or pose write.
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
    base = ET.SubElement(world, "body", name="base", pos=f"0 0 {BASE_HEIGHT_M:.9g}")
    ET.SubElement(base, "freejoint", name="root")
    ET.SubElement(base, "geom", name="chassis", type="box",
                  size=" ".join(str(s / 2) for s in CHASSIS_SIZE_M),
                  mass=str(CHASSIS_MASS_KG), rgba=".78 .70 .25 1", group="2")
    for name, pos in (("base_origin", "0 0 0"), ("imu", "0 0 .04"),
                      ("lidar", "0 0 .085"), ("camera_mount", ".31 0 .035")):
        ET.SubElement(base, "site", name=name, pos=pos, size=".002")
    # Round tire shoulders are ellipsoids; local Z rotates to hinge axis +Y.
    # Contact friction remains .8 along BOTH tangent directions, allowing genuine
    # dissipative tire scrub in a skid turn instead of pretending to have casters.
    for name, (sx, sy) in zip(WHEEL_NAMES, ((1, 1), (1, -1), (-1, 1), (-1, -1))):
        wheel = ET.SubElement(base, "body", name=name + "_wheel",
                              pos=f"{sx * WHEELBASE_M / 2} {sy * TRACK_M / 2} {WHEEL_Z_M:.9g}")
        ET.SubElement(wheel, "joint", name=name + "_wheel_joint", type="hinge", axis="0 1 0")
        ET.SubElement(wheel, "inertial", pos="0 0 0", mass=str(WHEEL_MASS_KG),
                      quat=".7071067811865476 -.7071067811865476 0 0",
                      diaginertia=" ".join(format(value, ".17g") for value in WHEEL_INERTIA_LOCAL_KG_M2))
        ET.SubElement(wheel, "geom", name=name + "_tire", type="ellipsoid",
                      size=f"{WHEEL_RADIUS_M} {WHEEL_RADIUS_M} {WHEEL_WIDTH_M / 2}",
                      quat=".7071067811865476 -.7071067811865476 0 0",
                      mass=str(WHEEL_MASS_KG), rgba=".12 .12 .13 1", group="2", priority="1")
    actuators = ET.SubElement(root, "actuator")
    for name in WHEEL_NAMES:
        ET.SubElement(actuators, "velocity", name=name + "_wheel", joint=name + "_wheel_joint",
                      kv=str(WHEEL_VELOCITY_GAIN), ctrllimited="true",
                      ctrlrange=f"{-MAX_WHEEL_SPEED_RAD_S} {MAX_WHEEL_SPEED_RAD_S}",
                      forcelimited="true", forcerange=f"{-WHEEL_TORQUE_LIMIT_NM} {WHEEL_TORQUE_LIMIT_NM}")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="gyro", site="imu")
    ET.SubElement(sensors, "accelerometer", name="accelerometer", site="imu")
    ET.SubElement(sensors, "velocimeter", name="base_linvel", site="base_origin")
    ET.SubElement(sensors, "gyro", name="base_angvel", site="base_origin")
    return ET.tostring(root, encoding="unicode")


class ScoutMiniController:
    """Kinematic wheel mapping with ideal-gyro yaw PI and physical wheel servos.

    Actual chassis velocity is an outcome of tire contacts and slip, not imposed.
    Default yaw-rate PI compensates tire scrub using the ideal gyro sensor; it
    is a simulation control design, not a replay of AgileX firmware. Optional
    body-linear-velocity feedback belongs to the caller and is reported separately. The caller rejects lateral commands: a skid base cannot strafe.
    Joint signs and arrays are FL, FR, RL, RR. State snapshots are from the most
    recent update BEFORE its physics step; read_wheel_state(data) gives current
    state for post-step logs. Angles are unwrapped radians, speeds radians/sec.
    """
    wheel_names = WHEEL_NAMES

    def __init__(self, model, *, yaw_feedback=True):
        self.qpos_indices = np.array([model.joint(name).qposadr[0] for name in JOINT_NAMES])
        self.qvel_indices = np.array([model.joint(name).dofadr[0] for name in JOINT_NAMES])
        self.actuator_indices = np.array([model.actuator(name + "_wheel").id for name in WHEEL_NAMES])
        self.target_velocity = np.zeros(4)
        self.requested_target_velocity = np.zeros(4)
        self.wheel_positions = np.zeros(4)
        self.wheel_velocities = np.zeros(4)
        self.last_command = np.zeros(3)
        self.steps = self.saturation_count = 0
        self.saturation_scale = 1.
        self.yaw_feedback = bool(yaw_feedback)
        self.dt = float(model.opt.timestep)
        self.yaw_integral = 0.
        self.filtered_yaw_rate = 0.
        self.yaw_error = self.yaw_correction = self.effective_yaw_command = 0.
        self.yaw_kp, self.yaw_ki, self.yaw_filter_s = 2., 8., .05
        self.yaw_integral_limit = 2 * MAX_LINEAR_SPEED_M_S / TRACK_M

    @property
    def metadata(self):
        result = model_metadata()
        result["control"]["yaw_feedback_enabled"] = self.yaw_feedback
        return result

    @property
    def saturation_fraction(self):
        return self.saturation_count / max(1, self.steps)

    def read_wheel_state(self, data):
        return (data.qpos[self.qpos_indices].copy(), data.qvel[self.qvel_indices].copy())

    def update(self, model, data, command):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("Scout Mini command must contain three finite values [vx, vy, wz]")
        # Simulation.set_command validates vy; it is intentionally not an input
        # to skid-steer kinematics. A tracker must also suppress lateral output.
        vx, _, wz = command
        self.last_command[:] = command
        candidate_integral = self.yaw_integral
        effective_wz = float(wz)
        if self.yaw_feedback:
            measured_wz = float(data.sensor("gyro").data[2])
            if not np.isfinite(measured_wz):
                raise RuntimeError("Scout Mini gyro yaw rate is not finite")
            self.filtered_yaw_rate += -math.expm1(-self.dt / self.yaw_filter_s) * (
                measured_wz - self.filtered_yaw_rate)
            self.yaw_error = float(wz - self.filtered_yaw_rate)
            # A full zero command is a brake request: discard the prior turn's
            # integral rather than continuing wheel spin while it unwinds.
            braking = abs(vx) < 1e-12 and abs(wz) < 1e-12
            candidate_integral = 0. if braking else float(np.clip(
                self.yaw_integral + self.yaw_ki * self.yaw_error * self.dt,
                -self.yaw_integral_limit, self.yaw_integral_limit))
            effective_wz += self.yaw_kp * self.yaw_error + candidate_integral
        self.effective_yaw_command = effective_wz
        self.yaw_correction = effective_wz - float(wz)
        targets = np.array([vx - TRACK_M * effective_wz / 2,
                            vx + TRACK_M * effective_wz / 2] * 2) / WHEEL_RADIUS_M
        self.requested_target_velocity[:] = targets
        maximum = float(np.max(np.abs(targets)))
        self.saturation_scale = min(1., MAX_WHEEL_SPEED_RAD_S / maximum) if maximum else 1.
        self.target_velocity[:] = targets * self.saturation_scale
        # Freeze integration when it would push an already saturated wheel
        # target farther outward. Decay/unwinding is still allowed.
        if self.yaw_feedback and (self.saturation_scale == 1. or
                                  candidate_integral * self.yaw_error <= 0):
            self.yaw_integral = candidate_integral
        data.ctrl[self.actuator_indices] = self.target_velocity
        self.wheel_positions[:], self.wheel_velocities[:] = self.read_wheel_state(data)
        self.steps += 1
        self.saturation_count += int(self.saturation_scale < 1.)
