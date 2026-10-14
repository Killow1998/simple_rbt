"""MuJoCo stepping, robot control and ideal sensor readings; no ROS dependency."""
from pathlib import Path
from dataclasses import replace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from .robots import (ROBOT_NAMES, WHEEL_MODELS, NO_LATERAL_ROBOTS, COMMAND_LIMITS,
                     wheel_model, command_is_supported, validate_terrain)

from .paths import ROOT, SCAN_CSV
UPSTREAM = ROOT / ".vendor" / "MuJoCo-LiDAR"
UPSTREAM_COMMIT = "e6b879df4b96daaed30ef10fe02650d29048fa9b"
BODY_FROM_OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def _add_environment(root, terrain="flat", difficulty=1., seed=0, terrain_level="standard",
                     terrain_friction=None):
    from .terrains import append_terrain
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos="0 0 6", dir="0 0 -1", directional="true")
    ET.SubElement(world, "geom", name="floor", type="plane", size="12 12 0.1",
                  rgba="0.35 0.4 0.45 1", friction="1 0.01 0.001", group="1")
    for name, pos, size, color in [
        ("north", "0 6 1.5", "12 .1 1.5", ".7 .4 .3 1"),
        ("south", "0 -6 1.5", "12 .1 1.5", ".3 .6 .7 1"),
        ("east", "12 0 1.5", ".1 6 1.5", ".7 .7 .4 1"),
        ("west", "-12 0 1.5", ".1 6 1.5", ".4 .7 .4 1"),
    ]:
        ET.SubElement(world, "geom", name=name, type="box", pos=pos, size=size,
                      rgba=color, group="1")
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1024", offheight="1024")
    ET.SubElement(visual, "map", znear="0.005", zfar="50")
    profile = append_terrain(root, terrain, difficulty=difficulty, seed=seed, level=terrain_level)
    if terrain_friction is not None:
        coefficient = float(terrain_friction)
        if not np.isfinite(coefficient) or not 0 < coefficient <= 2:
            raise ValueError("Terrain friction must be finite and in (0, 2]")
        friction = (coefficient, .02, .01)
        # Foot/wheel priority=1 would otherwise override the requested surface.
        # Explicit test surfaces win the contact mix; default MJCF is untouched.
        for geom in world.findall("geom"):
            if geom.get("name") == "floor" or geom.get("name", "").startswith("terrain_"):
                geom.set("friction", " ".join(format(value, ".17g") for value in friction))
                geom.set("priority", "2")
                geom.set("condim", "6")
        profile = replace(profile, friction=friction, parameters={
            **profile.parameters, "friction_override": {
                "sliding": coefficient, "torsional": .02, "rolling": .01,
                "surface_priority": 2, "contact_dimension": 6,
                "scope": "floor and course; robot and walls unchanged"}})
    return profile


def _go2_xml(upstream):
    path = upstream / "models/unitree_go2/go2_mjx.xml"
    if not path.exists():
        raise FileNotFoundError("Go2 assets missing; run ./setup.sh first")
    root = ET.parse(path).getroot()
    for mesh in root.findall("./asset/mesh"):
        mesh.set("file", str((path.parent / mesh.attrib["file"]).resolve()))
    base = root.find("./worldbody/body[@name='base']")
    # A top-mounted MID360 is different from the upstream inverted Go2 front lidar.
    site = base.find("site[@name='lidar']")
    site.set("pos", "0.02 0 0.18")
    site.set("quat", "1 0 0 0")
    ET.SubElement(base, "site", name="base_origin", pos="0 0 0", size="0.002")
    ET.SubElement(base, "site", name="camera_mount", pos="0.32 0 0.1", size="0.002")
    sensors = root.find("sensor")
    ET.SubElement(sensors, "velocimeter", name="base_linvel", site="base_origin")
    ET.SubElement(sensors, "gyro", name="base_angvel", site="base_origin")
    key = ET.parse(upstream / "models/scene_go2.xml").getroot().find("keyframe")
    root.append(key)
    return ET.tostring(root, encoding="unicode")


def _diffdrive_xml():
    # Minimal physical two-wheel base, with passive low-friction casters.
    root = ET.fromstring('''<mujoco model="diffdrive">
      <compiler angle="radian"/>
      <option timestep="0.004" integrator="implicitfast"/>
      <default><geom friction="1 0.005 0.001"/><joint damping="0.05"/></default>
      <worldbody><body name="base" pos="0 0 0.16">
        <freejoint/>
        <geom name="chassis" type="box" size=".23 .15 .06" mass="8" rgba=".2 .4 .8 1"/>
        <site name="base_origin"/><site name="imu" pos="0 0 .07"/>
        <site name="lidar" pos="0 0 .15"/><site name="camera_mount" pos=".24 0 .08"/>
        <geom name="front_caster_collision" type="sphere" size=".025" pos=".18 0 -.13" mass=".01" friction=".001 .001 .001"/>
        <geom name="rear_caster_collision" type="sphere" size=".025" pos="-.18 0 -.13" mass=".01" friction=".001 .001 .001"/>
        <body name="left_wheel" pos="0 .18 -.04">
          <joint name="left" type="hinge" axis="0 1 0"/>
          <geom type="cylinder" size=".12 .025" quat=".70710678 .70710678 0 0" mass=".3" rgba=".1 .1 .1 1"/>
        </body>
        <body name="right_wheel" pos="0 -.18 -.04">
          <joint name="right" type="hinge" axis="0 1 0"/>
          <geom type="cylinder" size=".12 .025" quat=".70710678 .70710678 0 0" mass=".3" rgba=".1 .1 .1 1"/>
        </body>
      </body></worldbody>
      <actuator><velocity joint="left" kv="3" forcelimited="true" forcerange="-8 8"/>
        <velocity joint="right" kv="3" forcelimited="true" forcerange="-8 8"/></actuator>
      <sensor><gyro name="gyro" site="imu"/><accelerometer name="accelerometer" site="imu"/>
        <velocimeter name="base_linvel" site="base_origin"/><gyro name="base_angvel" site="base_origin"/></sensor>
    </mujoco>''')
    return ET.tostring(root, encoding="unicode")


class Go2Controller:
    """The upstream Go2 observation/action contract with an external velocity command."""
    def __init__(self, model, policy_path):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.policy = ort.InferenceSession(str(policy_path), sess_options=options,
                                          providers=["CPUExecutionProvider"])
        self.default = model.key_qpos[0, 7:19].copy()
        self.last_action = np.zeros(12, dtype=np.float32)
        self.steps = 0
        self.substeps = round(0.02 / model.opt.timestep)

    def update(self, model, data, command):
        if self.steps % self.substeps == 0:
            rotation = data.site_xmat[model.site("imu").id].reshape(3, 3)
            obs = np.concatenate([
                data.sensor("local_linvel").data, data.sensor("gyro").data,
                rotation.T @ np.array([0., 0., -1.]),
                data.qpos[7:19] - self.default, data.qvel[6:18],
                self.last_action, command,
            ]).astype(np.float32)
            self.last_action = self.policy.run(["continuous_actions"], {"obs": obs[None]})[0][0]
            data.ctrl[:] = self.default + 0.5 * self.last_action
        self.steps += 1


class Simulation:
    """One robot in a small obstacle room; sensors and transport remain optional."""
    def __init__(self, robot="go2", upstream=UPSTREAM, *, terrain="flat", difficulty=1., seed=0,
                 controller="auto", velocity_feedback=False, terrain_level="standard",
                 terrain_friction=None, go2_torque_mode="configured", go2_startup="crouched"):
        if robot not in ROBOT_NAMES:
            raise ValueError(f"Unknown robot: {robot}")
        validate_terrain(robot, terrain)
        self.robot = robot
        if robot == "go2" and controller == "auto":
            controller = "robot_lab"
        if go2_torque_mode not in ("configured", "upstream"):
            raise ValueError("Go2 torque mode must be configured or upstream")
        if go2_startup not in ("crouched", "upstream_getup"):
            raise ValueError("Go2 startup must be crouched or upstream_getup")
        if (robot != "go2" or controller == "legacy") and (
                go2_torque_mode != "configured" or go2_startup != "crouched"):
            raise ValueError("Deployment alternatives require the Go2 HIMLoco or RobotLab controller")
        self.go2_torque_mode = go2_torque_mode
        self.go2_startup = go2_startup
        if robot == "go2w":
            if controller not in ("auto", "rl_sar"):
                raise ValueError(f"Unsupported controller {controller} for go2w")
            from .go2w import build_go2w_xml
            xml = build_go2w_xml()
            controller = "rl_sar"
        elif robot in WHEEL_MODELS:
            builder, wheel_controller, required_controller = wheel_model(robot)
            if controller not in ("auto", required_controller):
                raise ValueError(f"Unsupported controller {controller} for {robot}")
            xml = builder()
            controller = required_controller
        elif robot == "go2" and controller in ("himloco", "robot_lab"):
            from .go2 import build_go2_xml
            xml = build_go2_xml(controller)
        elif robot == "go2" and controller in ("auto", "legacy"):
            xml = _go2_xml(Path(upstream))
            controller = "legacy"
        elif robot == "diffdrive" and controller in ("auto", "none"):
            xml = _diffdrive_xml()
            controller = "none"
        else:
            raise ValueError(f"Unsupported controller {controller} for {robot}")
        root = ET.fromstring(xml)
        self.terrain = _add_environment(root, terrain, difficulty, seed, terrain_level, terrain_friction)
        from .mid360 import append_mid360_imu
        self.mid360_metadata = append_mid360_imu(root)
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.controller_name = controller
        if robot == "go2w":
            from .go2w import Go2WController
            self.controller = Go2WController(self.model)
        elif robot in WHEEL_MODELS:
            self.controller = wheel_controller(self.model)
        elif robot == "go2" and controller != "legacy":
            from .go2 import Go2Controller as RLSarController
            self.controller = RLSarController(self.model, controller, torque_mode=go2_torque_mode)
        elif robot == "go2":
            self.controller = Go2Controller(self.model, Path(upstream) / "examples/onnx/go2_policy.onnx")
        else:
            self.controller = None
        self.startup_controller = None
        if robot == "go2" and go2_startup == "upstream_getup":
            from .go2_deployment import Go2GetUp
            self.startup_controller = Go2GetUp(self.model, self.data)
        self.command = np.zeros(3)
        self.policy_command = np.zeros(3)
        self.command_limits = np.array(COMMAND_LIMITS[robot])
        self.command_tracker = None
        if velocity_feedback and self.controller:
            from .command_tracking import CommandTracker
            if robot in NO_LATERAL_ROBOTS:
                # Skid steering has no lateral actuator. Feedback adjusts only
                # longitudinal speed and yaw through the ordinary wheel command.
                self.command_tracker = CommandTracker(kp=(.8, 0., 1.), ki=(.8, 0., .5),
                    command_limits=(self.command_limits[0], .6, self.command_limits[2]))
            elif robot in WHEEL_MODELS:
                self.command_tracker = CommandTracker(command_limits=self.command_limits)
            else:
                self.command_tracker = CommandTracker()
            self.command_tracker.brake_on_zero = robot in WHEEL_MODELS
        self.steps = 0

    @property
    def startup_complete(self):
        return self.startup_controller is None or self.startup_controller.complete

    @property
    def startup_phase(self):
        return self.startup_controller.phase if not self.startup_complete else "policy"

    @property
    def startup_steps(self):
        return self.startup_controller.steps if self.startup_controller else 0

    @property
    def startup_elapsed_s(self):
        return self.startup_controller.elapsed_s if self.startup_controller else 0.

    @property
    def deployment_metadata(self):
        if self.robot in WHEEL_MODELS:
            return {"robot": self.robot, "controller": self.controller_name,
                    "go2_options_applicable": False, **self.controller.metadata}
        if self.robot != "go2" or self.controller_name == "legacy":
            return {"robot": self.robot, "controller": self.controller_name,
                    "go2_options_applicable": False}
        return {"robot": self.robot, "controller": self.controller_name,
                "go2_torque_mode": self.go2_torque_mode,
                "go2_startup": self.go2_startup,
                "policy_period_s": self.controller.substeps * self.model.opt.timestep,
                "physics_period_s": self.model.opt.timestep,
                "configured_torque_limits_nm": self.controller.torque_limits.tolist(),
                "actuator_ctrlrange_nm": self.model.actuator_ctrlrange[
                    self.controller.actuator_indices].tolist(),
                "startup": self.startup_controller.metadata if self.startup_controller else None,
                "limitation": "Deterministic simulator comparison; automatic startup transition and ideal state feedback are not a hardware FSM replay"}

    @property
    def stamp_ns(self):
        return round(self.steps * self.model.opt.timestep * 1_000_000_000)

    def set_command(self, vx=0., vy=0., wz=0.):
        try:
            command = np.array([vx, vy, wz], dtype=float)
        except (TypeError, ValueError):
            self.command[:] = 0.
            raise ValueError("Velocity command must contain three finite numbers") from None
        if command.shape != (3,):
            self.command[:] = 0.
            raise ValueError("Velocity command must contain three scalar numbers")
        if not np.isfinite(command).all():
            self.command[:] = 0.
            raise ValueError("Velocity command must be finite")
        if not command_is_supported(self.robot, *command):
            self.command[:] = 0.
            raise ValueError(f"Unsupported planar command for {self.robot}; lateral motion or in-place Ackermann turning is unavailable")
        self.command[:] = np.clip(command, -self.command_limits, self.command_limits)

    def step(self):
        self.policy_command[:] = self.command
        starting = not self.startup_complete
        if starting:
            self.policy_command[:] = 0.
        elif self.command_tracker:
            rotation = self.data.site_xmat[self.model.site("base_origin").id].reshape(3, 3)
            linear, angular = self.velocity()
            self.policy_command[:] = self.command_tracker.update(
                self.command, linear, angular, np.arctan2(rotation[1, 0], rotation[0, 0]), self.model.opt.timestep)
            if self.robot in NO_LATERAL_ROBOTS:
                self.policy_command[1] = 0.
            if self.robot == "fr_mini":
                if np.max(np.abs(self.command)) == 0.:
                    self.policy_command[:] = 0.
                else:
                    yaw_limit = abs(self.policy_command[0]) * self.controller.max_curvature
                    self.policy_command[2] = np.clip(self.policy_command[2], -yaw_limit, yaw_limit)
                    if abs(self.policy_command[0]) <= 1e-9:
                        self.policy_command[2] = 0.
        if starting:
            self.startup_controller.update(self.model, self.data)
        elif self.controller:
            self.controller.update(self.model, self.data, self.policy_command)
        else:
            vx, _, wz = self.command
            self.data.ctrl[:] = [(vx - .18 * wz) / .12, (vx + .18 * wz) / .12]
        mujoco.mj_step(self.model, self.data)
        self.steps += 1
        # mj_step ends with integrated qpos; refresh derived poses and sensors to the same time.
        mujoco.mj_forward(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or abs(self.data.time - self.steps * self.model.opt.timestep) > 1e-6:
            raise RuntimeError("Simulation became unstable or MuJoCo reset the state")

    def pose(self, site="base_origin"):
        site_id = self.model.site(site).id
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, self.data.site_xmat[site_id])
        return self.data.site_xpos[site_id].copy(), quaternion[[1, 2, 3, 0]]

    def extrinsics(self, site):
        idx = self.model.site(site).id
        quat = self.model.site_quat[idx]
        return self.model.site_pos[idx].copy(), quat[[1, 2, 3, 0]].copy()

    def imu(self):
        """Device IMU in SI; quaternion is simulation truth, not a hardware reading.

        The policy still uses its original robot IMU. The MID360 accelerometer
        is evaluated at its own site, including rigid-body lever-arm effects.
        ROS raw output does not publish this truth quaternion.
        """
        return (self.data.sensor("mid360_gyro").data.copy(),
                self.data.sensor("mid360_accelerometer").data.copy(), self.pose("mid360_imu")[1])

    def velocity(self):
        return self.data.sensor("base_linvel").data.copy(), self.data.sensor("base_angvel").data.copy()

    def camera_pose(self):
        idx = self.model.site("camera_mount").id
        return self.data.site_xpos[idx].copy(), self.data.site_xmat[idx].reshape(3, 3) @ BODY_FROM_OPTICAL
