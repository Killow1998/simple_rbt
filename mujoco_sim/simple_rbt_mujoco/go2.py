"""Explicit Go2 deployment policies; ONNX CPU inference, no Torch dependency.

Canonical rl_sar torque dynamics and policy contracts are pinned in the asset
manifest. These alternatives have measured tracking limitations; retaining the
original MuJoCo-LiDAR controller as ``legacy`` allows honest comparisons.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mujoco_sim/assets/go2_manifest.json"
ASSETS = ROOT / ".vendor/rl_sar-go2"
TORQUE_MODES = ("configured", "upstream")


def _config(controller):
    manifest = json.loads(MANIFEST.read_text())
    if controller not in manifest["controllers"]:
        raise ValueError("Go2 deployment controller must be 'himloco' or 'robot_lab'")
    return manifest, manifest["controllers"][controller]


def build_go2_xml(controller="himloco", *, asset_directory=ASSETS):
    """Canonical torque-driven model with sensor sites; caller adds environment.

    Joint/inertial/contact parameters remain from pinned rl_sar_zoo. The 5 ms
    physics and 20 ms policy periods follow policy/go2/base.yaml. implicitfast
    integrates the model's physical damping. Canonical crouched home pose stays.
    """
    _config(controller)
    assets = Path(asset_directory)
    path = assets / "mjcf/go2.xml"
    if not path.exists():
        raise FileNotFoundError("Go2 assets missing; run bootstrap_rl_sar_assets.py "
                                "--manifest mujoco_sim/assets/go2_manifest.json")
    root = ET.parse(path).getroot()
    root.find("compiler").set("meshdir", str((assets / "mjcf/assets").resolve()))
    root.find("option").set("timestep", "0.005")
    root.find("option").set("integrator", "implicitfast")
    base = root.find("./worldbody/body[@name='base_link']")
    base.set("name", "base")  # Uniform simulator body lookup; physical parameters unchanged.
    for name, pos in (("base_origin", "0 0 0"), ("lidar", "0.02 0 0.18"),
                      ("camera_mount", "0.32 0 0.1")):
        ET.SubElement(base, "site", name=name, pos=pos, size="0.002")
    sensors = root.find("sensor")
    for sensor in sensors:
        if sensor.get("name") == "imu_gyro":
            sensor.set("name", "gyro")
        elif sensor.get("name") == "imu_acc":
            sensor.set("name", "accelerometer")
    ET.SubElement(sensors, "velocimeter", name="base_linvel", site="base_origin")
    ET.SubElement(sensors, "gyro", name="base_angvel", site="base_origin")
    for key in root.findall("./keyframe/key"):
        key.set("ctrl", " ".join(["0"] * 12))  # These actuators command torque.
    return ET.tostring(root, encoding="unicode")


class Go2Controller:
    """Named joint mapping, newest-first history and physics-rate torque PD."""

    def __init__(self, model, controller="himloco", *, torque_mode="configured"):
        if torque_mode not in TORQUE_MODES:
            raise ValueError(f"Unknown Go2 torque mode {torque_mode!r}; choose from {TORQUE_MODES}")
        self.torque_mode = torque_mode
        import onnxruntime as ort
        manifest, self.config = _config(controller)
        self.name = controller
        export = manifest["policy_exports"][controller]
        policy = MANIFEST.parent / export["onnx_file"]
        if hashlib.sha256(policy.read_bytes()).hexdigest() != export["onnx_sha256"]:
            raise ValueError("Go2 ONNX policy differs from its recorded export")
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.policy = ort.InferenceSession(str(policy), sess_options=options,
                                          providers=["CPUExecutionProvider"])
        self.joint_names = [manifest["sdk_joint_names"][i]
                            for i in self.config["joint_mapping"]]
        joints = [model.joint(name).id for name in self.joint_names]
        self.qpos_indices = np.asarray([model.jnt_qposadr[j] for j in joints])
        self.qvel_indices = np.asarray([model.jnt_dofadr[j] for j in joints])
        actuators = []
        for joint in joints:
            matches = np.flatnonzero(model.actuator_trnid[:, 0] == joint)
            if len(matches) != 1:
                raise ValueError("Each policy joint needs exactly one torque actuator")
            actuators.append(matches[0])
        self.actuator_indices = np.asarray(actuators)
        self.default = np.asarray(self.config["default_dof_pos"])
        self.action_scale = np.asarray(self.config["action_scale"])
        self.kp = np.asarray(self.config["rl_kp"])
        self.kd = np.asarray(self.config["rl_kd"])
        self.torque_limits = np.asarray(self.config["torque_limits"])
        self.command_scale = np.asarray(self.config["commands_scale"])
        self.last_action = np.zeros(12, dtype=np.float32)
        self.steps = 0
        self.substeps = round(0.02 / model.opt.timestep)
        if self.substeps < 1 or not np.isclose(self.substeps * model.opt.timestep, 0.02):
            raise ValueError("Physics timestep must divide the 20 ms policy period")
        self.history_indices = self.config["observations_history"] or [0]
        self.history = np.zeros((max(self.history_indices) + 1, 45), dtype=np.float32)
        self.imu_site = model.site("imu").id

    def observation(self, model, data, command):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("Velocity command must contain three finite values")
        rotation = data.site_xmat[self.imu_site].reshape(3, 3)
        terms = {
            "commands": command * self.command_scale,
            "ang_vel": data.sensor("gyro").data * self.config["ang_vel_scale"],
            "gravity_vec": rotation.T @ np.array([0., 0., -1.]),
            "dof_pos": (data.qpos[self.qpos_indices] - self.default) * self.config["dof_pos_scale"],
            "dof_vel": data.qvel[self.qvel_indices] * self.config["dof_vel_scale"],
            "actions": self.last_action,
        }
        observation = np.concatenate([terms[name] for name in self.config["observations"]])
        return np.clip(observation, -self.config["clip_obs"], self.config["clip_obs"]).astype(np.float32)

    def update(self, model, data, command):
        if self.steps % self.substeps == 0:
            observation = self.observation(model, data, command)
            self.history[1:] = self.history[:-1]
            self.history[0] = observation
            if self.config["observations_history_priority"] != "time":
                raise ValueError("Unsupported policy history layout")
            inputs = self.history[self.history_indices].reshape(1, -1)
            action = self.policy.run(["actions"], {"obs": inputs})[0][0]
            if action.shape != (12,) or not np.isfinite(action).all():
                raise RuntimeError("Go2 policy produced an invalid action")
            self.last_action = np.clip(action, self.config["clip_actions_lower"],
                                       self.config["clip_actions_upper"])
        torque = self.kp * (self.default + self.last_action * self.action_scale
                            - data.qpos[self.qpos_indices]) - self.kd * data.qvel[self.qvel_indices]
        # The pinned upstream RLControl sends position targets and zero feedforward
        # torque; SetCommand computes this PD torque without the config clamp.
        # MuJoCo still applies the unchanged native actuator ctrlrange in either mode.
        data.ctrl[self.actuator_indices] = (np.clip(torque, -self.torque_limits, self.torque_limits)
                                            if self.torque_mode == "configured" else torque)
        self.steps += 1
