"""Experimental Go2W adapter for the unchanged public rl_sar RobotLab policy.

This is a simulator integration, not the canonical f1reRL LLC. That policy
previously failed a native MuJoCo zero-command parking gate (0.219321 m / 10 s,
limit 0.20 m). Its rough/stair training coverage is not certified by a manifest.
The robot starts already standing; the upstream three-second GetUp FSM is not
replayed. Physical robot parameters, policy scales and PD gains remain native.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = ROOT / ".vendor/rl_sar-go2w"
POLICY_PATH = ROOT / "mujoco_sim/assets/go2w_policy.onnx"
SOURCE_COMMIT = "36bac3930bca95239de9bcfade283832e6ee4409"
MODEL_COMMIT = "d4cca961c70c9265e92a58639f1fe9aaccb2ad0f"
MODEL_SHA256 = "a17198047a92470cf616863a65d58482b7765f57c345dd3e3f70310b8bc773b4"
POLICY_SHA256 = "d6773ed965b3f58b014aed006a0bd2ed1afcb843f4768e5b210e8f27a839e3e7"
JOINT_NAMES = tuple(f"{leg}_{part}_joint" for leg in ("FR", "FL", "RR", "RL")
                    for part in ("hip", "thigh", "calf")) + tuple(
                        f"{leg}_wheel_joint" for leg in ("FR", "FL", "RR", "RL"))
DEFAULT_POS = np.array([0., .5, -1.5, 0., .5, -1.5,
                        0., .8, -1.5, 0., .8, -1.5, 0., 0., 0., 0.], dtype=np.float32)
ACTION_SCALE = np.array([.125, .25, .25] * 4 + [5.] * 4, dtype=np.float32)
POLICY_PERIOD_S = .02
PD_PERIOD_S = .005


def build_go2w_xml(asset_root: Path = ASSET_ROOT) -> str:
    """Return pinned robot-only MJCF plus ideal sites/sensors and a ready keyframe.

    Root integration adds the selected terrain. Existing model mesh references
    become absolute paths into the local ignored vendor copy; external source
    workspaces are never opened at runtime or modified.
    """
    asset_root = Path(asset_root)
    path = asset_root / "mjcf/go2w.xml"
    if not path.is_file():
        raise FileNotFoundError("Go2W assets missing; run mujoco_sim/scripts/bootstrap_rl_sar_assets.py")
    if hashlib.sha256(path.read_bytes()).hexdigest() != MODEL_SHA256:
        raise ValueError("Go2W MJCF differs from the pinned native model")
    root = ET.parse(path).getroot()
    root.set("model", "go2w_rl_sar_experimental")
    root.find("compiler").set("meshdir", str((path.parent / "assets").resolve()))
    root.find("option").set("timestep", "0.002")
    base = root.find("./worldbody/body[@name='base_link']")
    for name, pos in (("base_origin", "0 0 0"), ("lidar", ".02 0 .18"),
                      ("camera_mount", ".32 0 .1")):
        ET.SubElement(base, "site", name=name, pos=pos, size=".002")
    sensors = root.find("sensor")
    sensors.find("gyro[@name='imu_gyro']").set("name", "gyro")
    sensors.find("accelerometer[@name='imu_acc']").set("name", "accelerometer")
    ET.SubElement(sensors, "velocimeter", name="base_linvel", site="base_origin")
    ET.SubElement(sensors, "gyro", name="base_angvel", site="base_origin")
    # MuJoCo qpos follows XML tree order, which differs from policy/motor order.
    position_by_name = dict(zip(JOINT_NAMES, DEFAULT_POS))
    tree_joints = [joint.get("name") for joint in root.findall(".//body/joint")]
    qpos = [0., 0., .45, 1., 0., 0., 0.] + [float(position_by_name[name]) for name in tree_joints]
    keyframes = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframes, "key", name="ready", qpos=" ".join(map(str, qpos)),
                  ctrl=" ".join(["0"] * 16))  # actuators take torque, not q targets
    return ET.tostring(root, encoding="unicode")


def build_observation(angular_velocity, gravity, command, position, velocity, previous_action):
    """Exact published 57-D order; continuous wheel positions are zeroed."""
    values = [np.asarray(x, dtype=np.float32) for x in
              (angular_velocity, gravity, command, position, velocity, previous_action)]
    if [v.shape for v in values] != [(3,), (3,), (3,), (16,), (16,), (16,)]:
        raise ValueError("Go2W observation shape mismatch")
    gyro, gravity, command, position, velocity, previous_action = values
    relative = position - DEFAULT_POS
    relative[12:] = 0.
    obs = np.concatenate((gyro * .25, gravity, command, relative, velocity * .05, previous_action))
    if not np.isfinite(obs).all():
        raise ValueError("Go2W observation contains non-finite values")
    return np.clip(obs, -100., 100.)


class Go2WController:
    """50 Hz ONNX policy, native 200 Hz torque-PD updates on the physics clock."""

    def __init__(self, model, policy_path: Path = POLICY_PATH):
        import onnxruntime as ort
        policy_path = Path(policy_path)
        if hashlib.sha256(policy_path.read_bytes()).hexdigest() != POLICY_SHA256:
            raise ValueError("Go2W ONNX policy differs from the verified public export")
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.policy = ort.InferenceSession(str(policy_path), sess_options=options,
                                          providers=["CPUExecutionProvider"])
        if self.policy.get_inputs()[0].shape != [1, 57] or self.policy.get_outputs()[0].shape != [1, 16]:
            raise ValueError("Go2W policy must map [1,57] to [1,16]")
        self.qpos_indices = np.array([model.joint(name).qposadr[0] for name in JOINT_NAMES])
        self.qvel_indices = np.array([model.joint(name).dofadr[0] for name in JOINT_NAMES])
        self.actuator_indices = np.array([model.actuator(name.removesuffix("_joint")).id for name in JOINT_NAMES])
        self.default = DEFAULT_POS.copy()
        self.last_action = np.zeros(16, dtype=np.float32)
        self.last_observation = np.zeros(57, dtype=np.float32)
        self.target_position = self.default.copy()
        self.target_velocity = np.zeros(16, dtype=np.float32)
        self.steps = self.policy_calls = self.pd_calls = 0
        self.dt = float(model.opt.timestep)
        if self.dt > PD_PERIOD_S or abs(round(POLICY_PERIOD_S / self.dt) * self.dt - POLICY_PERIOD_S) > 1e-9:
            raise ValueError("Go2W needs physics dt <= .005 s dividing the .02 s policy period")
        self.substeps = round(POLICY_PERIOD_S / self.dt)
        self.next_pd_time = 0.

    def update(self, model, data, command):
        if self.steps % self.substeps == 0:
            rotation = data.site("imu").xmat.reshape(3, 3)
            self.last_observation = build_observation(
                data.sensor("gyro").data, rotation.T @ np.array([0., 0., -1.]), command,
                data.qpos[self.qpos_indices], data.qvel[self.qvel_indices], self.last_action)
            output = self.policy.run(["actions"], {"obs": self.last_observation[None]})[0][0]
            if not np.isfinite(output).all():
                raise RuntimeError("Go2W policy returned non-finite actions")
            self.last_action = np.clip(output, -100., 100.)
            self.target_position[:12] = self.default[:12] + self.last_action[:12] * ACTION_SCALE[:12]
            self.target_velocity[12:] = self.last_action[12:] * ACTION_SCALE[12:]
            self.policy_calls += 1
        # Native PD period .005 is not an integer multiple of .002 physics.
        # Quantize each deadline to the first following physics tick (2/3 steps).
        if self.steps * self.dt + 1e-10 >= self.next_pd_time:
            q, dq = data.qpos[self.qpos_indices], data.qvel[self.qvel_indices]
            torque = .5 * (self.target_velocity - dq)
            torque[:12] += 20. * (self.target_position[:12] - q[:12])
            # Native motor ctrlranges implement limiting (wheel15Nm, other native
            # MJCF limits). The C++ FSM sets feed-forward tau=0, not a second PD.
            data.ctrl[self.actuator_indices] = torque
            self.pd_calls += 1
            self.next_pd_time = self.pd_calls * PD_PERIOD_S
        self.steps += 1
