import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from simple_rbt_mujoco.go2w import (ASSET_ROOT, DEFAULT_POS, JOINT_NAMES, POLICY_PATH,
                                   POLICY_SHA256, Go2WController, build_go2w_xml,
                                   build_observation)


def flat_robot():
    xml = ET.fromstring(build_go2w_xml())
    ET.SubElement(xml.find("worldbody"), "geom", type="plane", size="0 0 .05")
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


def test_public_policy_and_native_asset_manifest():
    from simple_rbt_mujoco.paths import MANIFESTS
    manifest = json.loads((MANIFESTS / "go2w_manifest.json").read_text())
    for entry in manifest["files"]:
        raw = (ASSET_ROOT / entry["path"]).read_bytes()
        assert len(raw) == entry["bytes"]
        assert hashlib.sha256(raw).hexdigest() == entry["sha256"]
    assert hashlib.sha256(POLICY_PATH.read_bytes()).hexdigest() == POLICY_SHA256


def test_ready_pose_uses_named_joint_order_and_complete_sensor_contract():
    model, data = flat_robot()
    controller = Go2WController(model)
    assert model.nu == 16 and model.nv == 22
    np.testing.assert_allclose(data.qpos[controller.qpos_indices], DEFAULT_POS)
    assert tuple(model.joint(i).name for i in range(1, model.njnt)) != JOINT_NAMES
    for name in ("base_origin", "imu", "lidar", "camera_mount"):
        assert model.site(name).id >= 0
    for name in ("gyro", "accelerometer", "base_linvel", "base_angvel"):
        assert model.sensor(name).dim[0] == 3


def test_observation_order_scales_and_continuous_wheel_mask():
    position = DEFAULT_POS + np.arange(16)
    obs = build_observation([4, 8, 12], [0, 0, -1], [.4, -.2, .3],
                            position, np.arange(16) * 20, np.arange(16) / 10)
    np.testing.assert_array_equal(obs[:6], [1, 2, 3, 0, 0, -1])
    np.testing.assert_allclose(obs[6:9], [.4, -.2, .3])
    np.testing.assert_allclose(obs[9:21], np.arange(12))
    np.testing.assert_array_equal(obs[21:25], 0)
    np.testing.assert_array_equal(obs[25:41], np.arange(16))
    np.testing.assert_allclose(obs[41:], np.arange(16) / 10)


def test_policy_pd_schedule_and_short_flat_motion():
    model, data = flat_robot()
    controller = Go2WController(model)
    command = np.zeros(3)
    start = None
    min_height = 1.
    for i in range(2500):
        if i == 1000:
            start = data.qpos[:3].copy()
            command[0] = .3
        controller.update(model, data, command)
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        assert np.isfinite(data.qpos).all()
        min_height = min(min_height, data.qpos[2])
    assert controller.policy_calls == 250
    assert controller.pd_calls == 1000
    assert min_height > .25
    assert data.qpos[0] - start[0] > .3
    assert abs(data.qpos[1] - start[1]) < .3
