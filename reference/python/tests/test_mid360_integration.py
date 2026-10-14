"""MID-360 sensor attachment must not alter robot dynamics or policy IMUs."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from simple_rbt_mujoco import mid360
from simple_rbt_mujoco.core import Simulation


@pytest.mark.parametrize("robot,controller", [("diffdrive", "auto"), ("go2", "auto"),
                                               ("go2w", "auto"), ("go2", "legacy"),
                                               ("scout_mini", "auto"), ("fr_mini", "auto"),
                                               ("ridgeback", "auto"), ("ranger_mini", "auto")])
def test_device_imu_attachment_preserves_motion_and_original_imu(robot, controller, monkeypatch):
    # Capture the exact source compiled by Simulation. Saving compiled MJCF could
    # change serialized precision, which would introduce an unrelated variable.
    original_append = mid360.append_mid360_imu
    captured = []

    def capture(root):
        metadata = original_append(root)
        captured.append(ET.tostring(root, encoding="unicode"))
        return metadata

    monkeypatch.setattr(mid360, "append_mid360_imu", capture)
    sim = Simulation(robot, controller=controller)
    assert len(captured) == 1
    baseline_root = ET.fromstring(captured[0])
    removed = []
    expected = {"mid360_imu": "site", "mid360_gyro": "gyro",
                "mid360_accelerometer": "accelerometer"}
    for parent in baseline_root.iter():
        for child in list(parent):
            if child.get("name") in expected:
                assert child.tag == expected[child.get("name")]
                removed.append(child.get("name"))
                parent.remove(child)
    assert sorted(removed) == sorted(expected)
    baseline = mujoco.MjModel.from_xml_string(ET.tostring(baseline_root, encoding="unicode"))
    baseline_data = mujoco.MjData(baseline)
    if baseline.nkey:
        mujoco.mj_resetDataKeyframe(baseline, baseline_data, 0)
    mujoco.mj_forward(baseline, baseline_data)
    assert sim.model.nsite == baseline.nsite + 1
    assert sim.model.nsensor == baseline.nsensor + 2
    np.testing.assert_array_equal(sim.model.body_mass, baseline.body_mass)
    np.testing.assert_array_equal(sim.model.body_inertia, baseline.body_inertia)

    for name in ("gyro", "accelerometer"):
        # Original policy sensors still point to the original site, not the device.
        assert sim.model.sensor(name).objid[0] == sim.model.site("imu").id
        assert baseline.sensor(name).objid[0] == baseline.site("imu").id
    if robot == "go2" and controller != "legacy":
        assert sim.controller.imu_site == sim.model.site("imu").id

    sampler, emitted, delays = mid360.Mid360ImuSampler(), [], []

    def verify_state_and_sample():
        assert sim.data.time == baseline_data.time
        for name in ("qpos", "qvel", "ctrl"):
            np.testing.assert_array_equal(getattr(sim.data, name), getattr(baseline_data, name))
        for name in ("gyro", "accelerometer"):
            np.testing.assert_array_equal(sim.data.sensor(name).data, baseline_data.sensor(name).data)
        lidar = sim.data.site("lidar")
        device = sim.data.site("mid360_imu")
        np.testing.assert_allclose(lidar.xmat.reshape(3, 3).T @ (device.xpos - lidar.xpos),
                                   mid360.IMU_IN_LIDAR_POSITION, atol=3e-15, rtol=0)
        np.testing.assert_allclose(device.xmat, lidar.xmat, atol=3e-15, rtol=0)
        gyro, acceleration, truth_quaternion = sim.imu()
        np.testing.assert_array_equal(gyro, sim.data.sensor("mid360_gyro").data)
        np.testing.assert_array_equal(acceleration, sim.data.sensor("mid360_accelerometer").data)
        np.testing.assert_array_equal(truth_quaternion, sim.pose("mid360_imu")[1])
        # The hardware sample deliberately does not carry core's truth quaternion.
        samples = sampler.observe(sim.stamp_ns, gyro, acceleration)
        assert sampler.metadata["orientation_available"] is False
        for sample in samples:
            assert not hasattr(sample, "orientation")
            assert not hasattr(sample, "quaternion")
            assert sample.source_stamps_ns[0] <= sample.stamp_ns <= sample.source_stamps_ns[1]
            delays.append(sim.stamp_ns - sample.stamp_ns)
        emitted.extend(samples)

    verify_state_and_sample()  # stamp zero is a real sample, not a false sentinel.
    sim.set_command(.3, 0., 0.)
    for _ in range(100):
        sim.step()
        baseline_data.ctrl[:] = sim.data.ctrl
        mujoco.mj_step(baseline, baseline_data)
        mujoco.mj_forward(baseline, baseline_data)
        verify_state_and_sample()
    expected_stamps = list(range(0, sim.stamp_ns + 1, mid360.IMU_PERIOD_NS))
    assert [sample.stamp_ns for sample in emitted] == expected_stamps
    assert max(delays) <= round(sim.model.opt.timestep * 1e9)
    if robot == "go2w":
        assert sim.model.opt.timestep == .002
        assert sim.stamp_ns == 200_000_000
        assert len(emitted) == 41
        assert max(delays) == 1_000_000
        assert emitted[-1].stamp_ns == sim.stamp_ns
