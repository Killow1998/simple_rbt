"""Observable GetUp timing, interpolation and unassisted physical startup."""
import json
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from simple_rbt_mujoco.go2 import ASSETS, build_go2_xml
from simple_rbt_mujoco.go2_deployment import Go2GetUp


def model_and_data():
    root = ET.fromstring(build_go2_xml("himloco"))
    ET.SubElement(root.find("worldbody"), "geom", type="plane", size="10 10 .1",
                  friction="1 .01 .001")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


@unittest.skipUnless((ASSETS / "mjcf/go2.xml").exists(), "optional Go2 assets not installed")
class Go2GetUpTests(unittest.TestCase):
    def test_schedule_preserves_float32_boundary_and_changes_only_motor_control(self):
        model, data = model_and_data()
        startup = Go2GetUp(model, data)
        initial_qpos, initial_qvel = data.qpos.copy(), data.qvel.copy()
        self.assertEqual(startup.joint_names[:3], ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"])
        pre = np.tile(np.array([0., 1.36, -2.65], dtype=np.float32), 4)
        stand = np.tile(np.array([0., .8, -1.5], dtype=np.float32), 4)
        self.assertFalse(startup.update(model, data))
        first = np.float32(1.) / np.float32(200)
        np.testing.assert_array_equal(startup.target_position,
                                      (np.float32(1.) - first) * startup.start_position + first * pre)
        for _ in range(199):
            startup.update(model, data)
        self.assertEqual(startup.steps, 200)
        self.assertEqual(startup.phase, "pre_getup")
        startup.update(model, data)
        self.assertEqual(startup.phase, "getup")
        second = np.float32(1.) / np.float32(400)
        np.testing.assert_array_equal(startup.target_position, (np.float32(1.) - second) * pre + second * stand)
        for _ in range(399):
            startup.update(model, data)
        self.assertEqual(startup.steps, 600)
        self.assertFalse(startup.complete)
        self.assertTrue(startup.update(model, data))
        self.assertEqual(startup.steps, 601)
        self.assertAlmostEqual(startup.elapsed_s, 3.005)
        np.testing.assert_array_equal(startup.target_position, stand)
        np.testing.assert_array_equal(data.qpos, initial_qpos)
        np.testing.assert_array_equal(data.qvel, initial_qvel)
        np.testing.assert_array_equal(data.qfrc_applied, np.zeros(model.nv))
        np.testing.assert_array_equal(data.xfrc_applied, np.zeros((model.nbody, 6)))
        self.assertEqual(data.time, 0.)
        json.dumps(startup.metadata, allow_nan=False)
        # A completed helper still holds via fresh PD if invoked again, without
        # changing its recorded startup duration or advancing the simulator.
        data.qvel[startup.qvel_indices] = 1.
        startup.update(model, data)
        self.assertEqual(startup.steps, 601)
        expected = 80. * (stand.astype(float) - data.qpos[startup.qpos_indices]) - 3.
        np.testing.assert_allclose(data.ctrl[startup.actuator_indices], expected)

    def test_near_pose_skip_is_strictly_below_point_one_radian(self):
        for delta, expected_phase in ((np.float32(.05), "getup"), (np.float32(.1), "pre_getup")):
            model, data = model_and_data()
            setup = Go2GetUp(model, data)
            pre = np.tile(np.array([0., 1.36, -2.65], dtype=np.float32), 4)
            data.qpos[setup.qpos_indices] = pre
            data.qpos[setup.qpos_indices[0]] += delta
            startup = Go2GetUp(model, data)
            startup.update(model, data)
            self.assertEqual(startup.phase, expected_phase)
            if expected_phase == "getup":
                while not startup.complete:
                    startup.update(model, data)
                self.assertEqual(startup.steps, 401)

    def test_real_getup_reaches_standing_using_only_native_actuators(self):
        model, data = model_and_data()
        startup = Go2GetUp(model, data)
        original_limits = model.actuator_ctrlrange.copy()
        original_qpos = data.qpos.copy()
        while not startup.complete:
            startup.update(model, data)
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            self.assertTrue(np.isfinite(data.qpos).all())
            self.assertTrue(np.isfinite(data.qvel).all())
            self.assertTrue(np.all(np.abs(data.actuator_force) <= original_limits[:, 1] + 1e-9))
        self.assertAlmostEqual(data.time, startup.elapsed_s)
        self.assertGreater(data.qpos[2], .2)
        self.assertGreater(float(np.linalg.norm(data.qpos - original_qpos)), .1)
        np.testing.assert_allclose(data.qpos[startup.qpos_indices], startup.target_position, atol=.15)
        np.testing.assert_array_equal(model.actuator_ctrlrange, original_limits)
        np.testing.assert_array_equal(data.qfrc_applied, np.zeros(model.nv))
        np.testing.assert_array_equal(data.xfrc_applied, np.zeros((model.nbody, 6)))

    def test_incompatible_timestep_is_rejected(self):
        model, data = model_and_data()
        model.opt.timestep = .004
        with self.assertRaisesRegex(ValueError, "5 ms"):
            Go2GetUp(model, data)


if __name__ == "__main__":
    unittest.main()
