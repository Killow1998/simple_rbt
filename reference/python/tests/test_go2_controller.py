"""Real-policy rollout and exact deployment contracts; no Torch required."""
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from simple_rbt_mujoco.go2 import ASSETS, Go2Controller, build_go2_xml


def model_and_data(controller):
    root = ET.fromstring(build_go2_xml(controller))
    ET.SubElement(root.find("worldbody"), "geom", type="plane", size="10 10 .1",
                  friction="1 .01 .001")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


@unittest.skipUnless((ASSETS / "mjcf/go2.xml").exists(), "optional Go2 assets not installed")
class Go2ControllerTests(unittest.TestCase):
    def test_him_contract_uses_newest_first_history_and_named_policy_joints(self):
        model, data = model_and_data("himloco")
        control = Go2Controller(model, "himloco")
        self.assertEqual(control.joint_names[:3], ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"])
        self.assertEqual(control.joint_names[3:6], ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"])
        data.qpos[control.qpos_indices] = control.default
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        observation = control.observation(model, data, (.3, -.2, .5))
        np.testing.assert_allclose(observation[:3], [.6, -.4, .125])
        np.testing.assert_allclose(observation[6:9], [0, 0, -1])
        np.testing.assert_array_equal(observation[9:33], np.zeros(24))
        control.update(model, data, (.3, -.2, .5))
        first = control.history[0].copy()
        self.assertEqual(control.history.shape, (6, 45))
        for _ in range(control.substeps):
            control.update(model, data, (.4, 0, 0))
        np.testing.assert_array_equal(control.history[1], first)
        np.testing.assert_allclose(control.history[0, :3], [.8, 0, 0])

    def test_both_policies_move_forward_without_falling(self):
        for kind in ("himloco", "robot_lab"):
            with self.subTest(controller=kind):
                model, data = model_and_data(kind)
                control = Go2Controller(model, kind)
                heights = []
                for step in range(1000):
                    control.update(model, data, (.3 if step >= 200 else 0, 0, 0))
                    self.assertTrue(np.isfinite(data.ctrl).all())
                    self.assertTrue((np.abs(data.ctrl[control.actuator_indices]) <= control.torque_limits + 1e-6).all())
                    mujoco.mj_step(model, data)
                    mujoco.mj_forward(model, data)
                    heights.append(data.qpos[2])
                self.assertGreater(data.qpos[0], .5)
                self.assertGreater(min(heights), .2)

    def test_torque_modes_only_change_the_software_limit_and_preserve_native_limits(self):
        records = {}
        for mode in ("configured", "upstream"):
            model, data = model_and_data("himloco")
            original_limits = model.actuator_ctrlrange.copy()
            control = Go2Controller(model, "himloco", torque_mode=mode)
            data.qpos[control.qpos_indices] = control.default
            data.qvel[:] = 0.
            control.steps = 1  # Isolate PD action handling from inference.
            control.last_action[:] = 100.
            control.update(model, data, (0., 0., 0.))
            expected = control.kp * control.last_action * control.action_scale
            if mode == "configured":
                expected = np.minimum(expected, control.torque_limits)
            np.testing.assert_allclose(data.ctrl[control.actuator_indices], expected)
            mujoco.mj_forward(model, data)
            native = model.actuator_ctrlrange[control.actuator_indices, 1]
            np.testing.assert_allclose(data.actuator_force[control.actuator_indices], np.minimum(expected, native))
            np.testing.assert_array_equal(model.actuator_ctrlrange, original_limits)
            records[mode] = data.actuator_force[control.actuator_indices].copy()
        np.testing.assert_allclose(records["configured"], np.tile([23.7, 23.7, 33.5], 4))
        np.testing.assert_allclose(records["upstream"], np.tile([23.7, 23.7, 45.43], 4))
        with self.assertRaises(ValueError):
            Go2Controller(model, "himloco", torque_mode="unlimited")

    def test_torque_pd_updates_between_policy_steps(self):
        model, data = model_and_data("himloco")
        control = Go2Controller(model, "himloco")
        control.update(model, data, (0, 0, 0))
        action = control.last_action.copy()
        before = data.ctrl.copy()
        data.qvel[control.qvel_indices] += 1
        control.update(model, data, (0, 0, 0))
        np.testing.assert_array_equal(control.last_action, action)
        self.assertGreater(float(np.linalg.norm(before - data.ctrl)), .1)


if __name__ == "__main__":
    unittest.main()
