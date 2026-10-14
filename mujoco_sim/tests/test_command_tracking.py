import math

import numpy as np
import pytest

from simple_rbt_mujoco.command_tracking import CommandTracker


def test_pi_rejects_constant_simulated_velocity_bias_and_tracks_stop():
    tracker = CommandTracker(hold_heading=False)
    measured = np.zeros(3)
    dt = .01
    for i in range(2000):
        target = np.array([.3 if i < 1000 else 0., 0., 0.])
        output = tracker.update(target, measured, [0, 0, 0], 0., dt)
        # A lagged plant with a constant native-policy bias, not tracker math.
        measured[:2] += dt / .15 * (output[:2] + [.03, -.015] - measured[:2])
        if i == 999:
            np.testing.assert_allclose(measured[:2], [.3, 0.], atol=.001)
    np.testing.assert_allclose(measured[:2], 0., atol=.001)
    assert tracker.saturation_count == 0


def test_saturation_is_reported_and_antiwindup_allows_release():
    tracker = CommandTracker(hold_heading=False, filter_time_s=0.)
    for _ in range(200):
        output = tracker.update([5, 0, 0], [0, 0, 0], [0, 0, 0], 0., .01)
        assert output[0] == 1.
    assert tracker.saturation_fraction == 1.
    assert tracker.requested_target[0] == 5.
    assert tracker.integral[0] == 0.
    output = tracker.update([0, 0, 0], [0, 0, 0], [0, 0, 0], 0., .01)
    assert output[0] == 0.


def test_heading_wrap_and_explicit_turn_release():
    tracker = CommandTracker()
    tracker.reset(heading_yaw=math.pi - .02)
    output = tracker.update([.3, 0, 0], [.3, 0, 0], [0, 0, 0], -math.pi + .02, .02)
    assert -.2 < output[2] < 0.  # short negative correction, not a full circle
    output = tracker.update([.3, 0, .2], [.3, 0, 0], [0, 0, .2], 1.0, .02)
    assert tracker.heading_reference == 1.
    tracker.update([.3, 0, 0], [.3, 0, 0], [0, 0, 0], 1.1, .02)
    assert tracker.heading_reference == 1.1


def test_rejects_nonfinite_inputs():
    tracker = CommandTracker()
    with pytest.raises(ValueError, match="finite"):
        tracker.update([np.nan, 0, 0], [0, 0, 0], [0, 0, 0], 0., .02)


def test_command_only_feedback_completes_physical_go2w_ramp_and_stops():
    """Regression for the actual policy/physics loop, not just the PI equations."""
    import xml.etree.ElementTree as ET
    import mujoco
    from simple_rbt_mujoco.go2w import build_go2w_xml, Go2WController
    from simple_rbt_mujoco.terrains import append_terrain

    root = ET.fromstring(build_go2w_xml())
    ET.SubElement(root.find("worldbody"), "geom", type="plane", size="12 12 .1",
                  friction="1 .01 .001")
    terrain = append_terrain(root, "ramp")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    native, tracker = Go2WController(model), CommandTracker()
    dt = float(model.opt.timestep)

    def step(target):
        rotation = data.site("base_origin").xmat.reshape(3, 3)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        command = tracker.update(target, data.sensor("base_linvel").data,
                                 data.sensor("base_angvel").data, yaw, dt)
        native.update(model, data, command)
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

    for _ in range(round(2 / dt)):
        step([0, 0, 0])
    for _ in range(round(16 / dt)):
        step([.5, 0, 0])
        assert abs(data.qpos[1]) < .1
        assert data.qpos[2] - terrain.height_at(data.qpos[0], data.qpos[1]) > .25
        assert data.site("base_origin").xmat.reshape(3, 3)[2, 2] > math.cos(math.radians(30))
        if data.qpos[0] >= terrain.finish_x:
            break
    assert data.qpos[0] >= terrain.finish_x
    finish = data.qpos[:2].copy()
    stop_distances, final_speeds = [], []
    for i in range(round(5 / dt)):
        step([0, 0, 0])
        stop_distances.append(float(np.linalg.norm(data.qpos[:2] - finish)))
        if i >= round(4 / dt):
            final_speeds.append(float(np.linalg.norm(data.sensor("base_linvel").data[:2])))
    assert max(stop_distances) < .15
    assert np.mean(final_speeds) < .02
    assert tracker.saturation_fraction == 0.
