"""R100 acceptance requires real passive-roller contact, not chassis actuation."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from simple_rbt_mujoco.ridgeback import (HALF_TRACK_M, HALF_WHEELBASE_M,
    MAX_WHEEL_SPEED_RAD_S, ROLLER_COUNT, ROLLER_SLOPES, WHEEL_NAMES,
    RidgebackController, build_ridgeback_xml, model_metadata)


def scene(ground=True):
    root = ET.fromstring(build_ridgeback_xml())
    if ground:
        ET.SubElement(root.find("worldbody"), "geom", name="floor", type="plane",
                      size="20 20 .1", friction="1 .01 .001")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, RidgebackController(model)


def test_compiled_four_hubs_forty_free_rollers_mass_and_isotropic_contact():
    model, data, controller = scene()
    assert model.nu == 4 and model.nv == 50
    assert model.body_mass.sum() == pytest.approx(135.)
    assert model.opt.timestep == .005
    active_joints = set(model.actuator_trnid[:, 0])
    for name, slope in zip(WHEEL_NAMES, ROLLER_SLOPES):
        assert model.joint(name + "_wheel_joint").id in active_joints
        for index in range(ROLLER_COUNT):
            joint = model.joint(f"{name}_roller_{index}_joint").id
            assert joint not in active_joints and not model.jnt_limited[joint]
        axis = data.xaxis[model.joint(name + "_roller_0_joint").id]
        assert axis[1] / axis[0] == pytest.approx(slope)
        assert axis[2] == pytest.approx(0.)
    # The named robot default must not leak its collision mask into later floors.
    assert model.geom_contype[model.geom("floor").id] == 1
    for _ in range(50):
        controller.update(model, data, [0., 0., 0.])
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    assert data.ncon >= 4
    for contact in data.contact:
        names = {model.geom(int(contact.geom1)).name, model.geom(int(contact.geom2)).name}
        assert "floor" in names
        assert any("roller" in name for name in names)
        np.testing.assert_allclose(contact.friction[:2], [.8, .8])
    assert model_metadata()["control"]["native_body_velocity_feedback"] is False


def test_kinematic_signs_uniform_limit_and_controller_cannot_move_base_directly():
    model, data, controller = scene()
    qpos, qvel = data.qpos.copy(), data.qvel.copy()
    controller.update(model, data, [0., .3, 0.])
    assert np.sign(controller.target_velocity).tolist() == [-1., 1., 1., -1.]
    controller.update(model, data, [0., 0., .5])
    assert np.sign(controller.target_velocity).tolist() == [-1., 1., -1., 1.]
    controller.update(model, data, [1., .6, 1.])
    expected = np.array([1. - .6 - (HALF_TRACK_M + HALF_WHEELBASE_M),
                         1. + .6 + (HALF_TRACK_M + HALF_WHEELBASE_M),
                         1. + .6 - (HALF_TRACK_M + HALF_WHEELBASE_M),
                         1. - .6 + (HALF_TRACK_M + HALF_WHEELBASE_M)])
    np.testing.assert_allclose(controller.target_velocity / np.max(np.abs(controller.target_velocity)), expected / np.max(np.abs(expected)))
    assert np.max(np.abs(controller.target_velocity)) == pytest.approx(MAX_WHEEL_SPEED_RAD_S)
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.qvel, qvel)
    assert not data.qfrc_applied.any() and not data.xfrc_applied.any()
    for bad in ([np.nan, 0., 0.], [0., 0.], [0., 0., np.inf]):
        with pytest.raises(ValueError):
            controller.update(model, data, bad)


@pytest.mark.parametrize("command", [[.3, 0., 0.], [-.3, 0., 0.],
    [0., .3, 0.], [0., -.3, 0.], [0., 0., .5], [0., 0., -.5], [.3, .2, .3]])
def test_contact_driven_forward_lateral_turn_compound_and_stop(command):
    model, data, controller = scene()
    measured, parked, roller_speed = [], [], []
    passive_dof = model.joint("front_left_roller_0_joint").dofadr[0]
    for step in range(1600):
        controller.update(model, data, command if 400 <= step < 1200 else [0., 0., 0.])
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        velocity = np.r_[data.sensor("base_linvel").data[:2], data.sensor("base_angvel").data[2]]
        if 800 <= step < 1200:
            measured.append(velocity.copy())
            roller_speed.append(abs(data.qvel[passive_dof]))
        if 1400 <= step:
            parked.append(velocity.copy())
        assert np.isfinite(data.qpos).all()
    assert data.time == pytest.approx(8.)
    mean = np.mean(measured, axis=0)
    assert np.all(np.abs(mean - command) <= np.maximum(.05, .15 * np.abs(command)))
    # Norm/absolute statistics do not hide oscillating motion behind zero means.
    assert np.linalg.norm(np.asarray(parked)[:, :2], axis=1).mean() < .05
    assert np.abs(np.asarray(parked)[:, 2]).mean() < .05
    if command[1] != 0.:
        assert np.mean(roller_speed) > 1.
    assert not data.qfrc_applied.any() and not data.xfrc_applied.any()
    qpos, qvel = controller.read_wheel_state(data)
    np.testing.assert_array_equal(qpos, data.qpos[controller.qpos_indices])
    np.testing.assert_array_equal(qvel, data.qvel[controller.qvel_indices])


def test_without_ground_hub_motors_cannot_translate_system_centre_of_mass():
    model, data, controller = scene(ground=False)
    model.opt.gravity[:] = 0.
    mujoco.mj_forward(model, data)
    start = data.subtree_com[model.body("base").id].copy()
    for _ in range(600):
        controller.update(model, data, [.3, .2, .3])
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.subtree_com[model.body("base").id], start, atol=1e-7)
    assert data.ncon == 0
    assert np.max(np.abs(controller.read_wheel_state(data)[1])) > 1.
