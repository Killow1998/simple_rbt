"""Physical and command-contract checks for the rear-driven FR-mini adapter."""
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from simple_rbt_mujoco.fr_mini import (
    FrMiniController, build_fr_mini_xml, validate_command, model_metadata,
    WHEEL_NAMES, WHEELBASE_M, TRACK_M, WHEEL_RADIUS_M,
    MAX_STEERING_ANGLE_RAD, MAX_CURVATURE, MAX_WHEEL_SPEED_RAD_S,
)


def scene(kind="flat"):
    root = ET.fromstring(build_fr_mini_xml())
    world, base = root.find("worldbody"), root.find("./worldbody/body[@name='base']")
    if kind == "ramp":
        pitch = -math.radians(8.)
        quat = f"{math.cos(pitch/2)} 0 {math.sin(pitch/2)} 0"
        ET.SubElement(world, "geom", name="floor", type="plane", size="10 10 .1", quat=quat)
        base.set("quat", quat)
        base.set("pos", f"{math.sin(pitch)*.1255} 0 {math.cos(pitch)*.1255}")
    elif kind == "hills":
        asset = ET.SubElement(root, "asset")
        ET.SubElement(asset, "hfield", name="hill", nrow="5", ncol="401", size="4 2 .06 .1")
        ET.SubElement(world, "geom", name="hill", type="hfield", hfield="hill")
        base.set("pos", "-3.8 0 .21")
    elif kind == "flat":
        ET.SubElement(world, "geom", name="floor", type="plane", size="10 10 .1")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    if kind == "hills":
        x = np.linspace(-4., 4., 401)
        # Continuous raised-cosine hills, 6 cm peak and 2 m period; no steps.
        model.hfield_data[:] = np.tile(.5 * (1 - np.cos(math.pi * (x + 4))), (5, 1)).ravel()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, FrMiniController(model)


def advance(model, data, controller, seconds, command):
    samples = []
    tires = {model.geom(name + "_tire").id for name in WHEEL_NAMES}
    for _ in range(round(seconds / model.opt.timestep)):
        controller.update(model, data, command)
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        assert np.isfinite(data.qpos).all()
        assert np.max(np.abs(data.actuator_force[controller.drive_actuator_indices])) <= 4. + 1e-8
        assert np.max(np.abs(data.actuator_force[controller.steering_actuator_indices])) <= 3. + 1e-8
        for contact in data.contact:
            a, b = int(contact.geom1), int(contact.geom2)
            if contact.dist <= 1e-6 and (model.geom_bodyid[a] == 0) != (model.geom_bodyid[b] == 0):
                assert (b if model.geom_bodyid[a] == 0 else a) in tires
        samples.append(np.r_[data.sensor("base_linvel").data, data.sensor("base_angvel").data])
    return np.asarray(samples)


def test_geometry_mass_rear_axle_reference_and_passive_front_wheels():
    model, data, controller = scene()
    assert (model.nq, model.nv, model.nu) == (13, 12, 4)
    assert model.body_mass.sum() == pytest.approx(18.)
    assert model.opt.timestep == pytest.approx(.005)
    np.testing.assert_allclose(data.site("base_origin").xpos, [0., 0., .1255])
    assert data.geom("chassis").xpos[0] == pytest.approx(.2)
    assert data.geom("chassis").xpos[2] - model.geom("chassis").size[2] == pytest.approx(.061)
    assert data.geom("upper_deck").xpos[2] + model.geom("upper_deck").size[2] == pytest.approx(.190)
    driven_joints = set(model.actuator_trnid[:, 0])
    for name in WHEEL_NAMES:
        geom = model.geom(name + "_tire")
        assert geom.type == mujoco.mjtGeom.mjGEOM_ELLIPSOID
        assert geom.size[0] == pytest.approx(.070)
        assert geom.friction[0] == pytest.approx(.8)
        front = name.startswith("front")
        assert (model.joint(name + "_wheel_joint").id in driven_joints) is (not front)
        assert data.body(name + "_wheel").xpos[0] == pytest.approx(.4 if front else 0.)
    assert len(controller.actuator_indices) == 2
    for name in ("base_origin", "imu", "lidar", "camera_mount"):
        assert model.site(name).bodyid == model.body("base").id
    assert model_metadata()["control"]["command_reference"].startswith("rear axle")


@pytest.mark.parametrize("command", [[0, 0, .1], [1e-9, 0, .1], [0, 1e-14, 0], [.3, .1, 0], [np.nan, 0, 0], [0, 0]])
def test_unsupported_commands_rejected(command):
    with pytest.raises(ValueError):
        validate_command(command)


def test_ackermann_geometry_reverse_curvature_and_logged_limits():
    model, data, controller = scene()
    before_q, before_v = data.qpos.copy(), data.qvel.copy()
    controller.update(model, data, [.3, 0, .2])
    left, right = controller.steering_targets
    assert 0 < right < left < MAX_STEERING_ANGLE_RAD
    assert WHEELBASE_M / math.tan(left) + TRACK_M / 2 == pytest.approx(1.5)
    assert WHEELBASE_M / math.tan(right) - TRACK_M / 2 == pytest.approx(1.5)
    expected_rear = np.array([.3 - .2 * TRACK_M / 2, .3 + .2 * TRACK_M / 2]) / WHEEL_RADIUS_M
    np.testing.assert_allclose(data.ctrl[controller.drive_actuator_indices], expected_rear)
    np.testing.assert_array_equal(data.qpos, before_q)
    np.testing.assert_array_equal(data.qvel, before_v)
    assert not data.qfrc_applied.any() and not data.xfrc_applied.any()
    controller.update(model, data, [-.3, 0, .2])
    assert np.all(controller.steering_targets < 0.)
    assert np.all(controller.target_velocity < 0.)
    controller.update(model, data, [.3, 0, .5])
    assert controller.curvature_saturated
    assert controller.last_command[2] == .5
    assert controller.applied_command[2] == pytest.approx(.3 * MAX_CURVATURE)
    assert max(abs(controller.steering_targets)) == pytest.approx(MAX_STEERING_ANGLE_RAD)
    assert np.max(abs(controller.target_velocity)) <= MAX_WHEEL_SPEED_RAD_S
    controller.update(model, data, [0, 0, 0])
    np.testing.assert_array_equal(data.ctrl, 0.)


@pytest.mark.parametrize("vx,wz", [(.3, 0.), (-.3, 0.), (.3, .2), (.3, -.2), (-.3, .2)])
def test_contact_driven_forward_reverse_turning_and_motor_braking(vx, wz):
    model, data, controller = scene()
    advance(model, data, controller, 2., [0., 0., 0.])
    samples = advance(model, data, controller, 5., [vx, 0., wz])
    assert samples[-400:, 0].mean() == pytest.approx(vx, abs=.02)
    assert samples[-400:, 5].mean() == pytest.approx(wz, abs=.03)
    assert np.abs(samples[-400:, 1]).mean() < .01  # rear axle reference, not body center
    assert data.site("base_origin").xmat[8] > .98
    stop_start = data.site("base_origin").xpos.copy()
    advance(model, data, controller, 2., [0., 0., 0.])
    assert np.linalg.norm(data.site("base_origin").xpos[:2] - stop_start[:2]) < .08
    assert np.linalg.norm(data.sensor("base_linvel").data) < .01
    assert np.linalg.norm(data.sensor("gyro").data) < .01
    np.testing.assert_allclose(data.sensor("accelerometer").data, [0., 0., 9.81], atol=.03)


def test_eight_degree_ramp_and_continuous_hills_are_contact_driven():
    for kind, duration, minimum_progress in (("ramp", 5., 1.), ("hills", 12., 2.5)):
        model, data, controller = scene(kind)
        advance(model, data, controller, 2., [0, 0, 0])
        start = data.site("base_origin").xpos.copy()
        samples = advance(model, data, controller, duration, [.3, 0, 0])
        assert data.site("base_origin").xpos[0] - start[0] > minimum_progress
        assert .23 < samples[-400:, 0].mean() < .35
        assert data.site("base_origin").xmat[8] > .9
        assert not data.qfrc_applied.any() and not data.xfrc_applied.any()
