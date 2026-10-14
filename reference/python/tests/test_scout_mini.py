"""Observable physical checks for the simplified Scout Mini integration."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from simple_rbt_mujoco.scout_mini import (
    ScoutMiniController, build_scout_mini_xml, model_metadata,
    WHEEL_NAMES, WHEEL_RADIUS_M, WHEELBASE_M, TRACK_M,
    MAX_WHEEL_SPEED_RAD_S, WHEEL_TORQUE_LIMIT_NM,
)


def scene(floor=True, yaw_feedback=True):
    root = ET.fromstring(build_scout_mini_xml())
    if floor:
        ET.SubElement(root.find("worldbody"), "geom", name="floor", type="plane",
                      size="10 10 .1", friction="1 .01 .001")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, ScoutMiniController(model, yaw_feedback=yaw_feedback)


def advance(model, data, controller, seconds, command):
    values = []
    for _ in range(round(seconds / model.opt.timestep)):
        controller.update(model, data, command)
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        assert np.isfinite(data.qpos).all()
        assert np.max(np.abs(data.actuator_force)) <= WHEEL_TORQUE_LIMIT_NM + 1e-8
        values.append(np.r_[data.sensor("base_linvel").data,
                            data.sensor("base_angvel").data])
    return np.asarray(values)


def test_compiled_physical_geometry_mass_and_sensor_contract():
    model, data, controller = scene()
    assert (model.nq, model.nv, model.nu) == (11, 10, 4)
    assert model.body_mass.sum() == pytest.approx(23.)
    assert round(.1 / model.opt.timestep) * model.opt.timestep == pytest.approx(.1)
    np.testing.assert_allclose(model.geom("chassis").size, [.306, .2, .065])
    np.testing.assert_allclose(data.site("base_origin").xpos, [0., 0., .18])
    for name in ("base_origin", "imu", "lidar", "camera_mount"):
        assert model.site(name).bodyid == model.body("base").id
    for name in ("gyro", "accelerometer", "base_linvel", "base_angvel"):
        assert data.sensor(name).data.shape == (3,)
    for name in WHEEL_NAMES:
        geom = model.geom(name + "_tire")
        assert geom.type == mujoco.mjtGeom.mjGEOM_ELLIPSOID
        np.testing.assert_allclose(geom.size, [.08, .08, .045])
        body = model.body(name + "_wheel")
        assert body.mass[0] == pytest.approx(1.)
        # The shape update preserves the v1 solid-cylinder inertia tensor,
        # rather than silently changing dynamics to a uniform solid ellipsoid.
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, body.iquat)
        rotation = rotation.reshape(3, 3)
        tensor = rotation @ np.diag(body.inertia) @ rotation.T
        np.testing.assert_allclose(tensor, np.diag([.002275, .0032, .002275]), atol=1e-15)
        tire_R = data.geom(name + "_tire").xmat.reshape(3, 3)
        np.testing.assert_allclose(np.sqrt((tire_R ** 2) @ geom.size ** 2), [.08, .045, .08])
        assert geom.friction[0] == pytest.approx(.8)  # both tangents, no caster shortcut
        assert geom.priority == 1
        np.testing.assert_allclose(model.joint(name + "_wheel_joint").axis, [0., 1., 0.])
    left = data.body("front_left_wheel").xpos
    right = data.body("front_right_wheel").xpos
    rear = data.body("rear_left_wheel").xpos
    assert left[1] - right[1] == pytest.approx(TRACK_M)
    assert left[0] - rear[0] == pytest.approx(WHEELBASE_M)
    manifest = model_metadata()
    assert manifest["model_version"].endswith("rounded-rigid-v2")
    assert manifest["implemented_geometry"]["tire_shape"] == "ellipsoid"
    assert manifest["license_and_assets"]["third_party_meshes_copied"] is False
    assert manifest["control"]["yaw_feedback"]["source"].startswith("ideal MuJoCo gyro")


def test_controller_never_writes_pose_or_external_wrench_and_exposes_saturation():
    model, data, controller = scene()
    before_qpos, before_qvel = data.qpos.copy(), data.qvel.copy()
    controller.update(model, data, [1.5, 0, 1.])
    np.testing.assert_array_equal(data.qpos, before_qpos)
    np.testing.assert_array_equal(data.qvel, before_qvel)
    assert not data.qfrc_applied.any() and not data.xfrc_applied.any()
    assert controller.saturation_scale < 1.
    assert np.max(np.abs(controller.target_velocity)) == pytest.approx(MAX_WHEEL_SPEED_RAD_S)
    np.testing.assert_allclose(controller.target_velocity,
                               controller.requested_target_velocity * controller.saturation_scale)
    with pytest.raises(ValueError, match="finite"):
        controller.update(model, data, [np.nan, 0, 0])
    with pytest.raises(ValueError):
        controller.update(model, data, [0, 0])


def test_airborne_wheel_motors_do_not_translate_free_body_forward():
    model, data, controller = scene(floor=False)
    base_id = model.body("base").id
    initial_com = data.subtree_com[base_id].copy()
    advance(model, data, controller, .2, [.5, 0, 0])
    # Wheel reaction torque can pitch the chassis and shift its frame origin;
    # without ground contact the entire robot COM has no horizontal force.
    np.testing.assert_allclose(data.subtree_com[base_id, :2], initial_com[:2], atol=1e-7)
    assert data.qpos[2] < 0.  # gravity acts; no hidden pose/velocity drive
    assert np.max(np.abs(controller.read_wheel_state(data)[1])) > 1.


@pytest.mark.parametrize("speed", [.5, -.3])
def test_contact_driven_forward_reverse_and_stop(speed):
    model, data, controller = scene()
    advance(model, data, controller, 1., [0, 0, 0])
    start = data.qpos[:3].copy()
    samples = advance(model, data, controller, 4., [speed, 0, 0])
    assert data.qpos[0] - start[0] == pytest.approx(speed * 4, abs=.08)
    assert np.mean(samples[-200:, 0]) == pytest.approx(speed, abs=.015)
    np.testing.assert_allclose(controller.read_wheel_state(data)[1], speed / .08, atol=.08)
    assert abs(data.qpos[1]) < .01
    assert data.site("base_origin").xmat[8] > .99
    stop_position = data.qpos[:3].copy()
    advance(model, data, controller, 2., [0, 0, 0])
    assert np.linalg.norm(data.qpos[:2] - stop_position[:2]) < .04
    assert np.linalg.norm(data.sensor("base_linvel").data) < .01
    np.testing.assert_allclose(data.sensor("accelerometer").data, [0., 0., 9.81], atol=.02)


@pytest.mark.parametrize("yaw_rate", [.5, -.5])
def test_gyro_feedback_skid_turns_both_directions_then_brakes(yaw_rate):
    model, data, controller = scene()
    advance(model, data, controller, 1., [0, 0, 0])
    samples = advance(model, data, controller, 8., [0, 0, yaw_rate])
    assert np.mean(samples[-600:, 5]) == pytest.approx(yaw_rate, abs=.04)
    assert np.linalg.norm(data.qpos[:2]) < .03
    # Real tires scrub: this is a compensated turn, not ideal no-slip kinematics.
    assert abs(controller.yaw_correction) > abs(yaw_rate)
    assert data.site("base_origin").xmat[8] > .99
    advance(model, data, controller, 2., [0, 0, 0])
    assert abs(controller.yaw_integral) < 1e-10
    assert np.linalg.norm(data.sensor("gyro").data) < .01
    assert np.max(np.abs(controller.read_wheel_state(data)[1])) < .02


def test_max_yaw_reports_wheel_saturation_and_meets_existing_speed_tolerance():
    model, data, controller = scene()
    advance(model, data, controller, 1., [0, 0, 0])
    samples = advance(model, data, controller, 8., [0, 0, 1.])
    actual = float(np.mean(samples[-600:, 5]))
    # Reuse the already-declared 15% tracking tolerance, not the obsolete
    # cylinder-only prediction that this command must miss its target.
    assert actual == pytest.approx(1., abs=.15)
    assert controller.saturation_fraction > .1
    assert np.max(np.abs(controller.target_velocity)) <= MAX_WHEEL_SPEED_RAD_S + 1e-10
    assert controller.requested_target_velocity.max() > controller.target_velocity.max()
