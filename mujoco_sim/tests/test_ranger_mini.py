import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from simple_rbt_mujoco.ranger_mini import (
    RangerMiniController, build_ranger_mini_xml, model_metadata,
    WHEEL_NAMES, JOINT_NAMES, STEERING_JOINT_NAMES, MODULE_XY,
    WHEEL_RADIUS_M, WHEELBASE_M, TRACK_M, BASE_HEIGHT_M,
    MAX_WHEEL_SPEED_RAD_S, WHEEL_TORQUE_LIMIT_NM, STEERING_TORQUE_LIMIT_NM,
    STEERING_RATE_LIMIT_RAD_S, CHASSIS_COLLISION_NAMES,
)


def scene(floor=True):
    root = ET.fromstring(build_ranger_mini_xml())
    if floor:
        ET.SubElement(root.find("worldbody"), "geom", name="floor", type="plane",
                      size="20 20 .1", friction="1 .01 .001")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, RangerMiniController(model)


def advance(model, data, controller, seconds, command):
    values = []
    for _ in range(round(seconds / model.opt.timestep)):
        controller.update(model, data, command)
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        assert np.isfinite(data.qpos).all()
        assert np.max(np.abs(data.actuator_force[controller.actuator_indices])) <= WHEEL_TORQUE_LIMIT_NM + 1e-8
        assert np.max(np.abs(data.actuator_force[controller.steering_actuator_indices])) <= STEERING_TORQUE_LIMIT_NM + 1e-8
        values.append(np.r_[data.sensor("base_linvel").data,
                            data.sensor("base_angvel").data])
    return np.asarray(values)


def test_current_variant_mass_geometry_and_real_joint_contract():
    model, data, controller = scene()
    assert (model.nq, model.nv, model.nu) == (15, 14, 8)
    assert model.body_mass.sum() == pytest.approx(73.)
    assert model.opt.timestep == .005
    np.testing.assert_allclose(data.site("base_origin").xpos, [0, 0, BASE_HEIGHT_M])
    for name in ("base_origin", "imu", "lidar", "camera_mount"):
        assert model.site(name).bodyid == model.body("base").id
    for name in ("gyro", "accelerometer", "base_linvel", "base_angvel"):
        assert data.sensor(name).data.shape == (3,)
    for name in CHASSIS_COLLISION_NAMES:
        assert model.geom(name).contype != 0
    for name, xy in zip(WHEEL_NAMES, MODULE_XY):
        np.testing.assert_allclose(data.body(name + "_wheel").xpos, [*xy, WHEEL_RADIUS_M])
        np.testing.assert_allclose(model.joint(name + "_steering_joint").axis, [0, 0, 1])
        np.testing.assert_allclose(model.joint(name + "_wheel_joint").axis, [0, 1, 0])
        np.testing.assert_allclose(model.joint(name + "_steering_joint").range, [-np.pi, np.pi])
        tire = model.geom(name + "_tire")
        assert tire.type == mujoco.mjtGeom.mjGEOM_ELLIPSOID
        assert tire.friction[0] == .8
        assert tire.priority == 1
    assert data.body("front_left_wheel").xpos[0] - data.body("rear_left_wheel").xpos[0] == pytest.approx(WHEELBASE_M)
    assert data.body("front_left_wheel").xpos[1] - data.body("front_right_wheel").xpos[1] == pytest.approx(TRACK_M)
    manifest = model_metadata()
    assert manifest["identity"]["product"] == "RANGER MINI 3.0"
    assert manifest["manufacturer_specifications"]["track_m"] == .370
    assert manifest["manufacturer_specifications"]["max_linear_speed_m_s"] == 2.
    assert manifest["control"]["simulator_command_limits_vx_vy_wz"] == [1., .6, 1.]
    assert "not exact AgileX" in manifest["control"]["firmware_scope"]
    assert manifest["license_and_assets"]["third_party_meshes_copied"] is False
    assert set(controller.actuator_indices).isdisjoint(controller.steering_actuator_indices)


def test_alignment_holds_all_drive_and_invalid_commands_do_not_write_state():
    model, data, controller = scene()
    before_qpos, before_qvel = data.qpos.copy(), data.qvel.copy()
    controller.update(model, data, [0, .4, 0])
    assert controller.mode == "aligning"
    np.testing.assert_array_equal(controller.target_velocity, np.zeros(4))
    assert np.max(np.abs(controller.steering_velocity_targets)) <= STEERING_RATE_LIMIT_RAD_S
    assert np.min(np.abs(controller.requested_target_velocity)) > 0
    np.testing.assert_array_equal(data.qpos, before_qpos)
    np.testing.assert_array_equal(data.qvel, before_qvel)
    assert not data.qfrc_applied.any() and not data.xfrc_applied.any()
    for command in ([np.nan, 0, 0], [0, np.inf, 0], [0, 0], [[0, 0, 0]]):
        previous_ctrl = data.ctrl.copy()
        with pytest.raises(ValueError, match="three finite"):
            controller.update(model, data, command)
        np.testing.assert_array_equal(data.ctrl, previous_ctrl)


def test_reverse_uses_short_steering_path_and_wheel_limit_preserves_ratios():
    model, data, controller = scene()
    controller.update(model, data, [-.5, 0, 0])
    np.testing.assert_allclose(controller.steering_targets, 0., atol=1e-15)
    assert np.all(controller.target_velocity < 0)
    # Near the mechanical endpoint, choose reversed drive at +pi rather than
    # dragging the steering joint through a nearly complete half-turn.
    data.qpos[controller.steering_qpos_indices] = 3.05
    controller.update(model, data, [.5, 0, 0])
    np.testing.assert_allclose(controller.steering_targets, np.pi)
    assert np.all(controller.target_velocity < 0)
    # An out-of-envelope direct controller request is still bounded at motors;
    # public body-command limits are enforced by shared Simulation separately.
    data.qpos[controller.steering_qpos_indices] = 0
    controller.update(model, data, [3., 0, 0])
    assert controller.saturation_fraction > 0
    np.testing.assert_allclose(controller.target_velocity, MAX_WHEEL_SPEED_RAD_S)
    np.testing.assert_allclose(controller.target_velocity,
                               controller.requested_target_velocity * controller.saturation_scale)


def test_zero_during_alignment_latches_current_azimuth_and_physically_brakes():
    model, data, controller = scene()
    advance(model, data, controller, 1., [0, 0, 0])
    advance(model, data, controller, .15, [0, .4, 0])
    assert controller.mode == "aligning"
    angles_before, _ = controller.read_steering_state(data)
    controller.update(model, data, [0, 0, 0])
    np.testing.assert_array_equal(controller.steering_targets, angles_before)
    np.testing.assert_array_equal(controller.target_velocity, np.zeros(4))
    latched = controller.steering_targets.copy()
    advance(model, data, controller, 2., [0, 0, 0])
    assert controller.mode == "brake"
    np.testing.assert_array_equal(controller.steering_targets, latched)
    np.testing.assert_allclose(controller.read_steering_state(data)[0], latched, atol=.003)
    assert np.max(np.abs(controller.read_steering_state(data)[1])) < .005
    assert np.linalg.norm(data.sensor("base_linvel").data) < .005


def test_airborne_actuators_do_not_drive_horizontal_center_of_mass():
    model, data, controller = scene(floor=False)
    base_id = model.body("base").id
    initial_com = data.subtree_com[base_id].copy()
    advance(model, data, controller, .2, [.5, .2, .3])
    np.testing.assert_allclose(data.subtree_com[base_id, :2], initial_com[:2], atol=1e-7)
    assert data.qpos[2] < .1
    assert not data.xfrc_applied.any() and not data.qfrc_applied.any()


@pytest.mark.parametrize("command", [
    [.6, 0, 0], [-.3, 0, 0], [0, .4, 0], [0, -.4, 0],
    [0, 0, .5], [0, 0, -.5], [.3, .2, .3], [1., .6, 1.],
])
def test_contact_driven_motion_and_zero_command_stop(command):
    model, data, controller = scene()
    advance(model, data, controller, 1., [0, 0, 0])
    samples = advance(model, data, controller, 6., command)
    observed = samples[-200:, [0, 1, 5]].mean(axis=0)
    # Existing project velocity tolerance, not adjusted to this model's results.
    limits = np.maximum(.05, .15 * np.abs(command))
    assert np.all(np.abs(observed - command) <= limits)
    assert data.site("base_origin").xmat[8] > .99
    if command[2] == 0:
        assert abs(samples[-200:, 5]).mean() < .05
    if command[:2] == [0, 0]:
        assert np.linalg.norm(data.qpos[:2]) < .03
    if command[1] != 0 or command[2] != 0:
        assert controller.alignment_count > 0
        assert np.max(np.abs(controller.read_steering_state(data)[0])) > .1
    stopped = advance(model, data, controller, 2., [0, 0, 0])[-200:]
    assert np.linalg.norm(stopped[:, :2], axis=1).mean() <= .05
    assert np.abs(stopped[:, 5]).mean() <= .05
    assert np.max(np.abs(controller.read_wheel_state(data)[1])) < .01
    assert np.max(np.abs(controller.read_steering_state(data)[1])) < .01
    np.testing.assert_allclose(data.sensor("accelerometer").data, [0, 0, 9.81], atol=.03)
