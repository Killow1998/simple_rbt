"""Physical mounting/specific-force contracts and asynchronous 200 Hz sampling."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from simple_rbt_mujoco.mid360 import (
    ACCEL_LIMIT_M_S2, G0, GYRO_LIMIT_RAD_S, IMU_IN_LIDAR_POSITION,
    IMU_PERIOD_NS, Mid360ImuSampler, append_mid360_imu,
)


def compile_root(root):
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_mount_translation_rotation_and_static_specific_force_preserve_policy_imu():
    root = ET.fromstring("""<mujoco><option gravity="0 0 -9.81"/>
      <worldbody><body pos="1 2 3" quat=".7071067811865476 .7071067811865476 0 0">
        <body name="base" pos=".1 .2 .3">
          <joint name="resting_hinge" axis="0 0 1"/>
          <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
          <site name="imu" pos="0 0 0"/>
          <site name="lidar" pos=".2 .3 .4" quat=".7071067811865476 0 0 .7071067811865476"/>
        </body></body></worldbody>
      <sensor><gyro name="gyro" site="imu"/><accelerometer name="accelerometer" site="imu"/></sensor>
    </mujoco>""")
    original = [ET.tostring(child) for child in root.find("sensor")]
    metadata = append_mid360_imu(root)
    model, data = compile_root(root)
    imu_id, lidar_id = model.site("mid360_imu").id, model.site("lidar").id
    np.testing.assert_allclose(model.site_pos[imu_id], [.2 - .02329, .3 + .011, .4 - .04412], atol=1e-15)
    np.testing.assert_allclose(data.site_xmat[imu_id], data.site_xmat[lidar_id], atol=1e-15)
    r_world_lidar = data.site_xmat[lidar_id].reshape(3, 3)
    np.testing.assert_allclose(data.site_xpos[imu_id] - data.site_xpos[lidar_id],
                               r_world_lidar @ IMU_IN_LIDAR_POSITION, atol=1e-15)
    # X_lidar points up after the two distinct mount rotations.
    np.testing.assert_allclose(data.sensor("mid360_accelerometer").data, [9.81, 0, 0], atol=1e-13)
    np.testing.assert_allclose(data.sensor("mid360_gyro").data, 0., atol=1e-15)
    assert [ET.tostring(child) for child in root.find("sensor")][:2] == original
    assert model.sensor("gyro").objid[0] == model.site("imu").id
    assert model.sensor("mid360_gyro").objid[0] == imu_id
    assert metadata["imu_in_lidar_position_m"] == list(IMU_IN_LIDAR_POSITION)
    before_second_append = ET.tostring(root)
    with pytest.raises(ValueError, match="already exist"):
        append_mid360_imu(root)
    assert ET.tostring(root) == before_second_append


def test_accelerometer_measures_rotating_lever_arm_at_its_own_site():
    root = ET.fromstring("""<mujoco><option gravity="0 0 0"/>
      <worldbody><body name="rotor">
        <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
        <joint name="spin" type="hinge" axis="0 0 1"/>
        <site name="lidar" pos=".3 .1 .2" quat=".7071067811865476 0 .7071067811865476 0"/>
      </body></worldbody><actuator><motor joint="spin"/></actuator></mujoco>""")
    append_mid360_imu(root)
    model, data = compile_root(root)
    data.qpos[0], data.qvel[0], data.ctrl[0] = .4, 2., .3
    mujoco.mj_forward(model, data)
    assert data.qacc[0] == pytest.approx(3.)
    imu_id = model.site("mid360_imu").id
    position = data.site_xpos[imu_id]
    rotation = data.site_xmat[imu_id].reshape(3, 3)
    omega, alpha = np.array([0., 0., 2.]), np.array([0., 0., 3.])
    expected = np.cross(alpha, position) + np.cross(omega, np.cross(omega, position))
    np.testing.assert_allclose(data.sensor("mid360_gyro").data, [-2., 0., 0.], atol=1e-14)
    np.testing.assert_allclose(data.sensor("mid360_accelerometer").data, rotation.T @ expected, atol=1e-13)
    assert np.linalg.norm(expected) > 1.  # Copying a body-centre IMU would report zero.


def test_two_ms_physics_waits_for_right_sample_and_emits_exact_five_ms_ticks():
    sampler = Mid360ImuSampler()
    first = sampler.observe(0, [0, 0, 0], [0, 0, G0])
    assert [s.stamp_ns for s in first] == [0]
    assert first[0].sampling_method == "exact"
    assert first[0].source_stamps_ns == (0, 0)
    assert sampler.observe(2_000_000, [2, 0, 0], [0, 2, G0]) == []
    assert sampler.observe(4_000_000, [4, 0, 0], [0, 4, G0]) == []
    sample, = sampler.observe(6_000_000, [6, 0, 0], [0, 6, G0])
    assert sample.stamp_ns == 5_000_000
    assert sample.source_stamps_ns == (4_000_000, 6_000_000)
    assert sample.sampling_method == "linear_interpolation"
    np.testing.assert_allclose(sample.angular_velocity_rad_s, [5, 0, 0])
    np.testing.assert_allclose(sample.specific_force_m_s2, [0, 5, G0])
    assert not sample.angular_velocity_rad_s.flags.writeable
    assert sampler.metadata["orientation_available"] is False


@pytest.mark.parametrize("physics_dt_ns", [2_000_000, 4_000_000, 5_000_000])
def test_inclusive_one_second_window_has_201_grid_samples(physics_dt_ns):
    sampler, emitted = Mid360ImuSampler(), []
    for stamp in range(0, 1_000_000_001, physics_dt_ns):
        emitted.extend(sampler.observe(stamp, [0, 0, 0], [0, 0, G0]))
    assert [s.stamp_ns for s in emitted] == list(range(0, 1_000_000_001, IMU_PERIOD_NS))
    assert sampler.samples_emitted == 201


def test_off_grid_start_does_not_extrapolate_and_observations_are_snapshots():
    sampler, gyro = Mid360ImuSampler(), np.array([1., 0., 0.])
    assert sampler.observe(3_000_000, gyro, [0, 0, G0]) == []
    gyro[:] = 100.
    sample, = sampler.observe(7_000_000, [3, 0, 0], [0, 0, G0])
    assert sample.stamp_ns == 5_000_000
    np.testing.assert_allclose(sample.angular_velocity_rad_s, [2, 0, 0])


def test_optional_range_clipping_is_explicit_and_default_retains_ideal_values():
    gyro, accel = [2 * GYRO_LIMIT_RAD_S, 0, -2 * GYRO_LIMIT_RAD_S], [0, 8 * G0, -8 * G0]
    ideal, = Mid360ImuSampler().observe(0, gyro, accel)
    clipped, = Mid360ImuSampler(saturation=True).observe(0, gyro, accel)
    np.testing.assert_allclose(ideal.angular_velocity_rad_s, gyro)
    np.testing.assert_allclose(ideal.specific_force_m_s2, accel)
    assert not ideal.saturation_enabled and not any(ideal.gyro_saturated)
    np.testing.assert_allclose(clipped.angular_velocity_rad_s, [GYRO_LIMIT_RAD_S, 0, -GYRO_LIMIT_RAD_S])
    np.testing.assert_allclose(clipped.specific_force_m_s2, [0, ACCEL_LIMIT_M_S2, -ACCEL_LIMIT_M_S2])
    assert clipped.gyro_saturated == (True, False, True)
    assert clipped.accel_saturated == (False, True, True)


@pytest.mark.parametrize("stamp,gyro,accel", [
    (True, [0, 0, 0], [0, 0, 0]), (5.0, [0, 0, 0], [0, 0, 0]),
    (-1, [0, 0, 0], [0, 0, 0]), (np.nan, [0, 0, 0], [0, 0, 0]),
    (0, [True, 0, 0], [0, 0, 0]), (0, [0, np.inf, 0], [0, 0, 0]),
    (0, [0, 0, 0], [0, np.nan, 0]), (0, [0, 0], [0, 0, 0]),
])
def test_invalid_inputs_do_not_advance_the_sampler(stamp, gyro, accel):
    sampler = Mid360ImuSampler()
    with pytest.raises(ValueError):
        sampler.observe(stamp, gyro, accel)
    assert sampler.observe(0, [0, 0, 0], [0, 0, G0])[0].stamp_ns == 0


def test_duplicate_or_reversed_time_is_rejected_without_advancing_state():
    sampler = Mid360ImuSampler()
    sampler.observe(5_000_000, [0, 0, 0], [0, 0, G0])
    for stamp in (5_000_000, 4_000_000):
        with pytest.raises(ValueError, match="strictly increase"):
            sampler.observe(stamp, [0, 0, 0], [0, 0, G0])
    assert sampler.observe(10_000_000, [0, 0, 0], [0, 0, G0])[0].stamp_ns == 10_000_000


@pytest.mark.parametrize("site", ['name="lidar" quat="0 0 0 0"',
                                  'name="lidar" pos="0 nan 0"',
                                  'name="lidar" euler="0 0 90"', 'name="other"'])
def test_invalid_mount_fails_without_mutating_tree(site):
    root = ET.fromstring(f'<mujoco><worldbody><body><site {site}/></body></worldbody></mujoco>')
    before = ET.tostring(root)
    with pytest.raises(ValueError):
        append_mid360_imu(root)
    assert ET.tostring(root) == before


@pytest.mark.parametrize("defaults,bodies", [
    # The legacy Go2 foot class has its own site position; it does not apply here.
    ('<default><default class="robot"/><default class="unrelated_foot">'
     '<site pos="0 0 -.213" euler="0 0 90"/></default></default>',
     '<body childclass="robot"><site name="lidar" pos=".2 .3 .4" quat="1 0 0 0"/></body>'),
    # Inherit the main pose without explicit site attributes.
    ('<default><site pos=".1 .2 .3" quat=".7071067811865476 0 0 .7071067811865476"/></default>',
     '<body><site name="lidar"/></body>'),
    # Resolve nearest ancestor childclass and nested default-class inheritance.
    ('<default><site pos="1 2 3"/><default class="robot"><site pos=".2 .3 .4"/>'
     '<default class="mount"><site quat=".7071067811865476 0 .7071067811865476 0"/>'
     '</default></default></default>',
     '<body childclass="mount"><body><site name="lidar"/></body></body>'),
    # Explicit site class overrides its parent's unsafe childclass. The newly
    # appended IMU must also select that class, not inherit the unsafe parent.
    ('<default><default class="unsafe"><site euler="0 0 90"/></default>'
     '<default class="mount"><site pos=".1 .2 .3" quat="1 0 0 0"/></default></default>',
     '<body childclass="unsafe"><site name="lidar" class="mount"/></body>'),
    # Explicit pos/quat do override inherited pos/quat (unlike inherited Euler).
    ('<default><site pos="4 5 6" quat="0 1 0 0"/></default>',
     '<body><site name="lidar" pos=".2 .3 .4" quat="1 0 0 0"/></body>'),
])
def test_only_effective_site_defaults_control_the_mount(defaults, bodies):
    root = ET.fromstring(f'<mujoco>{defaults}<worldbody>{bodies}</worldbody></mujoco>')
    original_model, _ = compile_root(root)
    expected_position = original_model.site("lidar").pos.copy()
    expected_quaternion = original_model.site("lidar").quat.copy()
    metadata = append_mid360_imu(root)
    model, data = compile_root(root)
    np.testing.assert_allclose(model.site("lidar").pos, expected_position, atol=1e-15)
    np.testing.assert_allclose(model.site("lidar").quat, expected_quaternion, atol=1e-15)
    lidar, imu = data.site("lidar"), data.site("mid360_imu")
    np.testing.assert_allclose(lidar.xmat, imu.xmat, atol=1e-15)
    np.testing.assert_allclose(lidar.xmat.reshape(3, 3).T @ (imu.xpos - lidar.xpos),
                               IMU_IN_LIDAR_POSITION, atol=1e-15, rtol=0)
    assert len(metadata["calibration_sha256"]) == 64
    assert metadata["lidar_in_imu_position_m"] == [-v for v in IMU_IN_LIDAR_POSITION]


def test_effective_alternate_orientation_is_not_silently_overridden_by_quaternion():
    root = ET.fromstring("""<mujoco><default><site euler="0 0 90"/>
      <default class="mount"><site quat="1 0 0 0"/></default></default>
      <worldbody><body childclass="mount"><site name="lidar" quat="1 0 0 0"/></body></worldbody>
    </mujoco>""")
    # Independent compiler evidence: explicit quaternion does not override this Euler default.
    model, _ = compile_root(root)
    np.testing.assert_allclose(model.site("lidar").quat, [2**-.5, 0, 0, 2**-.5], atol=1e-15)
    before = ET.tostring(root)
    with pytest.raises(ValueError, match="Effective lidar"):
        append_mid360_imu(root)
    assert ET.tostring(root) == before
