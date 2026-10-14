"""Independent geometric/time tests, including a snapshot negative control."""
import os

import mujoco
import numpy as np
import pytest

from simple_rbt_mujoco.lidar import CsvLidar, CsvScanPattern
from simple_rbt_mujoco.rolling_lidar import PoseHistory, RollingCsvLidar, rotate_vectors


def quat_yaw(angle):
    return np.array([np.cos(angle / 2), 0., 0., np.sin(angle / 2)])


def pattern(azimuth=(0., 10., -10., 20.)):
    return CsvScanPattern(np.arange(1, len(azimuth) + 1), np.asarray(azimuth),
                          np.full(len(azimuth), 90.))


def wall_scene(extra=""):
    # Plane at world X=5, normal -X; robot's articulated child must be excluded.
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <geom name="wall" type="plane" pos="5 0 0" quat=".7071067811865476 0 -.7071067811865476 0" size="0 0 .1"/>
      <body name="robot"><freejoint/><geom type="sphere" size=".2" mass="1"/><site name="lidar"/>
        <body name="leg" pos="1 0 0"><joint axis="0 0 1"/><geom type="sphere" size=".2" mass=".1"/></body>
      </body>{extra}</worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def history_motion(start_ns=0):
    history = PoseHistory()
    for t in range(0, 100_000_001, 5_000_000):
        seconds = t / 1e9
        history.add(start_ns + t, [seconds, 0., 0.], quat_yaw(.5 * seconds))
    return history


def test_history_interpolates_short_arc_and_keeps_ns_precision():
    h = PoseHistory()
    start = 9_000_000_000_000_000_000
    h.add(start, [0, 0, 0], quat_yaw(np.radians(179)))
    h.add(start + 10, [2, 4, 6], -quat_yaw(np.radians(-179)))
    p, q = h.poses(np.array([start, start + 5, start + 10], dtype=np.uint64))
    np.testing.assert_allclose(p, [[0, 0, 0], [1, 2, 3], [2, 4, 6]])
    direction = rotate_vectors(q, np.tile([1., 0, 0], (3, 1)))
    np.testing.assert_allclose(direction[1], [-1, 0, 0], atol=1e-12)
    with pytest.raises(ValueError, match="extrapolation"):
        h.poses(np.array([start - 1], dtype=np.uint64))
    with pytest.raises(ValueError, match="extrapolation"):
        h.poses(np.array([start + 11], dtype=np.uint64))


def test_history_pruning_preserves_knot_before_window_and_rejects_invalid_state():
    h = PoseHistory()
    for i in range(101):
        h.add(i * 5_000_000, [i, 0, 0], [1, 0, 0, 0])
    assert h.bounds_ns == (245_000_000, 500_000_000)
    assert len(h) == 52
    np.testing.assert_allclose(h.poses(np.array([247_500_000]))[0], [[49.5, 0, 0]])
    with pytest.raises(ValueError, match="strictly increasing"):
        h.add(500_000_000, [0, 0, 0], [1, 0, 0, 0])
    with pytest.raises(ValueError, match="finite"):
        h.add(505_000_000, [np.nan, 0, 0], [1, 0, 0, 0])
    with pytest.raises(ValueError, match="nonzero"):
        h.add(505_000_000, [0, 0, 0], [0, 0, 0, 0])
    with np.errstate(over="ignore"), pytest.raises(ValueError, match="finite nonzero norm"):
        h.add(505_000_000, [0, 0, 0], [1e308, 1e308, 1e308, 1e308])
    for invalid in (True, np.bool_(False)):
        with pytest.raises(ValueError, match="not bool"):
            PoseHistory().add(invalid, [0, 0, 0], [1, 0, 0, 0])
    with pytest.raises(ValueError, match="at least 250"):
        PoseHistory(100_000_000)


def test_moving_raw_points_match_analytic_plane_and_snapshot_deskew_does_not():
    model, data = wall_scene()
    original_groups = model.geom_group.copy()
    history = history_motion()
    lidar = RollingCsvLidar(model, pattern())
    scan = lidar.scan(data, 0, 20000, history=history)
    assert scan.point_num == scan.rays_cast == 20000
    assert scan.offsets_ns[-1] == 99_995_000
    positions, quaternions = history.poses(scan.offsets_ns.astype(np.uint64))
    world = positions + rotate_vectors(quaternions, scan.xyz)
    np.testing.assert_allclose(world[:, 0], 5., atol=1e-6)
    expected_range = (5. - positions[:, 0]) / rotate_vectors(
        quaternions, np.tile(lidar.pattern.directions, (5000, 1)))[:, 0]
    np.testing.assert_allclose(np.linalg.norm(scan.xyz, axis=1), expected_range, atol=1e-6)
    assert scan.metadata["acquisition_model"] == "rolling_static_environment"
    assert scan.metadata["robot_self_occlusion"] is False
    assert scan.metadata["deskewed"] is False
    assert scan.metadata["nominal_point_interval_ns"] == 5000
    assert scan.metadata["maximum_pose_knot_interval_ns"] == 5_000_000
    np.testing.assert_array_equal(model.geom_group, original_groups)
    # Reproduce the OLD broken model analytically: ranges all observed at END,
    # but falsely interpreted as acquired at the individual earlier timestamps.
    local = np.tile(lidar.pattern.directions, (5000, 1))
    end_rotation = np.tile(quat_yaw(.05), (20000, 1))
    snapshot_ranges = 4.9 / rotate_vectors(end_rotation, local)[:, 0]
    wrongly_compensated = positions + rotate_vectors(quaternions, snapshot_ranges[:, None] * local)
    assert np.max(np.abs(wrongly_compensated[:, 0] - 5.)) > .09


def test_full_robot_subtree_excluded_and_missing_history_does_not_consume_pattern():
    model, data = wall_scene()
    history = history_motion()
    lidar = RollingCsvLidar(model, pattern())
    with pytest.raises(ValueError, match="extrapolation"):
        lidar.scan(data, 100_000_000, 4, history=history)
    assert lidar.pattern.cursor == 0 and lidar.pattern.rays_emitted == 0
    first = lidar.scan(data, 0, 4, history=history)
    data.qpos[-1] = 1.4
    mujoco.mj_forward(model, data)
    second = lidar.scan(data, 0, 4, history=history)
    np.testing.assert_array_equal(first.xyz, second.xyz)
    assert np.all(first.xyz[:, 0] > 4.9)
    assert lidar.metadata["environment_geom_count"] == 1


@pytest.mark.parametrize("body", [
    '<body name="other"><freejoint/><geom type="sphere" size=".2" mass="1"/></body>',
    '<body name="other" mocap="true"><geom type="sphere" size=".2"/></body>'])
def test_dynamic_obstacles_are_rejected(body):
    model, _ = wall_scene(body)
    with pytest.raises(ValueError, match="non-robot dynamic"):
        RollingCsvLidar(model, pattern())


def test_misses_preserve_original_offsets_indices_and_lines_across_wrap():
    # Infinite wall receives +X rays and misses -X rays.
    model, data = wall_scene()
    h = history_motion()
    scan_pattern = pattern([0., 180., 15., 180.])
    scan_pattern.cursor = 2
    scan = RollingCsvLidar(model, scan_pattern).scan(data, 0, 6, history=h)
    np.testing.assert_array_equal(scan.ray_indices, [2, 0, 2])
    np.testing.assert_array_equal(scan.lines, [2, 0, 2])
    np.testing.assert_array_equal(scan.offsets_ns, [0, 10000, 20000])
    assert scan.missed_count == 3


def test_snapshot_has_no_synthetic_acquisition_times():
    model, data = wall_scene()
    scan = CsvLidar(model, pattern()).scan(data, 100_000_000, 4)
    assert scan.stamp_ns == 100_000_000
    assert not np.any(scan.offsets_ns)
    assert scan.metadata["acquisition_model"] == "snapshot_end"
    with pytest.raises(ValueError, match="timestamp"):
        CsvLidar(model, pattern()).scan(data, True, 4)


@pytest.mark.skipif(os.environ.get("SIMPLE_RBT_TEST_CUDA") != "1", reason="Explicit host CUDA opt-in required")
def test_warp_cpu_moving_rays_agree_on_same_static_scene():
    model, data = wall_scene('<geom type="box" pos="3 2 0" size=".2 .2 .4"/>')
    history = history_motion()
    cpu = RollingCsvLidar(model, pattern([0., 180., 20., -25.]), backend="cpu")
    gpu = RollingCsvLidar(model, pattern([0., 180., 20., -25.]), backend="warp", device="cuda:0")
    a = cpu.scan(data, 0, 1200, history=history)
    b = gpu.scan(data, 0, 1200, history=history)
    np.testing.assert_array_equal(a.ray_indices, b.ray_indices)
    np.testing.assert_array_equal(a.offsets_ns, b.offsets_ns)
    np.testing.assert_array_equal(a.lines, b.lines)
    np.testing.assert_allclose(a.xyz, b.xyz, rtol=0, atol=2e-5)
