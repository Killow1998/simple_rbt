from pathlib import Path

import numpy as np
import pytest

from simple_rbt_mujoco.lidar import CsvLidar, CsvScanPattern


def write_pattern(tmp_path, rows):
    path = tmp_path / "pattern.csv"
    path.write_text("Time/s,Azimuth/deg,Zenith/deg\n" + "\n".join(
        ",".join(str(x) for x in row) for row in rows) + "\n")
    return CsvScanPattern.from_csv(path)


def test_zenith_conversion_and_wrapped_sample_identity(tmp_path):
    pattern = write_pattern(tmp_path, [(1, 0, 90), (2, 90, 90), (3, 0, 0), (4, 0, 180), (5, 180, 90)])
    first = pattern.next_frame(3)
    np.testing.assert_allclose(first.directions, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], atol=1e-7)
    second = pattern.next_frame(5)
    np.testing.assert_array_equal(second.csv_sample_ids, [4, 5, 1, 2, 3])
    np.testing.assert_array_equal(second.ray_indices, [3, 4, 0, 1, 2])
    np.testing.assert_array_equal(second.lines, [3, 0, 0, 1, 2])
    np.testing.assert_array_equal(second.offsets_ns, [0, 5000, 10000, 15000, 20000])
    assert second.offsets_ns.dtype == np.uint32
    assert pattern.rays_emitted == 8 and pattern.cursor == 3
    pattern.reset()
    assert pattern.next_frame(1).csv_sample_ids[0] == 1


def test_bundled_pattern_is_sample_ids_not_seconds():
    from simple_rbt_mujoco.paths import SCAN_CSV
    pattern = CsvScanPattern.from_csv(SCAN_CSV)
    assert len(pattern) == 800000
    first = pattern.next_frame()
    assert len(first.theta) == 20000
    assert first.offsets_ns[-1] == 99995000
    np.testing.assert_allclose(first.directions[0], [0.010812, 0.613282, 0.78979], atol=2e-4)
    pattern.cursor = len(pattern) - 2
    wrapped = pattern.next_frame(4)
    np.testing.assert_array_equal(wrapped.csv_sample_ids, [799999, 800000, 1, 2])
    np.testing.assert_array_equal(wrapped.lines, [2, 3, 0, 1])


def test_snapshot_misses_preserve_indices_but_share_snapshot_time(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("mujoco_lidar")
    # Base and site face +world Y. Only local +/-X rays hit. Every hit belongs
    # to the same snapshot time, while original ray/line indices are preserved.
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <worldbody>
        <body name="base" pos="1 2 0" quat="0.7071067812 0 0 0.7071067812">
          <geom type="sphere" size="0.2"/><site name="lidar"/>
        </body>
        <geom type="sphere" size="0.5" pos="1 7 0"/>
        <geom type="sphere" size="0.5" pos="1 -1 0"/>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    pattern = write_pattern(tmp_path, [(1, 0, 90), (2, 90, 90), (3, 180, 90), (4, 0, 0)])
    lidar = CsvLidar(model, pattern, backend="cpu")
    scan = lidar.scan(data, stamp_ns=12_300_000_000, n_rays=4)
    assert scan.timebase == 12_300_000_000
    assert scan.rays_cast == 4 and scan.valid_count == 2 and scan.missed_count == 2
    np.testing.assert_allclose(scan.xyz, [[4.5, 0, 0], [-2.5, 0, 0]], atol=1e-6)
    np.testing.assert_array_equal(scan.offsets_ns, [0, 0])
    assert scan.metadata["acquisition_model"] == "snapshot_end"
    np.testing.assert_array_equal(scan.lines, [0, 2])
    np.testing.assert_array_equal(scan.ray_indices, [0, 2])
    assert scan.xyz.dtype == np.float32 and scan.lines.dtype == np.uint8


def test_invalid_ranges_share_the_original_ray_mask(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("mujoco_lidar")
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><site name="lidar"/></worldbody></mujoco>')
    pattern = write_pattern(tmp_path, [(i + 1, 0, 90) for i in range(8)])
    lidar = CsvLidar(model, pattern)
    class Ranges:
        def trace_rays(self, data, theta, phi):
            return np.array([1., np.nan, -1., np.inf, 40., 0.05, 3., 0.1])
    lidar.raycaster = Ranges()
    scan = lidar.scan(None, 0, 8)
    np.testing.assert_array_equal(scan.ray_indices, [0, 6, 7])
    np.testing.assert_array_equal(scan.offsets_ns, [0, 0, 0])
    np.testing.assert_array_equal(scan.lines, [0, 2, 3])


def test_rejects_seconds_and_offset_overflow(tmp_path):
    with pytest.raises(ValueError, match="integer sample IDs"):
        write_pattern(tmp_path, [(0.0, 0, 90), (0.000005, 0, 90)])
    pattern = write_pattern(tmp_path, [(1, 0, 90)])
    with pytest.raises(ValueError, match="uint32"):
        pattern.next_frame(1000000)
