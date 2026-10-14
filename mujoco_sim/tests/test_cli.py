import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


@pytest.mark.skipif(os.environ.get("SIMPLE_RBT_TEST_RENDER") != "1", reason="opt-in host OpenGL rendering")
def test_saved_sensor_times_and_depth_metadata(tmp_path):
    output = tmp_path / "capture"
    result = subprocess.run([
        sys.executable, "-m", "simple_rbt_mujoco.cli", "--robot", "diffdrive",
        "--duration", ".252", "--settle", "0", "--rays", "1000", "--cmd", "2", "0", "0",
        "--cameras", "--output", str(output),
    ], env={**os.environ, "MUJOCO_GL": "egl"}, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    summary = json.loads((output / "summary.json").read_text())
    camera = json.loads((output / "camera-metadata.json").read_text())
    scan = np.load(output / "last-scan.npz")
    assert scan["stamp_ns"] == 100_000_000
    assert scan["reference_stamp_ns"] == summary["last_lidar_frame_end_ns"] == 200_000_000
    assert scan["acquisition_mode"] == "rolling"
    assert scan["capture_position_world_m"].shape == scan["xyz"].shape
    assert scan["capture_quaternion_world_xyzw"].shape == (len(scan["xyz"]), 4)
    imu = np.loadtxt(output / "mid360-imu-si.csv", delimiter=",", skiprows=1)
    np.testing.assert_array_equal(imu[:, 0], np.arange(51) * 5_000_000)
    assert summary["imu_samples"] == 51
    assert summary["imu_model"]["orientation_available"] is False
    assert camera["stamp_ns"] == summary["saved_camera_stamp_ns"] == 252_000_000
    assert np.all((scan["ray_indices"] >= 20000) & (scan["ray_indices"] < 21000))
    assert summary["requested_command_m_s_rad_s"] == [2, 0, 0]
    assert summary["final_applied_command_m_s_rad_s"] == [1, 0, 0]
    assert camera["cameras"]["perspective"]["depth_kind"] == "z"
    assert camera["cameras"]["panorama"]["depth_kind"] == "range"
    depth = np.load(output / "panorama-depth-range.npy")
    assert np.isposinf(depth).any()
    assert not np.isnan(depth).any()
    assert (output / "perspective-rgb.png").is_file()
