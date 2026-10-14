"""Compare real Noetic payloads with files saved by the C++ runtime."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(value):
    return hashlib.sha256(value).hexdigest()


def compare_payloads(output, ros1, images):
    output = Path(output)
    latest = ros1["latest"]
    checks = {}
    summary = json.loads((output / "summary.json").read_text())
    lidar = json.loads((output / "lidar-metadata.json").read_text())
    xyz = np.load(output / "lidar-xyz.npy").astype("<f4", copy=False)
    offsets = np.load(output / "lidar-offsets-ns.npy").astype("<u4", copy=False)
    lines = np.load(output / "lidar-lines.npy").astype("u1", copy=False)
    custom = latest["/livox/lidar"]
    checks["custom_msg_final_stamp"] = custom["stamp_ns"] == custom["timebase"] == lidar["stamp_ns"]
    checks["custom_msg_attributes"] = custom["attributes_match"] and custom["lidar_id"] == 0 and custom["reserved"] == [0, 0, 0]
    checks["custom_msg_all_points"] = custom["point_num"] == custom["actual_points"] == len(xyz)
    checks["custom_msg_payload_bytes"] = custom["sha256"] == sha(xyz.tobytes() + offsets.tobytes() + lines.tobytes())
    dtype = np.dtype([(n, t) for n, t in zip(
        ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
        ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"])])
    points = np.zeros(len(xyz), dtype=dtype)
    for index, name in enumerate(["x", "y", "z"]):
        points[name] = xyz[:, index]
    points["intensity"] = 100
    points["line"] = lines
    points["timestamp"] = np.asarray(offsets, dtype="u8") + lidar["stamp_ns"]
    cloud = latest["/livox/points"]
    checks["pointcloud2_payload_bytes"] = cloud["sha256"] == sha(points.tobytes())
    checks["pointcloud2_layout"] = cloud["point_step"] == 26 and cloud["row_step"] == len(xyz) * 26
    camera = json.loads((output / "camera-metadata.json").read_text())
    for prefix, name in [("/camera", "perspective"), ("/camera/panorama", "panorama")]:
        for suffix in ["/image_raw", "/depth/image_raw"]:
            received = latest[prefix + suffix]
            dimensions = camera[name]
            checks[name + suffix.replace("/", "_") + "_layout"] = (received["frame_id"] == "camera_optical_frame"
                and received["width"] == dimensions["width"] and received["height"] == dimensions["height"]
                and received["encoding"] == ("rgb8" if suffix == "/image_raw" else "32FC1"))
            checks[name + suffix.replace("/", "_") + "_stamp"] = received["stamp_ns"] == camera["stamp_ns"]
            checks[name + suffix.replace("/", "_") + "_bytes"] = (received["sha256"] ==
                images[prefix + suffix].get(str(received["stamp_ns"])))
    checks["camera_intrinsics"] = np.array_equal(latest["/camera/camera_info"]["K"], np.asarray(camera["perspective"]["K"]).ravel())
    with (output / "mid360-imu-si.csv").open() as stream:
        samples = {int(row["stamp_ns"]): row for row in csv.DictReader(stream)}
    for topic, wire in [("/livox/imu", True), ("/livox/imu_si", False)]:
        received = latest[topic]
        checks[topic + "_frame"] = received["frame_id"] == ("livox_frame" if wire else "imu_frame")
        checks[topic + "_covariance"] = received["orientation_covariance"] == ([0.] * 9 if wire else [-1.] + [0.] * 8)
        row = samples.get(received["stamp_ns"])
        checks[topic + "_timestamp"] = row is not None
        if row:
            gyro = np.asarray([float(row[k]) for k in ["wx_rad_s", "wy_rad_s", "wz_rad_s"]])
            accel = np.asarray([float(row[k]) for k in ["specific_force_x_m_s2", "specific_force_y_m_s2", "specific_force_z_m_s2"]])
            if wire:
                gyro = gyro.astype("f4").astype("f8")
                accel = (accel / 9.80665).astype("f4").astype("f8")
            checks[topic + "_values"] = np.array_equal(received["gyro"], gyro) and np.array_equal(received["acceleration"], accel)
    static = {transform["child"]: transform for transform in latest["/tf_static"]}
    checks["static_frames"] = {"livox_frame", "imu_frame", "camera_optical_frame"}.issubset(static)
    if checks["static_frames"]:
        lidar_tf, imu_tf = static["livox_frame"], static["imu_frame"]
        q = np.asarray(lidar_tf["xyzw"])
        offset = np.asarray([.011, .02329, -.04412])
        rotated = offset + 2 * np.cross(q[:3], np.cross(q[:3], offset) + q[3] * offset)
        checks["mid360_imu_extrinsics"] = np.allclose(np.asarray(imu_tf["position"]) - lidar_tf["position"], rotated, rtol=0, atol=1e-10)
    checks["command_namespace"] = summary["ros_command_topic"] == ros1["command_topic"] == "/" + ros1["robot"] + "/cmd_vel"
    checks["final_deadman_command"] = summary["final_applied_command_m_s_rad_s"] == [0., 0., 0.]
    checks["real_time_pacing"] = .95 <= summary["real_time_factor"] <= 1.05
    return checks, summary
