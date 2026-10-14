"""Join same-run worker evidence with independent received DDS evidence."""
import numpy as np
from full_run_evidence import digest_json


def full_run_checks(config, evidence, received, commands, child_returncode, initial_sources, worker_pid=None, *, current_sources):
    """Join independent DDS evidence with the same worker's formal gates."""
    from simple_rbt_mujoco.ros2 import robot_command_topic
    from simple_rbt_mujoco.robots import DEFAULT_CONTROLLERS
    expected_controller = (DEFAULT_CONTROLLERS[config["robot"]]
                           if config["controller"] == "auto" else config["controller"])
    command_topic = robot_command_topic(config["robot"])
    transport = evidence.get("ros_transport") or {}
    course = evidence.get("formal_course", {})
    expected = evidence.get("emitted", {})
    required = ("imu", "imu_si", "lidar", "pointcloud2", "odom", "tf", "rgb", "depth", "panorama_rgb", "panorama_depth", "camera_info")
    checks = {
        "worker_exited_cleanly": child_returncode == 0 and not evidence.get("error"),
        "same_run_identity": bool(received.status and received.status.get("run_id") == config["run_id"] == evidence.get("run_id")
            and received.status.get("pid") == evidence.get("pid") == worker_pid
            and received.status.get("config_sha256") == evidence.get("config_sha256") == digest_json(config)),
        "native_controller_unchanged": evidence.get("controller") == expected_controller,
        "source_unchanged_during_run": evidence.get("sources") == initial_sources == current_sources,
        "robot_command_namespace_matches": bool(received.status and
            config.get("command_topic") == evidence.get("command_topic") ==
            received.status.get("command_topic") == transport.get("command_topic") == command_topic and
            transport.get("robot_name") == config["robot"] and
            transport.get("node_namespace") == "/" + config["robot"]),
        "formal_route_pass": course.get("success") is True and course.get("failure_reason") is None,
        "original_speed_thresholds_pass": course.get("speed_tracking", {}).get("passed") is True,
        "effective_contact_friction_verified": evidence.get("friction", {}).get("passed") is True,
        "ros_message_contracts": not received.errors,
        "all_reliable_messages_received": all(expected.get(key, 0) > 0 and received.counts[key] == expected[key] for key in required),
        "rtf_at_least_requested_minimum": evidence.get("real_time_factor", 0) >= config["min_real_time_factor"],
    }
    end = evidence.get("simulation_end_ns", 0)
    final_clock = received.last_stamp.get("clock", 0)
    checks["clock_monotonic_and_complete"] = bool(received.first_clock is not None and received.first_clock < 1_000_000_000
        and final_clock <= end and end - final_clock <= 10_000_000 and final_clock > 1_000_000_000)
    checks["imu_payloads_match_worker"] = bool(received.counts["imu"] and received.imu_hash.hexdigest() == evidence.get("imu_sha256"))
    checks["imu_si_payloads_match_worker"] = bool(received.counts["imu_si"] and received.imu_si_hash.hexdigest() == evidence.get("imu_si_sha256"))
    expected_imu_stamps = list(range(0, end + 1, 5_000_000))
    checks["mid360_imu_exact_200hz_grid"] = bool(end and received.imu_stamps == expected_imu_stamps == received.imu_si_stamps)
    samples = evidence.get("imu_samples", [])
    dt_ns = round(evidence.get("physics_timestep_s", 0) * 1e9)
    checks["imu_sampling_brackets_valid"] = bool(samples and dt_ns > 0 and
        [row["stamp_ns"] for row in samples] == expected_imu_stamps and all(
            len(row.get("source_stamps_ns", [])) == 2 and
            0 <= row["source_stamps_ns"][0] <= row["stamp_ns"] <= row["source_stamps_ns"][1] <= end and
            row["source_stamps_ns"][1] - row["source_stamps_ns"][0] <= dt_ns and
            ((row.get("sampling_method") == "exact" and row["source_stamps_ns"] == [row["stamp_ns"]] * 2) or
             (row.get("sampling_method") == "linear_interpolation" and
              row["source_stamps_ns"][0] < row["stamp_ns"] < row["source_stamps_ns"][1]))
            for row in samples))
    imu_stamp_set = set(received.imu_stamps) & set(received.imu_si_stamps)
    checks["imu_covers_every_complete_scan"] = bool(received.scans and all(
        all(t in imu_stamp_set for t in range(row["stamp_ns"], row["stamp_ns"] + 100_000_001, 5_000_000))
        for row in received.scans))
    checks["rolling_lidar_acquisition"] = bool(evidence.get("lidar_model", {}).get("acquisition_model") == "rolling_static_environment"
        and evidence.get("scans") and all(row.get("metadata", {}).get("acquisition_model") == "rolling_static_environment"
            for row in evidence["scans"]))
    checks["rolling_lidar_pose_history_covers_acquisition"] = bool(evidence.get("scans") and all(
        len(row.get("metadata", {}).get("history_coverage_ns", [])) == 2 and
        row["metadata"].get("first_scheduled_point_ns") == row["stamp_ns"] and
        row["metadata"].get("last_scheduled_point_ns") == row["stamp_ns"] + 99_995_000 and
        0 <= row["metadata"]["history_coverage_ns"][0] <= row["stamp_ns"] and
        row["stamp_ns"] + 99_995_000 <= row["metadata"]["history_coverage_ns"][1] <= end
        for row in evidence["scans"]))
    checks["lidar_payloads_match_worker"] = bool(received.scans and
        [{k: row[k] for k in ("stamp_ns", "point_count", "scan_sha256")} for row in received.scans] ==
        [{k: row[k] for k in ("stamp_ns", "point_count", "scan_sha256")} for row in evidence.get("scans", [])]
        and max(row["last_point_ns"] for row in received.scans) <= final_clock)
    checks["pointcloud_payloads_match_worker"] = bool(received.points and received.points ==
        [{k: row[k] for k in ("stamp_ns", "pointcloud2_sha256")} for row in evidence.get("scans", [])])
    checks["all_image_payloads_match_worker"] = bool(len(received.images) == 4 and received.images == evidence.get("cameras"))
    period = round(1e9 / config["camera_hz"])
    expected_stamps = list(range(period, end + 1, period))
    checks["cameras_at_requested_simulation_rate"] = bool(expected_stamps and len(received.images) == 4 and
        all([row["stamp_ns"] for row in received.images[key]] == expected_stamps for key in received.images))
    checks["open_panorama_has_infinite_sky_depth"] = received.image_stats.get("panorama_depth", {}).get("no_hit_count", 0) > 0
    keys = ("stamp_ns", "position", "quaternion", "linear", "angular")
    expected_odom = [{key: row[key] for key in keys} for row in evidence.get("odometry", [])]
    checks["odometry_matches_same_physics_run"] = bool(received.odometry and received.odometry == expected_odom)
    checks["dynamic_tf_matches_odometry"] = bool(received.odometry and len(received.dynamic_tf) == len(received.odometry) and all(
        row["stamp_ns"] in received.dynamic_tf and received.dynamic_tf[row["stamp_ns"]]["position"] == row["position"]
        and received.dynamic_tf[row["stamp_ns"]]["quaternion"] == row["quaternion"] for row in received.odometry))
    try:
        from check_ros2 import model_sensor_transforms, mid360_extrinsics_match
        actual_model = model_sensor_transforms(config["robot"], evidence.get("controller", config["controller"]))
        checks["static_tf_matches_model"] = all(frame in received.static_tf and received.static_tf[frame]["parent"] == "base_link"
            and np.allclose(received.static_tf[frame]["position"], transform["translation"], rtol=0, atol=1e-7)
            and abs(abs(np.dot(received.static_tf[frame]["quaternion"], transform["quaternion"])) - 1) < 1e-7
            for frame, transform in actual_model.items())
        lidar_tf, imu_tf = received.static_tf.get("livox_frame"), received.static_tf.get("imu_frame")
        checks["mid360_internal_rigid_transform"] = bool(lidar_tf and imu_tf and mid360_extrinsics_match(
            lidar_tf["position"], lidar_tf["quaternion"], imu_tf["position"], imu_tf["quaternion"]))
    except Exception:
        checks["static_tf_matches_model"] = False
        checks["mid360_internal_rigid_transform"] = False
    finish = course.get("finish_stamp_ns")
    finish_x = course.get("thresholds", {}).get("finish_x_m")
    checks["finish_observed_in_received_odometry"] = bool(finish is not None and finish_x is not None
        and received.status and received.status.get("finish_x_m") == finish_x and any(
            row["stamp_ns"] >= finish and row["position"][0] >= finish_x for row in received.odometry))
    terminal = [np.linalg.norm(row["linear"][:2]) for row in received.odometry
                if finish is not None and row["stamp_ns"] >= finish + 1_000_000_000]
    terminal_speed = float(np.mean(terminal)) if terminal else None
    stopped_transition = next((row for row in evidence.get("command_transitions", [])
        if finish is not None and row["stamp_ns"] >= finish and row["command"] == [0., 0., 0.]), None)
    checks["command_sent_via_ros_through_course"] = bool(commands and finish is not None and
        commands[0]["sim_stamp_ns"] >= round(config["settle_s"] * 1e9)
        and commands[-1]["sim_stamp_ns"] >= finish - 150_000_000)
    checks["deadman_after_finish"] = bool(finish is not None and commands and stopped_transition
        and evidence.get("final_command") == [0., 0., 0.] and end >= finish + 2_000_000_000
        and stopped_transition["monotonic_s"] - commands[-1]["monotonic_s"] >= .45
        and terminal_speed is not None and terminal_speed < .08)
    route_ros_realtime = all(value for key, value in checks.items() if key != "original_speed_thresholds_pass")
    return checks, {"overall_acceptance": all(checks.values()),
                    "route_ros_realtime_pass": route_ros_realtime,
                    "terminal_mean_planar_speed_m_s": terminal_speed,
                    "minimum_realtime_factor": config["min_real_time_factor"],
                    "speed_tracking_is_required_for_overall_acceptance": True}
