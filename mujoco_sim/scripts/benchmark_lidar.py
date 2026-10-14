#!/usr/bin/env python3
"""Measure completed CPU/GPU scans and compare them on identical moving poses.

Timing includes CSV indexing, ray tracing, device-to-host results, filtering and
point construction. Physics, policy inference, setup and warmup are excluded.
Each pair uses the SAME native-step pose history. Rolling mode evaluates every
ray at its acquisition pose; snapshot mode is an explicit frame-end debug path.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simple_rbt_mujoco.core import SCAN_CSV, UPSTREAM_COMMIT, Simulation
from simple_rbt_mujoco.lidar import CsvLidar, CsvScanPattern
from simple_rbt_mujoco.rolling_lidar import PoseHistory, RollingCsvLidar


def distribution(values):
    x = np.asarray(values, dtype=np.float64)
    if len(x) == 0:
        return {"count": 0, "median": None, "p95": None, "max": None}
    return {"count": int(len(x)), "median": float(np.median(x)),
            "p95": float(np.percentile(x, 95)), "max": float(x.max())}


def benchmark(args):
    import mujoco
    import warp as wp
    wp.init()
    gpu = wp.get_device(args.gpu_device)
    if not gpu.is_cuda:
        raise ValueError("GPU comparison requires a CUDA device")
    # Preserve the controller used by the original LiDAR comparison even when
    # the interactive simulator's default changes. Scene code is hashed below.
    sim = Simulation("go2", controller="legacy")
    sensor_class = RollingCsvLidar if args.lidar_motion == "rolling" else CsvLidar
    lidar = {
        "cpu": sensor_class(sim.model, CsvScanPattern.from_csv(SCAN_CSV), backend="cpu"),
        "warp_cuda": sensor_class(sim.model, CsvScanPattern.from_csv(SCAN_CSV),
                             backend="warp", device=args.gpu_device),
    }
    frame_dt = args.rays / lidar["cpu"].pattern.sample_rate_hz
    frame_ns = round(frame_dt * 1e9)
    history = PoseHistory(retention_ns=max(250_000_000, frame_ns))
    position, quaternion = sim.pose("lidar")
    history.add(0, position, quaternion[[3, 0, 1, 2]])
    history.add(frame_ns, position, quaternion[[3, 0, 1, 2]])

    def completed_scan(name, stamp):
        if name == "warp_cuda":
            wp.synchronize_device(gpu)
        start = time.perf_counter_ns()
        scan = lidar[name].scan(sim.data, stamp, args.rays,
                               **({"history": history} if args.lidar_motion == "rolling" else {}))
        if name == "warp_cuda":
            wp.synchronize_device(gpu)
        return scan, (time.perf_counter_ns() - start) / 1e6

    for _ in range(args.warmup):
        for name in lidar:
            completed_scan(name, 0)
    for sensor in lidar.values():
        sensor.pattern.reset()
    history = PoseHistory(retention_ns=max(250_000_000, frame_ns))
    history.add(0, position, quaternion[[3, 0, 1, 2]])
    sim.set_command(args.speed, 0, 0.1)
    steps_per_frame = round(frame_dt / sim.model.opt.timestep)
    if abs(steps_per_frame * sim.model.opt.timestep - frame_dt) > 1e-9:
        raise ValueError("Ray frame duration must be an integer number of physics steps")

    latencies = {name: [] for name in lidar}
    frame_results = []
    all_errors = []
    total_mismatches = 0
    total_common = 0
    mismatch_details = []
    bad_time_or_line = 0
    poses, joints = [sim.data.qpos[:3].copy()], [sim.data.qpos[7:19].copy()]
    for frame_idx in range(args.frames):
        start_stamp = sim.stamp_ns
        for _ in range(steps_per_frame):
            sim.step()
            position, quaternion = sim.pose("lidar")
            history.add(sim.stamp_ns, position, quaternion[[3, 0, 1, 2]])
        poses.append(sim.data.qpos[:3].copy())
        joints.append(sim.data.qpos[7:19].copy())
        scans = {}
        # Alternate ordering to avoid systematically rewarding one backend.
        names = list(lidar) if frame_idx % 2 == 0 else list(reversed(lidar))
        for name in names:
            scans[name], elapsed = completed_scan(name, start_stamp if args.lidar_motion == "rolling" else sim.stamp_ns)
            latencies[name].append(elapsed)
        cpu, gpu_scan = scans["cpu"], scans["warp_cuda"]
        common, ci, gi = np.intersect1d(cpu.ray_indices, gpu_scan.ray_indices,
                                      assume_unique=True, return_indices=True)
        mismatch = len(cpu.ray_indices) + len(gpu_scan.ray_indices) - 2 * len(common)
        if mismatch:
            differing = np.setxor1d(cpu.ray_indices, gpu_scan.ray_indices)
            for ray_id in differing:
                detail = {"frame": frame_idx, "ray_index": int(ray_id)}
                for name, scan in scans.items():
                    selected = scan.xyz[scan.ray_indices == ray_id]
                    detail[name + "_point_local"] = selected[0].tolist() if len(selected) else None
                    # Raw rolling coordinates belong to each point's own pose;
                    # do not label a frame-end transform as its world position.
                mismatch_details.append(detail)
        errors = np.abs(np.linalg.norm(cpu.xyz[ci].astype(np.float64), axis=1)
                        - np.linalg.norm(gpu_scan.xyz[gi].astype(np.float64), axis=1))
        times_match = np.array_equal(cpu.offsets_ns[ci], gpu_scan.offsets_ns[gi])
        lines_match = np.array_equal(cpu.lines[ci], gpu_scan.lines[gi])
        if not times_match or not lines_match or cpu.stamp_ns != gpu_scan.stamp_ns:
            bad_time_or_line += 1
        all_errors.append(errors)
        total_common += len(common)
        total_mismatches += mismatch
        frame_results.append({"frame": frame_idx, "stamp_ns": cpu.stamp_ns,
                              "cpu_returns": cpu.point_num, "gpu_returns": gpu_scan.point_num,
                              "common_returns": len(common), "hit_mask_mismatches": mismatch,
                              "max_range_error_m": float(errors.max()) if len(errors) else None})
    errors = np.concatenate(all_errors)
    poses, joints = np.asarray(poses), np.asarray(joints)
    timings = {name: distribution(values) for name, values in latencies.items()}
    differences_above_tolerance = int((errors > args.tolerance_m).sum())
    passed = total_mismatches == 0 and differences_above_tolerance == 0 and bad_time_or_line == 0
    return {
        "robot": "go2", "controller": sim.controller_name, "control_mode": "native",
        "upstream_commit": UPSTREAM_COMMIT,
        "core_source_sha256": hashlib.sha256(
            (Path(__file__).resolve().parents[1] / "simple_rbt_mujoco/core.py").read_bytes()).hexdigest(),
        "scene_note": "Current core room; legacy controller matches the historical comparison, but geometry may differ across core revisions.",
        "command": [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]],
        "csv": str(SCAN_CSV), "csv_sha256": hashlib.sha256(SCAN_CSV.read_bytes()).hexdigest(),
        "mujoco_version": mujoco.__version__, "warp_version": wp.__version__,
        "python_version": platform.python_version(), "gpu_device": args.gpu_device,
        "gpu_name": gpu.name, "frames": args.frames, "warmup_scans_per_backend": args.warmup,
        "rays_per_scan": args.rays, "ray_rate_hz": lidar["cpu"].pattern.sample_rate_hz,
        "command_m_s_rad_s": [args.speed, 0, 0.1],
        "simulated_duration_s": args.frames * frame_dt,
        "timing_scope": "completed adapter scan including readback; excluding physics, setup and warmup",
        "scan_geometry": lidar["cpu"].metadata,
        "completed_scan_latency_ms": timings,
        "raw_median_cpu_over_gpu_latency_ratio": timings["cpu"]["median"] / timings["warp_cuda"]["median"],
        "comparison": {
            "ray_slots_compared": args.frames * args.rays,
            "common_returns": total_common,
            "hit_mask_mismatches": total_mismatches,
            "hit_mask_mismatch_fraction": total_mismatches / (args.frames * args.rays),
            "range_abs_error_m": distribution(errors),
            "range_tolerance_m": args.tolerance_m,
            "common_returns_exceeding_tolerance": differences_above_tolerance,
            "fraction_common_returns_exceeding_tolerance": differences_above_tolerance / max(1, total_common),
            "frames_with_time_or_line_mismatch": bad_time_or_line,
            "numerical_agreement_passed": passed,
            "criteria": "identical valid-ray mask, timestamps and lines; all common ranges within tolerance",
        },
        "motion": {"initial_base_xyz": poses[0].tolist(), "final_base_xyz": poses[-1].tolist(),
                   "base_displacement_m": float(np.linalg.norm(poses[-1] - poses[0])),
                   "base_height_min_m": float(poses[:, 2].min()),
                   "base_height_max_m": float(poses[:, 2].max()),
                   "joint_peak_to_peak_rad": np.ptp(joints, axis=0).tolist()},
        "frames_detail": frame_results,
        "hit_mask_mismatch_details": mismatch_details,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--rays", type=int, default=20000)
    parser.add_argument("--lidar-motion", choices=["rolling", "snapshot"], default="rolling")
    parser.add_argument("--gpu-device", default="cuda:0")
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--tolerance-m", type=float, default=0.001)
    parser.add_argument("--require-agreement", action="store_true",
                        help="exit 1 unless valid-ray masks/times/lines match and common ranges meet tolerance")
    parser.add_argument("--output", type=Path, default=Path("output/validation-20261013/lidar-benchmark.json"))
    args = parser.parse_args()
    if args.frames <= 0 or args.warmup < 0 or not 0 < args.rays <= 800000:
        parser.error("frames/rays must be positive; rays <= CSV length; warmup >= 0")
    result = benchmark(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"report": str(args.output), "latency_ms": result["completed_scan_latency_ms"],
                      "comparison": result["comparison"], "motion": result["motion"]}, indent=2))
    return 1 if args.require_agreement and not result["comparison"]["numerical_agreement_passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
