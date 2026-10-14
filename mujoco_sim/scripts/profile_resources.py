#!/usr/bin/env python3
"""Measure real-time CPU/RAM/VRAM and disk use in fresh simulator processes."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import statistics
import time

from simple_rbt_mujoco.core import ROOT
from simple_rbt_mujoco.resources import directory_size


def process_tree(root_pid):
    todo, found = [root_pid], []
    while todo:
        pid = todo.pop()
        if pid in found:
            continue
        found.append(pid)
        try:
            children = Path(f"/proc/{pid}/task/{pid}/children").read_text().split()
            todo.extend(map(int, children))
        except FileNotFoundError:
            pass
    return found


def resident_kib(pids):
    total = 0
    for pid in pids:
        try:
            for line in Path(f"/proc/{pid}/status").read_text().splitlines():
                if line.startswith("VmRSS:"):
                    total += int(line.split()[1])
        except (FileNotFoundError, ProcessLookupError):
            pass
    return total


def gpu_memory_mib(pids):
    # Per-process compute memory. Global GPU memory would include the desktop.
    try:
        result = subprocess.run([
            "nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits",
        ], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode:
        return None
    total = 0
    for row in result.stdout.splitlines():
        parts = [x.strip() for x in row.split(",")]
        if len(parts) == 2 and parts[0].isdigit() and int(parts[0]) in pids and parts[1].isdigit():
            total += int(parts[1])
    return total


def main():
    from simple_rbt_mujoco.robots import ROBOT_NAMES, COMMAND_LIMITS, validate_terrain
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", choices=("cpp", "python"), default="cpp")
    parser.add_argument("--robots", nargs="+", choices=ROBOT_NAMES, default=["go2", "go2w"])
    parser.add_argument("--terrain", default="hills")
    parser.add_argument("--terrain-level", choices=["standard", "moderate", "hard", "extreme"], default="standard")
    parser.add_argument("--controller", default="auto")
    parser.add_argument("--control-mode", choices=["native", "tracked"], default="tracked")
    parser.add_argument("--go2-torque-mode", choices=["configured", "upstream"], default="configured")
    parser.add_argument("--go2-startup", choices=["crouched", "upstream_getup"], default="crouched")
    parser.add_argument("--friction", type=float)
    parser.add_argument("--duration", type=float, default=6.)
    parser.add_argument("--speed", type=float, default=.3)
    parser.add_argument("--profiles", nargs="+", choices=["physics", "lidar_cpu", "lidar_cuda", "full_1hz", "full_10hz", "full_10hz_unpaced"],
                        help="Run only the requested profiles; default is all six")
    parser.add_argument("--output", type=Path, required=True, help="New report directory")
    args = parser.parse_args()
    try:
        for robot in args.robots:
            validate_terrain(robot, args.terrain)
    except ValueError as error:
        parser.error(str(error))
    max_speed = min(COMMAND_LIMITS[robot][0] for robot in args.robots)
    if not math.isfinite(args.speed) or not 0 < args.speed <= max_speed or not math.isfinite(args.duration) or args.duration <= 0:
        parser.error(f"speed must be in (0,{max_speed:g}] and duration must be positive and finite")
    if args.friction is not None and (not math.isfinite(args.friction) or not 0 < args.friction <= 2):
        parser.error("friction must be in (0,2] and finite")
    if any(robot != "go2" for robot in args.robots) and (
            args.go2_torque_mode != "configured" or args.go2_startup != "crouched"):
        parser.error("Go2 deployment alternatives require --robots go2")
    args.output.mkdir(parents=True, exist_ok=False)
    profiles = [
        ("physics", ["--sensors", "physics", "--backend", "cpu"]),
        ("lidar_cpu", ["--sensors", "lidar", "--backend", "cpu"]),
        ("lidar_cuda", ["--sensors", "lidar", "--backend", "warp"]),
        ("full_1hz", ["--sensors", "full", "--backend", "warp", "--camera-hz", "1"]),
        ("full_10hz", ["--sensors", "full", "--backend", "warp", "--camera-hz", "10"]),
        ("full_10hz_unpaced", ["--sensors", "full", "--backend", "warp", "--camera-hz", "10"]),
    ]
    if args.profiles:
        profiles = [profile for profile in profiles if profile[0] in args.profiles]
    rows = []
    for robot in args.robots:
        for name, flags in profiles:
            output = args.output / f"{robot}-{name}"
            launcher = "run.sh" if args.runtime == "cpp" else "run-python.sh"
            command = [str(ROOT / "mujoco_sim" / launcher), "--robot", robot,
                       "--terrain", args.terrain, "--controller", args.controller,
                       "--control-mode", args.control_mode,
                       "--terrain-level", args.terrain_level,
                       "--go2-torque-mode", args.go2_torque_mode, "--go2-startup", args.go2_startup,
                       *([] if args.friction is None else ["--friction", str(args.friction)]),
                       "--duration", str(args.duration), "--settle", "2", "--cmd", str(args.speed), "0", "0",
                       *([] if name.endswith("_unpaced") else ["--realtime"]),
                       "--summary-only", "--output", str(output.resolve()), *flags]
            peak_ram = peak_gpu = 0
            samples = gpu_samples = 0
            ram_samples = []
            started = time.perf_counter()
            with (args.output / f"{robot}-{name}.log").open("x") as log:
                process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                try:
                    next_gpu = 0.
                    while process.poll() is None:
                        pids = process_tree(process.pid)
                        rss = resident_kib(pids) / 1024.
                        peak_ram = max(peak_ram, rss)
                        ram_samples.append((time.perf_counter(), rss))
                        samples += 1
                        now = time.perf_counter()
                        if now >= next_gpu:
                            value = gpu_memory_mib(pids)
                            if value is not None:
                                peak_gpu = max(peak_gpu, value)
                                gpu_samples += 1
                            next_gpu = now + .5
                        if now - started > max(120, args.duration * 20):
                            raise RuntimeError(f"Profile timed out: {robot}/{name}")
                        time.sleep(.05)
                finally:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
            if process.returncode:
                raise RuntimeError(f"Profile failed: {robot}/{name}; see log")
            summary = json.loads((output / "summary.json").read_text())
            interval = summary["resources_loop"]
            loop_rss = [rss for stamp, rss in ram_samples
                        if interval["interval_start_monotonic_s"] <= stamp <= interval["interval_end_monotonic_s"]]
            row = {"robot": robot, "runtime": args.runtime, "profile": name, "argv": command,
                   "sampled_process_tree_peak_rss_mib": peak_ram,
                   "sampled_compute_vram_peak_mib": peak_gpu if gpu_samples else None,
                   "ram_samples": samples, "gpu_samples": gpu_samples,
                   "loop_sampled_rss_median_mib": statistics.median(loop_rss) if loop_rss else None,
                   "loop_sampled_rss_max_mib": max(loop_rss) if loop_rss else None,
                   "whole_process_wall_seconds": time.perf_counter() - started,
                   "run_disk": directory_size(output), "simulation": summary}
            rows.append(row)
            print(json.dumps({"robot": robot, "profile": name, "peak_ram_mib": peak_ram,
                              "peak_vram_mib": peak_gpu, "rtf": summary["real_time_factor"],
                              "cpu_percent_one_core": summary["resources_loop"]["mean_cpu_percent_one_core"]}), flush=True)
            (args.output / "results.json").write_text(json.dumps({"cases": rows}, indent=2) + "\n")
    footprint = {name: directory_size(ROOT / name)
                 for name in [".venv", ".vendor", ".git", "mujoco_sim", "mujoco_sim_cpp", "output", "mid360_sim", "simple_rbt", "livox_ros_driver2"]}
    report = {
        "cpu_definition": "100% is one fully used logical core; full-machine percentage is also saved",
        "rss_definition": "per-case fresh process ru_maxrss plus 50ms sampled process-tree RSS; shared pages may count per process",
        "gpu_definition": "500ms sampled nvidia-smi per-process compute allocation; may omit graphics-only EGL allocations; not GPU utilization",
        "disk_definition": "logical and allocated bytes, no symlink target traversal; per-directory hardlink deduplication",
        "timing": "paced to real time except full_10hz_unpaced; loop includes physics, requested sensors and rendering but no ROS transport",
        "runtime": args.runtime,
        "startup": ("fresh native process; whole-process RAM includes libraries and initialization; loop begins after sensor warmup"
                    if args.runtime == "cpp" else
                    "process-tree sampled RAM and total elapsed include interpreter/imports; main() CPU meter excludes top-level imports; first EGL creation is inside loop"),
        "cases": rows, "project_footprint": footprint,
    }
    (args.output / "results.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
