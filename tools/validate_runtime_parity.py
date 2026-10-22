#!/usr/bin/env python3
"""Compare native runtimes; non-RGB payloads always require exact parity."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
# Process identity and wall-clock measurements cannot be bit-identical.
VOLATILE = {
    "output", "runtime_pid", "wall_seconds", "real_time_factor",
    "resources_loop", "resources_run", "lidar_median_ms", "lidar_p95_ms",
    "camera_pair_median_ms", "camera_pair_p95_ms",
}


def cases():
    robots = ("go2", "go2w", "scout_mini", "fr_mini", "ridgeback",
              "ranger_mini", "turtlebot3_waffle_pi")
    for robot in robots:
        yield robot, ["--robot", robot, "--sensors", "physics", "--duration", "12",
                      "--cmd", ".3", "0", ".15"]
    for motion in ("rolling", "snapshot"):
        yield motion, ["--robot", "scout_mini", "--sensors", "lidar", "--duration", "12",
                       "--cmd", ".6", "0", ".3", "--lidar-motion", motion]
    yield "full-d435i", ["--robot", "go2w", "--sensors", "full", "--duration", "2",
                         "--camera-model", "d435i", "--camera-hz", "10"]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rgb_difference(left, right, mae_limit, outlier_limit):
    import numpy as np
    from PIL import Image
    a, b = np.asarray(Image.open(left)), np.asarray(Image.open(right))
    if a.shape != b.shape or a.dtype != b.dtype or a.ndim != 3 or a.shape[2] != 3:
        raise RuntimeError(f"RGB format changed: {left.name}")
    difference = np.abs(a.astype(np.float32) - b)
    result = {"mean_absolute_channel_error": float(difference.mean()),
              "max_channel_error": float(difference.max()),
              "pixels_above_10_fraction": float((difference.max(axis=2) > 10).mean()),
              "before_sha256": digest(left), "after_sha256": digest(right)}
    if result["mean_absolute_channel_error"] > mae_limit or result["pixels_above_10_fraction"] > outlier_limit:
        raise RuntimeError(f"RGB quality budget exceeded: {left.name}: {result}")
    return result


def compare(left, right, rgb_mae_limit=0, rgb_outlier_limit=0):
    names = {p.name for p in left.iterdir() if p.is_file()}
    other = {p.name for p in right.iterdir() if p.is_file()}
    if names != other:
        raise RuntimeError(f"Artifact set changed: {names ^ other}")
    result = {}
    for name in sorted(names):
        a, b = left / name, right / name
        if name == "summary.json":
            before, after = json.loads(a.read_text()), json.loads(b.read_text())
            before = {k: v for k, v in before.items() if k not in VOLATILE}
            after = {k: v for k, v in after.items() if k not in VOLATILE}
            if before != after:
                raise RuntimeError(f"Summary changed: {[k for k in before.keys() | after.keys() if before.get(k) != after.get(k)]}")
            result[name] = "all deterministic fields equal"
        elif rgb_mae_limit > 0 and name in ("panorama-rgb.png", "perspective-rgb.png"):
            result[name] = rgb_difference(a, b, rgb_mae_limit, rgb_outlier_limit)
        else:
            if digest(a) != digest(b):
                raise RuntimeError(f"Artifact bytes changed: {name}")
            result[name] = digest(a)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rgb-mae-limit", type=float, default=0,
                        help="Opt-in RGB-only mean channel difference limit, 0..255; default exact")
    parser.add_argument("--rgb-outlier-limit", type=float, default=0,
                        help="Allowed fraction of RGB pixels with any channel difference >10, 0..1")
    args = parser.parse_args()
    if not 0 <= args.rgb_mae_limit <= 255 or not 0 <= args.rgb_outlier_limit <= 1:
        parser.error("RGB quality limits are outside their valid ranges")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, LIBGL_ALWAYS_SOFTWARE="1", EGL_PLATFORM="surfaceless",
               __EGL_VENDOR_LIBRARY_FILENAMES="/usr/share/glvnd/egl_vendor.d/50_mesa.json",
               SIMPLE_RBT_LOG_DIR=str(output / "logs"), SIMPLE_RBT_TRACE="0")
    report = {"before_sha256": digest(args.before), "after_sha256": digest(args.after),
              "rgb_mae_limit": args.rgb_mae_limit, "rgb_outlier_limit": args.rgb_outlier_limit,
              "excluded_summary_fields": sorted(VOLATILE), "cases": [], "passed": False}
    try:
        for name, options in cases():
            case = output / name
            case.mkdir()
            for label, binary in (("before", args.before), ("after", args.after)):
                with (case / (label + ".log")).open("w") as log:
                    subprocess.run([str(binary.resolve()), "--project-root", str(ROOT),
                                    "--backend", "cpu", "--output", str(case / label),
                                    *options], env=env, stdout=log, stderr=subprocess.STDOUT,
                                   check=True, timeout=180)
            result = compare(case / "before", case / "after", args.rgb_mae_limit, args.rgb_outlier_limit)
            report["cases"].append({"name": name, "artifacts": result})
            print(f"PASS {name}: {len(result)} artifacts", flush=True)
        report["passed"] = True
    finally:
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
