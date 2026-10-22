#!/usr/bin/env python3
"""Compare two native runtimes on identical inputs without relaxing payload checks."""
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
                         "--camera-model", "d435i"]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compare(left, right):
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
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, LIBGL_ALWAYS_SOFTWARE="1", EGL_PLATFORM="surfaceless",
               __EGL_VENDOR_LIBRARY_FILENAMES="/usr/share/glvnd/egl_vendor.d/50_mesa.json",
               SIMPLE_RBT_LOG_DIR=str(output / "logs"), SIMPLE_RBT_TRACE="0")
    report = {"before_sha256": digest(args.before), "after_sha256": digest(args.after),
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
            result = compare(case / "before", case / "after")
            report["cases"].append({"name": name, "artifacts": result})
            print(f"PASS {name}: {len(result)} artifacts", flush=True)
        report["passed"] = True
    finally:
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
