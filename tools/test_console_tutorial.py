#!/usr/bin/env python3
"""Native UI launch/observe/stop of an optional tutorial, on an isolated display."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests/console"))
from native_x11 import Inputs
from observe_session import Observer, child_session
from observe_session import owned_workers
from profile_console import process_stats


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--display", required=True)
    p.add_argument("--build", type=Path, default=ROOT / "build/flight-jammy")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--adapter-image", required=True)
    p.add_argument("--ros", choices=["ros1", "ros2"], default="ros1")
    p.add_argument("--measure-seconds", type=int, default=0)
    args = p.parse_args()
    if not 0 <= args.measure_seconds <= 60: p.error("measure-seconds must be 0..60")
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    cfg = json.loads((ROOT / "examples/tutorial-go2w.json").read_text())
    cfg.update(tutorial="fast-livo2", tutorial_ros=args.ros, adapters={args.ros: True})
    (out / "session.json").write_text(json.dumps(cfg))
    env = dict(os.environ, DISPLAY=args.display, LC_ALL="C.UTF-8", LIBGL_ALWAYS_SOFTWARE="1",
        EGL_PLATFORM="surfaceless", LP_NUM_THREADS="4", ROS_MASTER_URI="http://127.0.0.1:11993",
        ROS_IP="127.0.0.1", SIMPLE_RBT_START_MASTER="1", ROS_DOMAIN_ID="196", ROS_LOCALHOST_ONLY="1",
        XDG_CONFIG_HOME=str(out / "config"), XDG_STATE_HOME=str(out / "state"), XDG_CACHE_HOME=str(out / "cache"))
    env["SPROBO_" + args.ros.upper() + "_IMAGE"] = args.adapter_image
    checks = {}; proc = observer = inputs = window = None
    def check(name, value):
        checks[name] = bool(value)
        if not value: raise RuntimeError(name)
    with (out / "console.log").open("w") as log:
        try:
            proc = subprocess.Popen([str(args.build.resolve() / "simple-rbt-console"), "--worker",
                str(args.build.resolve() / "simple-rbt-sim"), "--assets", str(ROOT / "assets"),
                "--config", str(out / "session.json")], env=env, stdout=log, stderr=subprocess.STDOUT)
            directory = child_session(proc.pid)
            observer = Observer(directory, view=True)
            observer.wait(lambda m, v: observer.preview_ready(m), timeout=20)
            inputs = Inputs(args.display); window = inputs.window(); inputs.focus(window)
            time.sleep(5)  # Optional adapter's image inspection runs before enabling Start.
            inputs.capture(window, out / "configured.png")
            inputs.click(180, 825)
            observer.wait(lambda m, v: m.get("session_state") == "running", timeout=30)
            deadline = time.monotonic() + 35
            trajectory = None
            while time.monotonic() < deadline:
                files = list((out / "state").rglob("estimated-poses.csv"))
                if files and len(files[0].read_text().splitlines()) > 25:
                    trajectory = files[0]; break
                if proc.poll() is not None: raise RuntimeError("Console exited")
                time.sleep(.2)
            check("native_start_launches_cpp_frontend", trajectory is not None)
            contract = json.loads(trajectory.with_name("contract.json").read_text())
            check("live_calibration_without_truth_input", not contract["ground_truth_input"] and
                  contract["camera"]["cam_width"] == 320 and contract["camera"]["cam_height"] == 240)
            if args.measure_seconds:
                groups = {"console": [proc.pid], "core": [pid for pid, _ in owned_workers(proc.pid)]}
                containers = {}; samples = []
                for group, name in [("adapter", "simple-rbt-"+args.ros+"-"+directory.name),
                                    ("frontend", "sprobo-tutorial-"+directory.name)]:
                    rows = subprocess.check_output(["docker", "top", name, "-eo", "pid"], text=True).splitlines()[1:]
                    groups[group] = [int(row) for row in rows if row.strip()]
                    containers.update({pid: name for pid in groups[group]})
                def snapshot():
                    result = {}
                    for group, pids in groups.items():
                        rows = [process_stats(pid, containers.get(pid)) for pid in pids]
                        result[group] = {key: sum(row[key] for row in rows) for key in rows[0]}
                    return result
                first = snapshot(); start = time.monotonic(); initial_stamp = observer.snapshot()[0]["stamp_ns"]
                for _ in range(args.measure_seconds):
                    time.sleep(1)
                    sample = snapshot()
                    samples.append({"elapsed_s": time.monotonic()-start, "groups": sample})
                last = samples[-1]; elapsed = last["elapsed_s"]
                resources = {"conditions": "stationary Go2W, live first/third views, software GL, FAST-LIVO2; X server and measurement tool excluded",
                    "mean_cores": sum(last["groups"][key]["cpu_seconds"]-first[key]["cpu_seconds"] for key in groups)/elapsed,
                    "peak_rss_mib": max(sum(row["rss_mib"] for row in s["groups"].values()) for s in samples),
                    "peak_pss_mib": max(sum(row["pss_mib"] for row in s["groups"].values()) for s in samples),
                    "real_time_factor": (observer.snapshot()[0]["stamp_ns"]-initial_stamp)/1e9/elapsed,
                    "samples": samples}
                (out / "resources.json").write_text(json.dumps(resources, indent=2)+"\n")
            inputs.click(574, 90)
            time.sleep(.3)
            inputs.capture(window, out / "tutorial-running.png")
            # During operation, only Back and Stop frontend are enabled after this navigation button.
            inputs.tab(); inputs.tab(); inputs.press("Return")
            name = "sprobo-tutorial-" + directory.name
            deadline = time.monotonic() + 12
            stopped = False
            while time.monotonic() < deadline:
                result = subprocess.run(["docker", "inspect", "--format", "{{.State.Running}}", name],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                if result.returncode or result.stdout.strip() == "false": stopped = True; break
                time.sleep(.2)
            check("native_stop_ends_only_frontend", stopped and observer.snapshot()[0]["session_state"] == "running")
            inputs.capture(window, out / "tutorial-stopped.png")
            inputs.close_window(window); proc.wait(timeout=20)
            check("clean_console_exit", proc.returncode == 0)
        except Exception as e:
            checks["exception: " + str(e)] = False
            if inputs and window: inputs.capture(window, out / "failure.png")
        finally:
            if observer: observer.close()
            if inputs: inputs.close()
            if proc and proc.poll() is None: proc.terminate(); proc.wait(timeout=20)
    report = {"passed": bool(checks) and all(checks.values()), "checks": checks, "ros": args.ros,
        "console_sha256": hashlib.sha256((args.build / "simple-rbt-console").read_bytes()).hexdigest()}
    (out / "report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2)); raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__": main()
