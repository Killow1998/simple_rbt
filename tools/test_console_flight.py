#!/usr/bin/env python3
"""Actual XTEST flight controls, focus ownership and bilingual theme checks."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests/console"))
from native_x11 import Inputs
from observe_session import Observer, child_session, owned_workers


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--display", required=True)
    p.add_argument("--console", type=Path, default=ROOT / "build/flight-jammy/simple-rbt-console")
    p.add_argument("--core", type=Path, default=ROOT / "build/flight-jammy/simple-rbt-sim")
    p.add_argument("--assets", type=Path, default=ROOT / "assets")
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    cfg = json.loads((ROOT / "examples/skydio-x2.json").read_text())
    (out / "session.json").write_text(json.dumps(cfg))
    env = dict(os.environ, DISPLAY=args.display, LC_ALL="C.UTF-8",
               LIBGL_ALWAYS_SOFTWARE="1", EGL_PLATFORM="surfaceless", LP_NUM_THREADS="4",
               XDG_CONFIG_HOME=str(out / "config"), XDG_STATE_HOME=str(out / "state"),
               XDG_CACHE_HOME=str(out / "cache"))
    checks = {}
    observer = program = inputs = process = sender = window = None
    stop = threading.Event()

    def check(name, value):
        checks[name] = bool(value)
        if not value:
            raise RuntimeError(name)

    def xyz():
        return observer.snapshot()[0]["base_pose"]["position"]

    def flight_state(name):
        return observer.wait(lambda m, v: m.get("flight", {}).get("state") == name, timeout=12)

    def held(key, seconds=1.5):
        inputs.key(key, True)
        time.sleep(seconds)
        inputs.key(key, False)
        flight_state("hover")
        time.sleep(.8)

    with (out / "console.log").open("w") as log:
        try:
            process = subprocess.Popen([str(args.console.resolve()), "--worker", str(args.core.resolve()),
                "--assets", str(args.assets.resolve()), "--config", str(out / "session.json")],
                env=env, stdout=log, stderr=subprocess.STDOUT)
            directory = child_session(process.pid)
            observer = Observer(directory, view=True)
            observer.wait(lambda m, v: observer.preview_ready(m), timeout=20)
            inputs = Inputs(args.display)
            window = inputs.window()
            inputs.focus(window)
            inputs.capture(window, out / "launch.png")
            inputs.click(180, 825)
            observer.wait(lambda m, v: m.get("session_state") == "running", timeout=15)
            inputs.capture(window, out / "grounded.png")
            inputs.click(72, 328)
            flight_state("hover")
            check("native_takeoff", .9 < xyz()[2] < 1.1)
            inputs.click(900, 340)
            start = xyz()
            held("w")
            check("keyboard_forward_and_release", xyz()[0] > start[0] + .25)
            start = xyz()
            held("a")
            check("keyboard_lateral", xyz()[1] > start[1] + .25)
            start = xyz()
            held("r")
            check("keyboard_climb", xyz()[2] > start[2] + .2)
            held("q")
            q = observer.snapshot()[0]["base_pose"]["quaternion"]
            yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            check("keyboard_yaw", yaw > .4)
            inputs.key("w", True)
            time.sleep(.6)
            inputs.press("space")
            flight_state("hover")
            inputs.key("w", False)
            check("space_holds_position", True)
            inputs.key("w", True)
            time.sleep(.6)
            inputs.blur()
            flight_state("hover")
            inputs.key("w", False)
            observer.wait(lambda m, v: math.sqrt(sum(x*x for x in v)) < .025 and
                          math.dist(m["base_pose"]["position"], m["flight"]["target_position"]) < .025, timeout=5)
            before = xyz()
            time.sleep(1)
            check("manual_blur_holds_position", math.dist(before, xyz()) < .05)
            inputs.focus(window)
            inputs.click(180, 196)
            inputs.press("Home"); inputs.press("Down"); inputs.press("Return")
            observer.wait(lambda m, v: m.get("source") == "program" or m.get("command_source") == "program")
            program = Observer(directory, "program")

            def send():
                payload = json.dumps({"frame": "world_enu", "action": "velocity",
                    "velocity": [0., 0., .2], "yaw_rate": 0.}).encode()
                while not stop.wait(.04):
                    program.connections[0].sendall(struct.pack("<4sHHQII", b"SRBT", 1, 18, 0, len(payload), 0)+payload)

            before = xyz()
            sender = threading.Thread(target=send, daemon=True)
            sender.start()
            inputs.blur()
            observer.wait(lambda m, v: m["base_pose"]["position"][2] > before[2] + .2)
            check("program_continues_without_focus", True)
            stop.set(); sender.join(timeout=2)
            flight_state("hover")
            check("program_timeout_holds_position", True)
            inputs.focus(window)
            inputs.click(180, 196); inputs.press("Home"); inputs.press("Return")
            observer.wait(lambda m, v: m.get("source") == "gui" or m.get("command_source") == "gui")
            inputs.capture(window, out / "runtime-dark-en.png")
            check("dark_contrast", inputs.readability(window)[0])
            inputs.click(1030, 32); inputs.press("End"); inputs.press("Return")
            inputs.click(1160, 32); inputs.press("End"); inputs.press("Return")
            time.sleep(.4)
            inputs.capture(window, out / "runtime-light-zh.png")
            check("light_contrast", inputs.readability(window)[0])
            inputs.click(145, 356)  # Chinese row uses the native font's line height.
            flight_state("grounded")
            check("native_land", xyz()[2] < .06)
            inputs.click(55, 356)
            flight_state("hover")
            inputs.click(318, 356)
            flight_state("disarmed")
            inputs.blur()
            observer.wait(lambda m, v: m["base_pose"]["position"][2] < .1)
            check("native_motor_cut_does_not_rearm_on_blur", observer.snapshot()[0]["flight"]["state"] == "disarmed")
            workers = owned_workers(process.pid)
            inputs.close_window(window)
            process.wait(timeout=15)
            check("window_close_stops_owned_core", process.returncode == 0 and
                  all(not Path(f"/proc/{pid}").exists() for pid, _ in workers))
        except Exception as error:
            checks["exception: " + str(error)] = False
            if inputs and window:
                inputs.capture(window, out / "failure.png")
        finally:
            stop.set()
            if sender: sender.join(timeout=2)
            if program: program.close()
            if observer: observer.close()
            if inputs: inputs.close()
            if process and process.poll() is None:
                process.terminate(); process.wait(timeout=15)
    report = {"passed": bool(checks) and all(checks.values()), "checks": checks,
              "core_sha256": hashlib.sha256(args.core.read_bytes()).hexdigest(),
              "console_sha256": hashlib.sha256(args.console.read_bytes()).hexdigest()}
    (out / "report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
