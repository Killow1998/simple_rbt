#!/usr/bin/env python3
"""Real native input: arm controls, both camera mounts, presets and focus ownership."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"tests/console"))
from native_x11 import Inputs
from observe_session import Observer, child_session, owned_workers
from test_manipulator import Peer
from test_console_native import project, gizmo_length


def rotate(point, quaternion):
    w, x, y, z = quaternion
    cross = lambda a, b: [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    c = cross([x, y, z], point)
    cc = cross([x, y, z], c)
    return [point[i]+2*w*c[i]+2*cc[i] for i in range(3)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--display", required=True)
    parser.add_argument("--console", type=Path, default=ROOT/"build/software-panorama/simple-rbt-console")
    parser.add_argument("--core", type=Path, default=ROOT/"build/software-panorama/simple-rbt-sim")
    parser.add_argument("--assets", type=Path, default=ROOT/"assets")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = {"robot": "panda", "backend": "cpu", "camera_hz": 10, "camera_model": "d435",
              "sensors": {"mid360": False, "camera": True, "depth": True, "panorama": False, "external_camera": True}}
    (output/"session.json").write_text(json.dumps(config))
    env = dict(os.environ, DISPLAY=args.display, LC_ALL="C.UTF-8", XDG_CONFIG_HOME=str(output/"config"),
               XDG_STATE_HOME=str(output/"state"), XDG_CACHE_HOME=str(output/"cache"))
    command = [str(args.console.resolve()), "--worker", str(args.core.resolve()), "--assets", str(args.assets.resolve())]
    process = observer = peer = inputs = window = sender = None
    stop = threading.Event()
    checks = {}

    def check(name, condition):
        checks[name] = bool(condition)
        if not condition:
            raise RuntimeError(name)

    def wait_joints(predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            joints, _ = peer.snapshot()
            if joints and predicate(joints[-1][1]["position"]):
                return joints[-1][1]["position"]
            time.sleep(.04)
        raise RuntimeError("Arm did not respond to native input: " + str(peer.snapshot()[0][-1:]))

    def scroll_top():
        inputs.move(180, 400)
        for _ in range(28):
            inputs.button(True, 4)
            inputs.button(False, 4)
        time.sleep(.15)

    def drag_mount(name):
        if name == "external_camera":
            inputs.move(850, 500)
            for _ in range(5):
                inputs.button(True, 5)
                inputs.button(False, 5)
            time.sleep(.5)
        metadata, _ = observer.snapshot()
        time.sleep(.5)
        rectangle = inputs.viewport(window, metadata["observer"]["width"], metadata["observer"]["height"])
        mount = metadata["mounts"][name]["position"]
        parent = metadata["mount_parents"][name]
        world = lambda p: [a+b for a, b in zip(parent["position"], rotate(p, parent["quaternion"]))]
        projection = json.loads(json.dumps(metadata))
        projection["base_pose"]["position"] = [0, 0, 0]
        length = gizmo_length(projection, world(mount), rectangle)
        origin = project(projection, world(mount), rectangle)
        tips = [project(projection, world([mount[j]+(length if j == i else 0) for j in range(3)]), rectangle) for i in range(3)]
        axis = max(range(3), key=lambda i: math.dist(origin, tips[i]))
        tip = tips[axis]
        direction = [(tip[i]-origin[i])/math.dist(origin, tip) for i in range(2)]
        inputs.move(*tip)
        inputs.button(True)
        inputs.move(tip[0]+24*direction[0], tip[1]+24*direction[1])
        inputs.button(False)
        updated, _ = observer.wait(lambda m, v: math.dist(m["mounts"][name]["position"], mount) > .005)
        delta = [a-b for a, b in zip(updated["mounts"][name]["position"], mount)]
        check(name+"_drag_respects_parent_axes", abs(delta[axis]) > .005 and all(abs(delta[i]) < 1e-6 for i in range(3) if i != axis))

    with (output/"console.log").open("w") as log:
        try:
            process = subprocess.Popen(command+["--config", str(output/"session.json")], env=env, stdout=log, stderr=subprocess.STDOUT)
            directory = child_session(process.pid)
            observer = Observer(directory, view=True)
            observer.wait(lambda m, v: m.get("session_state") == "config" and observer.preview_ready(m), timeout=20)
            inputs = Inputs(args.display)
            window = inputs.window()
            inputs.focus(window)
            inputs.click(180, 90)
            time.sleep(.4)
            inputs.number(180, 174, "bench")
            inputs.click(180, 346)
            inputs.press("Escape")
            for _ in range(10):
                inputs.tab()
            inputs.type_number("0.06000")
            observer.wait(lambda m, v: abs(m["mounts"]["camera"]["position"][0]-.06) < 1e-7)
            check("wrist_numeric_mount", True)
            drag_mount("camera")
            scroll_top()
            inputs.click(180, 754)
            inputs.press("End")
            inputs.press("Return")
            inputs.tab()
            inputs.type_number("1.10000")
            observer.wait(lambda m, v: abs(m["mounts"]["external_camera"]["position"][0]-1.1) < 1e-7)
            check("external_numeric_mount", True)
            drag_mount("external_camera")
            saved_mounts = observer.snapshot()[0]["mounts"]
            scroll_top()
            inputs.click(75, 247)
            time.sleep(.3)
            workspace_path = output/"config/simple-rbt/workspace.json"
            workspace = json.loads(workspace_path.read_text())
            preset = workspace["robots"][workspace["robot_id"]]
            check("named_preset_keeps_both_cameras", preset["name"] == "bench" and preset["sensors"]["external_camera"] and
                  all(preset["mounts"][key] == saved_mounts[key] for key in ("camera", "external_camera")))
            inputs.capture(window, output/"mount-editor.png")
            inputs.click(60, 90)
            inputs.click(190, 825)
            observer.wait(lambda m, v: m.get("session_state") == "running", timeout=15)
            peer = Peer(directory)
            wait_joints(lambda q: len(q) == 9)
            inputs.number(240, 262, "20")
            wait_joints(lambda q: abs(q[0]-math.radians(20)) < .02)
            check("native_joint_target", True)
            inputs.number(240, 550, "0")
            wait_joints(lambda q: max(abs(x) for x in q[7:]) < .005)
            inputs.number(240, 550, "80")
            wait_joints(lambda q: min(q[7:]) > .035)
            check("native_gripper_open_close", True)
            inputs.number(240, 262, "80")
            inputs.blur()
            time.sleep(.8)
            held = wait_joints(lambda q: True)[0]
            time.sleep(1)
            after = wait_joints(lambda q: True)[0]
            check("manual_blur_holds_pose", after < 1.2 and abs(after-held) < .01)
            inputs.focus(window)
            inputs.click(180, 196)
            inputs.press("Home")
            inputs.press("Down")
            inputs.press("Return")
            observer.wait(lambda m, v: m.get("command_source") == "program" or m.get("source") == "program")

            def send_program():
                while not stop.wait(.04):
                    peer.send(15, {"name": ["joint1"], "position": [-.2]})

            sender = threading.Thread(target=send_program, daemon=True)
            sender.start()
            inputs.blur()
            wait_joints(lambda q: abs(q[0]+.2) < .02)
            check("program_control_continues_without_focus", True)
            inputs.focus(window)
            inputs.click(95, 783)
            observer.wait(lambda m, v: m.get("command_source") == "gui" or m.get("source") == "gui")
            stop.set()
            sender.join(timeout=2)
            check("stop_returns_manual_ownership", True)
            time.sleep(.5)
            inputs.capture(window, output/"runtime-dark-en.png")
            check("dark_text_contrast", inputs.readability(window)[0])
            inputs.click(1030, 32); inputs.press("End"); inputs.press("Return")
            inputs.click(1160, 32); inputs.press("End"); inputs.press("Return")
            time.sleep(.4)
            inputs.capture(window, output/"runtime-light-zh.png")
            check("light_text_contrast", inputs.readability(window)[0])
            _, frames = peer.snapshot()
            check("both_live_rgb_depth_streams", all(len([k for k in frames[channel] if isinstance(k, int)]) > 50 for channel in (8, 17)))
            workers = owned_workers(process.pid)
            inputs.close_window(window)
            process.wait(timeout=10)
            check("window_close_stops_owned_core", process.returncode == 0 and all(not Path(f"/proc/{pid}").exists() for pid, _ in workers))
            peer.close(); peer = None
            observer.close()
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
            observer = Observer(child_session(process.pid), view=True)
            restored, _ = observer.wait(lambda m, v: m.get("session_state") == "config" and observer.preview_ready(m), timeout=20)
            check("restart_restores_mounts", all(restored["mounts"][key] == saved_mounts[key] for key in ("camera", "external_camera")))
            window = inputs.window()
            inputs.capture(window, output/"restored-preset.png")
            inputs.close_window(window)
            process.wait(timeout=10)
        except Exception as error:
            checks["exception: "+str(error)] = False
            if inputs and window:
                inputs.capture(window, output/"failure.png")
        finally:
            stop.set()
            if sender: sender.join(timeout=2)
            if peer: peer.close()
            if observer: observer.close()
            if inputs: inputs.close()
            if process and process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
    report = {"passed": all(checks.values()), "checks": checks,
              "core_sha256": hashlib.sha256(args.core.read_bytes()).hexdigest(),
              "console_sha256": hashlib.sha256(args.console.read_bytes()).hexdigest()}
    (output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
