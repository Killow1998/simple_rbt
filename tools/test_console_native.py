#!/usr/bin/env python3
"""Drive the installed/native MyGo window through real X11 events on a test display."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import struct
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests/console"))
from native_x11 import Inputs, luminance
from observe_session import Observer, child_session, owned_workers


def project(metadata, point, viewport):
    observer, base = metadata["observer"], metadata["base_pose"]["position"]
    eye, look, up = observer["position"], observer["lookat"], observer["up"]
    def unit(v):
        length = math.sqrt(sum(x*x for x in v))
        return [x/length for x in v]
    def cross(a, b):
        return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    def dot(a, b):
        return sum(x*y for x, y in zip(a, b))
    forward = unit([look[i]-eye[i] for i in range(3)])
    right = unit(cross(forward, up)); up = cross(right, forward)
    relative = [point[i]+base[i]-eye[i] for i in range(3)]
    width, height = observer["width"], observer["height"]
    vx, vy, vw, vh = viewport
    scale = min(vw/width, vh/height)
    focal = height*scale*.5/math.tan(math.radians(observer["vertical_fov_deg"])*.5)
    depth = dot(relative, forward)
    return vx+vw*.5+focal*dot(relative, right)/depth, vy+vh*.5-focal*dot(relative, up)/depth


def gizmo_length(metadata, mount, viewport):
    observer = metadata["observer"]
    forward = [b-a for a,b in zip(observer["position"],observer["lookat"])]
    norm = math.sqrt(sum(x*x for x in forward))
    relative = [mount[i]+metadata["base_pose"]["position"][i]-observer["position"][i] for i in range(3)]
    depth = sum(x*y/norm for x,y in zip(relative,forward))
    height = min(viewport[2]/observer["width"],viewport[3]/observer["height"])*observer["height"]
    focal = height*.5/math.tan(math.radians(observer["vertical_fov_deg"])*.5)
    return min(96*min(1,height/400)*depth/focal,depth*.4)


def color_near(inputs, window, position, color, radius=3):
    width,height,pixels = inputs.pixels(window)
    x,y = map(round,position)
    return any(max(abs(pixels[(v*width+u)*3+c]-color[c]) for c in range(3))<24
               for v in range(max(0,y-radius),min(height,y+radius+1))
               for u in range(max(0,x-radius),min(width,x+radius+1)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--display", required=True, help="Isolated Xvfb display; not your desktop")
    parser.add_argument("--console", type=Path, default=ROOT / "build/desktop/simple-rbt-console")
    parser.add_argument("--core", type=Path, default=ROOT / "build/desktop/simple-rbt-sim")
    parser.add_argument("--backend", choices=("cpu", "auto", "cuda"), default="auto")
    parser.add_argument("--assets", type=Path, default=ROOT / "assets")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(); output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    robot_names = sorted(robot["name"] for robot in json.loads(subprocess.check_output(
        [str(args.core.resolve()), "--list-robots"], text=True)))
    config = {"robot": "ranger_mini", "controller": "auto", "control_mode": "tracked",
              "terrain": "flat", "terrain_level": "standard", "backend": args.backend, "camera_hz": 10, "camera_model":"d435i",
              "sensors": {"mid360": True, "camera": True, "depth": True, "panorama": False}}
    (output / "config.json").write_text(json.dumps(config))
    env = dict(os.environ, DISPLAY=args.display, LC_ALL="C.UTF-8", XDG_CONFIG_HOME=str(output / "config"),
               XDG_STATE_HOME=str(output / "state"), XDG_CACHE_HOME=str(output / "cache"))
    checks, observer, inputs, window = [], None, None, None
    program, program_thread = None, None
    program_stop = threading.Event()
    def check(label, passed, evidence=None):
        checks.append({"check": label, "passed": bool(passed), "evidence": evidence})
        if not passed:
            raise RuntimeError(label)
    with (output / "console.log").open("w") as log:
        console_sha = hashlib.sha256(args.console.read_bytes()).hexdigest()
        core_sha = hashlib.sha256(args.core.read_bytes()).hexdigest()
        process = subprocess.Popen([str(args.console.resolve()), "--worker", str(args.core.resolve()),
            "--assets", str(args.assets.resolve()), "--config", str(output / "config.json"),
            "--capture", str(output / "live-scene.png"), "--capture-after-ms", "80000"], env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            directory = child_session(process.pid); observer = Observer(directory, view=True)
            metadata, _ = observer.wait(lambda m, v: m.get("session_state") == "config" and observer.preview_ready(m))
            time.sleep(1)
            inputs = Inputs(args.display); window = inputs.window(); inputs.focus(window)
            inputs.click(175,90); time.sleep(.4)
            old_workers = owned_workers(process.pid)
            def choose_robot(last=False, index=0):
                inputs.move(180,300)
                for _ in range(30):
                    inputs.t.XTestFakeButtonEvent(inputs.d,4,True,0);inputs.t.XTestFakeButtonEvent(inputs.d,4,False,0)
                inputs.x.XFlush(inputs.d);time.sleep(.12)
                inputs.click(180, 346)
                name = "End" if last else "Home"
                inputs.key(name, True); inputs.key(name, False)
                for _ in range(index):
                    inputs.key("Down", True); inputs.key("Down", False)
                inputs.key("Return", True); inputs.key("Return", False)
            choose_robot(last=True)
            time.sleep(.23)
            choose_robot()
            time.sleep(.23)
            choose_robot(index=robot_names.index("ranger_mini"))
            observer.close()
            directory = child_session(process.pid, exclude=directory, robot="ranger_mini")
            observer = Observer(directory, view=True)
            metadata, _ = observer.wait(lambda m, v: m.get("session_state") == "config" and observer.preview_ready(m) and m.get("robot") == "ranger_mini")
            time.sleep(.5)
            check("rapid native robot changes retain one latest worker", len(owned_workers(process.pid)) == 1)
            check("superseded worker processes exit", all(not Path(f"/proc/{pid}").exists() for pid,_ in old_workers))
            inputs.capture(window, output / "native-configuration.png")
            appearance_path = output / "config/simple-rbt-console/appearance.json"
            appearance_worker = owned_workers(process.pid)
            translated_labels = {}
            def appearance(theme, language, saved=True, phase="native"):
                time.sleep(.25)
                readable, colors = inputs.readability(window)
                preferences = json.loads(appearance_path.read_text()) if saved else None
                inputs.find_window()
                expected_title = "机器人操作台" if language == "zh-CN" else "Robot console"
                brightness = luminance(colors[0]["background_rgb"])
                theme_matches = brightness < .1 if theme == "dark" else brightness > .5
                preferences_match = not saved or preferences == {"theme": theme, "language": language}
                inputs.capture(window, output / f"{phase}-{theme}-{language}.png")
                if theme == "light":
                    width, _, pixels = inputs.pixels(window)
                    translated_labels[language] = b"".join(
                        pixels[(y*width+18)*3:(y*width+340)*3] for y in range(78, 132))
                check(f"{phase} {theme}/{language} renders readable text and selected appearance",
                    readable and theme_matches and preferences_match
                    and any(expected_title in title for title in inputs.titles),
                    {"text": colors, "preferences": preferences, "window_titles": inputs.titles})
            appearance("dark", "en", saved=False)
            def appearance_select(x,last):
                inputs.click(x,32);inputs.press("End" if last else "Home");inputs.press("Return")
            appearance_select(1030,True);appearance("light","en")
            appearance_select(1160,True);appearance("light","zh-CN")
            changed_text=sum(a!=b for a,b in zip(translated_labels["en"],translated_labels["zh-CN"]))
            check("language selection changes visible configuration labels",changed_text>100)
            appearance_select(1030,False);appearance("dark","zh-CN")
            appearance_select(1160,False);appearance("dark","en")
            check("changing appearance keeps the same C++ worker",owned_workers(process.pid)==appearance_worker)
            inputs.click(180,346);inputs.press("Escape")
            # The Robot selector retains focus after Enter. Walk the real
            # keyboard order: three other selectors, four sensor checkboxes,
            # camera model, the sensor selector, then X. Font metrics cannot change this order.
            for _ in range(10):
                inputs.press("Tab")
            inputs.type_number("0.03000")
            metadata, _ = observer.wait(lambda m, v: abs(m.get("mounts", {}).get("mid360", {}).get("position", [99])[0]-.03)<1e-6)
            check("native keyboard numeric edit reaches C++ core", True)
            original = metadata["mounts"]["mid360"]["position"][:]
            viewport = inputs.viewport(window, metadata["observer"]["width"], metadata["observer"]["height"])
            (output / "native-layout.json").write_text(json.dumps({"observer_rectangle": viewport}, indent=2)+"\n")
            length = gizmo_length(metadata,original,viewport)
            origin = project(metadata,original,viewport)
            tips = [project(metadata,[original[j]+(length if i==j else 0) for j in range(3)],viewport) for i in range(3)]
            colors = [(242,116,116),(79,217,157),(111,162,255)]
            # Inspect actual native pixels on all three diagonals. The former
            # horizontal/vertical-only Painter.Line cannot pass this check.
            check("three visible axis shafts connect toward their common origin",
                  all(color_near(inputs,window,[origin[j]+(tip[j]-origin[j])*t for j in range(2)],color)
                      for tip,color in zip(tips,colors) for t in (.25,.45,.65)))
            inputs.capture(window,output / "gizmo-connected.png")
            tip = tips[0]
            inputs.move(*tip); time.sleep(.15)
            check("hover highlights the same diagonal that will be dragged",
                  color_near(inputs,window,[(a+b)*.5 for a,b in zip(origin,tip)],(255,225,154)))
            inputs.capture(window,output / "gizmo-hover.png")
            inputs.move(*tip); inputs.button(True); inputs.move(tip[0]+22, tip[1]-8); inputs.button(False)
            metadata, _ = observer.wait(lambda m, v: abs(m["mounts"]["mid360"]["position"][0]-.03)>.005)
            check("native XTEST translation drag changes core mount", True, metadata["mounts"]["mid360"])
            inputs.click(1050, 146)
            inputs.key("End", True); inputs.key("End", False)
            inputs.key("Return", True); inputs.key("Return", False)
            time.sleep(.2)
            mount = metadata["mounts"]["mid360"]["position"]
            radius = gizmo_length(metadata,mount,viewport)*.8
            ring = lambda angle: project(metadata, [mount[0]+radius*math.cos(angle), mount[1]+radius*math.sin(angle), mount[2]], viewport)
            inputs.move(700,140); time.sleep(.15)
            check("rotation ring follows its projected curve",
                  all(color_near(inputs,window,ring(angle),(111,162,255)) for angle in (.4,.8,1.2)))
            inputs.capture(window,output / "gizmo-rotation.png")
            inputs.move(*ring(.73)); inputs.button(True); inputs.move(*ring(1.13)); inputs.button(False)
            metadata, _ = observer.wait(lambda m, v: abs(m["mounts"]["mid360"]["quaternion"][0])<.995)
            check("native XTEST rotation ring changes core quaternion", True, metadata["mounts"]["mid360"])
            mount = metadata["mounts"]["mid360"]["position"]
            imu = metadata["mid360_internal_imu"]["position"]
            w, x, y, z = metadata["mounts"]["mid360"]["quaternion"]
            offset = [.011,.02329,-.04412]
            vector = [x,y,z]
            dot = sum(a*b for a,b in zip(vector,offset))
            cross = [y*offset[2]-z*offset[1],z*offset[0]-x*offset[2],x*offset[1]-y*offset[0]]
            rotated = [2*vector[i]*dot+(2*w*w-1)*offset[i]+2*w*cross[i] for i in range(3)]
            check("MID360 internal IMU remains rigid", all(abs(imu[i]-mount[i]-rotated[i])<1e-7 for i in range(3)))
            check("controller body IMU remains independent", metadata["body_imu_mount"]["position"] == [0,0,.04])
            # Exercise Xian's intended Go2W driving workflow after the wheel
            # chassis mount editor checks, using the real robot selector.
            choose_robot(index=robot_names.index("go2w"))
            observer.close()
            directory=child_session(process.pid,exclude=directory,robot="go2w")
            observer=Observer(directory, view=True)
            metadata,_=observer.wait(lambda m,v:m.get("session_state")=="config" and observer.preview_ready(m) and m.get("robot")=="go2w")
            check("Go2W retains selected D435i device",metadata.get("camera_model")=="d435i" and metadata.get("camera_imu_locked"))
            time.sleep(.5)
            inputs.click(60,90);time.sleep(.4)
            inputs.click(*inputs.primary_button(window))
            observer.wait(lambda m, v: m.get("session_state") == "running")
            time.sleep(2)
            initial, _ = observer.snapshot(); initial = initial["base_pose"]["position"]
            inputs.move(180, 296); inputs.button(True)
            metadata, _ = observer.wait(lambda m, v: m.get("applied_command", [0])[0]>.15)
            time.sleep(1.6)
            metadata, _ = observer.snapshot()
            distance = math.dist(initial, metadata["base_pose"]["position"])
            check("native joystick moves physical robot", distance>.05, {"displacement_m": distance})
            inputs.button(False)
            observer.wait(lambda m, v: m.get("applied_command") == [0,0,0] and math.hypot(v[0],v[1])<.1)
            check("native pointer release stops physical robot", True)
            # Actual X11 key events must reach the same core command path.
            inputs.click(180,348)
            for key,axis,sign in [('w',0,1),('s',0,-1),('a',1,1),('d',1,-1),('q',2,1),('e',2,-1)]:
                inputs.key(key,True)
                metadata,_=observer.wait(lambda m,v:sign*m.get("applied_command",[0,0,0])[axis]>.05)
                command=metadata["applied_command"]
                check("keyboard "+key.upper()+" maps to its signed axis",all(abs(command[i])<1e-9 for i in range(3) if i!=axis),command)
                inputs.key(key,False)
                observer.wait(lambda m,v:m.get("applied_command")==[0,0,0])
            inputs.key('w',True)
            started,_=observer.wait(lambda m,v:m.get("applied_command",[0])[0]>.1)
            initial=started["base_pose"]["position"]
            trace=[];deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                time.sleep(.1);m,v=observer.snapshot();trace.append({'stamp':m['stamp_ns'],'command':m['applied_command'],'position':m['base_pose']['position'],'velocity':v})
                if m['stamp_ns']>=started['stamp_ns']+2000000000:break
            (output/'keyboard-hold-trace.json').write_text(json.dumps(trace,indent=2))
            metadata,_=observer.wait(lambda m,v:m.get("applied_command",[0])[0]>.1)
            check("held keyboard moves the physical robot",math.dist(initial,metadata["base_pose"]["position"])>.05,{"initial":initial,"metadata":metadata,"velocity":observer.snapshot()[1]})
            inputs.press('space')
            observer.wait(lambda m,v:m.get("applied_command")==[0,0,0]);time.sleep(.8)
            check("Space stops without repeat restarting motion",observer.snapshot()[0]["applied_command"]==[0,0,0])
            inputs.key('w',False)
            inputs.key('w',True)
            observer.wait(lambda m,v:m.get("applied_command",[0])[0]>.1)
            inputs.tab()
            observer.wait(lambda m,v:m.get("applied_command")==[0,0,0])
            check("keyboard focus leaving the drive control stops motion",True)
            inputs.key('w',False)
            inputs.click(180,348);inputs.key('w',True)
            observer.wait(lambda m,v:m.get("applied_command",[0])[0]>.1)
            inputs.blur()
            observer.wait(lambda m,v:m.get("applied_command")==[0,0,0])
            check("window focus loss clears held keyboard keys",True)
            inputs.key('w',False);inputs.focus(window)
            inputs.move(180, 296); inputs.button(True)
            observer.wait(lambda m, v: m.get("applied_command", [0])[0]>.15)
            inputs.blur()
            observer.wait(lambda m, v: m.get("applied_command") == [0,0,0] and math.hypot(v[0],v[1])<.1)
            check("actual native window focus loss stops robot", True)
            inputs.button(False); inputs.focus(window)
            # Pick the non-ROS program owner through the real native selector.
            inputs.click(180,202)
            inputs.key("End",True); inputs.key("End",False)
            inputs.key("Return",True); inputs.key("Return",False)
            observer.wait(lambda m,v:m.get("source")=="program")
            program = Observer(directory, role="program")
            def command_program():
                payload = json.dumps({"velocity":[.25,0,0]}).encode()
                message = struct.pack("<4sHHQII",b"SRBT",1,3,0,len(payload),0)+payload
                while not program_stop.is_set():
                    program.connections[0].sendall(message)
                    program_stop.wait(.05)
            program_thread = threading.Thread(target=command_program,daemon=True)
            program_thread.start()
            metadata,_ = observer.wait(lambda m,v:m.get("applied_command",[0])[0]>.2)
            check("program receipt is visible in core status", metadata.get("command_status", {}).get("state") == "receiving")
            initial = metadata["base_pose"]["position"]
            inputs.blur(); time.sleep(1.2)
            metadata,_ = observer.wait(lambda m,v:m.get("source")=="program" and m.get("applied_command",[0])[0]>.2)
            distance = math.dist(initial,metadata["base_pose"]["position"])
            check("program control continues after actual native focus loss",distance>.1,{"displacement_m":distance})
            program_stop.set(); program_thread.join(timeout=2)
            observer.wait(lambda m,v:m.get("applied_command")==[0,0,0] and math.hypot(v[0],v[1])<.1)
            metadata,_ = observer.wait(lambda m,v:m.get("command_status",{}).get("state")=="timed_out")
            check("program silence still triggers command deadman",True)
            check("timeout remains distinct from a connected program", metadata["connections"]["program"] > 0)
            program.close(); program=None
            inputs.focus(window)
            time.sleep(.5)
            inputs.capture(window, output / "native-window.png")
            appearance_select(1030,True);appearance_select(1160,True)
            appearance("light","zh-CN",phase="before-reopen")
            process.wait(timeout=90)
            check("native console closes owned core gracefully", process.returncode == 0)
            check("actual X11 window screenshot exists", (output / "native-window.png").is_file())
            observer.close(); observer = None; window = None
            # Reuse the preferences saved by the previous native window. The
            # second process receives no language/theme flags or injected values.
            process = subprocess.Popen([str(args.console.resolve()), "--worker", str(args.core.resolve()),
                "--assets", str(args.assets.resolve()), "--config", str(output / "config.json"),
                "--capture", str(output / "reopened-live-scene.png"), "--capture-after-ms", "6500"],
                env=env, stdout=log, stderr=subprocess.STDOUT)
            directory = child_session(process.pid); observer = Observer(directory, view=True)
            observer.wait(lambda m, v: m.get("session_state") == "config" and observer.preview_ready(m))
            time.sleep(.5)
            window = inputs.window(); inputs.focus(window)
            appearance("light", "zh-CN", phase="reopened")
            process.wait(timeout=15)
            check("reopened native console closes owned core gracefully", process.returncode == 0)
        except Exception as error:
            if inputs and window:
                try:
                    inputs.capture(window, output / "native-failure.png")
                except Exception as capture_error:
                    checks.append({"check": "failure screenshot", "passed": False, "evidence": str(capture_error)})
            checks.append({"check": "exception", "passed": False, "evidence": str(error)})
        finally:
            program_stop.set()
            if program_thread: program_thread.join(timeout=2)
            if program: program.close()
            if inputs:
                inputs.button(False); inputs.close()
            if observer:
                observer.close()
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
    report = {"passed": all(c["passed"] for c in checks), "checks": checks,
              "console_sha256": console_sha, "core_sha256": core_sha,
              "input": "XTEST keyboard/pointer and XSetInputFocus; no application callbacks invoked"}
    (output / "report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
