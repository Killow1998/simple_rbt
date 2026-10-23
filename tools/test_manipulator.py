#!/usr/bin/env python3
"""Exercise the standalone arm through real IPC, including timed RGB-D output."""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def check_camera_geometry(output, joints, frames, check):
    """Reconstruct exported physics states independently of the camera worker."""
    import mujoco
    import numpy as np
    model = mujoco.MjModel.from_xml_path(str(output/"native/compiled-model.xml"))
    data = mujoco.MjData(model)
    states = dict(joints)
    errors = {}
    for kind, camera in [(8, "camera"), (17, "external_camera")]:
        stamp, payload = frames[kind]["last"]
        check(camera+"_same_physics_stamp", stamp in states)
        state = states[stamp]
        mujoco.mj_resetDataKeyframe(model, data, 0)
        for name, position in zip(state["name"], state["position"]):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[joint]] = position
        mujoco.mj_forward(model, data)
        tool, base = (mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
                      for name in ("arm_tool", "base_origin"))
        relative = data.site_xmat[base].reshape(3, 3).T @ (data.site_xpos[tool] - data.site_xpos[base])
        check(camera+"_tool_pose_matches_export", np.max(np.abs(relative-state["tool_pose"]["position"])) < 1e-9)
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, camera+"_depth")
        physical = data.site_xmat[site].reshape(3, 3)
        optical = np.column_stack((-physical[:, 1], -physical[:, 2], physical[:, 0]))
        depth = np.frombuffer(payload, dtype="<f4", offset=25+320*240*3).reshape(240, 320)
        focal = 120/np.tan(np.pi/6)
        differences = []
        for v in range(20, 240, 40):
            for u in range(20, 320, 40):
                ray = np.array([(u-159.5)/focal, (v-119.5)/focal, 1.])
                ray /= np.linalg.norm(ray)
                hit = np.array([-1], dtype=np.int32)
                # Start on the renderer's near plane: a device housing behind
                # that plane must not occlude the independent depth oracle.
                near = model.vis.map.znear*model.stat.extent/ray[2]
                direction = optical @ ray
                distance = near + mujoco.mj_ray(model, data, data.site_xpos[site]+near*direction, direction,
                                                np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8), 1, -1, hit)
                if hit[0] >= 0 and model.geom_type[hit[0]] == mujoco.mjtGeom.mjGEOM_PLANE:
                    differences.append(abs(float(depth[v, u]) - distance*ray[2]))
        errors[camera] = {"floor_pixels": len(differences), "max_error_m": max(differences)}
        (output/"geometry.json").write_text(json.dumps(errors, indent=2)+"\n")
        check(camera+"_analytic_floor_depth", len(differences) >= 8 and max(differences) < .002)
    return errors


class Peer:
    def __init__(self, directory, role="program"):
        self.closed = False
        self.lock = threading.Lock()
        self.joints = []
        self.frames = {8: {}, 17: {}}
        self.sockets = []
        token = uuid.uuid4().hex
        for channel in ("control", "data"):
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            connection.connect(str(directory / (channel+".sock")))
            self.sockets.append(connection)
            self.send(1, {"role": role, "token": token,
                          "subscriptions": ["joints", "camera", "external_camera"]}, connection)
            if channel == "control":
                kind, _, payload = self.read(connection)
                assert kind == 1
                self.hello = json.loads(payload)
        self.threads = [threading.Thread(target=self.receive, args=(c,), daemon=True) for c in self.sockets]
        for thread in self.threads:
            thread.start()

    @staticmethod
    def read(connection):
        def exact(size):
            result = bytearray()
            while len(result) < size:
                part = connection.recv(size-len(result))
                if not part:
                    raise EOFError
                result.extend(part)
            return bytes(result)
        magic, version, kind, stamp, size, reserved = struct.unpack("<4sHHQII", exact(24))
        assert magic == b"SRBT" and version == 1 and reserved == 0 and size <= 32*1024*1024
        return kind, stamp, exact(size)

    def send(self, kind, value, connection=None):
        payload = json.dumps(value).encode()
        (connection or self.sockets[0]).sendall(struct.pack("<4sHHQII", b"SRBT", 1, kind, 0, len(payload), 0)+payload)

    def receive(self, connection):
        try:
            while not self.closed:
                kind, stamp, payload = self.read(connection)
                with self.lock:
                    if kind == 16:
                        self.joints.append((stamp, json.loads(payload)))
                    elif kind in self.frames:
                        width, height, radial, rgb, depth = struct.unpack_from("<IIBQQ", payload)
                        assert (width, height, radial, rgb, depth) == (320, 240, 0, 320*240*3, 320*240)
                        self.frames[kind][stamp] = hashlib.sha256(payload[25:25+rgb]).hexdigest()
                        self.frames[kind]["last"] = (stamp, payload)
        except (EOFError, OSError):
            pass

    def snapshot(self):
        with self.lock:
            return list(self.joints), {k: dict(v) for k, v in self.frames.items()}

    def close(self):
        self.closed = True
        for connection in self.sockets:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        for thread in self.threads:
            thread.join(timeout=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core", type=Path, default=ROOT/"build/software-panorama/simple-rbt-sim")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    session = Path("/tmp")/("srbt-arm-"+uuid.uuid4().hex[:10]); session.mkdir(mode=0o700)
    config = {"robot": "panda", "backend": "cpu", "source": "program", "start_immediately": True,
              "duration_s": 12, "settle_s": 0, "camera_hz": 10, "camera_model": "d435",
              "sensors": {"mid360": False, "camera": True, "depth": True, "external_camera": True},
              "output": str(output/"native")}
    (output/"session.json").write_text(json.dumps(config, indent=2)+"\n")
    peer = wrong = process = None
    checks = {}
    def check(name, condition):
        checks[name] = bool(condition)
        if not condition:
            raise AssertionError(name)
    try:
        with (output/"core.log").open("w") as log:
            process = subprocess.Popen([str(args.core.resolve()), "--session-config", str(output/"session.json"),
                                        "--session-dir", str(session)], stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+20
            while not (session/"data.sock").exists():
                if process.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError("Core did not create the arm session")
                time.sleep(.05)
            peer = Peer(session)
            wrong = Peer(session, "ros1")
            deadline = time.monotonic()+30
            phases = {}
            while True:
                joints, _ = peer.snapshot()
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Core stopped before the control sequence completed")
                if not joints:
                    time.sleep(.02)
                    continue
                elapsed = joints[-1][0]/1e9
                if elapsed >= 10:
                    break
                if elapsed < 1:
                    wrong.send(15, {"name": ["joint1"], "position": [1.]})
                elif elapsed < 4:
                    peer.send(15, {"name": ["joint1", "finger_joint1"], "position": [.35, 0.]})
                elif elapsed < 6:
                    peer.send(15, {"name": ["joint1", "finger_joint1"], "position": [.35, .04]})
                elif elapsed < 6.25:
                    peer.send(15, {"name": ["joint1"], "position": [1.4]})
                phases[int(elapsed)] = joints[-1][1]
                time.sleep(.04)
            check("wrong_control_owner_ignored", abs(phases[0]["position"][0]) < .02)
            check("joint_position_reached", abs(phases[3]["position"][0]-.35) < .025)
            check("both_fingers_close", max(abs(q) for q in phases[3]["position"][7:]) < .005)
            check("both_fingers_open", min(phases[5]["position"][7:]) > .035)
            check("deadman_stops_before_target", phases[9]["position"][0] < 1.)
            check("deadman_holds_pose", abs(phases[9]["position"][0]-phases[8]["position"][0]) < .01)
            check("wrist_moves_with_joints", phases[0]["tool_pose"] != phases[3]["tool_pose"])
            check("wrist_tf_parent", peer.hello["extrinsics"]["camera_parent_frame"] == "arm_tool")
            check("external_camera_calibrated", peer.hello["extrinsics"]["has_external_camera"])
            process.wait(timeout=15)
            check("clean_exit", process.returncode == 0)
        joints, frames = peer.snapshot()
        for kind, name in [(8, "wrist"), (17, "external")]:
            stamps = sorted(k for k in frames[kind] if isinstance(k, int))
            check(name+"_10hz", len(stamps)>=100 and all(b-a==100000000 for a,b in zip(stamps,stamps[1:])))
            check(name+"_live_images", len({frames[kind][k] for k in stamps}) > 5)
            stamp, payload = frames[kind]["last"]
            (output/(name+"-rgb.u8")).write_bytes(payload[25:25+320*240*3])
            (output/(name+"-depth.f32")).write_bytes(payload[25+320*240*3:])
            from PIL import Image
            Image.frombytes("RGB", (320, 240), payload[25:25+320*240*3]).save(output/(name+".png"))
        (output/"joints.json").write_text(json.dumps(joints)+"\n")
        geometry = check_camera_geometry(output, joints, frames, check)
        summary=json.loads((output/"native/summary.json").read_text())
        check("complete_sensor_counts", summary["camera_frames_per_sensor"]==summary["external_camera_frames"]==120)
        report={"passed":all(checks.values()),"checks":checks,"phases":phases,"geometry":geometry,
                "real_time_factor":summary["real_time_factor"],"resources":summary["resources_loop"],
                "core_sha256":hashlib.sha256(args.core.read_bytes()).hexdigest()}
        (output/"report.json").write_text(json.dumps(report,indent=2)+"\n")
        print(json.dumps({"passed":report["passed"],"checks":len(checks),"real_time_factor":report["real_time_factor"]}))
    finally:
        for client in (wrong,peer):
            if client: client.close()
        if process and process.poll() is None:
            process.terminate();process.wait(timeout=10)
        for path in session.glob("*.sock"): path.unlink()
        session.rmdir()


if __name__ == "__main__":
    main()
