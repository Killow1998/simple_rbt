#!/usr/bin/env python3
"""Exercise session transitions through the public IPC connection."""
import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests/console"))
from observe_session import Observer


class Peer(Observer):
    def __init__(self, directory):
        self.errors = []
        super().__init__(directory)

    def read(self, connection):
        try:
            while not self.closed:
                kind, _, data = self.packet(connection)
                with self.lock:
                    if kind == 11:
                        self.metadata = json.loads(data)
                    elif kind == 13:
                        self.errors.append(json.loads(data))
        except (EOFError, OSError):
            pass

    def send(self, kind, payload):
        data = json.dumps(payload).encode()
        self.connections[0].sendall(struct.pack("<4sHHQII", b"SRBT", 1, kind, 0, len(data), 0) + data)

    def request(self, **payload):
        self.send(2, payload)

    def wait_error(self, message, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if any(message in item.get("message", "") for item in self.errors):
                    return
            time.sleep(.05)
        raise RuntimeError(f"Expected error not received: {message}")


def run(core, project_root, root, immediate):
    temporary = tempfile.TemporaryDirectory(prefix="srbt-lifecycle-")
    session = Path(temporary.name)
    output = root / "run"
    config = root / "config.json"
    config.write_text(json.dumps({
        "robot": "scout_mini", "backend": "cpu", "source": "gui",
        "start_immediately": immediate, "settle_s": 0,
        "sensors": {"mid360": False, "camera": False, "depth": False, "panorama": False},
        "output": str(output),
    }))
    with (root / "core.log").open("w") as log:
        process = subprocess.Popen([str(core), "--project-root", str(project_root),
                                    "--session-config", str(config), "--session-dir", str(session)],
                                   stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
        peer = None
        try:
            deadline = time.monotonic() + 20
            while not (session / "data.sock").exists():
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("Core did not create a session; inspect core.log")
                time.sleep(.05)
            peer = Peer(session)
            initial = "running" if immediate else "config"
            metadata, _ = peer.wait(lambda m, _: m.get("session_state") == initial)
            if not immediate:
                peer.request(op="pause")
                mount = metadata["mounts"]["mid360"]
                position = mount["position"][:]
                position[0] += .01
                peer.request(op="mount", sensor="mid360", position=position, quaternion=mount["quaternion"])
                metadata, _ = peer.wait(lambda m, _: m.get("mounts", {}).get("mid360", {}).get("position") == position)
                assert metadata["session_state"] == "config", "pause before start must preserve configuration"
                peer.request(op="start")
                metadata, _ = peer.wait(lambda m, _: m.get("session_state") == "running")
            stamp = metadata["stamp_ns"]
            peer.wait(lambda m, _: m.get("stamp_ns", 0) > stamp)
            peer.send(3, {"velocity": [.2, 0, 0]})
            peer.wait(lambda m, _: m.get("applied_command", [0])[0] > .1)
            peer.request(op="stop")
            peer.wait(lambda m, _: m.get("session_state") == "running" and m.get("applied_command") == [0, 0, 0])
            peer.request(op="pause")
            metadata, _ = peer.wait(lambda m, _: m.get("session_state") == "paused")
            stamp = metadata["stamp_ns"]
            time.sleep(.4)
            assert peer.snapshot()[0]["stamp_ns"] == stamp, "pause must freeze simulation time"
            mount = metadata["mounts"]["mid360"]
            peer.request(op="mount", sensor="mid360", position=mount["position"], quaternion=mount["quaternion"])
            peer.wait_error("Restart configuration before editing a started session")
            peer.request(op="start")
            peer.wait(lambda m, _: m.get("session_state") == "running" and m.get("stamp_ns", 0) > stamp)
            peer.request(op="shutdown")
            if process.wait(timeout=15) != 0:
                raise RuntimeError("Core did not shut down cleanly; inspect core.log")
            assert json.loads((output / "summary.json").read_text())["session_state"] == "closed"
        finally:
            if peer:
                peer.close()
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
            temporary.cleanup()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    core, project = args.core.resolve(), args.project_root.resolve()
    if args.output:
        args.output.mkdir(parents=True, exist_ok=False)
        for name, immediate in [("configuration", False), ("immediate", True)]:
            case = args.output.resolve() / name
            case.mkdir()
            run(core, project, case, immediate)
    else:
        with tempfile.TemporaryDirectory(prefix="srbt-lifecycle-") as temporary:
            for name, immediate in [("configuration", False), ("immediate", True)]:
                case = Path(temporary) / name
                case.mkdir()
                run(core, project, case, immediate)
    print("Session lifecycle: configuration, motion, pause, resume and shutdown passed")


if __name__ == "__main__":
    main()
