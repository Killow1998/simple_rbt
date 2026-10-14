#!/usr/bin/env python3
"""Direct Noetic / C++ core end-to-end test; requires host execution."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
import uuid
import xmlrpc.client

from project import ROOT
sys.path.insert(0, str(ROOT / "tests" / "ros1"))
from artifacts import compare_payloads

TESTS = ROOT / "tests" / "ros1"
IMAGE = "simple-rbt-ros1-test:local"


def build_image(base_image, image):
    context = ROOT / "build" / "ros1-test" / "context"
    context.mkdir(parents=True, exist_ok=True)
    shutil.copy2(TESTS / "Dockerfile", context / "Dockerfile")
    messages = ROOT / "interfaces" / "livox_ros_driver2"
    shutil.copytree(messages, context / "messages", dirs_exist_ok=True)
    subprocess.run(["docker", "build", "--pull=false", "--build-arg",
                    "NOETIC_IMAGE=" + base_image, "--tag", image, str(context)], check=True)


def stop(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()


def run_case(container, robot, output, env, backend, control_mode):
    case = output / robot
    case.mkdir()
    runtime = client = oracle = None
    with (case / "ros1.log").open("w") as client_log, (case / "native.log").open("w") as runtime_log:
        try:
            oracle_command = ["docker", "exec", container, "/bin/bash", "-c",
                "source /opt/ros/noetic/setup.bash; exec python3 /tests/images.py /results/" + robot]
            oracle = subprocess.Popen(oracle_command, cwd=ROOT, env=env,
                                      stdout=client_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 10
            while not (case / "images-ready").exists() and time.monotonic() < deadline:
                if oracle.poll() is not None:
                    raise RuntimeError("Noetic image receiver failed; see " + str(case))
                time.sleep(.05)
            if not (case / "images-ready").exists():
                raise RuntimeError("Noetic image receiver did not become ready")
            shell = ('source /opt/ros/noetic/setup.bash; source /opt/simple-rbt/ros1/setup.bash; '
                     'exec python3 /tests/probe.py "$1" "/results/$1"')
            client = subprocess.Popen(["docker", "exec", container, "/bin/bash", "--noprofile", "--norc",
                                       "-c", shell, "probe", robot], stdout=client_log, stderr=subprocess.STDOUT)
            runtime = subprocess.Popen([str(ROOT / "run.sh"), "--robot", robot, "--ros1",
                                        "--terrain", "flat", "--sensors", "full", "--backend", backend,
                                        "--camera-hz", "10", "--control-mode", control_mode, "--settle", "2", "--duration", "20",
                                        "--output", str(case / "native")], cwd=ROOT, env=env,
                                       stdout=runtime_log, stderr=subprocess.STDOUT)
            code = runtime.wait(timeout=45)
            (case / "stop").touch()
            client_code = client.wait(timeout=15)
            oracle_code = oracle.wait(timeout=10)
            if code or client_code or oracle_code:
                raise RuntimeError(f"{robot}: runtime={code}, Noetic={client_code}, oracle={oracle_code}; see {case}")
        finally:
            stop(runtime); stop(client); stop(oracle)
    report = json.loads((case / "ros1.json").read_text())
    checks, summary = compare_payloads(case / "native", report, json.loads((case / "images.json").read_text()))
    checks.update(report["checks"])
    result = {"robot": robot, "passed": all(checks.values()), "checks": checks,
              "failed_checks": [name for name, ok in checks.items() if not ok],
              "rtf": summary["real_time_factor"], "resources_loop": summary["resources_loop"],
              "received": report["received"]}
    (case / "report.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"robot": robot, "passed": result["passed"], "checks": len(checks),
                      "failed_checks": result["failed_checks"]}), flush=True)
    return result


def main():
    catalog = json.loads((ROOT / "assets" / "catalog.json").read_text())
    robots = sorted({key.split("-", 1)[0] for key in catalog["models"]})
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-image", default="ros:noetic-ros-base")
    parser.add_argument("--image", default=IMAGE)
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--robots", nargs="+", choices=robots, default=robots)
    parser.add_argument("--backend", choices=["cpu", "cuda"], default="cuda")
    parser.add_argument("--control-mode", choices=["native", "tracked"], default="native")
    parser.add_argument("--master-port", type=int, default=11383)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1024 <= args.master_port <= 65535:
        parser.error("master-port must be 1024..65535")
    if not args.skip_build:
        build_image(args.base_image, args.image)
    if args.build_only:
        return
    output = (args.output or ROOT / "output" / ("noetic-" + uuid.uuid4().hex[:12])).resolve()
    master = "http://127.0.0.1:" + str(args.master_port)
    env = dict(os.environ, ROS_MASTER_URI=master, ROS_IP="127.0.0.1")
    env["SIMPLE_RBT_BUILD_DIR"] = str(ROOT / "build" / "ros1")
    with socket.socket() as check:
        check.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        check.bind(("127.0.0.1", args.master_port))
    output.mkdir(parents=True, exist_ok=False)
    container = "simple-rbt-noetic-" + uuid.uuid4().hex[:12]
    files = [Path(__file__), *sorted(TESTS.iterdir())]
    fingerprint = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in files if p.is_file()}
    results = []
    started = False
    try:
        subprocess.run(["docker", "run", "--detach", "--rm", "--pull", "never", "--network", "host",
                        "--entrypoint", "/bin/bash",
                        "--ipc", "host", "--user", f"{os.getuid()}:{os.getgid()}", "--name", container,
                        "--env", "ROS_MASTER_URI=" + master, "--env", "ROS_IP=127.0.0.1",
                        "--env", "ROS_HOME=/tmp/simple-rbt-ros1", "--env", "ROS_LOG_DIR=/tmp/simple-rbt-ros1-log",
                        "--env", "SIMPLE_RBT_MASTER_PORT=" + str(args.master_port),
                        "--mount", "type=bind,src=" + str(TESTS) + ",dst=/tests,readonly",
                        "--mount", "type=bind,src=" + str(output) + ",dst=/results", args.image,
                        "/tests/entrypoint.sh", "sleep", "infinity"], check=True)
        started = True
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            try:
                if xmlrpc.client.ServerProxy(master).getPid("/simple_rbt_noetic_test")[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(.1)
        else:
            raise RuntimeError("Noetic master failed to start")
        for robot in args.robots:
            result = run_case(container, robot, output, env, args.backend, args.control_mode)
            results.append(result)
            if not result["passed"]:
                raise RuntimeError(robot + ": " + str(result["failed_checks"]))
    finally:
        if started:
            with (output / "roscore.log").open("w") as log:
                subprocess.run(["docker", "exec", container, "cat", "/tmp/roscore.log"], stdout=log, stderr=subprocess.STDOUT)
            subprocess.run(["docker", "rm", "--force", container], stdout=subprocess.DEVNULL, check=True)
        unchanged = all(hashlib.sha256((ROOT / p).read_bytes()).hexdigest() == value for p, value in fingerprint.items())
        report = {"passed": len(results) == len(args.robots) and all(r["passed"] for r in results) and unchanged,
                  "cases": results, "test_sources_unchanged": unchanged, "test_sources": fingerprint,
                  "transport": "ros1",
                  "native_sha256": hashlib.sha256((Path(env["SIMPLE_RBT_BUILD_DIR"]) / "simple-rbt-native").read_bytes()).hexdigest()}
        (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    if not report["passed"]:
        raise RuntimeError("Noetic test incomplete or test sources changed; see " + str(output))
    print("Report: " + str(output / "summary.json"))


if __name__ == "__main__":
    main()
