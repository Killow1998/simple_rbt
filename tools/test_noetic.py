#!/usr/bin/env python3
"""Optional Noetic Docker / Humble C++ end-to-end test; requires host execution."""
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
sys.path.insert(0, str(ROOT / "tests" / "noetic"))
from artifacts import compare_payloads

TESTS = ROOT / "tests" / "noetic"
IMAGE = "simple-rbt-noetic-test:local"


def build_image(base_image, image):
    context = ROOT / "build" / "noetic" / "context"
    context.mkdir(parents=True, exist_ok=True)
    for name in ["Dockerfile", "entrypoint.sh", "tf_static.patch"]:
        shutil.copy2(TESTS / name, context / name)
    messages = ROOT / "ros2_interfaces" / "livox_ros_driver2"
    shutil.copytree(messages, context / "ros2", dirs_exist_ok=True)
    shutil.copy2(TESTS / "ros1_mapping_rules.yaml", context / "ros2" / "ros1_mapping_rules.yaml")
    cmake = context / "ros2" / "CMakeLists.txt"
    cmake.write_text(cmake.read_text() + "\ninstall(FILES ros1_mapping_rules.yaml DESTINATION share/${PROJECT_NAME})\n")
    package = context / "ros2" / "package.xml"
    package.write_text(package.read_text().replace("</export>",
                       '<ros1_bridge mapping_rules="ros1_mapping_rules.yaml"/></export>'))
    ros1 = context / "ros1"
    (ros1 / "msg").mkdir(parents=True, exist_ok=True)
    for name in ["CustomMsg.msg", "CustomPoint.msg"]:
        shutil.copy2(messages / "msg" / name, ros1 / "msg" / name)
    shutil.copy2(messages / "LICENSE.txt", ros1 / "LICENSE.txt")
    (ros1 / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.0.2)\nproject(livox_ros_driver2)\n"
        "find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)\n"
        "add_message_files(FILES CustomMsg.msg CustomPoint.msg)\n"
        "generate_messages(DEPENDENCIES std_msgs)\n"
        "catkin_package(CATKIN_DEPENDS message_runtime std_msgs)\n"
        "install(FILES LICENSE.txt DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})\n")
    (ros1 / "package.xml").write_text(
        '<?xml version="1.0"?>\n<package format="2"><name>livox_ros_driver2</name>'
        '<version>1.0.0</version><description>Original Livox messages for Noetic tests.</description>'
        '<maintainer email="devnull@example.com">simple_rbt</maintainer><license>MIT</license>'
        '<buildtool_depend>catkin</buildtool_depend><build_depend>message_generation</build_depend>'
        '<depend>std_msgs</depend><exec_depend>message_runtime</exec_depend></package>\n')
    ca = os.environ.get("CODEX_PROXY_CERT", "/etc/ssl/certs/ca-certificates.crt")
    subprocess.run(["docker", "build", "--pull=false", "--network", "host",
                    "--build-arg", "HTTP_PROXY", "--build-arg", "HTTPS_PROXY",
                    "--build-arg", "NOETIC_IMAGE=" + base_image,
                    "--secret", "id=session_ca,src=" + ca, "--tag", image, str(context)], check=True)


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
            oracle_shell = 'source tools/ros_env.sh; exec .venv/bin/python tests/noetic/ros2_images.py "$1"'
            oracle = subprocess.Popen(["/bin/bash", "--noprofile", "--norc", "-c", oracle_shell,
                                       "oracle", str(case)], cwd=ROOT, env=env,
                                      stdout=client_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 10
            while not (case / "ros2-ready").exists() and time.monotonic() < deadline:
                if oracle.poll() is not None:
                    raise RuntimeError("Humble image receiver failed; see " + str(case))
                time.sleep(.05)
            if not (case / "ros2-ready").exists():
                raise RuntimeError("Humble image receiver did not become ready")
            shell = ('source /opt/ros/noetic/setup.bash; source /opt/simple-rbt/ros1/setup.bash; '
                     'exec python3 /tests/probe.py "$1" "/results/$1"')
            client = subprocess.Popen(["docker", "exec", container, "/bin/bash", "--noprofile", "--norc",
                                       "-c", shell, "probe", robot], stdout=client_log, stderr=subprocess.STDOUT)
            runtime = subprocess.Popen([str(ROOT / "run.sh"), "--robot", robot, "--ros2",
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
    checks, summary = compare_payloads(case / "native", report, json.loads((case / "ros2-images.json").read_text()))
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
    parser.add_argument("--domain", type=int, default=90)
    parser.add_argument("--master-port", type=int, default=11383)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not 1024 <= args.master_port <= 65535:
        parser.error("domain must be 0..232 and master-port must be 1024..65535")
    if not args.skip_build:
        build_image(args.base_image, args.image)
    if args.build_only:
        return
    output = (args.output or ROOT / "output" / ("noetic-" + uuid.uuid4().hex[:12])).resolve()
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1")
    master = "http://127.0.0.1:" + str(args.master_port)
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
                        "--ipc", "host", "--user", f"{os.getuid()}:{os.getgid()}", "--name", container,
                        "--env", "ROS_MASTER_URI=" + master, "--env", "ROS_IP=127.0.0.1",
                        "--env", "ROS_HOME=/tmp/simple-rbt-ros1", "--env", "ROS_LOG_DIR=/tmp/simple-rbt-ros2-log",
                        "--env", "SIMPLE_RBT_MASTER_PORT=" + str(args.master_port),
                        "--env", "ROS_DOMAIN_ID=" + str(args.domain), "--env", "ROS_LOCALHOST_ONLY=1",
                        "--mount", "type=bind,src=" + str(TESTS) + ",dst=/tests,readonly",
                        "--mount", "type=bind,src=" + str(output) + ",dst=/results", args.image], check=True)
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
            with (output / "bridge.log").open("w") as log:
                subprocess.run(["docker", "exec", container, "cat", "/tmp/bridge.log"], stdout=log, stderr=subprocess.STDOUT)
            subprocess.run(["docker", "rm", "--force", container], stdout=subprocess.DEVNULL, check=True)
        unchanged = all(hashlib.sha256((ROOT / p).read_bytes()).hexdigest() == value for p, value in fingerprint.items())
        report = {"passed": len(results) == len(args.robots) and all(r["passed"] for r in results) and unchanged,
                  "cases": results, "test_sources_unchanged": unchanged, "test_sources": fingerprint,
                  "native_sha256": hashlib.sha256((ROOT / "build/native/simple-rbt-native").read_bytes()).hexdigest()}
        (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    if not report["passed"]:
        raise RuntimeError("Noetic test incomplete or test sources changed; see " + str(output))
    print("Report: " + str(output / "summary.json"))


if __name__ == "__main__":
    main()
