#!/usr/bin/env python3
"""Official ROS containers connected directly to the standalone C++ simulation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import time
import uuid
ROOT = Path(__file__).resolve().parents[1]


def stop(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()


def build_image(version, image, base_image=None):
    context = ROOT / "build" / "ros-adapters" / "context"
    context.mkdir(parents=True, exist_ok=True)
    for directory in ["deployment/"+version, "include", "ipc", "wrappers",
                      "third_party/nlohmann", "interfaces/livox_ros_driver2"]:
        shutil.copytree(ROOT / directory, context / directory, dirs_exist_ok=True)
    (context / "src").mkdir(exist_ok=True)
    shutil.copy2(ROOT / "src/robots.cpp", context / "src/robots.cpp")
    command = ["docker", "build", "--pull=false", "-f", str(context / "deployment" / version / "Dockerfile"),
               "--tag", image]
    if base_image:
        command += ["--build-arg", "ROS_IMAGE="+base_image]
    subprocess.run(command + [str(context)], check=True)


def run_case(args, robot, output):
    case = output / robot; case.mkdir()
    session = Path("/tmp") / ("srbt-"+uuid.uuid4().hex[:12]); session.mkdir(mode=0o700)
    container = "simple-rbt-"+args.ros+"-"+uuid.uuid4().hex[:12]
    env = {"ROS_MASTER_URI": "http://127.0.0.1:"+str(args.master_port), "ROS_IP": "127.0.0.1",
           "SIMPLE_RBT_START_MASTER": "1"} if args.ros == "ros1" else {
           "ROS_DOMAIN_ID": str(args.domain_id), "ROS_LOCALHOST_ONLY": "1"}
    config = {"robot": robot, "terrain": args.terrain, "backend": args.backend,
              "control_mode": args.control_mode, "source": args.ros, "start_immediately": True,
              "duration_s": 20, "settle_s": 2, "camera_hz": 10,
              "sensors": {"mid360": True, "camera": not args.no_camera, "depth": not args.no_camera, "panorama": not args.no_camera},
              "output": str(case / "native")}
    (case / "config.json").write_text(json.dumps(config, indent=2)+"\n")
    core = adapter = probe = None
    with (case / "core.log").open("w") as core_log, (case / "adapter.log").open("w") as adapter_log:
        try:
            environment = dict(os.environ)
            if args.hide_gpu:
                environment.update(CUDA_VISIBLE_DEVICES="", NVIDIA_VISIBLE_DEVICES="void")
            core_sha = hashlib.sha256(args.core.read_bytes()).hexdigest()
            core = subprocess.Popen([str(args.core), "--session-config", str(case / "config.json"),
                                     "--session-dir", str(session)], stdout=core_log, stderr=subprocess.STDOUT, env=environment)
            command = ["docker", "run", "--rm", "--pull", "never", "--network", "host", "--name", container]
            for key, value in env.items(): command += ["--env", key+"="+value]
            command += ["--mount", f"type=bind,source={session},target=/session",
                        "--mount", f"type=bind,source={ROOT / 'tests/ipc'},target=/tests,readonly",
                        "--mount", f"type=bind,source={case},target=/results", args.image,
                        "--session-dir", "/session"]
            adapter = subprocess.Popen(command, stdout=adapter_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+15
            while time.monotonic()<deadline:
                if adapter.poll() is not None: raise RuntimeError("Adapter exited; see "+str(case))
                ready = subprocess.run(["docker", "exec", container, "test", "-S", "/session/control.sock"],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
                if ready: break
                time.sleep(.1)
            setup = "/opt/ros/"+("noetic" if args.ros == "ros1" else "humble")+"/setup.bash"
            interface = "/opt/simple-rbt/interfaces/"+("setup.bash" if args.ros == "ros1" else "share/livox_ros_driver2/local_setup.bash")
            shell = f'source {setup}; source {interface}; exec python3 /tests/ros_probe.py "$1" /results/probe.json --physical --robot "$2"'
            if args.no_camera: shell += ' --no-camera'
            probe = subprocess.Popen(["docker", "exec", container, "bash", "-c", shell, "probe", args.ros, robot],
                                     stdout=adapter_log, stderr=subprocess.STDOUT)
            probe_code = probe.wait(timeout=35)
            core_code = core.wait(timeout=40)
            adapter_code = adapter.wait(timeout=15)
            if probe_code or core_code or adapter_code:
                raise RuntimeError(f"{robot}: probe={probe_code}, core={core_code}, adapter={adapter_code}; see {case}")
        finally:
            stop(probe); stop(core)
            subprocess.run(["docker", "rm", "--force", container], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            stop(adapter)
            for socket_path in session.glob("*.sock"): socket_path.unlink()
            session.rmdir()
    result = json.loads((case / "probe.json").read_text())
    result["robot"] = robot
    summary = json.loads((case / "native/summary.json").read_text())
    result["real_time_factor"] = summary["real_time_factor"]
    result["resources_loop"] = summary["resources_loop"]
    result["controller"] = summary["controller"]
    result["deployment"] = summary["deployment"]
    result["backend"] = summary["backend"]
    result["core_sha256"] = core_sha
    result["gpu_hidden"] = args.hide_gpu
    result["camera_enabled"] = not args.no_camera
    (case / "report.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({"robot": robot, "ros": args.ros, "passed": result["passed"], "checks": len(result["checks"])}), flush=True)
    return result


def main():
    catalog = json.loads((ROOT / "assets/catalog.json").read_text())
    all_robots = sorted({key.split("-", 1)[0] for key in catalog["models"]})
    robots = sorted({key.split("-", 1)[0] for key, value in catalog["models"].items() if not value.get("legacy", False)})
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros", choices=["ros1", "ros2"], default="ros2")
    parser.add_argument("--image")
    parser.add_argument("--base-image", help="Optional public base override; production Dockerfile pins the official image")
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--robots", nargs="+", choices=all_robots, default=robots,
                        help="Default: seven production robots; a legacy alias may be requested explicitly")
    parser.add_argument("--backend", choices=["auto", "cpu", "cuda"], default="auto")
    parser.add_argument("--no-camera", action="store_true", help="Validate motion and MID360 without an image renderer")
    parser.add_argument("--hide-gpu", action="store_true", help="Hide CUDA devices for the focused CPU/no-camera acceptance")
    parser.add_argument("--control-mode", choices=["native", "tracked"], default="tracked")
    parser.add_argument("--terrain", choices=["flat", "hills", "ramp", "rough"], default="flat")
    parser.add_argument("--core", type=Path, default=ROOT / "build/desktop/simple-rbt-sim")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--master-port", type=int, default=11387)
    parser.add_argument("--domain-id", type=int, default=98)
    args = parser.parse_args()
    args.image = args.image or ("simple-rbt-ros1:noetic" if args.ros == "ros1" else "simple-rbt-ros2:humble")
    if not 1024 <= args.master_port <= 65535: parser.error("master-port must be 1024..65535")
    if not 0 <= args.domain_id <= 232: parser.error("domain-id must be 0..232")
    if args.hide_gpu and (args.backend != "cpu" or not args.no_camera):
        parser.error("--hide-gpu requires --backend cpu --no-camera")
    if not args.skip_build: build_image(args.ros, args.image, args.base_image)
    if args.build_only: return
    if not args.core.exists(): parser.error("standalone core missing; build the desktop target first")
    if args.ros == "ros1":
        with socket.socket() as check: check.bind(("127.0.0.1", args.master_port))
    output = (args.output or ROOT / "output" / ("ipc-"+args.ros+"-"+uuid.uuid4().hex[:12])).resolve()
    output.mkdir(parents=True, exist_ok=False)
    results = [run_case(args, robot, output) for robot in args.robots]
    report = {"passed": all(r["passed"] for r in results), "ros": args.ros, "cases": results,
              "core_sha256": hashlib.sha256(args.core.read_bytes()).hexdigest(), "image": args.image}
    (output / "summary.json").write_text(json.dumps(report, indent=2)+"\n")
    if not report["passed"]: raise RuntimeError("ROS IPC acceptance failed; see "+str(output))

if __name__ == "__main__": main()
