#!/usr/bin/env python3
"""Measure owned native console/core processes; optional isolated ROS containers."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
import uuid
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests/console"))
from native_x11 import Inputs
from observe_session import Observer, child_session, owned_workers


def process_stats(pid, container=None):
    directory = Path("/proc") / str(pid)
    fields = (directory / "stat").read_text().split(")", 1)[1].split()
    ticks = os.sysconf("SC_CLK_TCK")
    memory = (directory / "statm").read_text().split()
    rss = int(memory[1])*os.sysconf("SC_PAGESIZE")/1024**2
    try:
        rollup = (directory / "smaps_rollup").read_text().splitlines()
    except PermissionError:
        if not container:
            raise
        namespace_pid = next(line.split()[-1] for line in (directory / "status").read_text().splitlines()
                             if line.startswith("NSpid:"))
        rollup = subprocess.run(["docker", "exec", container, "cat",
                                 f"/proc/{int(namespace_pid)}/smaps_rollup"],
                                check=True, capture_output=True, text=True).stdout.splitlines()
    pss = next(float(line.split()[1])/1024 for line in rollup if line.startswith("Pss:"))
    return {"cpu_seconds": (int(fields[11])+int(fields[12]))/ticks, "rss_mib": rss, "pss_mib": pss}


def run(args):
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    config = {"robot": "ranger_mini", "terrain": "flat", "controller": "auto", "backend": args.backend,
              "camera_hz": 10, "sensors": {"mid360": True,"camera": True,"depth": True,"panorama": True}}
    (output / "config.json").write_text(json.dumps(config))
    env = dict(os.environ, DISPLAY=args.display, LC_ALL="C.UTF-8", XDG_CONFIG_HOME=str(output / "config"),
               XDG_STATE_HOME=str(output / "state"), XDG_CACHE_HOME=str(output / "cache"))
    containers, docker_clients, samples = [], [], []
    observer, inputs = None, None
    with (output / "console.log").open("w") as log:
        process = subprocess.Popen([str(ROOT / "build/desktop/simple-rbt-console"),
            "--worker", str(ROOT / "build/desktop/simple-rbt-sim"), "--assets", str(ROOT / "assets"),
            "--config", str(output / "config.json")], env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            directory = child_session(process.pid); observer = Observer(directory)
            observer.wait(lambda m,v:m.get("session_state")=="config")
            inputs = Inputs(args.display); window = inputs.window(); inputs.focus(window)
            if args.ros:
                for version in ("ros1", "ros2"):
                    name = "srbt-profile-"+version+"-"+uuid.uuid4().hex[:8]; containers.append(name)
                    options = ["docker", "run", "--rm", "--pull", "never", "--name", name, "--network", "host",
                               "--mount", "type=bind,source="+str(directory)+",target=/session"]
                    environment = {"ROS_MASTER_URI":"http://127.0.0.1:11391", "ROS_IP":"127.0.0.1", "SIMPLE_RBT_START_MASTER":"1"} if version=="ros1" else {"ROS_DOMAIN_ID":"101", "ROS_LOCALHOST_ONLY":"1"}
                    for key,value in environment.items(): options += ["-e",key+"="+value]
                    options += ["simple-rbt-ros1:noetic" if version=="ros1" else "simple-rbt-ros2:humble", "--session-dir", "/session"]
                    docker_clients.append(subprocess.Popen(options,stdout=log,stderr=subprocess.STDOUT))
                observer.wait(lambda m,v:all(m.get("connections",{}).get(x,0)>0 for x in ("ros1","ros2")), timeout=20)
            inputs.click(*inputs.primary_button(window))
            observer.wait(lambda m,v:m.get("session_state")=="running")
            time.sleep(2)
            # Approximately .30 m/s, through the actual joystick rather than IPC commands.
            inputs.move(180,248); inputs.button(True)
            observer.wait(lambda m,v:m.get("applied_command",[0])[0]>.15)
            pids = {"console": [process.pid],"core": [pid for pid,_ in owned_workers(process.pid)]}
            for version,name in zip(("ros1","ros2"),containers):
                result = subprocess.run(["docker","top",name,"-eo","pid"],check=True,capture_output=True,text=True)
                pids[version] = [int(line) for line in result.stdout.splitlines()[1:] if line.strip()]
            pid_containers = {pid:name for group,name in zip(("ros1","ros2"),containers) for pid in pids[group]}
            previous = {pid:process_stats(pid, pid_containers.get(pid)) for values in pids.values() for pid in values}
            previous_wall = time.monotonic(); start = previous_wall
            previous_stamp = observer.snapshot()[0]["stamp_ns"]
            while time.monotonic()-start < args.seconds:
                time.sleep(1)
                now = time.monotonic(); elapsed = now-previous_wall
                groups = {}
                for group,values in pids.items():
                    rows = []
                    for pid in values:
                        value = process_stats(pid, pid_containers.get(pid))
                        value["cpu_percent_one_core"] = 100*(value["cpu_seconds"]-previous[pid]["cpu_seconds"])/elapsed
                        previous[pid] = value.copy(); rows.append(value)
                    groups[group] = {key:sum(row[key] for row in rows) for key in ("cpu_percent_one_core","rss_mib","pss_mib")}
                metadata,_ = observer.snapshot()
                stamp = metadata["stamp_ns"]
                total = {key:sum(group[key] for group in groups.values()) for key in ("cpu_percent_one_core","rss_mib","pss_mib")}
                samples.append({"elapsed_s":now-start,"groups":groups,"total":total,"stamp_ns":stamp})
                previous_wall, previous_stamp = now, stamp
            inputs.button(False)
            observer.wait(lambda m,v:m.get("applied_command")==[0,0,0] and math.hypot(v[0],v[1])<.1)
            inputs.capture(window,output / "native-window.png")
            report = {"conditions":{"robot":"ranger_mini","terrain":"flat","backend":args.backend,"sensor_outputs":["MID360 20000 rays/10Hz","IMU200Hz","RGB/Z-depth10Hz","panorama RGB/depth10Hz"],"gui":"native MyGo/Xvfb software GL; live first+third views","ros":args.ros,"cpu_unit":"100%=one logical CPU","memory":"summed process RSS and PSS; tool/X server excluded","external_ros_receivers":False},
                      "pids":pids,"samples":samples,"container_pss_reader":"docker exec cat /proc/<namespace-pid>/smaps_rollup" if containers else None,
                      "mean_cpu_percent_one_core":statistics.mean(x["total"]["cpu_percent_one_core"] for x in samples),
                      "peak_rss_mib":max(x["total"]["rss_mib"] for x in samples),
                      "peak_pss_mib":max(x["total"]["pss_mib"] for x in samples),
                      "real_time_factor":(samples[-1]["stamp_ns"]-samples[0]["stamp_ns"])/1e9/(samples[-1]["elapsed_s"]-samples[0]["elapsed_s"]),
                      "executable_sha256":{name:hashlib.sha256((ROOT/"build/desktop"/name).read_bytes()).hexdigest() for name in ("simple-rbt-console","simple-rbt-sim")}}
            (output / "report.json").write_text(json.dumps(report,indent=2)+"\n")
            print(json.dumps({key:value for key,value in report.items() if key not in ("pids","samples")},indent=2))
        finally:
            if inputs: inputs.button(False); inputs.close()
            if observer: observer.close()
            for name in containers:
                subprocess.run(["docker","stop","--time","3",name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            for client in docker_clients:
                try: client.wait(timeout=5)
                except subprocess.TimeoutExpired: client.terminate(); client.wait()
            process.terminate()
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired: process.kill(); process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--display",required=True)
    parser.add_argument("--seconds",type=float,default=20)
    parser.add_argument("--ros",action="store_true")
    parser.add_argument("--backend", choices=("cpu", "auto", "cuda"), default="auto")
    parser.add_argument("--output",type=Path,required=True)
    args = parser.parse_args()
    if not 5<=args.seconds<=120: parser.error("seconds must be5..120")
    run(args)


if __name__=="__main__":main()
