#!/usr/bin/env python3
"""Export a local Noetic C++ SDK for a Jammy host; no system installation."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
from project import ROOT

EXPORT = r'''
import io, os, pathlib, re, subprocess, sys, tarfile
code = '\n'.join('#include <'+p+'>' for p in [
 'ros/ros.h','ros/callback_queue.h','geometry_msgs/Twist.h',
 'geometry_msgs/TransformStamped.h',
 'nav_msgs/Odometry.h','rosgraph_msgs/Clock.h','sensor_msgs/CameraInfo.h',
 'sensor_msgs/Image.h','sensor_msgs/Imu.h','sensor_msgs/PointCloud2.h',
 'std_msgs/String.h','tf2_msgs/TFMessage.h'])
deps = subprocess.check_output(['g++','-std=c++17','-M','-x','c++','-',
 '-I/opt/ros/noetic/include','-I/opt/simple-rbt/ros1/include'], input=code.encode()).decode()
files = {}
for p in pathlib.Path('/opt/ros/noetic/include').rglob('*'):
 if p.is_file(): files['include/'+str(p.relative_to('/opt/ros/noetic/include'))]=p
for token in deps.replace('\\\n',' ').split():
 if token.startswith('/usr/include/boost/'):
  files['include/'+token[len('/usr/include/'):]]=pathlib.Path(token)
env = dict(os.environ, LD_LIBRARY_PATH='/opt/ros/noetic/lib')
libs = subprocess.check_output(['ldd','/opt/ros/noetic/lib/libroscpp.so'],env=env,text=True)
skip = ('libc.so','libm.so','libpthread.so','libdl.so','librt.so','libgcc_s.so','libstdc++.so','ld-linux')
for value in re.findall(r'=> (/\S+)', libs):
 p=pathlib.Path(value)
 if not p.name.startswith(skip): files['lib/'+p.name]=p
for p in pathlib.Path('/opt/ros/noetic/lib').glob('lib*.so'):
 if p.name in ('libroscpp.so','librosconsole.so','libroscpp_serialization.so','librostime.so','libxmlrpcpp.so','libcpp_common.so'):
  files['lib/'+p.name]=p
for p in pathlib.Path('/usr/share/doc').glob('*/copyright'):
 if p.parent.name.startswith(('libboost','ros-noetic-roscpp','ros-noetic-rosconsole','ros-noetic-rostime','liblog4cxx','libicu','libapr','libconsole-bridge')):
  files['licenses/'+p.parent.name+'.txt']=p
with tarfile.open(fileobj=sys.stdout.buffer,mode='w|',dereference=True) as tar:
 for name,p in sorted(files.items()): tar.add(p,arcname=name,recursive=False)
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", default="simple-rbt-ros1-test:local")
    parser.add_argument("--output", type=Path, default=ROOT / "build" / "ros1-sdk")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError("SDK output must be empty")
    process = subprocess.Popen(["docker", "run", "--rm", "--entrypoint", "python3",
                                args.image, "-c", EXPORT], stdout=subprocess.PIPE)
    licenses = {}
    try:
        with tarfile.open(fileobj=process.stdout, mode="r|") as archive:
            for member in archive:
                destination = (output / member.name).resolve()
                if not destination.is_relative_to(output) or not member.isfile():
                    raise ValueError("Unexpected SDK archive member")
                destination.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source:
                    content = source.read()
                digest = hashlib.sha256(content).hexdigest()
                if member.name.startswith("licenses/") and digest in licenses:
                    destination.hardlink_to(licenses[digest])
                else:
                    destination.write_bytes(content)
                    if member.name.startswith("licenses/"):
                        licenses[digest] = destination
        if process.wait():
            raise RuntimeError("Noetic SDK export failed")
    finally:
        if process.poll() is None:
            process.terminate(); process.wait()
    identity = subprocess.check_output(["docker", "image", "inspect", args.image,
                                       "--format", "{{.Id}}"], text=True).strip()
    # glibc >= 2.34 makes PTHREAD_STACK_MIN a runtime expression. Preserve the
    # original bounds check while removing Boost 1.71's preprocessor evaluation.
    header = output / "include/boost/thread/pthread/thread_data.hpp"
    original = header.read_text()
    assert original.count("#if PTHREAD_STACK_MIN > 0") == 1
    header.write_text(original.replace("#if PTHREAD_STACK_MIN > 0", "#ifdef PTHREAD_STACK_MIN"))
    (output / "source.json").write_text(json.dumps({"image": args.image, "id": identity,
        "header_patch": "Boost 1.71 PTHREAD_STACK_MIN check supports Jammy glibc"}, indent=2)+"\n")
    print(output)

if __name__ == "__main__":
    main()
