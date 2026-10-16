#!/usr/bin/env python3
"""Measure startup, scene editing and Go2W/Noetic runtime with real ROS receivers."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests/console'))
from complex_scene import building
from native_x11 import Inputs
from observe_session import Observer,child_session
from process_resources import Sampler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--display',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=60)
    parser.add_argument('--flat',action='store_true')
    parser.add_argument('--software',action='store_true')
    parser.add_argument('--image',default='simple-rbt-ros1:noetic')
    args=parser.parse_args()
    if not 15<=args.seconds<=120:
        parser.error('seconds must be 15..120')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    scene=None if args.flat else building()
    config=dict(robot='go2w',controller='rl_sar',backend='cpu',camera_model='d435i',camera_hz=10,
                sensors=dict(mid360=True,camera=True,depth=True,panorama=True))
    if scene:
        config['scene']=scene
    (out/'config.json').write_text(json.dumps(config,indent=2))
    env=dict(os.environ,DISPLAY=args.display,LC_ALL='C.UTF-8',XDG_CONFIG_HOME=str(out/'config'),
             XDG_STATE_HOME=str(out/'state'),XDG_CACHE_HOME=str(out/'cache'))
    if args.software:
        env.update(LIBGL_ALWAYS_SOFTWARE='1',EGL_PLATFORM='surfaceless',__EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/50_mesa.json')
    core=ROOT/'build/desktop/simple-rbt-sim';console=ROOT/'build/desktop/simple-rbt-console'
    container='srbt-resource-'+uuid.uuid4().hex[:10]
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    observer=inputs=sampler=process=adapter=receiver=None
    report={}
    with (out/'console.log').open('w') as log,(out/'ros.log').open('w') as roslog:
        try:
            process=subprocess.Popen([str(console),'--worker',str(core),'--assets',str(ROOT/'assets'),'--config',str(out/'config.json')],env=env,stdout=log,stderr=subprocess.STDOUT)
            sampler=Sampler(process.pid)
            directory=child_session(process.pid);observer=Observer(directory)
            observer.wait(lambda m,v:m.get('session_state')=='config',timeout=30)
            inputs=Inputs(args.display);window=inputs.window();inputs.focus(window)
            if scene:
                sampler.phase='editor'
                inputs.click(260,90);time.sleep(.5)
                inputs.move(650,500);inputs.button(True,button=3)
                for i in range(80):
                    inputs.move(650+int(60*math.sin(i*.1)),500+int(30*math.cos(i*.1)));time.sleep(.12)
                inputs.button(False,button=3)
                inputs.capture(window,out/'editor.png')
                sampler.phase='editor-idle'
                time.sleep(5)
                inputs.click(50,90)
            sampler.phase='ros-startup'
            command=['docker','run','--rm','--pull','never','--network','host','--name',container,
                     '-e',f'ROS_MASTER_URI=http://127.0.0.1:{port}','-e','ROS_IP=127.0.0.1','-e','SIMPLE_RBT_START_MASTER=1',
                     '--mount',f'type=bind,source={directory},target=/session',
                     '--mount',f'type=bind,source={ROOT/"tests/console"},target=/tests,readonly',
                     '--mount',f'type=bind,source={out},target=/results',args.image,'--session-dir','/session']
            adapter=subprocess.Popen(command,stdout=roslog,stderr=subprocess.STDOUT)
            sampler.container=container;sampler.client=adapter.pid
            observer.wait(lambda m,v:m.get('connections',{}).get('ros1',0)>0,timeout=30)
            shell='source /opt/ros/noetic/setup.bash; exec python3 /tests/ros_resource_sink.py "$1" /results/receiver.json'
            receiver=subprocess.Popen(['docker','exec',container,'bash','-c',shell,'receiver',str(args.seconds+5)],stdout=roslog,stderr=subprocess.STDOUT)
            inputs.click(*inputs.primary_button(window))
            observer.wait(lambda m,v:m.get('session_state')=='running',timeout=20)
            time.sleep(2)
            inputs.move(180,296);inputs.button(True)
            observer.wait(lambda m,v:m.get('applied_command',[0])[0]>.15)
            sampler.phase='runtime'
            before=observer.snapshot()[0];began=time.monotonic()
            while time.monotonic()-began<args.seconds:
                if process.poll() is not None or adapter.poll() is not None:
                    raise RuntimeError('A measured process exited unexpectedly')
                time.sleep(.25)
            elapsed=time.monotonic()-began;after=observer.snapshot()[0]
            inputs.button(False)
            observer.wait(lambda m,v:m.get('applied_command')==[0,0,0])
            sampler.stop()
            receiver.wait(timeout=15)
            sink=json.loads((out/'receiver.json').read_text())
            if any(value['messages']<5 for value in sink['topics'].values()):
                raise RuntimeError('Expected ROS sensor outputs missing: '+repr(sink))
            inputs.capture(window,out/'runtime.png')
            report=dict(passed=True,conditions=dict(robot='go2w',floors=len(scene['floors']) if scene else 1,
                authored_objects=len(scene['objects']) if scene else 5,scene='complex' if scene else 'flat',
                lidar='CPU, 20000 points at 10 Hz',sensors='MID360 IMU, D435i IMU, RGB, depth, panorama; images at 10 Hz',
                renderer='software Mesa' if args.software else 'host default EGL',duration_s=elapsed,
                cpu_unit='100% = one logical CPU; sampled peaks, not worst-case guarantees',
                memory='Sum RSS and proportional PSS of owned GUI/core/ROS processes. Receiver separate; X server, sampler, Docker daemon and GPU memory excluded.'),
                summary=sampler.summary(),samples=sampler.samples,receiver=sink,
                real_time_factor=(after['stamp_ns']-before['stamp_ns'])/1e9/elapsed,
                displacement_m=math.dist(before['base_pose']['position'],after['base_pose']['position']),
                executable_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (core,console)},
                image=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',args.image],text=True).strip())
        except Exception as error:
            report=dict(passed=False,error=str(error),samples=sampler.samples if sampler else [])
            raise
        finally:
            if sampler and sampler.thread.is_alive():
                try:
                    sampler.stop()
                except RuntimeError as error:
                    report['sampling_error']=str(error)
                    report['passed']=False
            if inputs:
                inputs.button(False)
            if observer:
                observer.close()
            subprocess.run(['docker','stop','--time','3',container],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            for child in (receiver,adapter):
                if child:
                    try:child.wait(timeout=8)
                    except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
            if process and process.poll() is None:
                if inputs:
                    inputs.close_window(window)
                else:
                    process.terminate()
                process.wait(timeout=10)
            if inputs:
                inputs.close()
            (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key not in ('samples','receiver')},indent=2))


if __name__=='__main__':
    main()
