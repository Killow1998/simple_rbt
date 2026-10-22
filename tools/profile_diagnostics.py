#!/usr/bin/env python3
"""Compare daily/trace logging with GUI-managed Noetic and real sensor receivers."""
import argparse,json,math,os,socket,subprocess,sys,time,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests/console'))
from native_x11 import Inputs
from diagnostic_image import image_path
from observe_session import Observer,child_session
from process_resources import Sampler
from complex_scene import building


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--display',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--image',required=True);p.add_argument('--trace',action='store_true');p.add_argument('--seconds',type=float,default=60);args=p.parse_args()
 if not 15<=args.seconds<=120:p.error('seconds must be 15..120')
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
 config=dict(robot='go2w',controller='rl_sar',backend='cpu',camera_model='d435i',camera_hz=10,sensors=dict(mid360=True,camera=True,depth=True,panorama=True),scene=building())
 (out/'config.json').write_text(json.dumps(config))
 env=dict(os.environ,DISPLAY=args.display,LC_ALL='C.UTF-8',PATH=image_path(out,'ros1',args.image),XDG_CONFIG_HOME=str(out/'config'),XDG_STATE_HOME=str(out/'state'),XDG_CACHE_HOME=str(out/'cache'),SIMPLE_RBT_TRACE='1' if args.trace else '0',ROS_MASTER_URI=f'http://127.0.0.1:{port}',ROS_IP='127.0.0.1',SIMPLE_RBT_START_MASTER='1')
 process=observer=sampler=receiver=inputs=None;window=container=None;report={}
 try:
  with (out/'console.log').open('w') as log:
   process=subprocess.Popen([str(ROOT/'build/desktop/simple-rbt-console'),'--worker',str(ROOT/'build/desktop/simple-rbt-sim'),'--assets',str(ROOT/'assets'),'--config',str(out/'config.json'),'--ros','ros1','--operate'],env=env,stdout=log,stderr=log)
   sampler=Sampler(process.pid);directory=child_session(process.pid);observer=Observer(directory)
   observer.wait(lambda m,v:m.get('session_state')=='running' and m.get('connections',{}).get('ros1',0)>0,timeout=40)
   container='simple-rbt-ros1-'+directory.name;sampler.container=container
   for task in Path(f'/proc/{process.pid}/task').glob('*/children'):
    for pid in task.read_text().split():
     try:cmd=Path('/proc',pid,'cmdline').read_text()
     except FileNotFoundError:continue
     if container in cmd:sampler.client=int(pid)
   subprocess.run(['docker','cp',str(ROOT/'tests/console/ros_resource_sink.py'),container+':/tmp/ros_resource_sink.py'],check=True,stdout=subprocess.DEVNULL)
   shell='source /opt/ros/noetic/setup.bash; exec python3 /tmp/ros_resource_sink.py "$1" /tmp/receiver.json'
   receiver=subprocess.Popen(['docker','exec',container,'bash','-c',shell,'receiver',str(args.seconds+5)],stdout=log,stderr=log)
   inputs=Inputs(args.display);window=inputs.window();inputs.focus(window);time.sleep(1)
   inputs.move(180,296);inputs.button(True);observer.wait(lambda m,v:m.get('applied_command',[0])[0]>.15)
   sampler.phase='runtime';before=observer.snapshot()[0];start=time.monotonic()
   while time.monotonic()-start<args.seconds:
    if process.poll() is not None:raise RuntimeError('GUI exited')
    time.sleep(.2)
   elapsed=time.monotonic()-start;after=observer.snapshot()[0];inputs.button(False);sampler.stop();receiver.wait(timeout=15)
   subprocess.run(['docker','cp',container+':/tmp/receiver.json',str(out/'receiver.json')],check=True,stdout=subprocess.DEVNULL)
   sink=json.loads((out/'receiver.json').read_text())
   if any(v['messages']<5 for v in sink['topics'].values()):raise RuntimeError('Missing sensor traffic')
   inputs.capture(window,out/'runtime.png')
   inputs.close_window(window);process.wait(timeout=15);window=None
   logs=out/'state/simple-rbt/logs';events=0;health=[];stopped=set();counts={};summary_bytes=0;raw_high_rate=0
   for f in logs.glob('srbt-*.jsonl'):
    for line in f.read_text().splitlines():
     event=json.loads(line);events+=1
     if event['event']=='logging.dropped':health.append(event)
     if event['event']=='process.stopped':stopped.add(event['component'])
     fields=event.get('fields') or {}
     if event['event']=='stream.summary':
      summary_bytes+=len(line.encode())+1
      key='/'.join((event['component'],fields['stream'],fields['stage']));counts[key]=counts.get(key,0)+fields['count']
     elif str(fields.get('sample','')).split(':')[0] in ('clock','imu','camera_imu'):raw_high_rate+=1
   summarized=args.trace and env.get('SIMPLE_RBT_TRACE_RAW_HIGH_RATE')!='1'
   checks={'shutdown_drained':{'gui','core'}<=stopped,'no_log_drops':not health}
   if summarized:
    checks['high_rate_summarized']=raw_high_rate==0 and bool(counts)
    for stream in ('clock','imu','camera_imu'):
     offered=counts.get('core/'+stream+'/ipc.offered',0);received=counts.get('ros1/'+stream+'/ipc.received',0);published=counts.get('ros1/'+stream+'/ros.publish_called',0)
     checks[stream+'_pipeline_counts']=offered>=received==published>0
   report=dict(passed=all(checks.values()),checks=checks,trace=args.trace,duration_s=elapsed,scenario='Go2W, four floors / 242 authored objects; CPU LiDAR 20000 points at 10 Hz; MID360 + D435i IMU, RGB/depth/panorama 10 Hz; 10 actual ROS subscribers; host EGL, GUI Xvfb software GL',
     real_time_factor=(after['stamp_ns']-before['stamp_ns'])/1e9/elapsed,summary=sampler.summary(),samples=sampler.samples,receiver=sink,
     logging=dict(bytes=sum(f.stat().st_size for f in logs.glob('srbt-*.jsonl')),retained_events=events,core_health=after['logging'],drop_markers=health,high_rate_summary_bytes=summary_bytes,pipeline_counts=counts),
     displacement_m=math.dist(before['base_pose']['position'],after['base_pose']['position']),image=args.image,
     executable_sha256={name:hashlib.sha256((ROOT/'build/desktop'/name).read_bytes()).hexdigest() for name in ('simple-rbt-console','simple-rbt-sim')})
 except Exception as error:
  report=dict(passed=False,error=str(error));raise
 finally:
  if sampler and sampler.thread.is_alive():sampler.stop()
  if observer:observer.close()
  if process and process.poll() is None:
   if inputs and window:inputs.close_window(window)
   else:process.terminate()
   process.wait(timeout=15)
  if container:subprocess.run(['docker','stop','--time','3',container],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  if receiver and receiver.poll() is None:receiver.wait(timeout=10)
  if inputs:inputs.close()
  (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({key:value for key,value in report.items() if key not in ('samples','receiver')},indent=2))
 if not report['passed']:raise SystemExit(1)
if __name__=='__main__':main()
