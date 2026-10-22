#!/usr/bin/env python3
"""Native GUI -> C++ -> real ROS diagnostics chain in isolated state directories."""
import argparse,json,os,shutil,socket,struct,subprocess,sys,time,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests/console'))
from native_x11 import Inputs
from diagnostic_image import image_path
from observe_session import Observer,child_session,owned_workers


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--display',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--ros',choices=['ros1','ros2'],default='ros1');p.add_argument('--image',required=True);p.add_argument('--console',type=Path,default=ROOT/'build/desktop/simple-rbt-console');p.add_argument('--core',type=Path,default=ROOT/'build/desktop/simple-rbt-sim');p.add_argument('--domain-id',type=int,default=97);args=p.parse_args()
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 docker=shutil.which('docker');test_path=image_path(out,args.ros,args.image)
 with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
 config=out/'initial.json';config.write_text(json.dumps(dict(robot='go2w',backend='cpu',camera_model='d435i',sensors=dict(mid360=True,camera=True,depth=True,panorama=True))))
 env=dict(os.environ,DISPLAY=args.display,LC_ALL='C.UTF-8',PATH=test_path,XDG_CONFIG_HOME=str(out/'config'),XDG_STATE_HOME=str(out/'state'),XDG_CACHE_HOME=str(out/'cache'),ROS_MASTER_URI=f'http://127.0.0.1:{port}',ROS_IP='127.0.0.1',SIMPLE_RBT_START_MASTER='1',ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
 inputs=Inputs(args.display);observer=proc=None;checks=[];window=None;container=None
 def check(name,ok,evidence=None):
  checks.append(dict(check=name,passed=bool(ok),evidence=evidence))
  if not ok:raise RuntimeError(name)
 def records():
  result=[]
  for f in sorted((out/'state/simple-rbt/logs').glob('srbt-*.jsonl')):
   for line in f.read_text().splitlines():
    try:result.append(json.loads(line))
    except json.JSONDecodeError:pass
  return result
 def wait_record(test):
  deadline=time.monotonic()+10
  while time.monotonic()<deadline:
   rows=records()
   if test(rows):return rows
   time.sleep(.2)
  raise RuntimeError('Expected persistent event absent')
 try:
  with (out/'console.log').open('w') as log:
   proc=subprocess.Popen([str(args.console.resolve()),'--worker',str(args.core.resolve()),'--assets',str(ROOT/'assets'),'--config',str(config),'--ros',args.ros],env=env,stdout=log,stderr=log)
   directory=child_session(proc.pid);observer=Observer(directory);observer.wait(lambda m,v:m.get('session_state')=='config',timeout=30)
   window=inputs.window();inputs.focus(window);time.sleep(1)
   inputs.click(410,90);time.sleep(1);inputs.capture(window,out/'logs-initial.png')
   inputs.click(29,181)
   observer.wait(lambda m,v:m.get('logging',{}).get('trace') is True)
   check('native trace checkbox reaches core',True)
   inputs.click(1265,146);time.sleep(.3)
   inputs.click(*inputs.primary_button(window))
   observer.wait(lambda m,v:m.get('session_state')=='running' and m.get('connections',{}).get(args.ros,0)>0,timeout=30)
   rows=wait_record(lambda rows:any(r.get('component')==args.ros and r.get('event')=='ros.publish_called' and (r.get('fields') or {}).get('stream')=='lidar' for r in rows))
   samples={}
   for r in rows:
    sample=(r.get('fields') or {}).get('sample','')
    if sample.startswith('lidar:'):samples.setdefault((r['session'],sample),set()).add(r['event'])
   linked=[sample for sample,events in samples.items() if {'sensor.generated','ipc.offered','ipc.received','ros.publish_called'}<=events]
   check('one scan correlates across generation IPC and ROS publish',bool(linked),linked[:3])
   rows=wait_record(lambda rows:{'clock','imu','camera_imu'}<={r['fields'].get('stream') for r in rows if r['event']=='stream.summary' and r['component']==args.ros and r['fields'].get('stage')=='ros.publish_called'})
   check('clock and both IMUs retain ROS stage summaries',True)
   actions={r['fields']['action_id'] for r in rows if r['event']=='ui.action'}
   check('button action is linked to core request',any(r['event']=='request.applied' and r['fields'].get('request',{}).get('action_id') in actions for r in rows))
   payload=json.dumps(dict(op='invalid-diagnostic-test',request_id='fault-test')).encode()
   observer.connections[0].sendall(struct.pack('<4sHHQII',b'SRBT',1,2,0,len(payload),0)+payload)
   wait_record(lambda rows:any(r['event']=='request.failed' and r['fields']['request'].get('request_id')=='fault-test' for r in rows))
   check('rejected request persists with reason and request ID',True)
   container='simple-rbt-'+args.ros+'-'+directory.name
   subprocess.run([docker,'stop','--time','3',container],check=True,stdout=subprocess.DEVNULL);container=None
   observer.wait(lambda m,v:m.get('connections',{}).get(args.ros,0)==0)
   wait_record(lambda rows:any(r['event']=='ipc.disconnected' and r['fields'].get('role')==args.ros for r in rows))
   check('adapter disconnect is retained',True)
   inputs.click(500,90);time.sleep(2);inputs.capture(window,out/'logs-runtime.png')
   inputs.click(1120,146)
   deadline=time.monotonic()+10
   while time.monotonic()<deadline and not list((out/'state/simple-rbt').glob('diagnostics-*.zip')):time.sleep(.2)
   archives=list((out/'state/simple-rbt').glob('diagnostics-*.zip'))
   check('native export creates diagnostic archive',bool(archives))
   with zipfile.ZipFile(archives[0]) as archive:
    manifest=json.loads(archive.read('manifest.json'));check('export includes configuration and event files',manifest['config']['robot']=='go2w' and any(n.endswith('.jsonl') for n in archive.namelist()))
   inputs.click(995,32);inputs.press('End');inputs.press('Return');inputs.click(1120,32);inputs.press('End');inputs.press('Return');time.sleep(.3);inputs.capture(window,out/'logs-light-zh.png')
   inputs.close_window(window);proc.wait(timeout=15)
   rows=[json.loads(line) for f in (out/'state/simple-rbt/logs').glob('srbt-*.jsonl') for line in f.read_text().splitlines()]
   check('concurrent writers preserve valid JSON and unique event IDs',len({(r['writer'],r['seq']) for r in rows})==len(rows))
   check('GUI and core shutdown records survive close',{'gui','core'}<={r['component'] for r in rows if r['event']=='process.stopped'})
   check('retained diagnostics stay below 100 MB',sum(f.stat().st_size for f in (out/'state/simple-rbt/logs').glob('srbt-*.jsonl'))<=100000000)
 except Exception as error:
  checks.append(dict(check='exception',passed=False,evidence=str(error)))
  if window and proc and proc.poll() is None:
   try:inputs.capture(window,out/'failure.png')
   except Exception:pass
 finally:
  if observer:observer.close()
  if proc and proc.poll() is None:
   if window:inputs.close_window(window)
   else:proc.terminate()
   proc.wait(timeout=15)
  if container:subprocess.run([docker,'stop','--time','3',container],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  inputs.close()
 report=dict(passed=all(c['passed'] for c in checks),checks=checks,image=args.image)
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
 if not report['passed']:raise SystemExit(1)
if __name__=='__main__':main()
