#!/usr/bin/env python3
"""Real X11 input: named presets, scene editing, uninterrupted preview and reopen."""
import argparse,json,os,subprocess,sys,time,math,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests/console'))
from native_x11 import Inputs
from observe_session import Observer,child_session,owned_workers
from test_console_native import project,gizmo_length

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--console',type=Path,default=ROOT/'build/desktop/simple-rbt-console');p.add_argument('--core',type=Path,default=ROOT/'build/desktop/simple-rbt-sim');p.add_argument('--assets',type=Path,default=ROOT/'assets');p.add_argument('--display',required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 config=out/'initial.json';config.write_text(json.dumps({'robot':'go2w','backend':'cpu','camera_model':'d435i'}))
 env=dict(os.environ,DISPLAY=args.display,LC_ALL='C.UTF-8',XDG_CONFIG_HOME=str(out/'config'),XDG_STATE_HOME=str(out/'state'),XDG_CACHE_HOME=str(out/'cache'))
 base=[str(args.console.resolve()),'--worker',str(args.core.resolve()),'--assets',str(args.assets.resolve())]
 inputs=Inputs(args.display);checks=[];proc=None;observer=None;window=None
 workspace=out/'config/simple-rbt/workspace.json'
 def check(name,ok,evidence=None):
  checks.append(dict(check=name,passed=bool(ok),evidence=evidence))
  if not ok:raise RuntimeError(name)
 def wait_saved(predicate):
  deadline=time.monotonic()+6
  while time.monotonic()<deadline:
   if workspace.exists():
    value=json.loads(workspace.read_text())
    if predicate(value):return value
   time.sleep(.1)
  raise RuntimeError('Workspace checkpoint timeout')
 def name(x,y,text):inputs.click(x,y);inputs.type_number(text)
 def screenshot(label):inputs.capture(window,out/(label+'.png'))
 def connect(exclude=None):
  nonlocal observer
  if observer:observer.close()
  directory=child_session(proc.pid,exclude=exclude)
  observer=Observer(directory);observer.wait(lambda m,v:m.get('session_state')=='config',timeout=15)
  time.sleep(.6);return directory
 def close():
  nonlocal proc,observer
  if observer:observer.close();observer=None
  if proc and proc.poll() is None:
   children=owned_workers(proc.pid)
   inputs.close_window(window);proc.wait(timeout=10)
   check('owned worker exits with console',all(not Path('/proc/'+str(pid)).exists() for pid,_ in children))
 with (out/'console.log').open('w') as log:
  try:
   proc=subprocess.Popen(base+['--config',str(config)],env=env,stdout=log,stderr=log)
   directory=connect();window=inputs.window();inputs.focus(window);screenshot('launch')
   original_window=window;inputs.click(175,90);time.sleep(.3)
   name(150,174,'survey-one')
   metadata,_=observer.snapshot();mount=metadata['mounts']['mid360']['position'];viewport=inputs.viewport(window,metadata['observer']['width'],metadata['observer']['height'])
   length=gizmo_length(metadata,mount,viewport);tip=project(metadata,[mount[0]+length,mount[1],mount[2]],viewport)
   inputs.move(*tip);inputs.button(True);inputs.move(tip[0]+25,tip[1]-5);inputs.button(False)
   metadata,_=observer.wait(lambda m,v:abs(m['mounts']['mid360']['position'][0]-mount[0])>.005)
   edited=metadata['mounts']['mid360'];inputs.click(75,246)
   w=wait_saved(lambda w:any(r['name']=='survey-one' for r in w['robots'].values()));robot_id=w['robot_id']
   check('user-named preset saves real dragged extrinsics',math.dist(w['robots'][robot_id]['mounts']['mid360']['position'],edited['position'])<1e-6)
   name(150,174,'survey-renamed');inputs.click(75,246)
   w=wait_saved(lambda w:w['robots'].get(robot_id,{}).get('name')=='survey-renamed')
   check('renaming keeps stable preset identity',len(w['robots'])==1)
   screenshot('robot-preset')
   inputs.click(310,90);time.sleep(.3);screenshot('scene-editor')
   check('page navigation keeps window and worker',inputs.find_window()==original_window and child_session(proc.pid)==directory)
   name(120,165,'warehouse-two-floors')
   # Ground-floor ramp is the second object in the tree.
   inputs.click(110,385);time.sleep(.2);screenshot('selected-ramp')
   # Edit connecting-ramp width through the native property inspector.
   name(1080,434,'2.5')
   w=wait_saved(lambda w:any(any(o['kind']=='ramp' and abs(o['size'][1]-2.5)<1e-6 for o in s['objects']) for s in w['scene_drafts'].values()))
   scene_id=w['scene_id'];check('native scene property edit persists',True)
   inputs.click(70,216);w=wait_saved(lambda w:scene_id in w['scenes'])
   check('named multilevel scene saves independently',len(w['scenes'][scene_id]['floors'])==2 and w['robots'][robot_id]['name']=='survey-renamed')
   screenshot('scene-properties')
   # Runtime scene switches keep the old frame until the new worker renders.
   inputs.click(120,825)
   colored=[];distinct=[]
   for i in range(14):
    width,height,pixels=inputs.pixels(window)
    count=sum(tuple(pixels[(y*width+x)*3:(y*width+x)*3+3]) not in [(0,0,0),(21,27,36),(25,29,36)] for y in range(350,750,8) for x in range(500,1250,8))
    colored.append(count)
    distinct.append(len({tuple(pixels[(y*width+x)*3:(y*width+x)*3+3]) for y in range(350,750,8) for x in range(500,1250,8)}))
    if i==1:screenshot('scene-loading')
    time.sleep(.08)
   directory=connect(exclude=directory)
   metadata,_=observer.snapshot();w=wait_saved(lambda w:w['config'].get('scene',{}).get('id')==scene_id)
   check('scene switch keeps a visible previous or new image',min(colored)>500 and min(distinct)>5,dict(nonblank_pixels=colored,distinct_colors=distinct))
   check('scene loading retains native window',inputs.find_window()==original_window)
   check('scene switch preserves robot extrinsics',math.dist(metadata['mounts']['mid360']['position'],edited['position'])<1e-6)
   check('only one simulation worker remains',len(owned_workers(proc.pid))==1)
   screenshot('custom-scene-launch')
   # Edit a draft without saving and verify the saved preset survives reopening.
   inputs.click(175,90);time.sleep(.2);name(150,174,'unfinished-draft')
   wait_saved(lambda w:w['robot_drafts'][robot_id]['name']=='unfinished-draft')
   close()
   proc=subprocess.Popen(base,env=env,stdout=log,stderr=log);directory=connect();window=inputs.window();inputs.focus(window)
   metadata,_=observer.snapshot();w=json.loads(workspace.read_text())
   check('reopen restores selected scene and named robot draft',w['robot_id']==robot_id and w['config']['scene']['id']==scene_id and w['robot_drafts'][robot_id]['name']=='unfinished-draft')
   check('reopen keeps saved preset separate from draft',w['robots'][robot_id]['name']=='survey-renamed')
   check('reopen restores extrinsics in real core',math.dist(metadata['mounts']['mid360']['position'],edited['position'])<1e-6)
   screenshot('reopened')
   close()
  except Exception as e:
   checks.append(dict(check='exception',passed=False,evidence=str(e)))
   if window:
    try:screenshot('failure')
    except Exception:pass
  finally:
   if observer:observer.close()
   if proc and proc.poll() is None:proc.terminate();proc.wait(timeout=10)
   inputs.x.XCloseDisplay(inputs.d)
 report=dict(passed=all(c['passed'] for c in checks),checks=checks,console_sha256=hashlib.sha256(args.console.read_bytes()).hexdigest(),core_sha256=hashlib.sha256(args.core.read_bytes()).hexdigest())
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
 if not report['passed']:raise SystemExit(1)
if __name__=='__main__':main()
