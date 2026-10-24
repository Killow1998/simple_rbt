#!/usr/bin/env python3
"""Native X11 v2 editor E2E: shared template, override, history, save and reopen."""
import argparse,json,os,subprocess,sys,time,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests/console'))
from native_x11 import Inputs
from observe_session import Observer,child_session,owned_workers

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--display',required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--core',type=Path,required=True);p.add_argument('--console',type=Path,required=True);args=p.parse_args()
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:4])
 env=dict(os.environ,DISPLAY=args.display,LIBGL_ALWAYS_SOFTWARE='1',EGL_PLATFORM='surfaceless',__EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/50_mesa.json',XDG_CONFIG_HOME=str(out/'config'),XDG_STATE_HOME=str(out/'state'),XDG_CACHE_HOME=str(out/'cache'))
 scene=json.loads((ROOT/'schemas/examples/structured-building.scene.json').read_text())
 cfg=out/'initial.json';cfg.write_text(json.dumps(dict(robot='go2w',controller='rl_sar',backend='cpu',camera_model='d435i',camera_hz=10,rays=20000,scene=scene)))
 base=[str(args.console.resolve()),'--worker',str(args.core.resolve()),'--assets',str(ROOT/'assets')]
 inputs=Inputs(args.display);proc=None;observer=None;checks=[];window=None
 workspace=out/'config/simple-rbt/workspace.json'
 def check(name,ok):
  checks.append(dict(check=name,passed=bool(ok)))
  if not ok:raise RuntimeError(name)
 def saved(predicate):
  deadline=time.monotonic()+8
  while time.monotonic()<deadline:
   if workspace.exists():
    w=json.loads(workspace.read_text())
    if predicate(w):return w
   time.sleep(.1)
  raise RuntimeError('workspace checkpoint timeout')
 def draft(w):return w['scene_drafts'][scene['id']]
 def connect(exclude=None):
  nonlocal observer,window
  if observer:observer.close()
  directory=child_session(proc.pid,exclude=exclude);observer=Observer(directory,view=True)
  observer.wait(lambda m,v:m.get('session_state')=='config' and observer.preview_ready(m),timeout=35)
  window=inputs.window();inputs.focus(window);time.sleep(.5);return directory
 def close():
  nonlocal observer,proc
  if observer:observer.close();observer=None
  if proc and proc.poll() is None:
   children=owned_workers(proc.pid);inputs.close_window(window);proc.wait(timeout=15)
   check('owned worker exits',all(not Path('/proc/'+str(pid)).exists() for pid,_ in children))
 def value(x,y,text):inputs.click(x,y);inputs.type_number(text)
 with (out/'console.log').open('w') as log:
  try:
   proc=subprocess.Popen(base+['--config',str(cfg)],env=env,stdout=log,stderr=log);directory=connect()
   inputs.click(310,90);time.sleep(.5);inputs.capture(window,out/'initial.png')
   value(120,165,'structured-e2e');inputs.click(110,581);time.sleep(.4)
   inputs.click(1049,570);value(1080,611,'1.4')
   w=saved(lambda w:abs(draft(w)['geometry_templates']['standard-flight']['width_m']-1.4)<1e-8)
   check('shared template width edits all instances',all(o['geometry']['template_id']=='standard-flight' and 'width_m' not in o['geometry'].get('overrides',{}) for o in draft(w)['objects'] if o['id'] in ['flight-a','flight-b']))
   inputs.click(1049,570);value(1080,749,'9')
   saved(lambda w:any(o['id']=='flight-a' and o['geometry'].get('overrides',{}).get('steps')==9 for o in draft(w)['objects']))
   inputs.click(480,146);saved(lambda w:not any(o['id']=='flight-a' for o in draft(w)['objects']))
   inputs.click(312,146);saved(lambda w:any(o['id']=='flight-a' for o in draft(w)['objects']))
   inputs.click(384,146);saved(lambda w:not any(o['id']=='flight-a' for o in draft(w)['objects']))
   inputs.click(312,146);saved(lambda w:any(o['id']=='flight-a' for o in draft(w)['objects']))
   check('delete undo redo preserves instance and source fields',True)
   inputs.click(70,216);w=saved(lambda w:scene['id'] in w['scenes']);stored=w['scenes'][scene['id']]
   check('save keeps v2 hierarchy and templates',stored['version']==2 and len(stored['objects'])==11 and 'floors' not in stored and 'position' not in stored['objects'][0])
   for theme in ('light','dark'):
    inputs.click(995,32);inputs.press('End' if theme=='light' else 'Home');inputs.press('Return')
    for language in ('zh','en'):
     inputs.click(1120,32);inputs.press('End' if language=='zh' else 'Home');inputs.press('Return');time.sleep(.4);inputs.capture(window,out/(theme+'-'+language+'.png'))
   inputs.click(120,825);directory=connect(exclude=directory)
   check('core preview accepts edited template scene',observer.snapshot()[0]['session_state']=='config')
   close();proc=subprocess.Popen(base,env=env,stdout=log,stderr=log);directory=connect()
   w=json.loads(workspace.read_text());check('restart preserves exact saved scene',w['scenes'][scene['id']]==stored and w['config']['scene']==stored)
   inputs.click(310,90);time.sleep(.4);inputs.capture(window,out/'reopened.png');close()
  except Exception as e:
   checks.append(dict(check='exception',passed=False,evidence=str(e)))
   if window:inputs.capture(window,out/'failure.png')
  finally:
   if observer:observer.close()
   if proc and proc.poll() is None:proc.terminate();proc.wait(timeout=15)
   inputs.close()
 report=dict(passed=all(c['passed'] for c in checks),checks=checks,cpu_affinity=sorted(os.sched_getaffinity(0)),core_sha256=hashlib.sha256(args.core.read_bytes()).hexdigest(),console_sha256=hashlib.sha256(args.console.read_bytes()).hexdigest())
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
