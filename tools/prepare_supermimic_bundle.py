#!/usr/bin/env python3
"""Export verified external profiles as a relocatable local SpRobo bundle."""
import argparse,hashlib,json,os,shutil,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--profiles',type=Path,required=True,help='Verified flat/slope/terrain_* asset roots')
 p.add_argument('--stairs',type=Path,help='Verified up_l1..down_l5 asset roots')
 p.add_argument('--output',type=Path,required=True)
 a=p.parse_args();dest=a.output.resolve();dest.mkdir(parents=True,exist_ok=False)
 shared=dest/'shared';shared.mkdir()
 base=a.profiles.resolve()/'flat'
 xml=ET.parse(base/'assets/models/go2w-supermimic.xml')
 meshes=Path(xml.find('compiler').get('meshdir'))
 shutil.copytree(meshes,shared/'robot-meshes')
 policy=base/'assets/supermimic/actor.onnx';shutil.copy2(policy,shared/'actor.onnx')
 shutil.copytree(ROOT/'assets/lidar',shared/'lidar')
 shutil.copytree(ROOT/'assets/tutorials',shared/'tutorials')
 shutil.copytree(ROOT/'assets/visual/sensors',shared/'visual/sensors')
 names={'flat':'flat','up':'terrain_up','down':'terrain_down','slope':'terrain_slope',
        'rubble':'terrain_rubble','incline-up':'slope_plus','incline-down':'slope_minus'}
 sources={name:a.profiles.resolve()/source for name,source in names.items()}
 if a.stairs:
  sources.update({f'{kind}-l{level}':a.stairs.resolve()/f'{kind}_l{level}' for level in range(1,6) for kind in ['up','down']})
 sources['mapping']=base
 bundle={'format':'sprobo.bundle.v1','name':'Go2W SuperMimic','default_profile':'up',
         'upstream_commit':'019c2d7300e82b59486996d87f6013108effb005',
         'policy_sha256':digest(policy),'command':['vx_m_s','vy_m_s','yaw_rad_s'],
         'command_observation_delay_s':.02,'multiccd':False,'profiles':{},'files':{}}
 for name,source in sources.items():
  target=dest/'profiles'/name;(target/'assets/models').mkdir(parents=True)
  catalog=json.loads((source/'assets/catalog.json').read_text());controller=catalog['controllers']['go2w-supermimic']
  original=source/'assets/models/go2w-supermimic.xml'
  if digest(original)!=catalog['models']['go2w-supermimic']['sha256']:raise ValueError('Source model digest mismatch: '+name)
  if controller['policy_sha256']!=bundle['policy_sha256']:raise ValueError('Source policy digest mismatch: '+name)
  tree=ET.parse(original);tree.find('compiler').set('meshdir','@PROJECT_ROOT@/../../shared/robot-meshes')
  # Warp 3.11 training uses one contact for cylinder/box wheel collisions.
  flag=tree.find('option/flag')
  if flag is None:flag=ET.SubElement(tree.find('option'),'flag')
  flag.set('multiccd','disable')
  if name=='mapping':
   world=tree.find('worldbody');world.remove(world.find("body[@name='terrain']"))
   # Training-only contact sensors reference the replaced upstream terrain body.
   sensors=tree.find('sensor')
   for sensor in list(sensors):
    if sensor.get('body1')=='terrain' or sensor.get('body2')=='terrain':sensors.remove(sensor)
  model=target/'assets/models/go2w-supermimic.xml';tree.write(model,encoding='unicode')
  controller['policy']='../../shared/actor.onnx'
  catalog['models']={'go2w-supermimic':{'file':'models/go2w-supermimic.xml','sha256':digest(model)}}
  catalog['controllers']={'go2w-supermimic':controller}
  (target/'assets/catalog.json').write_text(json.dumps(catalog,indent=2)+'\n')
  for part in ['lidar','visual','tutorials']:
   (target/'assets'/part).symlink_to(os.path.relpath(shared/part,target/'assets'),target_is_directory=True)
  if name=='mapping':
   config=json.loads((ROOT/'examples/tutorial-go2w.json').read_text())
  else:
   scene=json.loads((source/'validation.scene.json').read_text());scene.update(id='supermimic-'+name,name='SuperMimic · '+name)
   config=dict(robot='go2w',scene=scene,terrain='flat',terrain_level='standard',backend='cpu',camera_model='d435',lidar_model='mid360',
               camera_hz=10,mounts={},adapters={},sensors=dict.fromkeys(['mid360','camera','depth','panorama','external_camera'],False))
  config.update(controller='supermimic',control_mode='tracked')
  (target/'session.json').write_text(json.dumps(config,indent=2,ensure_ascii=False)+'\n')
  (target/'validation.scene.json').write_text(json.dumps(config['scene'],indent=2,ensure_ascii=False)+'\n')
  bundle['profiles'][name]={'root':str(target.relative_to(dest)),'config':str((target/'session.json').relative_to(dest)),
                            'original_model_sha256':digest(original),'model_sha256':digest(model)}
 for path in dest.rglob('*'):
  if path.is_symlink() and not path.resolve().is_relative_to(dest):raise ValueError('External symlink: '+str(path))
  if path.is_file():bundle['files'][str(path.relative_to(dest))]=digest(path)
 (dest/'bundle.json').write_text(json.dumps(bundle,indent=2,ensure_ascii=False)+'\n')
 print(dest)
if __name__=='__main__':main()
