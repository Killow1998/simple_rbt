#!/usr/bin/env python3
"""One-time model-data preparation; the native simulator never launches Python."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from simple_rbt_mujoco.core import ROOT, UPSTREAM, _go2_xml, _diffdrive_xml
from simple_rbt_mujoco.robots import WHEEL_MODELS, wheel_model
from simple_rbt_mujoco.mid360 import append_mid360_imu
from simple_rbt_mujoco.go2 import build_go2_xml, MANIFEST as GO2_MANIFEST
from simple_rbt_mujoco.go2w import build_go2w_xml, JOINT_NAMES, DEFAULT_POS, ACTION_SCALE
from simple_rbt_mujoco.terrains import append_terrain, TERRAIN_NAMES

out = ROOT / 'mujoco_sim_cpp/assets'
(out / 'models').mkdir(parents=True, exist_ok=True)
catalog = {'schema': 1, 'model_preparation': 'offline data export; no Python in native execution',
           'models': {}, 'controllers': {}, 'terrain_presets': {}}
go2 = json.loads(GO2_MANIFEST.read_text())
catalog['go2_manifest'] = go2
for robot in ('go2', 'go2w', 'diffdrive', *WHEEL_MODELS):
    choices = ('robot_lab','himloco','legacy') if robot == 'go2' else ('auto',)
    for choice in choices:
        if robot == 'go2':
            xml = _go2_xml(UPSTREAM) if choice == 'legacy' else build_go2_xml(choice)
            controller = choice
        elif robot == 'go2w':
            xml, controller = build_go2w_xml(), 'rl_sar'
        elif robot == 'diffdrive':
            xml, controller = _diffdrive_xml(), 'none'
        else:
            builder, _, controller = wheel_model(robot)
            xml = builder()
        root = ET.fromstring(xml)
        append_mid360_imu(root)
        text = ET.tostring(root, encoding='unicode').replace(str(ROOT), '@PROJECT_ROOT@')
        key = robot + '-' + controller
        path = out / 'models' / (key + '.xml')
        path.write_text(text + '\n')
        catalog['models'][key] = {'file': 'models/'+path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        if robot == 'go2' and controller != 'legacy':
            catalog['controllers'][key] = {**go2['controllers'][controller],
                'joint_names': [go2['sdk_joint_names'][i] for i in go2['controllers'][controller]['joint_mapping']],
                'policy': 'mujoco_sim/assets/'+go2['policy_exports'][controller]['onnx_file'],
                'policy_sha256':go2['policy_exports'][controller]['onnx_sha256']}
        elif robot == 'go2w':
            from simple_rbt_mujoco.go2w import POLICY_SHA256
            catalog['controllers'][key] = {'joint_names':list(JOINT_NAMES), 'default_dof_pos':DEFAULT_POS.tolist(),
                'action_scale':ACTION_SCALE.tolist(), 'policy':'mujoco_sim/assets/go2w_policy.onnx',
                'policy_sha256':POLICY_SHA256}
        elif robot == 'diffdrive':
            catalog['controllers'][key] = {
                'implemented_geometry': {'wheel_radius_m': .12, 'track_m': .36},
                'control': {'controller': 'none',
                            'description': 'Existing two-wheel velocity mapping; no outer command tracker.'}}
        elif robot in WHEEL_MODELS:
            import importlib
            catalog['controllers'][key] = importlib.import_module('simple_rbt_mujoco.'+robot).model_metadata()
        elif robot == 'go2':
            policy=UPSTREAM/'examples/onnx/go2_policy.onnx'
            catalog['controllers'][key]={'policy':str(policy.relative_to(ROOT)),
                'policy_sha256':hashlib.sha256(policy.read_bytes()).hexdigest()}

for terrain in TERRAIN_NAMES:
    for level in ('standard','moderate','hard','extreme'):
        root=ET.fromstring('<mujoco><worldbody/></mujoco>')
        profile=append_terrain(root, terrain, level=level, seed=0)
        catalog['terrain_presets'][terrain+'-'+level] = {
            'metadata':profile.metadata,
            'boxes':[{'name':b.name,'centre':b.centre,'half_size':b.half_size,'slope':b.slope} for b in profile._boxes]}

(out/'catalog.json').write_text(json.dumps(catalog,indent=2)+'\n')
print(json.dumps({'models':len(catalog['models']),'terrains':len(catalog['terrain_presets']),
                  'output':str(out)},indent=2))
