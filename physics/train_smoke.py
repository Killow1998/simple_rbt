"""Bounded adapter around upstream train.main; learner/rollout/optimizers stay upstream."""
import json,sys,hashlib
from pathlib import Path
import numpy as np
import torch
import mujoco
from warp_batch import solver_cfg


def shared_cfg(contract):
 from go2w_supermimic.env.flat import go2w_velocity_flat_env_cfg
 cfg=go2w_velocity_flat_env_cfg(play=True,history=5)
 cfg.scene.terrain=None;cfg.scene.env_spacing=0.
 cfg.scene.entities['robot'].init_state.pos=tuple(contract['initial']['qpos'][:3])
 # One authoritative static geometry set, no additional ground or generated terrain.
 def add_world(spec):
  for i,geom in enumerate(contract['static_geoms']):
   if geom['type'] not in [0,2,3,4,5,6]:raise ValueError('Training adapter requires analytic static geometry')
   spec.worldbody.add_geom(name='shared_world_'+str(i),mass=0.,**geom)
 cfg.scene.spec_fn=add_world
 cfg.sim.mujoco=solver_cfg(contract['solver']);cfg.decimation=4
 cfg.sim.nconmax=512;cfg.sim.njmax=2048
 cfg.actions['joint_pos'].scale={name.removeprefix('robot/'):value for name,value in zip(contract['controller']['joint_names'][:12],contract['controller']['action_scale'][:12])}
 cfg.actions['wheel_vel'].scale=contract['controller']['action_scale'][12]
 cfg.events['reset_base'].params['pose_range']={};cfg.events['reset_base'].params['velocity_range']={}
 return cfg


def audit_training(env,contract):
 source=mujoco.MjModel.from_xml_path(str(Path(env._shared_contract).parent/contract['model']));target=env.sim.mj_model
 errors={}
 # Compare by stable names: sensor visuals and terrain packing do not define robot indices.
 for kind,prefix,fields in [(mujoco.mjtObj.mjOBJ_BODY,'body',['mass','inertia','ipos','iquat']),
                            (mujoco.mjtObj.mjOBJ_JOINT,'jnt',['range','limited'])]:
  for name in contract['controller']['joint_names'] if prefix=='jnt' else [mujoco.mj_id2name(source,kind,i) for i in range(source.nbody) if source.body_mass[i]>0]:
   a=mujoco.mj_name2id(source,kind,name);b=mujoco.mj_name2id(target,kind,name)
   if b<0:raise ValueError('Training model name missing: '+name)
   for f in fields:errors[name+'/'+f]=float(np.max(np.abs(np.asarray(getattr(source,prefix+'_'+f)[a],dtype=float)-getattr(target,prefix+'_'+f)[b])))
 for name in contract['controller']['joint_names']:
  a=mujoco.mj_name2id(source,mujoco.mjtObj.mjOBJ_JOINT,name);b=mujoco.mj_name2id(target,mujoco.mjtObj.mjOBJ_JOINT,name)
  for field in ['dof_armature','dof_damping','dof_frictionloss']:
   errors[name+'/'+field]=float(abs(getattr(source,field)[source.jnt_dofadr[a]]-getattr(target,field)[target.jnt_dofadr[b]]))
  aa=int(np.where(source.actuator_trnid[:,0]==a)[0][0]);bb=int(np.where(target.actuator_trnid[:,0]==b)[0][0])
  for field in ['actuator_gainprm','actuator_biasprm','actuator_ctrlrange','actuator_forcerange']:
   errors[name+'/'+field]=float(np.max(np.abs(getattr(source,field)[aa]-getattr(target,field)[bb])))
 for i,geom in enumerate(contract['static_geoms']):
  g=mujoco.mj_name2id(target,mujoco.mjtObj.mjOBJ_GEOM,'shared_world_'+str(i))
  for key,value in geom.items():
   actual=getattr(target,'geom_'+key)[g]
   errors['static/'+str(i)+'/'+key]=float(np.max(np.abs(np.asarray(actual,dtype=float)-value)))
 for key,value in contract['solver'].items():
  errors['solver/'+key]=float(np.max(np.abs(np.asarray(getattr(target.opt,key),dtype=float)-value)))
 if max(errors.values())>1e-6:raise ValueError('Training robot parameters differ: '+str({k:v for k,v in errors.items() if v>1e-6}))
 return {'passed':True,'robot_max_abs_error':max(errors.values()),'fields':errors,'shared_static_geoms':len(contract['static_geoms']),
         'physics_backend':'warp','physics_device':str(env.sim.wp_device)}


class InitializationCaptured(Exception):
 pass


def run(contract_path,job,out):
 from go2w_supermimic.core import train
 from go2w_supermimic.env import flat
 from go2w_supermimic.core.policy import load_actor
 from mjlab import envs
 c=json.loads(Path(contract_path).read_text());cfg=shared_cfg(c)
 original_cfg=flat.go2w_velocity_flat_env_cfg;original_init=train.Learner.__init__;original_env=envs.ManagerBasedRlEnv
 baseline={};buffers={};audits=[]
 def initialize(learner,args,field=None):
  original_init(learner,args,field)
  frozen=load_actor(torch.load(job['checkpoint'],map_location=args.device,weights_only=True),args.device)
  learner.actor.load_state_dict(frozen.state_dict())
  for name,module in [('world_model',learner.model),('actor',learner.actor),('critic',learner.critic)]:
   baseline[name]={k:v.detach().cpu().clone() for k,v in module.named_parameters()}
   buffers[name]={k:v.detach().cpu().clone() for k,v in module.named_buffers()}
  torch.save({'parameters':baseline,'buffers':buffers},out/'initial-network-state.pt')
  if job.get('audit_endpoint'):raise InitializationCaptured()
 def environment(*args,**kwargs):
  env=original_env(*args,**kwargs);env._shared_contract=str(contract_path)
  audits.append(audit_training(env,c));return env
 # These are the upstream's dependency injection points; no upstream files are edited.
 flat.go2w_velocity_flat_env_cfg=lambda **kwargs:cfg
 train.Learner.__init__=initialize;envs.ManagerBasedRlEnv=environment
 original_constants=train.constants_from_bridge
 def constants(bridge):
  values=original_constants(bridge);values['leg_action_scale']=torch.tensor(c['controller']['action_scale'][:12],device=bridge.env.device);values['wheel_action_scale']=c['controller']['action_scale'][12];return values
 train.constants_from_bridge=constants
 old=sys.argv
 sys.argv=['sprobo-train-smoke','--device',job['device'],'--num-envs',str(job['training_worlds']),
  '--iterations',str(job['training_iterations']),'--buffer-len','32','--wm-window','4','--wm-batch','16','--wm-updates','1',
  '--imag-batch','16','--horizon','4','--warmup','4','--critic-epochs','1','--critic-minibatches','1',
  '--log-interval','8','--save-interval',str(job['training_iterations']),'--output',str(out/'training')]
 try:train.main()
 except InitializationCaptured:pass
 finally:
  sys.argv=old;flat.go2w_velocity_flat_env_cfg=original_cfg;train.Learner.__init__=original_init;envs.ManagerBasedRlEnv=original_env;train.constants_from_bridge=original_constants
 checkpoint=Path(job['audit_endpoint']) if job.get('audit_endpoint') else out/'training'/('model_'+str(job['training_iterations'])+'.pt');end=torch.load(checkpoint,map_location='cpu',weights_only=True)
 deltas={name:max(float((end[name][k]-v).abs().max()) for k,v in initial.items() if v.is_floating_point()) for name,initial in baseline.items() if name!='world_model'}
 # World-model checkpoint includes normalization metadata around its state_dict.
 from go2w_supermimic.core.world_model import Go2WWorldModel
 final_wm=Go2WWorldModel.from_checkpoint(end['world_model']).state_dict()
 deltas['world_model']=max(float((final_wm[k]-v).abs().max()) for k,v in baseline['world_model'].items() if v.is_floating_point())
 optimizer_steps=[max(int(v['step']) for v in optimizer['state'].values()) for optimizer in end['optimizers']]
 buffer_deltas={name:{k:float((({'world_model':final_wm}.get(name,end.get(name)))[k]-v).abs().max()) for k,v in values.items()} for name,values in buffers.items()}
 metrics=[json.loads(line) for line in (checkpoint.parent/'metrics.jsonl').read_text().splitlines()]
 final=metrics[-1];checks={'shared_robot':all(a['passed'] for a in audits),'actual_sampling':final['total_transitions']>=job['training_worlds']*job['training_iterations'],
  'wm_actor_critic_updates':min(optimizer_steps)>0 and min(deltas.values())>0,'imagination_policy_updates':final['policy_updates']>0,
  'finite_losses':all(np.isfinite(final[k]) for k in ['wm_loss','actor_loss','critic_loss']),
  'no_invalid_resets':final['invalid_env_resets']==0,'no_skipped_updates':final['skipped_updates']==0}
 return {'passed':all(checks.values()),'checks':checks,'physics_backend':'warp','physics_device':job['device'],
  'model_audits':audits,'optimizer_steps_wm_actor_critic':optimizer_steps,'parameter_max_deltas':deltas,'buffer_max_deltas':buffer_deltas,'final_metrics':final,
  'optimizer_initialization':'fresh Adam for this pipeline smoke; actor weights from 4h, not matched-Adam continuation',
  'audit_initialization_reconstructed':bool(job.get('audit_endpoint')),'audit_repeated_physical_steps':0 if job.get('audit_endpoint') else None,
  'audit_repeated_optimizer_updates':0 if job.get('audit_endpoint') else None,
  'imagination_horizon':4,'training_entry':'go2w_supermimic.core.train.main (unmodified source)',
  'training_scope':'bounded pipeline smoke; flat reward learner on shared scene, no curriculum/DR; not terrain policy acceptance',
  'peak_torch_allocated_mib':torch.cuda.max_memory_allocated()/2**20,'candidate_checkpoint':str(checkpoint),'checkpoint_adopted':False}
