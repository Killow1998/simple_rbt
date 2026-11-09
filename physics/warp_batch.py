"""GPU physics adapter. mjlab owns Warp step/reset; no simulator fallback."""
import hashlib,json,math
from pathlib import Path
import numpy as np
import torch
import mujoco
import warp as wp
from mjlab.sim import Simulation,SimulationCfg,MujocoCfg
from mjlab.entity.data import compute_velocity_from_cvel
from go2w_supermimic.core.policy import load_actor,push_history,frame


def solver_cfg(s):
 return MujocoCfg(timestep=s['timestep'],integrator={0:'euler',3:'implicitfast'}[s['integrator']],
  cone={0:'pyramidal',1:'elliptic'}[s['cone']],solver={0:'pgs',1:'cg',2:'newton'}[s['solver']],
  gravity=tuple(s['gravity']),impratio=s['impratio'],iterations=s['iterations'],ls_iterations=s['ls_iterations'],
  tolerance=s['tolerance'],ls_tolerance=s['ls_tolerance'],ccd_iterations=s['ccd_iterations'])


def model_audit(model,contract):
 errors={}
 visual_size=0.
 for k,v in contract['model_fields'].items():
  actual=np.asarray(getattr(model,k),dtype=float);expected=np.asarray(v,dtype=float).reshape(actual.shape)
  if k=='geom_size':
   collision=(model.geom_contype|model.geom_conaffinity)!=0
   visual_size=float(np.max(np.abs(actual[~collision]-expected[~collision])))
   actual,expected=actual[collision],expected[collision]
  errors[k]=float(np.max(np.abs(actual-expected))) if actual.size else 0.
 for k in ('disableflags','enableflags'):
  if k in contract['solver']:errors['solver.'+k]=abs(int(getattr(model.opt,k))-contract['solver'][k])
 return {'passed':max(errors.values())<=1e-6,'errors':errors,'visual_only_mesh_aabb_difference_m':visual_size}


class WarpBatch:
 def __init__(self,contract_path,checkpoint,worlds,device):
  self.contract=json.loads(Path(contract_path).read_text());c=self.contract
  path=Path(contract_path).parent/c['model']
  if hashlib.sha256(path.read_bytes()).hexdigest()!=c['model_sha256']:raise ValueError('Shared model changed')
  self.model=mujoco.MjModel.from_xml_path(str(path));self.audit=model_audit(self.model,c)
  if not self.audit['passed']:raise ValueError('Model fields differ: '+str(self.audit))
  self.sim=Simulation(worlds,SimulationCfg(mujoco=solver_cfg(c['solver']),nconmax=512,njmax=2048),model=self.model,device=device)
  if not self.sim.wp_device.is_cuda:raise ValueError('Real CUDA Warp device required')
  self.device=device;self.worlds=worlds;self.data=self.sim.data
  cfg=c['controller'];names=cfg['joint_names'];j=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in names]
  self.q=self.model.jnt_qposadr[j].tolist();self.v=self.model.jnt_dofadr[j].tolist()
  self.base=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,cfg['base_body'])
  self.actuators=[]
  for joint in j:self.actuators.append(int(np.where(self.model.actuator_trnid[:,0]==joint)[0][0]))
  self.to_env=cfg['env_action_order'];self.to_canonical=np.argsort(self.to_env).tolist()
  t=lambda x:torch.tensor(x,device=device,dtype=torch.float32)
  self.default=t(cfg['default_dof_pos']);self.scale=t(cfg['action_scale']);self.offset=t(cfg['action_offset'])
  self.previous=torch.zeros(worlds,16,device=device);self.pending=torch.zeros(worlds,3,device=device)
  self.history=torch.zeros(worlds,245,device=device);self.first=torch.ones(worlds,dtype=torch.bool,device=device)
  self.initial_q=t(c['initial']['qpos']);self.initial_v=t(c['initial']['qvel']);self.initial_ctrl=t(c['initial']['ctrl'])
  ckpt=torch.load(checkpoint,map_location=device,weights_only=True);self.actor=load_actor(ckpt,device)
  if self.actor.actor_wheel_vel or self.actor.history!=5:raise ValueError('Expected current 5x49 actor')
  self.reset(torch.arange(worlds,device=device))

 def reset(self,ids):
  self.sim.reset(ids);self.data.qpos[ids]=self.initial_q;self.data.qvel[ids]=self.initial_v;self.data.ctrl[ids]=self.initial_ctrl
  self.previous[ids]=0;self.pending[ids]=0;self.history[ids]=0;self.first[ids]=True;self.sim.forward()

 def state(self):
  d=self.data;b=self.base;r=d.xmat[:,b].reshape(-1,3,3)
  vel=compute_velocity_from_cvel(d.xpos[:,b],d.subtree_com[:,self.model.body_rootid[b]],d.cvel[:,b])
  linear=torch.bmm(r.transpose(1,2),vel[:,:3,None]).squeeze(-1)
  angular=torch.bmm(r.transpose(1,2),vel[:,3:,None]).squeeze(-1)
  return torch.cat((d.xpos[:,b,2:3],-r[:,2,:],linear,angular,d.qpos[:,self.q[:12]]-self.default[:12],d.qvel[:,self.v]),-1)

 def observe(self):
  latest=frame(self.state(),self.previous[:,self.to_env],self.pending,torch.zeros(self.worlds,12,device=self.device))
  shifted=push_history(self.history,latest,5)
  repeated=push_history(torch.cat([latest[:,sum((3,3,12,12,16,3)[:i]):sum((3,3,12,12,16,3)[:i+1])].repeat(1,5) for i in range(6)],-1),latest,5)
  self.history=torch.where(self.first[:,None],repeated,shifted);self.first[:]=False
  return self.history

 @torch.no_grad()
 def step(self,command):
  self.consumed=self.pending.clone();obs=self.observe();self.pending.copy_(command);action=self.actor(obs)
  self.data.ctrl[:,self.actuators]=self.offset+self.scale*action
  self.previous.copy_(action)
  for _ in range(4):self.sim.step()
  self.sim.forward()
  return obs,action


@torch.no_grad()
def run(contract_path,job,out):
 import onnxruntime as ort
 batch=WarpBatch(contract_path,job['checkpoint'],job['worlds'],job['device']);c=batch.contract
 cpu=json.loads((Path(contract_path).parent/'cpu-frames.json').read_text());errors={'observation':0.,'actor':0.,'onnx':0.}
 options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
 actor=ort.InferenceSession(c['controller']['policy'],options,providers=['CPUExecutionProvider'])
 for row in cpu:
  batch.data.qpos[:]=torch.tensor(row['qpos'],device=batch.device);batch.data.qvel[:]=torch.tensor(row['qvel'],device=batch.device);batch.sim.forward()
  obs=batch.observe();a=batch.actor(obs);onnx=actor.run(None,{actor.get_inputs()[0].name:obs[:1].cpu().numpy()})[0]
  errors['observation']=max(errors['observation'],float((obs[0]-torch.tensor(row['observation'],device=batch.device)).abs().max()))
  errors['actor']=max(errors['actor'],float((a[0]-torch.tensor(row['action'],device=batch.device)).abs().max()))
  errors['onnx']=max(errors['onnx'],float(np.max(np.abs(onnx-a[:1].cpu().numpy()))))
  batch.previous[:]=torch.tensor(row['action'],device=batch.device);batch.pending[:]=torch.tensor(row['command'],device=batch.device)
 batch.reset(torch.arange(batch.worlds,device=batch.device));rows=[];reset_checks={};origin=batch.data.xpos[:,batch.base].clone()
 for step in range(job['policy_steps']):
  command=torch.zeros(batch.worlds,3,device=batch.device)
  if 50<=step<200:
   command[:,0]=.3;command[1::4]=torch.tensor([0.,0.,.3],device=batch.device)
   command[2::4]=torch.tensor([-.2,0.,0.],device=batch.device);command[3::4]=torch.tensor([0.,.15,0.],device=batch.device)
  if step==150:
   old=batch.data.qpos[1:].clone();ids=torch.tensor([0],device=batch.device);batch.reset(ids)
   reset_checks={'qpos':bool(torch.allclose(batch.data.qpos[0],batch.initial_q)),'qvel':bool(torch.allclose(batch.data.qvel[0],batch.initial_v)),
    'time':float(batch.data.time[0])==0,'previous_action':not bool(batch.previous[0].any()),'command_latch':not bool(batch.pending[0].any()),
    'history':not bool(batch.history[0].any()),'other_worlds_unchanged':bool(torch.equal(old,batch.data.qpos[1:]))}
  obs,a=batch.step(command);s=batch.state()
  if not torch.isfinite(s).all():raise RuntimeError('Nonfinite GPU state')
  contacts=batch.sim.wp_data.contact;n=int(batch.sim.wp_data.nacon.numpy()[0])
  world=contacts.worldid.numpy()[:n];geom=contacts.geom.numpy()[:n];address=contacts.efc_address.numpy()[:n]
  force=batch.sim.wp_data.efc.force.numpy()
  contact_rows=[{'world':int(w),'geom':g.tolist(),'dist':float(dist),'constraint_address':int(adr[0]),
    'constraint_force':force[int(w),int(adr[0]):int(adr[0])+int(batch.model.geom_condim[g[0]])].tolist() if adr[0]>=0 else []}
    for w,g,dist,adr in zip(world,geom,contacts.dist.numpy()[:n],address)]
  rows.append({'step':step,'time':batch.data.time.cpu().tolist(),'command':command.cpu().tolist(),'observed_command':batch.consumed.cpu().tolist(),'state':s.cpu().tolist(),
   'qpos':batch.data.qpos.cpu().tolist(),'qvel':batch.data.qvel.cpu().tolist(),'action':a.cpu().tolist(),'ctrl':batch.data.ctrl.cpu().tolist(),
   'actuator_force':batch.data.actuator_force.cpu().tolist(),'qfrc_actuator':batch.data.qfrc_actuator.cpu().tolist(),
   'contacts':contact_rows})
 tilt=max(math.acos(max(-1.,min(1.,-s[3]))) for r in rows for s in r['state'])
 movement=(batch.data.xpos[:,batch.base]-origin).cpu().tolist()
 checks={'model':batch.audit['passed'],'observation':errors['observation']<=1e-5,'actor':errors['actor']<=4e-5,'onnx':errors['onnx']<=4e-5,
         'reset':all(reset_checks.values()),'upright':tilt<c['termination']['tilt_rad'],'motion':max(abs(x) for xyz in movement for x in xyz[:2])>.05}
 (out/'warp-trajectory.json').write_text(json.dumps(rows)+'\n')
 return {'passed':all(checks.values()),'checks':checks,'model_audit':batch.audit,'same_state_errors':errors,'reset_checks':reset_checks,
  'physics_backend':'warp','physics_device':str(batch.sim.wp_device),'device_name':batch.sim.wp_device.name,'mujoco_version':mujoco.__version__,'warp_version':wp.__version__,
  'worlds':batch.worlds,'policy_steps':len(rows),'physical_world_steps':len(rows)*4*batch.worlds,'max_tilt_rad':tilt,'final_displacement':movement,
  'peak_torch_allocated_mib':torch.cuda.max_memory_allocated()/2**20,'sensor_backend':'none (headless batch)'}
