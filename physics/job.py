"""Optional isolated Warp process; native C++ interaction does not import this module."""
import argparse,json,os,sys,time,traceback,hashlib,resource
from importlib.metadata import version
from pathlib import Path

def write(path,data):
 tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(path)

def main():
 p=argparse.ArgumentParser();p.add_argument('job',type=Path);p.add_argument('--contract',type=Path,required=True);p.add_argument('--task',choices=['batch','train-smoke'],required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 job=json.loads(a.job.read_text());out=a.output;out.mkdir(exist_ok=False)
 started=time.monotonic();status={'state':'running','physics_backend':'warp','task':a.task,'output':str(out)};write(out/'status.json',status)
 try:
  sys.path.insert(0,str(Path(job['reference']).resolve()))
  import torch
  torch.set_num_threads(2)
  if a.task=='batch':
   from warp_batch import run
  else:
   from train_smoke import run
  result=run(a.contract,job,out);result.update(elapsed_s=time.monotonic()-started,
   contract_sha256=hashlib.sha256(a.contract.read_bytes()).hexdigest(),checkpoint_sha256=hashlib.sha256(Path(job['checkpoint']).read_bytes()).hexdigest())
  import warp as wp,mujoco
  result.update(mujoco_version=mujoco.__version__,warp_version=wp.__version__,mjlab_version=version('mjlab'),mujoco_warp_version=version('mujoco-warp'),
   peak_warp_pool_mib=wp.get_mempool_used_mem_high(job['device'])/2**20,peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
  result['resource_checks']={'ram':result['peak_rss_mib']<8192,'allocator_pools':result['peak_warp_pool_mib']+result['peak_torch_allocated_mib']<8192}
  result['passed'] &= all(result['resource_checks'].values())
  write(out/'report.json',result);status.update(state='passed' if result['passed'] else 'failed',report=str(out/'report.json'))
 except Exception as e:
  status.update(state='failed',error=str(e));traceback.print_exc()
 write(out/'status.json',status);print(json.dumps(status),flush=True)
 return 0 if status['state']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
