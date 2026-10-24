#!/usr/bin/env python3
"""Audit transient voxel candidates; approximate visibility is not a human label.

Offline only. Inputs retain keyframe IDs and optimized frame-end LiDAR origins.
Never edits source clouds. A candidate exclusion mask is separate from evidence.
"""
import argparse,hashlib,json,os,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--points',type=Path,required=True);p.add_argument('--keyframes',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 args.output.mkdir(parents=True,exist_ok=False);os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:4]);began=time.monotonic()
 data=np.load(args.points);xyz,ids=data['xyz'],data['ids'];frames=json.loads(args.keyframes.read_text());cell=.1
 keys,first,inverse=np.unique(np.floor(xyz/cell).astype(np.int32),axis=0,return_index=True,return_inverse=True)
 points=xyz[first].astype(np.float64);n=len(points)
 pairs=np.unique(np.column_stack((inverse,ids)),axis=0)
 hits=np.bincount(pairs[:,0],minlength=n);start=np.full(n,np.inf);end=np.full(n,-np.inf);stamps=np.array([f['stamp_s'] for f in frames]);np.minimum.at(start,pairs[:,0],stamps[pairs[:,1]]);np.maximum.at(end,pairs[:,0],stamps[pairs[:,1]])
 free=np.zeros(n,dtype=np.uint16);free_first=np.full(n,-1,dtype=np.int32);free_last=np.full(n,-1,dtype=np.int32);tree=cKDTree(points)
 order=np.argsort(ids,kind='stable');sorted_ids=ids[order];breaks=np.searchsorted(sorted_ids,np.arange(len(frames)+1))
 for k,f in enumerate(frames):
  origin=np.asarray(f['lidar_origin']);scan=xyz[order[breaks[k]:breaks[k+1]]]-origin;ranges=np.linalg.norm(scan,axis=1);valid=ranges>.4;scan,ranges=scan[valid],ranges[valid];directions=scan/ranges[:,None]
  selected=np.asarray(tree.query_ball_point(origin,8),dtype=np.int64)
  if not len(selected):continue
  relative=points[selected]-origin;distance=np.linalg.norm(relative,axis=1);valid=distance>.6;selected,relative,distance=selected[valid],relative[valid],distance[valid]
  angular,nearest=cKDTree(directions).query(relative/distance[:,None],k=3,workers=4)
  # A narrow beam match plus 25 cm clearance; repeated in >=3 distinct
  # keyframes. Origins are frame-end approximations, retained in the report.
  near=(angular<.004)&(angular*distance[:,None]<.03)
  beyond=ranges[nearest]>distance[:,None]+.25
  clear=near[:,0]&beyond[:,0]&np.all(~near|beyond,axis=1)
  selected=selected[clear];free[selected]+=1;free_first[selected[free_first[selected]<0]]=k;free_last[selected]=k
  if k%100==0:print(k,'voxels',n,'clear evidence',int(sum(free>=3)),flush=True)
 suspect=(free>=3)&(free>=2*hits)&(end-start<2.)&(free_last-free_first>=5)
 stable=(hits>=3)&(end-start>=2.)&~suspect
 np.savez_compressed(args.output/'temporal-voxels.npz',xyz=points,voxel_keys=keys,hits=hits,free_votes=free,first_stamp=start,last_stamp=end,suspect=suspect,stable=stable)
 # No original point is deleted: this separate file is a mask for review.
 report=dict(source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [args.points,args.keyframes]},
  source_points=len(xyz),voxels=n,voxel_m=cell,keyframes=len(frames),suspect_voxels=int(sum(suspect)),stable_supported_voxels=int(sum(stable)),
  raw_points_in_suspect_voxels=int(sum(suspect[inverse])),runtime_s=time.monotonic()-began,
  classifier='suspect requires >=3 free-space votes, >=2x occupied keyframe votes, occupied duration <2 s, free-space span >=5 keyframes',
  status='candidate mask only; no original PCD change; not certified human removal',
  limitations=['Single-scan deskewed cloud; frame-end origins approximate per-point origins.', 'SLAM misregistration, thin structures and sparse sampling can mimic dynamics.', 'No human identity or complete dynamic-object segmentation is claimed.', 'Unknown/occluded space is not free space.'])
 (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
