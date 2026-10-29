#!/usr/bin/env python3
"""Offline tutorial preparation from locked Git commits; no private image."""
import argparse, hashlib, io, json, shutil, subprocess, tarfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def prepare(args):
    context=args.output.resolve();context.mkdir(parents=True,exist_ok=False)
    locks=json.loads((ROOT/'tutorials/sources.json').read_text())
    for name,path in [('fast-lio',args.fast_lio),('fast-livo2',args.fast_livo2)]:
        entry=locks[name]
        revision=subprocess.check_output(['git','-C',str(path),'rev-parse',entry['revision']+'^{commit}'],text=True).strip()
        if revision!=entry['revision']:raise ValueError(f'{name}: expected {entry["revision"]}, found {revision}')
        # Export committed objects, leaving active branches and pending edits untouched.
        archive=subprocess.check_output(['git','-C',str(path),'archive',revision])
        entry['archive_sha256']=hashlib.sha256(archive).hexdigest()
        target=context/'sources'/entry['directory'];target.mkdir(parents=True)
        with tarfile.open(fileobj=io.BytesIO(archive)) as source:
            # Some Python 3.10 backports resolve symlinks from the archive root.
            # Validate relative links from their own parent, after normal files.
            source.extractall(target,members=[m for m in source if not m.issym()],filter='data')
            for member in source:
                if not member.issym():continue
                link=target/member.name
                if not link.resolve().is_relative_to(target) or not (link.parent/member.linkname).resolve().is_relative_to(target):
                    raise ValueError('Source link escapes checkout: '+member.name)
                link.symlink_to(member.linkname)
    for source,target in [('tutorials/Dockerfile','Dockerfile'),('tutorials/entrypoint.sh','entrypoint.sh'),
                          ('tutorials/observer','observer'),('interfaces/livox_ros_driver2','interfaces'),
                          ('third_party/nlohmann','nlohmann'),('deployment/ros2/udp.xml','udp.xml')]:
        p=ROOT/source;q=context/target
        if p.is_dir():shutil.copytree(p,q)
        else:shutil.copy2(p,q)
    (context/'sources.json').write_text(json.dumps(locks,indent=2)+'\n')
    print('Self-contained, portable build context: '+str(context),flush=True)
    if args.build:
        image='simple-rbt-tutorials:'+args.ros
        command=['docker','build','--pull=false','-t',image]
        if args.ros=='humble':
            command+=['--build-arg','ENTRY=humble','--build-arg','ROS_IMAGE=ros:humble-ros-base-jammy@sha256:6b4abb5340e430c632bcf4710fb34996df4f80e502993e5755f8fc59a39c7b2e']
        subprocess.run(command+[str(context)],check=True)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fast-lio',type=Path,required=True);p.add_argument('--fast-livo2',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--ros',choices=['noetic','humble'],default='noetic')
    p.add_argument('--build',action='store_true');prepare(p.parse_args())
