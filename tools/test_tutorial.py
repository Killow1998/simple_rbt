#!/usr/bin/env python3
"""Live C++ simulator -> native ROS wrapper -> external frontend, scored separately."""
import argparse, hashlib, json, math, os, socket, struct, subprocess, sys, time, uuid
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests/console'))
from observe_session import Observer

class Telemetry(Observer):
    def __init__(self,directory,role):self.poses=[];super().__init__(directory,role)
    def send(self,kind,value):
        data=json.dumps(value).encode();self.connections[0].sendall(struct.pack('<4sHHQII',b'SRBT',1,kind,0,len(data),0)+data)
    def read(self,connection):
        try:
            while not self.closed:
                kind,stamp,data=self.packet(connection)
                with self.lock:
                    if kind==11:self.metadata=json.loads(data)
                    if kind==7:self.poses.append([stamp*1e-9,*struct.unpack('<13d',data)[:7]])
        except (EOFError,OSError):pass

def score(truth,estimated,extrinsics):
    truth=np.asarray(truth);estimated=estimated[(estimated[:,0]>=truth[0,0])&(estimated[:,0]<=truth[-1,0])]
    t=estimated[:,0];rotation=Slerp(truth[:,0],Rotation.from_quat(truth[:,[5,6,7,4]]))(t)
    p=np.column_stack([np.interp(t,truth[:,0],truth[:,i]) for i in [1,2,3]])
    p+=rotation.apply(extrinsics['imu']['position'])
    iq=extrinsics['imu']['quaternion'];rotation=rotation*Rotation.from_quat([*iq[1:],iq[0]])
    e=Rotation.from_quat(estimated[:,[5,6,7,4]])
    align=rotation[0]*e[0].inv();offset=p[0]-align.apply(estimated[0,1:4])
    error=np.linalg.norm(align.apply(estimated[:,1:4])+offset-p,axis=1)
    angle=(rotation.inv()*align*e).magnitude()*180/np.pi
    return {'frames':len(t),'position_max_m':float(error.max()),'position_rmse_m':float(np.sqrt(np.mean(error**2))),
            'angle_max_deg':float(angle.max()),'alignment':'one fixed first-frame SE3; no scale/trajectory fitting',
            'first_stamp_s':float(t[0]),'last_stamp_s':float(t[-1]),
            'passed':bool(error.max()<1. and angle.max()<5.)}

def stats(pids):
    rows=[]
    for pid in pids:
        try:
            base=Path('/proc')/str(pid);fields=(base/'stat').read_text().split(')',1)[1].split()
            rows.append(((int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),int((base/'statm').read_text().split()[1])*os.sysconf('SC_PAGESIZE')/1024**2))
        except FileNotFoundError:pass
    return [sum(x[i] for x in rows) for i in range(2)]

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ros',choices=['ros1','ros2'],required=True)
    p.add_argument('--algorithm',choices=['fast-lio','fast-livo2'],required=True);p.add_argument('--robot',choices=['go2w','skydio_x2'],default='go2w')
    p.add_argument('--build',type=Path,default=ROOT/'build/flight-jammy');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--adapter-image',required=True);p.add_argument('--tutorial-image');p.add_argument('--duration',type=float,default=140)
    p.add_argument('--port',type=int,default=11992);p.add_argument('--domain',type=int,default=195);a=p.parse_args()
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False);build=a.build.resolve()
    session=Path('/tmp')/('srbt-tutorial-'+uuid.uuid4().hex[:10]);session.mkdir()
    stem=session.name;adapter_name=stem+'-ros';front_name=stem+'-front'
    image=a.tutorial_image or 'simple-rbt-tutorials:'+('noetic' if a.ros=='ros1' else 'humble')
    core_sha=hashlib.sha256((build/'simple-rbt-sim').read_bytes()).hexdigest()
    image_ids={name:subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',tag],text=True).strip()
               for name,tag in [('adapter',a.adapter_image),('frontend',image)]}
    config=json.loads((ROOT/('examples/tutorial-go2w.json' if a.robot=='go2w' else 'examples/tutorial-quadrotor.json')).read_text())
    config.update(source='program',start_immediately=False,duration_s=a.duration,settle_s=1.,output=str(out/'native'))
    (out/'session.json').write_text(json.dumps(config,indent=2)+'\n')
    environment=dict(os.environ,LIBGL_ALWAYS_SOFTWARE='1',EGL_PLATFORM='surfaceless',LP_NUM_THREADS='4',XDG_STATE_HOME=str(out/'state'))
    ros_env={'ROS_MASTER_URI':f'http://127.0.0.1:{a.port}','ROS_IP':'127.0.0.1','SIMPLE_RBT_START_MASTER':'1'} if a.ros=='ros1' else {'ROS_DOMAIN_ID':str(a.domain),'ROS_LOCALHOST_ONLY':'1'}
    ros_args=[item for k,v in ros_env.items() for item in ['--env',k+'='+v]]
    handles=[];processes=[];gui=program=None;resources=[];complete=0
    def start(name,command):
        f=(out/(name+'.log')).open('w');handles.append(f);proc=subprocess.Popen(command,env=environment,stdout=f,stderr=subprocess.STDOUT);processes.append(proc);return proc
    def docker(name,extra):return ['docker','run','--rm','--pull','never','--cpuset-cpus','0,2,4,6','--network','host','--name',name,*ros_args,*extra]
    try:
        core=start('core',[str(build/'simple-rbt-sim'),'--project-root',str(ROOT),'--session-config',str(out/'session.json'),'--session-dir',str(session)])
        deadline=time.monotonic()+15
        while not (session/'data.sock').exists():
            if core.poll() is not None or time.monotonic()>deadline:raise RuntimeError('core did not create IPC')
            time.sleep(.05)
        gui=Telemetry(session,'gui');program=Telemetry(session,'program');gui.wait(lambda m,v:m.get('session_state')=='config')
        adapter=start('adapter',docker(adapter_name,['--mount',f'type=bind,source={session},target=/session',a.adapter_image,'--session-dir','/session']))
        gui.wait(lambda m,v:m.get('connections',{}).get(a.ros,0)>0,timeout=20)
        calibration=start('calibration',[str(build/'simple-rbt-tutorial-config'),'--session-dir',str(session),'--data-root',str(ROOT),'--output',str(out/'frontend'),'--algorithm',a.algorithm,'--ros',a.ros])
        gui.send(2,{'op':'start'});gui.wait(lambda m,v:m.get('session_state')=='running')
        if calibration.wait(timeout=25):raise RuntimeError('calibration failed')
        frontend=start('frontend',docker(front_name,['--user',f'{os.getuid()}:{os.getgid()}','--mount',f'type=bind,source={out/"frontend"},target=/run',image,a.algorithm]))
        estimate=out/'frontend/estimated-poses.csv';ready=time.monotonic()+25
        while not estimate.exists() or len(estimate.read_text().splitlines())<5:
            if frontend.poll() is not None or time.monotonic()>ready:raise RuntimeError('frontend did not produce estimated poses')
            time.sleep(.1)
        pids={'core':[core.pid]}
        for name,container in [('adapter',adapter_name),('frontend',front_name)]:
            rows=subprocess.check_output(['docker','top',container,'-eo','pid'],text=True).splitlines()[1:];pids[name]=[int(row) for row in rows if row.strip()]
        before={k:stats(v) for k,v in pids.items()};begin=time.monotonic();last_resource=begin
        # Truth is used by this independent route driver/scorer, never by a frontend.
        goals=[(8,0,0),(8,.65,math.pi/2),(8,0,-math.pi/2),(8,0,math.pi),(-8,0,math.pi),(-8,0,0)]
        if a.robot=='skydio_x2':goals=[(0,0,0),(4,0,0),(4,.65,math.pi/2),(4,0,-math.pi/2),(4,0,math.pi),(-8,0,math.pi),(-8,0,0)]
        hold_until=8.;airborne=False
        while core.poll() is None:
            metadata,_=program.snapshot();stamp=metadata.get('stamp_ns',0)*1e-9;pose=metadata.get('base_pose')
            if not pose:time.sleep(.04);continue
            q=pose['quaternion'];x,y,z=pose['position'];yaw=math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2))
            command=[0.,0.,0.];flight={'frame':'world_enu','action':'hold','velocity':[0.,0.,0.],'yaw_rate':0.,'takeoff_height':1.}
            if a.robot=='skydio_x2' and stamp>8 and not airborne:
                flight['action']='takeoff'
                if metadata.get('flight',{}).get('state')=='hover' and z>.9:airborne=True;hold_until=stamp+1
            elif stamp>=hold_until and complete<len(goals):
                gx,gy,heading=goals[complete];dx,dy=gx-x,gy-y;angle=math.remainder(heading-yaw,2*math.pi)
                if math.hypot(dx,dy)<.08 and abs(angle)<.05:complete+=1;hold_until=stamp+1
                else:
                    wz=max(-.6,min(.6,1.5*angle));vx=max(-.5,min(.5,.8*dx));vy=max(-.25,min(.25,.8*dy))
                    if abs(angle)>.25:vx=vy=0.
                    if a.robot=='skydio_x2':
                        flight.update(action='velocity',velocity=[vx,vy,max(-.25,min(.25,1.2*((1.4 if complete in [2,3] else 1.)-z)))],yaw_rate=wz)
                    else:
                        command=[max(-.5,min(.5,.8*(math.cos(yaw)*dx+math.sin(yaw)*dy))),max(-.25,min(.25,.8*(-math.sin(yaw)*dx+math.cos(yaw)*dy))),wz]
                        if abs(angle)>.25:command[:2]=[0.,0.]
            elif a.robot=='skydio_x2' and complete==len(goals):flight['action']='land'
            try:program.send(18,flight) if a.robot=='skydio_x2' else program.send(3,{'velocity':command})
            except OSError:break
            now=time.monotonic()
            if now-last_resource>=1:
                resources.append({'elapsed_s':now-begin,'groups':{k:stats(v) for k,v in pids.items()}});last_resource=now
            if now-begin>a.duration*2:raise RuntimeError('simulation exceeded E2E wall deadline')
            time.sleep(.04)
        if core.wait(timeout=10):raise RuntimeError('core failed')
        subprocess.run(['docker','stop','--time','5',front_name],check=True,stdout=subprocess.DEVNULL)
        frontend.wait(timeout=10);adapter.wait(timeout=15)
        contract=json.loads((out/'frontend/contract.json').read_text());estimated=np.loadtxt(estimate,delimiter=',',skiprows=1,ndmin=2)
        with program.lock:truth=list(program.poses)
        np.savetxt(out/'truth-base.csv',truth,delimiter=',',header='stamp_s,x,y,z,qw,qx,qy,qz',comments='')
        result=score(truth,estimated,contract['extrinsics']);result.update(algorithm=a.algorithm,ros=a.ros,robot=a.robot,route_completed=complete,route_goals=len(goals))
        summary=json.loads((out/'native/summary.json').read_text());result['real_time_factor']=summary['real_time_factor']
        result['sensor_counts']={key:summary[key] for key in ['lidar_frames','imu_samples','camera_frames_per_sensor','camera_imu_samples']}
        result['estimate_max_gap_s']=float(np.diff(estimated[:,0]).max())
        if a.robot=='skydio_x2':
            result['landed']=program.snapshot()[0].get('flight',{}).get('state')=='grounded'
            result['passed']&=result['landed']
        result['resources']={k:{'mean_cores':(resources[-1]['groups'][k][0]-before[k][0])/resources[-1]['elapsed_s'],'peak_rss_mib':max(row['groups'][k][1] for row in resources)} for k in pids}
        result['passed']&=complete==len(goals) and result['real_time_factor']>.97 and result['last_stamp_s']>a.duration-2
        result['core_sha256']=core_sha;result['image_ids']=image_ids
        (out/'resources.json').write_text(json.dumps(resources,indent=2)+'\n');(out/'score.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
        if not result['passed']:raise RuntimeError('tutorial E2E failed')
    finally:
        for peer in [gui,program]:
            if peer:peer.close()
        for name in [front_name,adapter_name]:subprocess.run(['docker','rm','--force',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.terminate()
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
        for f in handles:f.close()
        for path in session.glob('*.sock'):path.unlink()
        session.rmdir()
if __name__=='__main__':main()
