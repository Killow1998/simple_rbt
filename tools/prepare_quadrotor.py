#!/usr/bin/env python3
"""Prepare the pinned Menagerie Skydio X2; Python is offline preparation only."""
import hashlib,json,urllib.request,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
REV='71f066ad0be9cd271f7ed58c030243ef157af9f4'
SHA256={'x2.xml': '5e8f14afe735d07e0b299444d68927058a757149761313c2f0d2f88264c2ae7b', 'LICENSE': 'd42cae9a0f292b7a713597ba26e1e1e23d50df906c488f0d36fd3538af200bc1', 'assets/X2_lowpoly.obj': '4d6df3d7b26676a6ade8cbc320599961321ac3e3c52474d7c98dd7adc8152411', 'assets/X2_lowpoly_texture_SpinningProps_1024.png': '24153733dd01f9ddd37a3c79fad5e24ac876228812e3a8e95f60ccd8cedb66e1'}
BASE=f'https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{REV}/skydio_x2/'
def split_rotor_visual(source, target):
    # The pinned OBJ has four disconnected swept discs. Separate their material
    # from the body so opaque white texture does not resemble solid plates.
    lines=source.read_text().splitlines();vertices=[];faces=[];parent={}
    def find(point):
        parent.setdefault(point,point)
        if parent[point]!=point:parent[point]=find(parent[point])
        return parent[point]
    for line in lines:
        if line.startswith('v '):vertices.append(tuple(map(float,line.split()[1:4])))
        if line.startswith('f '):
            face=[int(x.split('/')[0])-1 for x in line.split()[1:]];faces.append((line,face))
            root=find(vertices[face[0]])
            for v in face[1:]:parent[find(vertices[v])]=root
    components={}
    for line,face in faces:components.setdefault(find(vertices[face[0]]),[]).append((line,face))
    discs=[];body=[];count=0
    for group in components.values():
        points=[vertices[v] for _,f in group for v in f]
        size=[max(p[i] for p in points)-min(p[i] for p in points) for i in range(3)]
        rotor=size[0]>20 and size[2]>20 and size[1]<.2
        if rotor:count+=1
        (discs if rotor else body).extend(line for line,_ in group)
    if count!=4:raise ValueError('Expected four pinned rotor-disc components')
    records={key:[line for line in lines if line.startswith(key+' ')] for key in ['v','vt','vn']}
    for name,geometry in [('X2_body.obj',body),('X2_rotor_discs.obj',discs)]:
        parsed=[[[int(v) for v in token.split('/')] for token in line.split()[1:]] for line in geometry]
        remap=[];result=[]
        for i,key in enumerate(['v','vt','vn']):
            indices=sorted({v[i] for face in parsed for v in face});remap.append({old:new+1 for new,old in enumerate(indices)})
            result.extend(records[key][old-1] for old in indices)
        result.extend('f '+' '.join('/'.join(str(remap[i][value]) for i,value in enumerate(v)) for v in face) for face in parsed)
        (target/name).write_text('\n'.join(result)+'\n')

def prepare(source):
    target=ROOT/'assets/visual/skydio_x2';target.mkdir(parents=True,exist_ok=True)
    entries={}
    for name in ['x2.xml','LICENSE','assets/X2_lowpoly.obj','assets/X2_lowpoly_texture_SpinningProps_1024.png']:
        path=source/name
        if not path.exists():
            path.parent.mkdir(parents=True,exist_ok=True)
            with urllib.request.urlopen(BASE+name,timeout=30) as response:path.write_bytes(response.read())
        data=path.read_bytes();entries[name]=hashlib.sha256(data).hexdigest()
        if entries[name]!=SHA256[name]:raise ValueError(f'Pinned asset checksum mismatch: {name}')
        if name not in ['x2.xml','assets/X2_lowpoly.obj']:(target/Path(name).name).write_bytes(data)
    split_rotor_visual(source/'assets/X2_lowpoly.obj',target)
    tree=ET.parse(source/'x2.xml');r=tree.getroot();compiler=r.find('compiler');compiler.attrib.pop('assetdir',None)
    r.find("default/default/default[@class='collision']/geom").set('rgba','0 0 0 0')
    compiler.set('angle','radian');r.find('option').set('timestep','.005');r.find('option').set('integrator','implicitfast');r.find('option').set('gravity','0 0 -9.80665')
    for e in r.find('asset'):
        if 'file' in e.attrib:e.set('file','@PROJECT_ROOT@/assets/visual/skydio_x2/'+e.get('file'))
    r.find('asset/mesh').set('file','@PROJECT_ROOT@/assets/visual/skydio_x2/X2_body.obj')
    r.find('asset/mesh').set('name','X2_lowpoly')
    ET.SubElement(r.find('asset'),'mesh',name='rotor_discs',file='@PROJECT_ROOT@/assets/visual/skydio_x2/X2_rotor_discs.obj',scale='.01 .01 .01')
    ET.SubElement(r.find('asset'),'material',name='rotor_sweep',rgba='.12 .15 .18 .28',specular='.1')
    ET.SubElement(r.find('worldbody/body'),'geom',mesh='rotor_discs',material='rotor_sweep',attrib={'class':'visual','quat':'0 0 1 1'})
    body=r.find('worldbody/body');body.set('pos','0 0 .03');body.find('freejoint').set('name','root')
    for name,pos in [('base_origin','0 0 0'),('lidar','0 0 .18'),('mid360_imu','.011 .02329 .13588'),('camera_mount','.18 0 .1')]:
        ET.SubElement(body,'site',name=name,pos=pos,size='.002',rgba='0 0 0 0')
    for tag,name,site in [('velocimeter','base_linvel','base_origin'),('gyro','base_angvel','base_origin'),('gyro','mid360_gyro','mid360_imu'),('accelerometer','mid360_accelerometer','mid360_imu')]:
        ET.SubElement(r.find('sensor'),tag,name=name,site=site)
    key=r.find('keyframe/key');key.set('name','ground');key.set('qpos','0 0 .03 1 0 0 0');key.set('ctrl','0 0 0 0')
    ET.indent(tree,space='  ');model=ROOT/'assets/models/skydio_x2-flight_pid.xml';tree.write(model,encoding='unicode')
    provenance=dict(repository='https://github.com/google-deepmind/mujoco_menagerie',revision=REV,source_sha256=entries,changes=['Rotor swept discs retain source geometry with a separate translucent material; original source pinned by hash; body geometry and texture unchanged.','200 Hz implicitfast physics; original four rotor thrust/reaction-torque actuators and collision envelopes retained.','Ground initial state; common sensor sites and configurable rigid device mounts.','C++ controller is simulator-specific, not Skydio firmware. Nominal sensor payloads are added separately.'])
    provenance['parts']=[{'name':'Skydio X2 visual model','files':[{'file':str((target/name).relative_to(ROOT)),'sha256':hashlib.sha256((target/name).read_bytes()).hexdigest()} for name in ['X2_body.obj','X2_rotor_discs.obj','X2_lowpoly_texture_SpinningProps_1024.png','LICENSE']]}]
    (target/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    p=ROOT/'assets/catalog.json';c=json.loads(p.read_text());c['models']['skydio_x2-flight_pid']=dict(file='models/'+model.name,sha256=hashlib.sha256(model.read_bytes()).hexdigest());c['controllers']['skydio_x2-flight_pid']=dict(source='visual/skydio_x2/provenance.json',feedback='simulation_state',velocity_frame='world_enu',vertical_limit_m_s=.6,yaw_rate_limit_rad_s=1.,max_tilt_deg=25.,rotor_max_thrust_n=13.)
    p.write_text(json.dumps(c,indent=2)+'\n')
if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,default=ROOT/'build/skydio-x2-source');prepare(parser.parse_args().source)
