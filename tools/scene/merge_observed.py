#!/usr/bin/env python3
"""Merge qualified coplanar patches without expanding their observed footprint.

Offline preparation only. Requires numpy, scipy, shapely 2.1.2. Input patches
and per-patch audits come from the closed-cloud fitting stage; no gate is reset.
"""
import argparse, hashlib, json
from collections import defaultdict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from shapely import Polygon, LineString, union_all
from shapely.geometry.polygon import orient
from shapely.ops import split


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def strict_regions(polygon):
    """Split at point-touching rings, which OGC permits but scene v2 forbids."""
    rings=[polygon.exterior,*polygon.interiors]
    for i,ring in enumerate(rings):
        for other in rings[:i]:
            contact=ring.intersection(other)
            if contact.is_empty: continue
            point=contact if contact.geom_type=='Point' else list(contact.geoms)[0]
            x,y=point.x,point.y;lo=polygon.bounds;extent=max(lo[2]-lo[0],lo[3]-lo[1])+1
            for dx,dy in [(0,extent),(extent,0)]:
                pieces=list(split(polygon,LineString([(x-dx,y-dy),(x+dx,y+dy)])).geoms)
                if len(pieces)>1:
                    return [region for piece in pieces for region in strict_regions(piece)]
            raise ValueError('Could not split touching polygon rings without inventing space')
    return [polygon]


def reconstruct(patches_path, provenance_path, output):
    output.mkdir(parents=True, exist_ok=False)
    patches = json.loads(patches_path.read_text())
    source = json.loads(provenance_path.read_text())['objects']
    groups = defaultdict(list)
    for patch in patches:
        audit = source[patch['id']]
        groups[(patch['floor'], audit['plane_id'], audit['semantic'])].append(patch)
    scene = dict(format='sprobo.scene', version=2, id='observed-building-v2',
                 name='Observed building / 观测建筑', units=dict(length='m', angle='rad'),
                 coordinate_system='right_handed_z_up', geometry_templates={}, objects=[])
    report = dict(inputs={str(p): digest(p) for p in (patches_path, provenance_path)},
                  method='Exact union on each fitted plane; 1e-8 m numerical snap; no convex hull, closing, buffering or new surface coverage.',
                  limitations='Sampling holdout shares optimized SLAM poses. Collision thickness is assumed, not observed. Gaps stay unknown. No cross-storey route acceptance.',
                  objects={}, input_patches=len(patches))
    def node(key, name, kind='group', parent='world', position=(0,0,0), quaternion=(1,0,0,0), **kw):
        result=dict(id=key,name=name,kind=kind,parent=parent,
                    pose=dict(position_m=list(position),quaternion_wxyz=list(quaternion)),**kw)
        scene['objects'].append(result)
        return result
    for level in sorted({key[0] for key in groups}):
        node('level_'+level,level,semantic='storey')
    sum_before = sum_after = 0.
    for (level, plane, semantic), group in sorted(groups.items()):
        anchor = source[group[0]['id']]
        origin = np.array(anchor['observed_face_center_map_m'])
        q = np.array(group[0]['quaternion'])
        axes = Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        polygons = []
        for patch in group:
            a = source[patch['id']]
            center = np.array(a['observed_face_center_map_m'])
            rotation = Rotation.from_quat(np.array(patch['quaternion'])[[1,2,3,0]]).as_matrix()
            corners = np.array([[-1,-1],[1,-1],[1,1],[-1,1]]) * np.array(patch['size'][:2])/2
            points = center + corners @ rotation[:,:2].T
            polygons.append(Polygon(((points-origin) @ axes)[:,:2]))
        merged = union_all(polygons, grid_size=1e-8)
        pieces = [merged] if merged.geom_type=='Polygon' else list(merged.geoms)
        pieces = [region for piece in pieces for region in strict_regions(piece)]
        assert merged.is_valid
        before = union_all(polygons).area
        assert abs(merged.area-before)<1e-5
        sum_before += before; sum_after += merged.area
        plane_id = f'plane_{level}_{plane}_{semantic}'
        node(plane_id, f'{semantic} plane {plane}', parent='level_'+level,position=origin,quaternion=q)
        for index, polygon in enumerate(pieces):
            # Removes collinear vertices only, preserving topology and area.
            polygon = orient(polygon.simplify(1e-9,preserve_topology=True),sign=1)
            key=f'{plane_id}_region_{index}'
            geometry=dict(type='planar_region',outer_xy_m=[list(p) for p in polygon.exterior.coords[:-1]],
                          holes_xy_m=[[list(p) for p in ring.coords[:-1]] for ring in polygon.interiors],thickness_m=.04)
            node(key,f'{semantic} {plane} / {index}','primitive',plane_id,geometry=geometry,
                 semantic=semantic,evidence=dict(basis='fitted',report_ref='reconstruction.json#objects/'+key,
                  note='Boundary is restricted to qualified observed support; thickness is an explicit 0.04 m collision approximation.'))
            ids=[p['id'] for p,poly in zip(group,polygons) if polygon.intersection(poly).area>1e-8]
            audit=[source[i] for i in ids]
            report['objects'][key]=dict(source_patch_ids=ids,plane_coefficients=anchor['plane_coefficients'],area_m2=polygon.area,
                source_keyframe_ids=sorted({k for a in audit for k in a['source_keyframe_ids']}),
                source_patch_holdout_p95_max_m=max(a['sampling_holdout_plane_p50_p95_max_m'][1] for a in audit),
                source_patch_coverage_p95_max_m=max(a['sampling_holdout_coverage_p95_max_m'][0] for a in audit),
                unknown_regions_filled=0)
    report.update(plane_groups=len(groups),surface_regions=len(report['objects']),
                  input_union_area_sum_m2=sum_before,output_area_sum_m2=sum_after,added_area_m2=sum_after-sum_before)
    (output/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    (output/'reconstruction.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='objects'},indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--patches',type=Path,required=True);parser.add_argument('--provenance',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    reconstruct(args.patches,args.provenance,args.output)
