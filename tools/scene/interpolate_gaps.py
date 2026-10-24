#!/usr/bin/env python3
"""Join short sampling gaps in qualified plane regions, with an evidence audit.

Offline preparation only; numpy, scipy and shapely 2.1.2 are required.
Large unobserved interiors are not reconstructed by this operation.
"""
import argparse
import copy
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from shapely import Polygon, union_all
from shapely.geometry.polygon import orient

from merge_observed import digest, strict_regions
from surface_support import SurfaceSupport


def polygons(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == 'Polygon':
        return [geometry]
    return [p for g in getattr(geometry, 'geoms', ()) for p in polygons(g)]


def reconstruct(scene_path, cloud, normals, output, radius):
    output.mkdir(parents=True, exist_ok=False)
    scene = json.loads(scene_path.read_text())
    nodes = {o['id']: o for o in scene['objects']}
    groups = defaultdict(list)
    for obj in scene['objects']:
        if (obj['id'].startswith('plane_') and obj.get('semantic') in ('floor', 'wall', 'ceiling')
                and obj.get('geometry', {}).get('type') == 'planar_region'):
            assert obj['pose'] == dict(position_m=[0, 0, 0], quaternion_wxyz=[1, 0, 0, 0])
            groups[obj['parent']].append(obj)
    support = SurfaceSupport(cloud, normals)
    report = dict(inputs={str(p): digest(p) for p in (scene_path, cloud, normals)},
                  radius_m=radius, probe_spacing_m=.05, groups={},
                  method='Bounded polygon closing proposes sampling gaps. Each added component needs repeated, normal-aligned plane support, independent keyframe coverage and no two crossing rays. Existing surface is never removed.',
                  limitations='Added area is local interpolation, not directly measured surface. Source keyframes share optimized SLAM poses. Large unknown regions remain. Unchanged input objects retain their original evidence; this is not cross-storey acceptance.')
    replaced = set()
    replacements = []
    for key, objects in groups.items():
        parent = nodes[key]
        # The fitted-plane importer puts plane frames under identity storey groups.
        ancestor = nodes[parent['parent']]
        assert ancestor['parent'] == 'world'
        assert ancestor['pose'] == dict(position_m=[0, 0, 0], quaternion_wxyz=[1, 0, 0, 0])
        origin = np.array(parent['pose']['position_m'])
        q = np.array(parent['pose']['quaternion_wxyz'])
        axes = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
        before = union_all([Polygon(o['geometry']['outer_xy_m'], o['geometry']['holes_xy_m']) for o in objects], grid_size=1e-8)
        closed = before.buffer(radius, join_style='mitre').buffer(-radius, join_style='mitre')
        proposed = [p for p in polygons(closed.difference(before)) if p.area > 1e-8]
        if not proposed:
            continue
        evidence = support.on_plane(origin, axes, before.bounds)
        accepted, checks = [], []
        for p in proposed:
            check = evidence.check(p)
            # Mitred corners must not expand beyond the declared interpolation band.
            if not before.buffer(radius+1e-7).covers(p):
                check['accepted'] = False
                check['rejected_by'].append('beyond_local_gap_band')
            checks.append(check)
            if check['accepted']:
                accepted.append(p)
        after = union_all([before, *accepted], grid_size=1e-8)
        assert before.difference(after).area < 1e-7
        regions = [r for p in polygons(after) for r in strict_regions(p)]
        report['groups'][key] = dict(before_regions=len(objects), after_regions=len(regions),
                                    before_area_m2=before.area, added_area_m2=after.area-before.area,
                                    checks=checks, changed=bool(accepted))
        print(key, len(objects), '->', len(regions), 'added', round(after.area-before.area, 4), flush=True)
        if not accepted:
            continue
        replaced.update(o['id'] for o in objects)
        for i, p in enumerate(regions):
            p = orient(p.simplify(1e-9, preserve_topology=True), sign=1)
            obj = copy.deepcopy(objects[0])
            obj['id'] = key+'_continuous_'+str(i)
            obj['name'] = objects[0]['semantic']+' '+key+' / '+str(i)
            obj['geometry'].update(outer_xy_m=np.asarray(p.exterior.coords)[:-1].tolist(),
                                   holes_xy_m=[np.asarray(h.coords)[:-1].tolist() for h in p.interiors])
            obj['evidence'] = dict(basis='fitted', report_ref='surface-interpolation.json#groups/'+key,
                                   note='Continuous fitted plane; only locally supported sampling gaps are interpolated. Numerical collision thickness remains 0.04 m.')
            replacements.append(obj)
    scene['objects'] = [o for o in scene['objects'] if o['id'] not in replaced] + replacements
    report['summary'] = dict(replaced_regions=len(replaced), output_regions=len(replacements),
                            added_area_m2=sum(g['added_area_m2'] for g in report['groups'].values()),
                            added_components=sum(c['accepted'] for g in report['groups'].values() for c in g['checks']),
                            scene_nodes=len(scene['objects']))
    (output/'scene.json').write_text(json.dumps(scene, indent=2)+'\n')
    (output/'surface-interpolation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report['summary'], indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('scene', 'cloud', 'normals', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--radius', type=float, default=.13)
    args = parser.parse_args()
    if not 0 < args.radius <= .13:
        parser.error('Local interpolation radius must be in (0, 0.13] m.')
    reconstruct(args.scene, args.cloud, args.normals, args.output, args.radius)
