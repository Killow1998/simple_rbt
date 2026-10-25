#!/usr/bin/env python3
"""Apply reviewed, explicitly inferred planar surfaces to a v2 scene.

Offline preparation: the plan owns finite outlines, source IDs and one plane per
surface group. This exporter does not discover walls or silently fill unknown space.
Requires numpy and scipy; runtime remains C++.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def rotation(quaternion):
    return Rotation.from_quat(np.asarray(quaternion)[[1, 2, 3, 0]]).as_matrix()


def pose(position, axes):
    return dict(position_m=np.asarray(position).tolist(),
                quaternion_wxyz=Rotation.from_matrix(axes).as_quat()[[3, 0, 1, 2]].tolist())


def complete(scene_path, plan_path, output):
    scene = json.loads(scene_path.read_text())
    plan = json.loads(plan_path.read_text())
    output.mkdir(parents=True, exist_ok=False)
    nodes = {o['id']: o for o in scene['objects']}
    transforms = {'world': (np.zeros(3), np.eye(3))}

    def world(key):
        if key not in transforms:
            o = nodes[key]
            p, r = world(o['parent'])
            transforms[key] = (p+r @ o['pose']['position_m'],
                               r @ rotation(o['pose']['quaternion_wxyz']))
        return transforms[key]

    def relative(parent, position, axes):
        p, r = world(parent)
        return pose(r.T @ (position-p), r.T @ axes)

    organization = plan.get('organization', {})
    for key, name in organization.get('group_names', {}).items():
        assert nodes[key]['kind'] == 'group'
        nodes[key]['name'] = name
    for key, parent in organization.get('group_parents', {}).items():
        node = nodes[key]
        assert node['kind'] == 'group' and nodes[parent]['kind'] == 'group'
        old_p, old_r = world(node['parent'])
        new_p, new_r = world(parent)
        if not (np.array_equal(old_p, new_p) and np.array_equal(old_r, new_r)):
            position, axes = world(key)
            node['pose'] = relative(parent, position, axes)
        node['parent'] = parent

    building = Rotation.from_euler('Z', plan['building_yaw_rad']).as_matrix()
    removed, emptied, added, replacements = set(), set(), [], {}
    report = copy.deepcopy(plan)
    report_name = plan.get('report_name', 'floor-completion.json')
    report['inputs'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in (scene_path, plan_path)}
    for level, specification in plan['levels'].items():
        semantic = specification.get('semantic', 'floor')
        if semantic not in ('floor', 'ceiling'):
            raise ValueError('Completion supports floor or ceiling surfaces')
        if semantic == 'ceiling' and specification.get('landing_ids'):
            raise ValueError('Stair entrances can only follow a floor plane')
        a, b, c = specification['plane_z_abc_building']
        normal = np.array([-a, -b, 1.])
        normal /= np.linalg.norm(normal)
        if semantic == 'ceiling':
            normal = -normal
        ex = np.array([1., 0, a])
        ex /= np.linalg.norm(ex)
        basis = np.column_stack((ex, np.cross(normal, ex), normal))
        origin, axes = building @ [0, 0, c], building @ basis

        def project(ring):
            xy = np.asarray(ring)
            # Ceiling faces point down; their thickness extends above the room.
            local = (np.c_[xy, a*xy[:, 0]+b*xy[:, 1]] @ basis)[:, :2]
            return (local[::-1] if semantic == 'ceiling' else local).tolist()

        evidence = dict(basis='inferred', report_ref=report_name+'#levels/'+level,
                        note=specification['note'])
        group = level+'-continuous-'+semantic
        assert group not in nodes
        title = 'floor · inferred / 连续地面 · 推定' if semantic == 'floor' else 'ceiling · inferred / 连续天花板 · 推定'
        added.append(dict(id=group, name=specification.get('name', level.title()+' '+title),
                          kind='group', parent=specification['parent'], semantic=semantic,
                          pose=relative(specification['parent'], origin, axes),
                          evidence=copy.deepcopy(evidence)))
        for i, boundary in enumerate(specification['boundaries']):
            added.append(dict(id=group+'-'+str(i),
                              name=semantic.title()+' region '+str(i+1)+' · inferred / 推定',
                              kind='primitive', parent=group, semantic=semantic,
                              pose=pose(np.zeros(3), np.eye(3)),
                              geometry=dict(type='planar_region',
                                            outer_xy_m=project(boundary['outer_xy_m']),
                                            holes_xy_m=[project(h) for h in boundary['holes_xy_m']],
                                            thickness_m=specification.get('thickness_m', .08)),
                              evidence=copy.deepcopy(evidence)))
        for key in specification.get('region_ids', specification.get('floor_region_ids', [])):
            assert nodes[key].get('semantic') == semantic
            removed.add(key)
            emptied.add(nodes[key]['parent'])
        for key in specification.get('landing_ids', []):
            node = copy.deepcopy(nodes[key])
            p, r = world(key)
            geometry = node['geometry']
            assert geometry['type'] == 'planar_region'

            def landing_ring(ring):
                xy = np.asarray(ring)
                vertices = (p+np.c_[xy, np.zeros(len(xy))] @ r.T) @ building
                return project(vertices[:, :2])

            geometry['outer_xy_m'] = landing_ring(geometry['outer_xy_m'])
            geometry['holes_xy_m'] = [landing_ring(h) for h in geometry['holes_xy_m']]
            node['pose'] = relative(node['parent'], origin, axes)
            node['name'] += ' · planar floor / 层内平面'
            node['evidence'] = copy.deepcopy(evidence)
            replacements[key] = node
    children = {o['parent'] for o in scene['objects'] if o['id'] not in removed}
    removed.update(key for key in emptied if key not in children)
    # Put each new surface group first in its storey's editor list. Primitive order
    # stays unchanged, so this ordering does not change physics geometry IDs.
    scene['objects'] = ([o for o in added if o['kind'] == 'group']
                        + [replacements.get(o['id'], o) for o in scene['objects']
                           if o['id'] not in removed]
                        + [o for o in added if o['kind'] != 'group'])
    scene['id'] = plan.get('scene_id', 'wall-bounded-building-inferred-floors')
    scene['name'] = plan.get('scene_name', 'Building · inferred planar floors / 建筑 · 推定平面地面')
    report.update(removed_node_ids=sorted(removed), changed_landing_ids=sorted(replacements),
                  scene_nodes=len(scene['objects']), added_nodes=len(added),
                  limitation='Planar surface completion is structural inference. Residuals against the source map are retained; neither the original cloud nor observed-only validation is changed.')
    (output/'scene.json').write_text(json.dumps(scene, indent=2)+'\n')
    (output/report_name).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(scene_nodes=len(scene['objects']), added_nodes=len(added),
                          removed_nodes=len(removed), landings_changed=sorted(replacements))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('scene', 'plan', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    complete(args.scene, args.plan, args.output)
