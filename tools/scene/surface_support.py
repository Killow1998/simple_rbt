"""Offline checks for short interpolation gaps on an already fitted plane."""
import numpy as np
from scipy.spatial import cKDTree
from shapely import contains_xy


class SurfaceSupport:
    def __init__(self, cloud, normal_path):
        data = np.load(cloud)
        self.xyz, self.ids = data['xyz'], data['ids']
        self.origins = data['origins']
        self.tree = cKDTree(self.xyz)
        normals = np.fromfile(normal_path, dtype='<f4').reshape(-1, 6)
        self.normals = normals[np.isfinite(normals).all(axis=1)]
        self.normal_tree = cKDTree(self.normals[:, :3])
        # Preserve the existing independent-keyframe visibility veto. These rays
        # use scan poses; they are conservative exclusions, not free-space truth.
        indices = np.flatnonzero(self.ids % 5 == 0)[::12]
        self.ray_ends = self.xyz[indices]
        self.ray_origins = self.origins[self.ids[indices]]

    def on_plane(self, origin, axes, bounds):
        low, high = np.asarray(bounds[:2]), np.asarray(bounds[2:])
        center = origin + axes[:, :2] @ ((low + high) / 2)
        indices = self.tree.query_ball_point(center, np.linalg.norm(high-low)/2 + .4)
        indices = np.asarray(indices, dtype=int)
        local = (self.xyz[indices] - origin) @ axes
        inside = (np.all(local[:, :2] >= low-.3, axis=1)
                  & np.all(local[:, :2] <= high+.3, axis=1)
                  & (abs(local[:, 2]) < .15))
        indices, local = indices[inside], local[inside]
        distance, nearest = self.normal_tree.query(self.xyz[indices], workers=4)
        aligned = ((distance < .06)
                   & (abs(self.normals[nearest, 3:] @ axes[:, 2]) > np.cos(np.pi/12)))
        local, ids = local[aligned], self.ids[indices[aligned]]
        start = (self.ray_origins-origin) @ axes[:, 2]
        end = (self.ray_ends-origin) @ axes[:, 2]
        denominator = start-end
        t = np.divide(start, denominator, out=np.full(len(start), -1.),
                      where=abs(denominator) > 1e-8)
        length = np.linalg.norm(self.ray_ends-self.ray_origins, axis=1)
        valid = (t > 0) & (t < 1) & ((1-t)*length > .15) & (abs(start) > .1)
        hit = self.ray_origins[valid] + t[valid, None]*(self.ray_ends[valid]-self.ray_origins[valid])
        crossing = ((hit-origin) @ axes)[:, :2]
        return PlaneSupport(local, ids, crossing)


def probes(polygon, spacing=.05):
    """Sample both the interior and boundary; never rely on just its centroid."""
    x0, y0, x1, y1 = polygon.bounds
    xs = np.linspace(x0, x1, max(2, int(np.ceil((x1-x0)/spacing))+1))
    ys = np.linspace(y0, y1, max(2, int(np.ceil((y1-y0)/spacing))+1))
    grid = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)
    grid = grid[contains_xy(polygon, grid[:, 0], grid[:, 1])]
    boundary = np.concatenate([np.asarray(r.coords)[:, :2] for r in
                               [polygon.segmentize(spacing).exterior,
                                *polygon.segmentize(spacing).interiors]])
    return np.unique(np.vstack([grid, boundary]), axis=0)


class PlaneSupport:
    def __init__(self, local, ids, crossing):
        self.local, self.ids, self.crossing = local, ids, crossing
        consistent = abs(local[:, 2]) <= .06
        self.train = cKDTree(local[consistent & (ids % 5 != 0)])
        self.held = cKDTree(local[consistent & (ids % 5 == 0)])

    def check(self, polygon):
        if not self.train.n or not self.held.n:
            return dict(area_m2=polygon.area, accepted=False,
                        rejected_by=['insufficient_repeated_support'])
        xy = probes(polygon)
        samples = np.c_[xy, np.zeros(len(xy))]
        train_distance = self.train.query(samples)[0]
        held_distance = self.held.query(samples)[0]
        neighborhood = polygon.buffer(.15)
        near = contains_xy(neighborhood, self.local[:, 0], self.local[:, 1])
        held = near & (self.ids % 5 == 0)
        residual = abs(self.local[held, 2])
        crossing = int(np.count_nonzero(contains_xy(polygon.buffer(1e-7),
                                                   self.crossing[:, 0], self.crossing[:, 1])))
        result = dict(area_m2=polygon.area, probes=len(samples),
                      training_scans=len(np.unique(self.ids[near & (self.ids % 5 != 0)])),
                      held_points=len(residual), crossing_rays=crossing,
                      training_distance_p95_max_m=np.quantile(train_distance, [.95, 1]).tolist(),
                      held_distance_p95_max_m=np.quantile(held_distance, [.95, 1]).tolist(),
                      held_plane_p95_m=float(np.quantile(residual, .95)) if len(residual) else None)
        reasons = []
        if result['training_scans'] < 3 or len(residual) < 12:
            reasons.append('insufficient_repeated_support')
        if not len(residual) or result['held_plane_p95_m'] > .06:
            reasons.append('plane_residual')
        if max(result['training_distance_p95_max_m'][0], result['held_distance_p95_max_m'][0]) > .14:
            reasons.append('coverage_p95')
        if max(train_distance.max(), held_distance.max()) > .25:
            reasons.append('coverage_max')
        if crossing >= 2:
            reasons.append('observed_free_rays')
        result.update(accepted=not reasons, rejected_by=reasons)
        return result
