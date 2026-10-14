"""Collision-derived bounds and body-contact checks for articulated wheel robots.

Callers must run mj_forward/mj_step first. Helpers are read-only and return plain
JSON-compatible data. Bounds use the base_origin SITE as their origin, with
either site-aligned or world-aligned axes. They describe the current articulated
pose, not a swept envelope or contact margin. A route evaluator should request
world axes for a world-Y corridor and add its own clearance margin.
"""
from __future__ import annotations

import mujoco
import numpy as np


def _base_and_descendants(model):
    base = int(model.body("base").id)
    if base == 0:
        raise ValueError("Robot base must be a non-world body")
    return base, _descendants(model, {base})


def _descendants(model, roots):
    # MuJoCo's compiled body tree stores every parent before its children.
    result = set(roots)
    for body_id in range(1, model.nbody):
        if int(model.body_parentid[body_id]) in result:
            result.add(body_id)
    return result


def _contact_enabled(model, geom_id):
    return bool(int(model.geom_contype[geom_id]) | int(model.geom_conaffinity[geom_id]))


def _collision_ids(model, body_ids):
    return [gid for gid in range(model.ngeom)
            if int(model.geom_bodyid[gid]) in body_ids and _contact_enabled(model, gid)]


def _extents_in_frame(model, geom_ids, rotations):
    """Batch exact primitive support along each geom-to-reference matrix row."""
    kinds, sizes = model.geom_type[geom_ids], model.geom_size[geom_ids]
    extents = np.empty((len(geom_ids), 3))
    for kind in np.unique(kinds):
        selected = kinds == kind
        rotation, size = rotations[selected], sizes[selected]
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            extents[selected] = np.einsum("nij,nj->ni", np.abs(rotation), size)
        elif kind == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            extents[selected] = np.sqrt(np.sum((rotation * size[:, None, :]) ** 2, axis=2))
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            extents[selected] = size[:, 0, None]
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            # Radial disk support plus half-height along each local Z axis.
            extents[selected] = (size[:, 0, None] * np.linalg.norm(rotation[:, :, :2], axis=2)
                                 + size[:, 1, None] * np.abs(rotation[:, :, 2]))
        elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            # Segment + sphere: radius also contributes along the long axis.
            extents[selected] = size[:, 0, None] + size[:, 1, None] * np.abs(rotation[:, :, 2])
        else:
            gid = geom_ids[int(np.flatnonzero(selected)[0])]
            name = model.geom(gid).name or str(gid)
            raise ValueError(f"Unsupported collision shape {int(kind)} for {name}; exact primitive envelope required")
    return extents


def collision_envelope(model, data, *, reference_frame="base_origin"):
    """Union AABB of contact-enabled robot geoms, relative to base_origin SITE.

    ``base_origin`` (default) uses the site's own axes, preserving the original
    API. ``world`` uses global axes while retaining that site's world position
    as origin: world-Y bounds then include yaw/roll/pitch and steering effects.
    lower_m/upper_m are coordinate-neutral keys; legacy lower_base_m/upper_base_m
    contain the SAME values in the SELECTED axes, including in world mode.

    BOX/CYLINDER/ELLIPSOID/SPHERE/CAPSULE use exact rotated support. All wheel
    assembly descendants, including passive rollers, are included. Unsupported
    collision shapes raise rather than silently approximating or omitting them.
    Visual-only geometry and world/other-object geometry are excluded.
    """
    if reference_frame not in ("base_origin", "world"):
        raise ValueError("reference_frame must be 'base_origin' or 'world'")
    base, bodies = _base_and_descendants(model)
    site = int(model.site("base_origin").id)
    if int(model.site_bodyid[site]) not in bodies:
        raise ValueError("base_origin must belong to the robot base subtree")
    origin = data.site_xpos[site]
    ids = _collision_ids(model, bodies)
    if not ids:
        raise ValueError("Robot has no contact-enabled collision geometry")
    rotations = data.geom_xmat[ids].reshape(-1, 3, 3)
    centers = data.geom_xpos[ids] - origin
    if reference_frame == "base_origin":
        base_rotation = data.site_xmat[site].reshape(3, 3)
        rotations = base_rotation.T @ rotations
        centers = centers @ base_rotation
    extents = _extents_in_frame(model, ids, rotations)
    lower, upper = np.min(centers - extents, axis=0), np.max(centers + extents, axis=0)
    return {"lower_m": lower.tolist(), "upper_m": upper.tolist(),
            "lower_base_m": lower.tolist(), "upper_base_m": upper.tolist(),
            "outer_half_width_m": float(max(abs(lower[1]), abs(upper[1]))),
            "collision_geom_ids": ids,
            "collision_geom_names": [model.geom(gid).name or str(gid) for gid in ids],
            "base_body_id": base, "reference_frame": reference_frame,
            "origin_site": "base_origin", "origin_world_m": origin.tolist(),
            "shape_support": "exact_primitive_support",
            "snapshot_time_s": float(data.time),
            "scope": "Current articulated pose in selected axes relative to base_origin site; excludes visual-only geoms and contact margins. Not a swept envelope."}


def caster_contact_geom_ids(model):
    """Resolve explicit support-ball roles, never infer them from sphere shape.

    These named contact geoms are directly attached to base in the minimal
    differential chassis. Unknown base geoms (including identical spheres) are
    deliberately not excluded. Validate a complete pair when either is present.
    """
    base, _ = _base_and_descendants(model)
    roles = ("front_caster_collision", "rear_caster_collision")
    by_name = {model.geom(gid).name: gid for gid in range(model.ngeom)}
    present = set(roles) & by_name.keys()
    if not present:
        return []
    if present != set(roles):
        raise ValueError("Incomplete support caster roles")
    ids = [by_name[name] for name in roles]
    if any(int(model.geom_bodyid[gid]) != base or
           int(model.geom_type[gid]) != mujoco.mjtGeom.mjGEOM_SPHERE or
           not _contact_enabled(model, gid) for gid in ids):
        raise ValueError("Support caster roles must be active base-attached spheres")
    return ids


def wheel_contact_geom_ids(model):
    """Contact-enabled geom IDs in each *_wheel body and all its descendants.

    Explicitly named base-attached support casters are also included. A
    steering-parent body is deliberately not part of the wheel exclusion: a
    contact-enabled steering housing hitting the environment is a body contact.
    """
    _, robot_bodies = _base_and_descendants(model)
    hubs = {bid for bid in robot_bodies if (model.body(bid).name or "").endswith("_wheel")}
    if not hubs:
        raise ValueError("No *_wheel assemblies found in robot base subtree")
    return sorted(set(_collision_ids(model, _descendants(model, hubs)))
                  | set(caster_contact_geom_ids(model)))


def body_environment_contacts(model, data, wheel_geom_ids=None):
    """Sorted unique 'robot geom / world geom' touching/penetrating body pairs.

    Only base-subtree versus world-body(0) contacts are in this gate. Wheel
    assemblies and explicit support casters are ignored; shells, decks and
    steering housings remain. A tiny
    1e-6 m contact-distance tolerance matches the existing route evaluator and
    avoids counting distant contacts generated only by a positive geom margin.
    """
    _, robot_bodies = _base_and_descendants(model)
    wheels = set(wheel_contact_geom_ids(model) if wheel_geom_ids is None else wheel_geom_ids)
    names = set()
    for contact in data.contact:
        a, b = int(contact.geom1), int(contact.geom2)
        if a < 0 or b < 0 or contact.dist > 1e-6:
            continue
        body_a, body_b = int(model.geom_bodyid[a]), int(model.geom_bodyid[b])
        if body_a == 0 and body_b in robot_bodies:
            robot_geom, world_geom = b, a
        elif body_b == 0 and body_a in robot_bodies:
            robot_geom, world_geom = a, b
        else:
            continue
        if robot_geom not in wheels:
            robot_name = model.geom(robot_geom).name or str(robot_geom)
            world_name = model.geom(world_geom).name or str(world_geom)
            names.add(f"{robot_name} / {world_name}")
    return sorted(names)
