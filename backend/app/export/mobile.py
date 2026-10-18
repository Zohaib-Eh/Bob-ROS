"""A wheeled base, from the geometry the user built: each wheel's cylinder (about its joint axis), the floor
(the wheels' lowest point), the casters (the base's other lowest points) and the drive. World coordinates,
the robot at q = 0, z up; forward is +x and the axles lie along y (ROS REP-103)."""
from dataclasses import dataclass, field

import numpy as np

from app.models import Body, Joint, Link, Mobile

WHEEL_SPEED = 20.0    # rad/s: a wheel's top speed (1.6 m/s on an 8 cm wheel)
ROUND = 0.97          # a wheel's rim, around the axis, stays within 3% of its radius
BINS = 12             # directions around the axle that must reach the rim (a 24-sided CAD wheel fills them, a box can't)
CASTER_BAND = 0.005   # m: base points this close to the floor carry it
CASTER_RADIUS = 0.01  # m: the frictionless sphere standing in for a caster
CLUSTER = 0.05        # m: points closer than this (in plan) are one caster (a caster is a few cm across)


@dataclass
class WheelFit:
    radius: float
    width: float
    center: tuple
    round: bool


@dataclass
class Drive:
    drive: str                      # "diff" (one wheel per side, casters) | "skid" (more per side)
    left: list[str]                 # joint ids
    right: list[str]
    radius: float                   # m, the wheels' mean
    separation: float               # m, left to right wheel centres
    ground: float                   # z of the floor under the wheels, in the robot's coordinates
    wheels: dict = field(default_factory=dict)    # joint id -> WheelFit
    casters: list = field(default_factory=list)   # [(x, y, z, r)]: spheres resting on the floor
    footprint: tuple = (0.0, 0.0, 0.0, 0.0)       # (xmin, ymin, xmax, ymax) of the base and wheels
    center: tuple = (0.0, 0.0)                    # (x, y) the base turns about in place: the axles' middle
    height: float = 0.0                           # m from the floor to the top of the base and wheels


def fit_wheel(vertices, axis, point) -> WheelFit:
    """The cylinder a wheel's shape makes about its joint axis (through `point`). round: the rim is a circle
    (in each of 12 directions around the axis the shape reaches within 3% of its radius)."""
    v = np.asarray(vertices, float) - point
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    along = v @ a
    radial = v - np.outer(along, a)
    dist = np.linalg.norm(radial, axis=1)
    r = float(dist.max())
    u = np.cross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    angle = np.arctan2(radial @ np.cross(a, u), radial @ u)
    bins = ((angle + np.pi) / (2 * np.pi) * BINS).astype(int) % BINS
    reach = np.zeros(BINS)
    np.maximum.at(reach, bins, dist)
    mid = (along.max() + along.min()) / 2
    return WheelFit(radius=r, width=float(along.max() - along.min()),
                    center=tuple(float(x) for x in np.asarray(point, float) + mid * a), round=bool(reach.min() >= ROUND * r))


def _fixed_to_root(root: str, joints: list[Joint], skip: set[str]) -> set[str]:
    """The base: the root link and what is bolted to it (fixed joints), not the wheels."""
    out, todo = {root}, [root]
    while todo:
        p = todo.pop()
        for j in joints:
            if j.parent_link_id == p and j.type == "fixed" and j.child_link_id not in skip | out:
                out.add(j.child_link_id)
                todo.append(j.child_link_id)
    return out


def _casters(points: np.ndarray, ground: float) -> list[tuple]:
    low = points[points[:, 2] <= ground + CASTER_BAND][:, :2]
    groups: list[list] = []
    for p in low:   # ponytail: greedy clustering, fine for a handful of casters
        for g in groups:
            if np.linalg.norm(np.mean(g, axis=0) - p) < CLUSTER:
                g.append(p)
                break
        else:
            groups.append([p])
    return [(float(c[0]), float(c[1]), ground + CASTER_RADIUS, CASTER_RADIUS) for c in (np.mean(g, axis=0) for g in groups)]


def drive_info(links: list[Link], joints: list[Joint], bodies_by_link: dict[str, list[Body]], mobile: Mobile) -> Drive:
    """ValueError in plain words when the wheels don't make a drivable base."""
    children = {j.child_link_id for j in joints}
    root = next(l.id for l in links if l.id not in children)
    by_id = {j.id: j for j in joints}
    wheel_ids = mobile.left_joint_ids + mobile.right_joint_ids
    if len(set(wheel_ids)) != len(wheel_ids):
        raise ValueError("A wheel is ticked on both sides.")
    fits = {}
    for jid in wheel_ids:
        j = by_id.get(jid)
        if j is None or j.parent_link_id != root or j.type not in ("continuous", "revolute"):
            raise ValueError(f"Wheel '{jid}' is not a joint on the base (a turning joint from the root link).")
        if abs(j.axis[1]) < 0.99 * np.linalg.norm(j.axis):
            raise ValueError("The wheel axles must lie along y (forward is +x): turn the robot in the Mobile base card.")
        verts = [v for b in bodies_by_link.get(j.child_link_id, []) for v in b.vertices]
        if not verts:
            raise ValueError(f"Wheel '{jid}' has no shape.")
        fits[jid] = fit_wheel(verts, j.axis, j.origin_xyz)
    ly = np.mean([fits[i].center[1] for i in mobile.left_joint_ids])
    ry = np.mean([fits[i].center[1] for i in mobile.right_joint_ids])
    if ly <= ry:
        raise ValueError("The left wheels must be on the left (+y) of the right ones, facing forward (+x).")
    ground = min(f.center[2] - f.radius for f in fits.values())
    base_links = _fixed_to_root(root, joints, {by_id[i].child_link_id for i in wheel_ids})
    base_pts = np.array([v for l in base_links for b in bodies_by_link.get(l, []) for v in b.vertices], float).reshape(-1, 3)
    wheel_pts = np.array([v for i in wheel_ids for b in bodies_by_link.get(by_id[i].child_link_id, []) for v in b.vertices], float)
    allp = np.vstack([base_pts, wheel_pts])
    plan = allp[:, :2]
    mid = np.mean([f.center[:2] for f in fits.values()], axis=0)
    per_side = max(len(mobile.left_joint_ids), len(mobile.right_joint_ids))
    return Drive(drive="diff" if per_side == 1 else "skid", left=list(mobile.left_joint_ids), right=list(mobile.right_joint_ids),
                 radius=float(np.mean([f.radius for f in fits.values()])), separation=float(ly - ry), ground=float(ground),
                 wheels=fits, casters=_casters(base_pts, ground) if len(base_pts) else [],
                 footprint=(*map(float, plan.min(axis=0)), *map(float, plan.max(axis=0))),
                 center=(float(mid[0]), float(mid[1])), height=float(allp[:, 2].max() - ground))
