"""Joint limits from geometry: swing the moving parts until they would hit something.

fcl reports the same penetration depth for touching and deeply overlapping meshes, so
depth can't tell resting contact from a collision. Instead the moving side is shrunk
inward by TOL and only boolean collision is used: touching or <TOL overlap is free,
anything deeper is a hit.
"""
import math

import fcl
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix, translation_matrix

from app.models import Body

TOL = 0.0005                      # m
STEP_RAD = math.radians(3)
FINE_RAD = math.radians(0.25)
MARGIN_RAD = math.radians(1)
STEP_M_MIN = 0.001
FINE_M = 0.00025
MARGIN_M = 0.001


def _mesh(bodies: list[Body]) -> trimesh.Trimesh | None:
    vs, fs, offset = [], [], 0
    for b in bodies:
        if not b.vertices or not b.triangles:
            continue
        vs.append(np.asarray(b.vertices, dtype=float))
        fs.append(np.asarray(b.triangles, dtype=int) + offset)
        offset += len(b.vertices)
    if not vs:
        return None
    return trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=False)


def _object(vertices, faces) -> fcl.CollisionObject:
    bvh = fcl.BVHModel()
    bvh.beginModel(len(vertices), len(faces))
    bvh.addSubModel(np.asarray(vertices, dtype=float), np.asarray(faces, dtype=int))
    bvh.endModel()
    return fcl.CollisionObject(bvh)


def _sphere(m: trimesh.Trimesh) -> tuple[np.ndarray, float]:
    lo, hi = m.bounds
    return (lo + hi) / 2, float(np.linalg.norm(hi - lo)) / 2


def _collides(a: fcl.CollisionObject, b: fcl.CollisionObject) -> bool:
    return fcl.collide(a, b, fcl.CollisionRequest(), fcl.CollisionResult()) > 0


def reachable(moving_center, moving_radius, obstacle_center, obstacle_radius, origin, axis,
              kind: str, travel: float) -> bool:
    """Can the moving part's bounding sphere come near the obstacle's during the sweep?"""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    o, mc, oc = (np.asarray(p, dtype=float) for p in (origin, moving_center, obstacle_center))
    reach = moving_radius + obstacle_radius
    if kind == "prismatic":
        t = float(np.clip(np.dot(oc - mc, a), -travel, travel))
        return float(np.linalg.norm(oc - (mc + t * a))) <= reach
    h = float(np.dot(mc - o, a))
    c0 = o + h * a                               # centre of the circle the moving centre sweeps
    rc = float(np.linalg.norm(mc - c0))
    q = oc - c0
    z = float(np.dot(q, a))
    rho = float(np.linalg.norm(q - z * a))
    return math.hypot(rho - rc, z) <= reach


def sweep_limits(kind: str, origin, axis, moving: list[Body],
                 obstacles: list[tuple[str, list[Body]]]) -> dict:
    if kind not in ("revolute", "prismatic"):
        raise ValueError(f"only revolute and prismatic joints can be swept, not {kind}")
    a = np.asarray(axis, dtype=float)
    if not (np.all(np.isfinite(a)) and np.all(np.isfinite(np.asarray(origin, dtype=float)))):
        raise ValueError("joint axis and origin must be finite numbers")
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        raise ValueError("joint axis must have nonzero length")
    a = a / n
    o = np.asarray(origin, dtype=float)
    mv = _mesh(moving)
    if mv is None:
        raise ValueError("the moving parts have no geometry")
    mobj = _object(mv.vertices - mv.vertex_normals * TOL, mv.faces)
    mc, mr = _sphere(mv)
    travel = float(np.linalg.norm(mv.extents))

    obs = []
    for link_id, bodies in obstacles:
        m = _mesh(bodies)
        if m is not None:
            oc, orad = _sphere(m)
            obs.append((link_id, _object(m.vertices, m.faces), oc, orad))

    mobj.setTransform(fcl.Transform())
    overlap = [lid for lid, obj, _, _ in obs if _collides(mobj, obj)]
    if overlap:
        return {"lower": None, "upper": None, "free": False,
                "blocked_by": {"lower": None, "upper": None}, "at_rest_overlap": overlap}
    obs = [x for x in obs if reachable(mc, mr, x[2], x[3], o, a, kind, travel)]

    def hit(value: float) -> str | None:
        T = translation_matrix(a * value) if kind == "prismatic" else rotation_matrix(value, a, o)
        mobj.setTransform(fcl.Transform(T[:3, :3], T[:3, 3]))
        for lid, obj, _, _ in obs:
            if _collides(mobj, obj):
                return lid
        return None

    if kind == "revolute":
        step, fine, margin, full = STEP_RAD, FINE_RAD, MARGIN_RAD, 2 * math.pi
    else:
        step, fine, margin, full = max(STEP_M_MIN, travel / 200), FINE_M, MARGIN_M, travel

    def search(sign: int) -> tuple[float, str] | None:
        prev, v = 0.0, step
        while v <= full + 1e-12:
            lid = hit(sign * v)
            if lid is not None:
                lo, hi = prev, v
                while hi - lo > fine:
                    mid = (lo + hi) / 2
                    if hit(sign * mid) is None:
                        lo = mid
                    else:
                        hi = mid
                return sign * max(lo - margin, 0.0), lid
            prev, v = v, v + step
        return None

    up, down = search(+1), search(-1)
    if kind == "revolute" and up is None:
        return {"lower": -math.pi, "upper": math.pi, "free": True,
                "blocked_by": {"lower": None, "upper": None}, "at_rest_overlap": []}
    upper, upper_by = up if up else (full, None)
    lower, lower_by = down if down else (-full, None)
    return {"lower": lower, "upper": upper, "free": False,
            "blocked_by": {"lower": lower_by, "upper": upper_by}, "at_rest_overlap": []}
