import uuid

import numpy as np

from app.models import Face, Joint

_PERPENDICULAR_DOT_TOLERANCE = 0.05
_RING_TOL = 1e-4  # m: equal holes count as one bolt circle when their centres fit a circle this closely


def _dot(a, b) -> float:
    return sum(x * y for x, y in zip(a, b))


def _axis_point(cyls: list[Face]) -> tuple:
    """Where the rotation axis runs: the chosen cylinder's centre line, or, when the match is a ring
    of 3+ equal parallel holes (bolts or pins on a flange), the centre of that ring."""
    first = cyls[0]
    if first.radius is None:
        return first.center
    a = np.asarray(first.axis, float)
    a /= np.linalg.norm(a)
    ring = [f for f in cyls if f.radius is not None and abs(f.radius - first.radius) < _RING_TOL
            and abs(abs(np.dot(np.asarray(f.axis, float), a)) - 1) < 1e-6]
    u = np.cross(a, (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0))
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    o = np.asarray(first.center, float)
    d = np.array([f.center for f in ring], float) - o
    uv = np.c_[d @ u, d @ v]                                   # hole centres in the plane across the axis
    uv = uv[np.unique(np.round(uv, 7), axis=0, return_index=True)[1]]  # one point per hole
    A = np.c_[2 * uv, np.ones(len(uv))]
    if len(uv) < 3 or np.linalg.matrix_rank(A) < 3:              # 1-2 holes or holes in a row: no ring
        return first.center
    cx, cy, k = np.linalg.lstsq(A, (uv ** 2).sum(axis=1), rcond=None)[0]
    r = float(np.sqrt(k + cx * cx + cy * cy))
    if r <= first.radius or np.abs(np.linalg.norm(uv - (cx, cy), axis=1) - r).max() > _RING_TOL:
        return first.center
    return tuple(float(x) for x in o + cx * u + cy * v)


def classify_joint(
    matched_pairs: list[tuple[Face, Face]],
    parent_link_id: str,
    child_link_id: str,
) -> Joint:
    """IPV/IMG table (spec 6a step 3): cylinder -> cylindrical; cylinder plus
    perpendicular planar lock -> revolute; planar alone -> planar; nothing
    matched -> fixed. Uses the first pair (largest, per find_matching_face_pairs)
    and flags ambiguous when more than one cylindrical pair matched."""
    base = dict(id=str(uuid.uuid4()), parent_link_id=parent_link_id,
                child_link_id=child_link_id, origin_rpy=(0.0, 0.0, 0.0),
                source="suggested")
    if not matched_pairs:
        return Joint(type="fixed", origin_xyz=(0.0, 0.0, 0.0), axis=None, **base)

    # widest bore first (a bearing beats a bolt hole); ties keep the matcher's contact order
    cyl_pairs = sorted((p for p in matched_pairs if p[0].kind == "cylinder"),
                       key=lambda p: -(p[0].radius or 0.0))
    plane_pairs = [p for p in matched_pairs if p[0].kind == "plane"]

    if cyl_pairs:
        cyl_face = cyl_pairs[0][0]
        has_perpendicular_plane = any(
            abs(_dot(cyl_face.axis, p[0].normal)) > (1 - _PERPENDICULAR_DOT_TOLERANCE)
            for p in plane_pairs
        )
        return Joint(
            type="revolute" if has_perpendicular_plane else "cylindrical",
            origin_xyz=_axis_point([p[0] for p in cyl_pairs]), axis=cyl_face.axis,
            ambiguous=len(cyl_pairs) > 1, **base,
        )

    plane_face = plane_pairs[0][0]
    return Joint(type="planar", origin_xyz=plane_face.center,
                 axis=plane_face.normal, **base)
