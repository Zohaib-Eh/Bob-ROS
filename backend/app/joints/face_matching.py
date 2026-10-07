import math

import numpy as np

from app.models import Body, Face

_PLANE_DISTANCE_TOLERANCE_DEFAULT = 1e-4   # meters
_PLANE_NORMAL_PARALLEL_TOLERANCE = 1e-3    # dot product closeness to -1
_CYLINDER_AXIS_PARALLEL_TOLERANCE = 1e-3
_CYLINDER_RADIUS_RELATIVE_TOLERANCE = 0.02
_CYLINDER_CENTER_LINE_DISTANCE_TOLERANCE = 1e-3


def _dist(a: tuple, b: tuple) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _dot(a: tuple, b: tuple) -> float:
    return sum(x * y for x, y in zip(a, b))


def _face_aabbs(body: Body) -> list:
    """Tessellation AABB (min, max) per face, or None if it has no triangles.
    Arrays are built once per body, not per face pair."""
    verts = np.asarray(body.vertices, dtype=float).reshape(-1, 3)
    tris = np.asarray(body.triangles, dtype=int).reshape(-1, 3)
    boxes = []
    for face in body.faces:
        if not face.triangle_indices:
            boxes.append(None)
            continue
        pts = verts[tris[face.triangle_indices].ravel()]
        boxes.append((pts.min(axis=0), pts.max(axis=0)))
    return boxes


def _aabbs_overlap(box_a, box_b, tol: float) -> bool:
    """Only checked when both faces have triangles; otherwise no constraint."""
    if box_a is None or box_b is None:
        return True
    return bool(np.all(box_a[0] <= box_b[1] + tol) and np.all(box_b[0] <= box_a[1] + tol))


def _planar_match(face_a: Face, face_b: Face, tol: float, has_boxes: bool) -> bool:
    if face_a.kind != "plane" or face_b.kind != "plane":
        return False
    # opposing normals (the two mating faces point at each other)
    if _dot(face_a.normal, face_b.normal) > -(1 - _PLANE_NORMAL_PARALLEL_TOLERANCE):
        return False
    if not has_boxes:
        return _dist(face_a.center, face_b.center) < tol
    # centers are arbitrary points on the plane, so test coplanarity instead
    offset = tuple(b - a for a, b in zip(face_a.center, face_b.center))
    return abs(_dot(offset, face_a.normal)) < tol


def _cylindrical_match(face_a: Face, face_b: Face) -> bool:
    if face_a.kind != "cylinder" or face_b.kind != "cylinder":
        return False
    if abs(abs(_dot(face_a.axis, face_b.axis)) - 1.0) > _CYLINDER_AXIS_PARALLEL_TOLERANCE:
        return False
    if face_a.radius is None or face_b.radius is None:
        return False
    radius_diff = abs(face_a.radius - face_b.radius) / max(face_a.radius, 1e-9)
    if radius_diff > _CYLINDER_RADIUS_RELATIVE_TOLERANCE:
        return False
    # perpendicular distance from face_b's center to face_a's axis line
    to_point = tuple(b - a for a, b in zip(face_a.center, face_b.center))
    along_axis = _dot(to_point, face_a.axis)
    perp = [to_point[i] - along_axis * face_a.axis[i] for i in range(3)]
    perp_distance = math.sqrt(sum(p * p for p in perp))
    return perp_distance < _CYLINDER_CENTER_LINE_DISTANCE_TOLERANCE


def _contact_score(face_a: Face, face_b: Face) -> float:
    """Larger is a more-constraining, more-trustworthy match."""
    if face_a.kind == "cylinder":
        return face_a.radius
    return 1.0  # planar matches have no natural size signal here


def find_matching_face_pairs(
    body_a: Body, body_b: Body,
    distance_tolerance: float = _PLANE_DISTANCE_TOLERANCE_DEFAULT,
) -> list[tuple[Face, Face]]:
    pairs: list[tuple[Face, Face]] = []
    boxes_b = _face_aabbs(body_b)
    for face_a, box_a in zip(body_a.faces, _face_aabbs(body_a)):
        for face_b, box_b in zip(body_b.faces, boxes_b):
            if not _aabbs_overlap(box_a, box_b, distance_tolerance):
                continue
            has_boxes = box_a is not None and box_b is not None
            if _planar_match(face_a, face_b, distance_tolerance, has_boxes) or _cylindrical_match(
                face_a, face_b
            ):
                pairs.append((face_a, face_b))

    pairs.sort(key=lambda pair: _contact_score(*pair), reverse=True)
    return pairs
