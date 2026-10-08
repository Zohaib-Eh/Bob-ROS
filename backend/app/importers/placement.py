import numpy as np

from app.models import STATUS_ORDER, Body, PartWarning, Vec3


def combine_bodies(bodies: list[Body], body_id: str, name: str) -> Body:
    vertices, triangles, faces, warnings = [], [], [], []
    for b in bodies:
        v0, t0 = len(vertices), len(triangles)
        vertices += b.vertices
        triangles += [(a + v0, c + v0, d + v0) for a, c, d in b.triangles]
        faces += [f.model_copy(update={"triangle_indices": [i + t0 for i in f.triangle_indices]})
                  for f in b.faces]
        warnings += b.warnings
    live = [b.status for b in bodies if b.vertices]
    status = max(live, key=STATUS_ORDER.get, default="failed")
    if live and len(live) < len(bodies):  # a failed part is missing from the geometry
        status = max(status, "approximate", key=STATUS_ORDER.get)
    return Body(id=body_id, name=name, vertices=vertices, triangles=triangles,
                faces=faces, status=status, warnings=warnings)


def _translate(body: Body, d: np.ndarray) -> Body:
    shift = lambda p: tuple(float(x) for x in np.asarray(p) + d)
    return body.model_copy(update={
        "vertices": [shift(v) for v in body.vertices],
        "faces": [f.model_copy(update={"center": shift(f.center)}) for f in body.faces],
    })


def place_replacement(body: Body, bbox_min: Vec3 | None, bbox_max: Vec3 | None,
                      tol: float = 1e-4) -> Body:
    """Spec §4.2: keep coordinates if the replacement overlaps the original's box;
    otherwise centre it on the original; with no original box, keep and flag."""
    if bbox_min is None or bbox_max is None:
        w = PartWarning(code="position_unverified", fixes=["replace"],
                        message=f"{body.name}: original part had no geometry, so the "
                                "replacement's position could not be checked")
        return body.model_copy(update={"warnings": body.warnings + [w]})
    if not body.vertices:
        return body
    v = np.asarray(body.vertices, dtype=float)
    lo, hi = v.min(axis=0), v.max(axis=0)
    o_lo, o_hi = np.asarray(bbox_min, dtype=float), np.asarray(bbox_max, dtype=float)
    if np.all(lo <= o_hi + tol) and np.all(o_lo <= hi + tol):
        return body
    moved = _translate(body, (o_lo + o_hi) / 2 - (lo + hi) / 2)
    w = PartWarning(code="placed_by_bbox", fixes=["replace"],
                    message=f"{body.name}: placed by bounding-box centre; check its rotation")
    return moved.model_copy(update={"warnings": moved.warnings + [w]})
