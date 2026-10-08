import uuid

from app.models import Face, Joint

_PERPENDICULAR_DOT_TOLERANCE = 0.05


def _dot(a, b) -> float:
    return sum(x * y for x, y in zip(a, b))


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

    cyl_pairs = [p for p in matched_pairs if p[0].kind == "cylinder"]
    plane_pairs = [p for p in matched_pairs if p[0].kind == "plane"]

    if cyl_pairs:
        cyl_face = cyl_pairs[0][0]
        has_perpendicular_plane = any(
            abs(_dot(cyl_face.axis, p[0].normal)) > (1 - _PERPENDICULAR_DOT_TOLERANCE)
            for p in plane_pairs
        )
        return Joint(
            type="revolute" if has_perpendicular_plane else "cylindrical",
            origin_xyz=cyl_face.center, axis=cyl_face.axis,
            ambiguous=len(cyl_pairs) > 1, **base,
        )

    plane_face = plane_pairs[0][0]
    return Joint(type="planar", origin_xyz=plane_face.center,
                 axis=plane_face.normal, **base)
