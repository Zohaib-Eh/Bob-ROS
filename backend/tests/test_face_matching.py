import math
import numpy as np
import trimesh
from app.models import Body, Face
from app.joints.face_matching import find_matching_face_pairs


def _planar_face(center, normal, radius=None, kind="plane"):
    return Face(
        kind=kind, origin="brep", normal=normal, axis=None,
        center=center, radius=radius, residual=0.0, triangle_indices=[],
    )


def _cylindrical_face(center, axis, radius):
    return Face(
        kind="cylinder", origin="brep", normal=None, axis=axis,
        center=center, radius=radius, residual=0.0, triangle_indices=[],
    )


def _body(faces):
    return Body(id="b", name="n", vertices=[], triangles=[], faces=faces)


def test_coincident_planar_faces_match():
    face_a = _planar_face(center=(0, 0, 0), normal=(0, 0, 1))
    face_b = _planar_face(center=(0, 0, 0), normal=(0, 0, -1))
    pairs = find_matching_face_pairs(_body([face_a]), _body([face_b]))
    assert len(pairs) == 1
    assert pairs[0] == (face_a, face_b)


def test_concentric_cylindrical_faces_match():
    face_a = _cylindrical_face(center=(0, 0, 0), axis=(0, 0, 1), radius=0.1)
    face_b = _cylindrical_face(center=(0, 0, 0.5), axis=(0, 0, 1), radius=0.1)
    pairs = find_matching_face_pairs(_body([face_a]), _body([face_b]))
    assert len(pairs) == 1


def test_unrelated_far_apart_faces_do_not_match():
    face_a = _planar_face(center=(0, 0, 0), normal=(0, 0, 1))
    face_b = _planar_face(center=(10, 10, 10), normal=(0, 0, -1))
    pairs = find_matching_face_pairs(_body([face_a]), _body([face_b]))
    assert pairs == []


def test_multiple_concentric_pairs_ordered_largest_radius_first():
    small = _cylindrical_face(center=(0, 0, 0), axis=(0, 0, 1), radius=0.05)
    small_match = _cylindrical_face(center=(0, 0, 0.1), axis=(0, 0, 1), radius=0.05)
    large = _cylindrical_face(center=(1, 0, 0), axis=(0, 0, 1), radius=0.2)
    large_match = _cylindrical_face(center=(1, 0, 0.1), axis=(0, 0, 1), radius=0.2)
    pairs = find_matching_face_pairs(_body([small, large]), _body([small_match, large_match]))
    assert len(pairs) == 2
    assert pairs[0][0].radius == 0.2  # largest first
    assert pairs[1][0].radius == 0.05


def _quad_body(x0, x1, normal, center):
    """Single square face in the z=0 plane spanning x0..x1, y 0..1, with real tessellation."""
    verts = [(x0, 0, 0), (x1, 0, 0), (x1, 1, 0), (x0, 1, 0)]
    face = Face(kind="plane", origin="brep", normal=normal, center=center,
                residual=0.0, triangle_indices=[0, 1])
    return Body(id="b", name="n", vertices=verts, triangles=[(0, 1, 2), (0, 2, 3)], faces=[face])


def test_coplanar_touching_planes_with_different_centers_match():
    a = _quad_body(0, 1, (0, 0, 1), (0.1, 0.9, 0))    # arbitrary point on plane
    b = _quad_body(0.5, 1.5, (0, 0, -1), (0.7, 0.2, 0))
    assert len(find_matching_face_pairs(a, b)) == 1


def test_coplanar_planes_with_disjoint_tessellation_do_not_match():
    a = _quad_body(0, 1, (0, 0, 1), (0, 0, 0))
    b = _quad_body(5, 6, (0, 0, -1), (5, 0, 0))
    assert find_matching_face_pairs(a, b) == []


def _cyl_body(z0):
    m = trimesh.creation.cylinder(radius=0.1, height=0.2, sections=16)
    m.apply_translation((0, 0, z0))
    n = len(m.faces)
    face = Face(kind="cylinder", origin="brep", axis=(0, 0, 1), center=(0, 0, z0),
                radius=0.1, residual=0.0, triangle_indices=list(range(n)))
    return Body(id="b", name="n", vertices=[tuple(v) for v in m.vertices],
                triangles=[tuple(int(i) for i in t) for t in m.faces], faces=[face])


def test_coaxial_cylinders_separated_along_axis_do_not_match():
    assert find_matching_face_pairs(_cyl_body(0.0), _cyl_body(1.0)) == []
    assert len(find_matching_face_pairs(_cyl_body(0.0), _cyl_body(0.1))) == 1


def test_step_peg_in_hole_yields_cylindrical_pair(peg_in_hole_step_path):
    from app.importers.step_importer import import_step
    bodies = import_step(peg_in_hole_step_path)
    block, peg = bodies[0], bodies[1]
    pairs = find_matching_face_pairs(block, peg)
    assert any(a.kind == "cylinder" and math.isclose(a.radius, 0.1, rel_tol=1e-3) for a, _ in pairs)


def test_matching_large_body_against_itself_is_fast(holed_plate_step_path):
    import time
    from app.importers.step_importer import import_step
    body = import_step(holed_plate_step_path)[0]
    assert len(body.faces) >= 60 and len(body.triangles) >= 5000
    t = time.perf_counter()
    find_matching_face_pairs(body, body)
    assert time.perf_counter() - t < 1.0
