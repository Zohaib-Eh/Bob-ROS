import math

import pytest
import trimesh

from app.importers.step_importer import import_step


def _mesh(body):
    m = trimesh.Trimesh(vertices=body.vertices, faces=body.triangles, process=False)
    return m


def test_box_import_has_six_planar_brep_faces(box_step_path):
    bodies = import_step(box_step_path)
    assert len(bodies) == 1
    faces = bodies[0].faces
    assert len(faces) == 6
    assert all(f.kind == "plane" for f in faces)
    assert all(f.origin == "brep" for f in faces)
    assert all(f.residual == 0.0 for f in faces)


def test_box_mesh_is_watertight_with_volume_6(box_step_path):
    body = import_step(box_step_path)[0]
    m = _mesh(body)
    assert m.is_watertight
    assert math.isclose(m.volume, 6.0, rel_tol=1e-3)
    # face triangle indices must stay valid and cover every triangle once
    idx = sorted(i for f in body.faces for i in f.triangle_indices)
    assert idx == list(range(len(body.triangles)))


def test_cylinder_import_has_correct_radius_and_axis(cylinder_step_path):
    faces = import_step(cylinder_step_path)[0].faces
    cyl_faces = [f for f in faces if f.kind == "cylinder"]
    assert len(cyl_faces) == 1
    assert math.isclose(cyl_faces[0].radius, 0.5, rel_tol=1e-6)
    assert math.isclose(abs(cyl_faces[0].axis[2]), 1.0, rel_tol=1e-6)


def test_cylinder_mesh_volume(cylinder_step_path):
    m = _mesh(import_step(cylinder_step_path)[0])
    assert math.isclose(m.volume, math.pi * 0.25 * 2, rel_tol=0.01)


def test_sphere_import_has_correct_radius(sphere_step_path):
    faces = import_step(sphere_step_path)[0].faces
    sph_faces = [f for f in faces if f.kind == "sphere"]
    assert len(sph_faces) == 1
    assert math.isclose(sph_faces[0].radius, 0.75, rel_tol=1e-6)


def test_peg_in_hole_import_returns_two_bodies(peg_in_hole_step_path):
    bodies = import_step(peg_in_hole_step_path)
    assert len(bodies) == 2
    for b in bodies:
        radii = [f.radius for f in b.faces if f.kind == "cylinder"]
        assert any(math.isclose(r, 0.1, rel_tol=1e-6) for r in radii)


def test_step_declared_in_mm_imports_in_meters(cylinder_mm_step_path):
    faces = import_step(cylinder_mm_step_path)[0].faces
    cyl = [f for f in faces if f.kind == "cylinder"][0]
    assert math.isclose(cyl.radius, 0.5, rel_tol=1e-6)


def test_iges_import_in_meters(cylinder_iges_path):
    faces = import_step(cylinder_iges_path)[0].faces
    cyl = [f for f in faces if f.kind == "cylinder"][0]
    assert math.isclose(cyl.radius, 0.5, rel_tol=1e-6)


def test_brep_import_scaled_to_meters(cylinder_brep_path):
    faces = import_step(cylinder_brep_path)[0].faces
    cyl = [f for f in faces if f.kind == "cylinder"][0]
    assert math.isclose(cyl.radius, 0.5, rel_tol=1e-6)


def test_unknown_extension_raises(tmp_path):
    p = tmp_path / "x.foo"
    p.write_text("x")
    with pytest.raises(ValueError):
        import_step(str(p))
