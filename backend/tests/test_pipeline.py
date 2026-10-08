import numpy as np
import trimesh
from app.importers.pipeline import import_path, segment_mesh_body
from app.models import Body


def _body(mesh, status="ok"):
    return Body(id="b", name="m", vertices=[tuple(v) for v in mesh.vertices.tolist()],
                triangles=[tuple(t) for t in mesh.faces.tolist()], status=status)


def test_segment_closed_box_gives_six_planes():
    b = segment_mesh_body(_body(trimesh.creation.box(extents=(1, 1, 1))))
    assert len(b.faces) == 6 and {f.kind for f in b.faces} == {"plane"}
    assert {f.origin for f in b.faces} == {"segmented"}


def test_segment_open_box_still_gets_faces():
    m = trimesh.creation.box(extents=(1, 1, 1))
    m.update_faces(m.face_normals[:, 2] < 0.5)          # remove the +z side
    b = segment_mesh_body(_body(m, status="approximate"))
    assert len(b.faces) == 5
    assert b.status == "approximate"


def test_segment_triangle_soup_is_degenerate_not_error():
    rng = np.random.default_rng(0)
    verts = rng.random((30, 3)).tolist()
    tris = [(i, i + 1, i + 2) for i in range(0, 30, 3)]          # 10 unconnected triangles
    b = segment_mesh_body(Body(id="b", name="soup", vertices=[tuple(v) for v in verts],
                               triangles=tris, status="approximate"))
    assert b.faces == []
    assert b.warnings[-1].code == "degenerate"
    assert b.warnings[-1].fixes == ["replace", "ignore"]


def test_import_path_step_returns_unit_scale(cylinder_step_path):
    bodies, scale, warnings = import_path(cylinder_step_path, ".step", 0.001)
    assert scale == 1.0 and len(bodies) == 1 and warnings == []


def test_import_path_brep_uses_scale(mixed_topology_brep_path):
    bodies, scale, warnings = import_path(mixed_topology_brep_path, ".brep", 1.0)
    assert scale == 1.0 and len(bodies) == 4


def test_import_path_stl_segments(box_stl_path):
    bodies, scale, _ = import_path(box_stl_path, ".stl", 0.001)
    assert scale == 0.001 and len(bodies[0].faces) == 6


def test_import_path_display_name_names_mesh_body(box_stl_path):
    bodies, _, _ = import_path(box_stl_path, ".stl", 0.001, display_name="widget")
    assert bodies[0].name == "widget"
