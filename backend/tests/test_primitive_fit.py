import math
import trimesh
from app.segment.primitive_fit import fit_face
from app.segment.region_growing import grow_regions

def test_box_face_fits_as_plane_with_correct_normal():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    faces = [fit_face(vertices, triangles, r) for r in regions]
    assert all(f.kind == "plane" for f in faces)
    assert all(f.origin == "segmented" for f in faces)
    for f in faces:
        length = math.sqrt(sum(c * c for c in f.normal))
        assert math.isclose(length, 1.0, rel_tol=1e-3)
    # the six normals should be the three axes, each appearing twice (+/-)
    axis_hits = {0: 0, 1: 0, 2: 0}
    for f in faces:
        dominant_axis = max(range(3), key=lambda i: abs(f.normal[i]))
        axis_hits[dominant_axis] += 1
    assert axis_hits == {0: 2, 1: 2, 2: 2}

def test_cylinder_side_fits_with_correct_radius_and_axis():
    mesh = trimesh.creation.cylinder(radius=0.5, height=2.0, sections=64)
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    faces = [fit_face(vertices, triangles, r) for r in regions]
    cyl_faces = [f for f in faces if f.kind == "cylinder"]
    assert len(cyl_faces) == 1
    assert math.isclose(cyl_faces[0].radius, 0.5, rel_tol=0.02)
    dominant_axis = max(range(3), key=lambda i: abs(cyl_faces[0].axis[i]))
    assert dominant_axis == 2  # trimesh.creation.cylinder is Z-aligned
