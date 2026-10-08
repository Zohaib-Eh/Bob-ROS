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
        # each normal is exactly an outward axis vector
        axis = max(range(3), key=lambda i: abs(f.normal[i]))
        expected = [0.0, 0.0, 0.0]
        expected[axis] = 1.0 if f.center[axis] > 0 else -1.0
        assert all(
            math.isclose(f.normal[i], expected[i], abs_tol=1e-9) for i in range(3)
        )
        # face sits on the box surface at +/-0.5 along its normal axis
        assert math.isclose(f.center[axis], 0.5 * expected[axis], abs_tol=1e-9)
        assert f.residual < 1e-9
    # the six normals cover the three axes, each appearing twice (+/-)
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
    cyl = cyl_faces[0]
    assert cyl.origin == "segmented"
    assert math.isclose(cyl.radius, 0.5, rel_tol=1e-6)
    dominant_axis = max(range(3), key=lambda i: abs(cyl.axis[i]))
    assert dominant_axis == 2  # trimesh.creation.cylinder is Z-aligned
    # axis passes through the Z axis
    assert abs(cyl.center[0]) < 1e-9 and abs(cyl.center[1]) < 1e-9
    assert cyl.residual < 1e-6

def test_sphere_region_is_unclassified():
    mesh = trimesh.creation.icosphere(subdivisions=2, radius=0.5)
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    assert len(regions) == 1
    face = fit_face(vertices, triangles, regions[0])
    assert face.kind == "unclassified"
    assert face.origin == "segmented"
    assert face.residual == 1.0

def test_empty_region_is_unclassified():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    face = fit_face(vertices, triangles, [])
    assert face.kind == "unclassified"
    assert face.origin == "segmented"
    assert face.residual == 1.0
