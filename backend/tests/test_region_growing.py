import pytest
import trimesh
from app.segment.region_growing import grow_regions

def test_box_produces_six_regions():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    assert len(regions) == 6
    # every triangle accounted for exactly once
    all_tris = sorted(i for region in regions for i in region)
    assert all_tris == list(range(len(triangles)))

def test_cylinder_produces_three_regions():
    mesh = trimesh.creation.cylinder(radius=0.5, height=2.0, sections=64)
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    # one curved side region + two flat end-cap regions
    assert len(regions) == 3

def test_cylinder_covers_every_triangle_once_with_analytic_sizes():
    mesh = trimesh.creation.cylinder(radius=0.5, height=2.0, sections=64)
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    all_tris = sorted(i for region in regions for i in region)
    assert all_tris == list(range(len(triangles)))
    # side: 2*64 = 128 tris; each center-fan cap: 64 tris
    assert sorted(len(r) for r in regions) == [64, 64, 128]

def test_open_mesh_raises_watertight_error():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    triangles = [tuple(t) for t in mesh.faces.tolist()][:-1]  # drop one face
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    with pytest.raises(ValueError, match="watertight"):
        grow_regions(vertices, triangles)

def test_unwelded_triangle_soup_raises_watertight_error():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    soup_vertices = []
    soup_triangles = []
    for i, tri in enumerate(mesh.faces.tolist()):
        soup_vertices.extend(tuple(mesh.vertices[j]) for j in tri)
        soup_triangles.append((3 * i, 3 * i + 1, 3 * i + 2))
    with pytest.raises(ValueError, match="watertight"):
        grow_regions(soup_vertices, soup_triangles)


def test_open_mesh_grows_regions_when_allowed():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    mesh.update_faces(mesh.face_normals[:, 2] < 0.5)   # remove the +z side (2 triangles)
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles, allow_open=True)
    assert len(regions) == 5
    assert sorted(i for r in regions for i in r) == list(range(10))
