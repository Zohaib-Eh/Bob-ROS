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
