import math
import pytest
from app.importers.mesh_importer import import_mesh

def test_default_scale_converts_mm_to_meters(box_stl_path):
    body, applied_scale = import_mesh(box_stl_path)
    assert math.isclose(applied_scale, 0.001)
    xs = [v[0] for v in body.vertices]
    span_x = max(xs) - min(xs)
    assert math.isclose(span_x, 1.0, rel_tol=1e-6)  # 1000mm -> 1m

def test_scale_override_is_respected(box_stl_path):
    body, applied_scale = import_mesh(box_stl_path, scale=1.0)
    assert applied_scale == 1.0
    xs = [v[0] for v in body.vertices]
    span_x = max(xs) - min(xs)
    assert math.isclose(span_x, 1000.0, rel_tol=1e-6)

def test_non_watertight_mesh_raises_clear_error(non_watertight_stl_path):
    with pytest.raises(ValueError, match="watertight"):
        import_mesh(non_watertight_stl_path)

def test_non_positive_scale_is_rejected(box_stl_path):
    with pytest.raises(ValueError, match="scale"):
        import_mesh(box_stl_path, scale=0.0)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1.0])
def test_non_finite_or_negative_scale_rejected(box_stl_path, bad):
    with pytest.raises(ValueError):
        import_mesh(box_stl_path, scale=bad)


def test_empty_mesh_raises_could_not_read(tmp_path):
    p = tmp_path / "empty.stl"
    p.write_bytes(b"")
    with pytest.raises(ValueError, match="could not read mesh"):
        import_mesh(str(p))


def test_inverted_stl_is_reoriented_outward(tmp_path):
    import numpy as np
    import trimesh
    from app.massprops.link_mass import compute_link_mass_properties
    inv = trimesh.creation.box(extents=(10.0, 10.0, 10.0))
    inv.invert()
    p = tmp_path / "inv.stl"
    inv.export(str(p))
    body, _ = import_mesh(str(p))
    m = trimesh.Trimesh(body.vertices, body.triangles, process=False)
    assert m.volume == pytest.approx(1e-6, rel=1e-6)  # 10 mm cube in m^3
    # every face normal points away from the centroid
    assert np.all(np.einsum("ij,ij->i", m.face_normals, m.triangles_center - m.centroid) > 0)
    mass, _, _ = compute_link_mass_properties([body], 1000.0)
    assert mass == pytest.approx(1e-3, rel=1e-6)
