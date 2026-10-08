import numpy as np
import trimesh
from app.importers.placement import combine_bodies, place_replacement
from app.models import Body, Face, PartWarning


def _box(bid, offset=(0, 0, 0), status="ok"):
    m = trimesh.creation.box(extents=(1, 1, 1)); m.apply_translation(offset)
    return Body(id=bid, name=bid, vertices=[tuple(v) for v in m.vertices.tolist()],
                triangles=[tuple(t) for t in m.faces.tolist()], status=status,
                faces=[Face(kind="plane", origin="segmented", normal=(0, 0, 1), center=(0, 0, 0),
                            residual=0.0, triangle_indices=[0, 1])])


def test_combine_shifts_indices_and_takes_worst_status():
    a, b = _box("a"), _box("b", (5, 0, 0), status="approximate")
    b = b.model_copy(update={"warnings": [PartWarning(code="open_geometry", message="m")]})
    c = combine_bodies([a, b], "orig", "part")
    assert c.id == "orig" and c.name == "part"
    assert len(c.vertices) == 16 and len(c.triangles) == 24
    assert c.faces[1].triangle_indices == [12, 13]
    assert max(i for t in c.triangles for i in t) == 15
    assert c.status == "approximate" and [w.code for w in c.warnings] == ["open_geometry"]


def test_overlapping_replacement_keeps_coordinates():
    r = place_replacement(_box("r", (0.2, 0, 0)), (-0.5, -0.5, -0.5), (0.5, 0.5, 0.5))
    assert np.allclose(np.array(r.vertices).min(0), [-0.3, -0.5, -0.5])
    assert r.warnings == []


def test_non_overlapping_replacement_is_centred_with_warning():
    r = place_replacement(_box("r", (100, 0, 0)), (9.5, -0.5, -0.5), (10.5, 0.5, 0.5))
    v = np.array(r.vertices)
    assert np.allclose((v.min(0) + v.max(0)) / 2, [10, 0, 0])
    assert np.allclose(r.faces[0].center, (-90, 0, 0))     # faces move with the body
    assert [w.code for w in r.warnings] == ["placed_by_bbox"]
    assert r.warnings[0].fixes == ["replace"]


def test_no_original_box_keeps_coordinates_unverified():
    r = place_replacement(_box("r", (100, 0, 0)), None, None)
    assert np.allclose(np.array(r.vertices).min(0), [99.5, -0.5, -0.5])
    assert [w.code for w in r.warnings] == ["position_unverified"]


def test_combine_failed_plus_ok_is_approximate_with_geometry():
    failed = Body(id="f", name="f", vertices=[], triangles=[], status="failed")
    c = combine_bodies([failed, _box("a")], "orig", "part")
    assert c.status == "approximate" and len(c.vertices) == 8
    assert combine_bodies([failed], "orig", "part").status == "failed"
