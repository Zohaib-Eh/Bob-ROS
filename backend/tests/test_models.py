from app.models import Face, Body, Link, Joint

def test_face_requires_valid_origin():
    face = Face(
        kind="plane", origin="brep", normal=(0, 0, 1), axis=None,
        center=(0, 0, 0), radius=None, residual=0.0, triangle_indices=[0, 1],
    )
    assert face.origin == "brep"

def test_face_rejects_invalid_origin():
    import pytest
    from pydantic import ValidationError
    with pytest.raises(ValidationError):
        Face(
            kind="plane", origin="guessed", normal=(0, 0, 1), axis=None,
            center=(0, 0, 0), radius=None, residual=0.0, triangle_indices=[0],
        )

def test_body_holds_faces():
    body = Body(id="b1", name="box", vertices=[(0, 0, 0)], triangles=[], faces=[])
    assert body.id == "b1"
    assert body.faces == []

def test_link_defaults_unset_mass_properties_to_none():
    link = Link(id="l1", name="base", body_ids=["b1"], density_kg_m3=1000.0)
    assert link.mass_kg is None
    assert link.center_of_mass is None

def test_joint_defaults_source_to_manual_when_unspecified_is_rejected():
    import pytest
    from pydantic import ValidationError
    with pytest.raises(ValidationError):
        Joint(
            id="j1", parent_link_id="l1", child_link_id="l2", type="revolute",
            origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0), axis=(0, 0, 1),
            limit_lower=None, limit_upper=None,
        )  # missing required `source`
