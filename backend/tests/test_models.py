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


import pytest
from pydantic import ValidationError

_NAN = float("nan")


def _face(**kw):
    d = dict(kind="plane", origin="brep", normal=(0, 0, 1), center=(0, 0, 0),
             residual=0.0, triangle_indices=[])
    d.update(kw)
    return d


def _joint(**kw):
    d = dict(id="j", parent_link_id="a", child_link_id="b", type="revolute",
             origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0), axis=(0, 0, 1),
             limit_lower=-1.0, limit_upper=1.0, source="confirmed")
    d.update(kw)
    return d


@pytest.mark.parametrize("kw", [
    dict(normal=None),                                    # plane without normal
    dict(normal=(0, 0, 0)),                               # zero normal
    dict(center=(_NAN, 0, 0)),                            # non-finite
    dict(residual=float("inf")),
    dict(kind="cylinder", normal=None, axis=None, radius=1.0),
    dict(kind="cylinder", normal=None, axis=(0, 0, 0), radius=1.0),
    dict(kind="cylinder", normal=None, axis=(0, 0, 1), radius=None),
    dict(kind="cylinder", normal=None, axis=(0, 0, 1), radius=0.0),
    dict(kind="sphere", normal=None, radius=None),
    dict(kind="sphere", normal=None, radius=-1.0),
])
def test_bad_face_rejected(kw):
    with pytest.raises(ValidationError):
        Face(**_face(**kw))


def test_good_faces_accepted():
    Face(**_face())
    Face(**_face(kind="cylinder", normal=None, axis=(0, 0, 1), radius=0.1))
    Face(**_face(kind="sphere", normal=None, radius=0.1))
    Face(**_face(kind="unclassified", normal=None))


def test_body_triangle_index_out_of_range_rejected():
    with pytest.raises(ValidationError):
        Body(id="b", name="b", vertices=[(0, 0, 0), (1, 0, 0), (0, 1, 0)],
             triangles=[(0, 1, 3)])
    with pytest.raises(ValidationError):
        Body(id="b", name="b", vertices=[(0, 0, 0), (1, 0, 0), (0, 1, 0)],
             triangles=[(0, 1, -1)])


def test_body_face_triangle_index_out_of_range_rejected():
    with pytest.raises(ValidationError):
        Body(id="b", name="b", vertices=[(0, 0, 0), (1, 0, 0), (0, 1, 0)],
             triangles=[(0, 1, 2)], faces=[Face(**_face(triangle_indices=[1]))])
    Body(id="b", name="b", vertices=[(0, 0, 0), (1, 0, 0), (0, 1, 0)],
         triangles=[(0, 1, 2)], faces=[Face(**_face(triangle_indices=[0]))])


@pytest.mark.parametrize("kw", [
    dict(axis=(0, 0, 0)),
    dict(limit_lower=1.0, limit_upper=-1.0),
    dict(origin_xyz=(_NAN, 0, 0)),
])
def test_bad_joint_rejected(kw):
    with pytest.raises(ValidationError):
        Joint(**_joint(**kw))


def test_link_non_finite_density_rejected():
    with pytest.raises(ValidationError):
        Link(id="l", name="l", body_ids=[], density_kg_m3=float("inf"))


import math
from app.models import PartWarning, STATUS_ORDER


def test_body_defaults_to_ok_with_no_warnings():
    b = Body(id="b", name="n", vertices=[], triangles=[])
    assert b.status == "ok"
    assert b.warnings == []


def test_body_carries_status_and_warnings():
    w = PartWarning(code="open_geometry", message="p: 4 open edges", fixes=["set_mass", "replace", "ignore"])
    b = Body(id="b", name="p", vertices=[], triangles=[], status="approximate", warnings=[w])
    assert b.warnings[0].fixes == ["set_mass", "replace", "ignore"]


def test_unknown_status_or_fix_rejected():
    with pytest.raises(ValidationError):
        Body(id="b", name="n", vertices=[], triangles=[], status="broken")
    with pytest.raises(ValidationError):
        PartWarning(code="x", message="m", fixes=["delete_everything"])


def test_link_mass_override_defaults():
    l = Link(id="l", name="l", body_ids=[], density_kg_m3=1000.0)
    assert l.mass_override_kg is None
    assert l.accept_approximate_mass is False


@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf])
def test_link_mass_override_must_be_finite_positive(bad):
    with pytest.raises(ValidationError):
        Link(id="l", name="l", body_ids=[], density_kg_m3=1000.0, mass_override_kg=bad)


def test_status_order_is_worst_last():
    assert sorted(STATUS_ORDER, key=STATUS_ORDER.get) == ["ok", "repaired", "approximate", "failed"]
