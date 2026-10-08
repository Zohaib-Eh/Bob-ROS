import math
from app.models import Face
from app.joints.ipv_img import classify_joint


def _cyl(center, axis, radius):
    return Face(kind="cylinder", origin="brep", normal=None, axis=axis,
                center=center, radius=radius, residual=0.0, triangle_indices=[])


def _plane(center, normal):
    return Face(kind="plane", origin="brep", normal=normal, axis=None,
                center=center, radius=None, residual=0.0, triangle_indices=[])


def test_no_matched_pairs_is_fixed():
    joint = classify_joint([], "parent", "child")
    assert joint.type == "fixed"
    assert joint.axis is None
    assert joint.source == "suggested"


def test_cylindrical_pair_alone_is_cylindrical_joint():
    pair = (_cyl((0, 0, 0), (0, 0, 1), 0.1), _cyl((0, 0, 0.1), (0, 0, 1), 0.1))
    joint = classify_joint([pair], "parent", "child")
    assert joint.type == "cylindrical"
    assert math.isclose(abs(joint.axis[2]), 1.0, rel_tol=1e-6)
    assert joint.source == "suggested"
    assert joint.ambiguous is False


def test_cylindrical_plus_perpendicular_planar_is_revolute():
    cyl_pair = (_cyl((0, 0, 0), (0, 0, 1), 0.1), _cyl((0, 0, 0.1), (0, 0, 1), 0.1))
    plane_pair = (_plane((0, 0, 0.2), (0, 0, 1)), _plane((0, 0, 0.2), (0, 0, -1)))
    joint = classify_joint([cyl_pair, plane_pair], "parent", "child")
    assert joint.type == "revolute"
    assert joint.source == "suggested"


def test_planar_pair_alone_is_planar_joint():
    pair = (_plane((0, 0, 0), (0, 0, 1)), _plane((0, 0, 0), (0, 0, -1)))
    joint = classify_joint([pair], "parent", "child")
    assert joint.type == "planar"
    assert joint.source == "suggested"


def test_multiple_cylindrical_pairs_use_largest_and_flag_ambiguous():
    big = (_cyl((0, 0, 0), (0, 0, 1), 0.2), _cyl((0, 0, 0.1), (0, 0, 1), 0.2))
    small = (_cyl((1, 0, 0), (0, 1, 0), 0.05), _cyl((1, 0, 0.1), (0, 1, 0), 0.05))
    joint = classify_joint([big, small], "parent", "child")
    assert joint.type == "cylindrical"
    assert joint.origin_xyz == (0, 0, 0)
    assert math.isclose(abs(joint.axis[2]), 1.0, rel_tol=1e-6)
    assert joint.source == "suggested"
    assert joint.ambiguous is True
