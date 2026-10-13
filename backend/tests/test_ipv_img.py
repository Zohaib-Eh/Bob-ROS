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


def _ring(n, ring_r, hole_r, axis, z0=0.0):
    """n equal holes on a circle of radius ring_r around the line through (0,0,z0) along axis."""
    a = [x / math.sqrt(sum(c * c for c in axis)) for x in axis]
    u = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = [u[1] * a[2] - u[2] * a[1], u[2] * a[0] - u[0] * a[2], u[0] * a[1] - u[1] * a[0]]
    nu = math.sqrt(sum(c * c for c in u)); u = [c / nu for c in u]
    v = [a[1] * u[2] - a[2] * u[1], a[2] * u[0] - a[0] * u[2], a[0] * u[1] - a[1] * u[0]]
    pairs = []
    for k in range(n):
        t = 2 * math.pi * k / n + 0.3
        c = tuple(ring_r * (math.cos(t) * u[i] + math.sin(t) * v[i]) + (z0 * a[i]) for i in range(3))
        pairs.append((_cyl(c, tuple(a), hole_r), _cyl(c, tuple(a), hole_r)))
    return pairs, a


def _dist_to_line(p, a, point=(0.0, 0.0, 0.0)):
    d = [p[i] - point[i] for i in range(3)]
    along = sum(d[i] * a[i] for i in range(3))
    return math.sqrt(sum((d[i] - along * a[i]) ** 2 for i in range(3)))


def test_bolt_hole_ring_puts_the_axis_at_the_ring_centre():
    pairs, a = _ring(4, 0.02, 0.0025, (0.6757, 0.685, 0.2723), z0=1.2)
    joint = classify_joint(pairs, "parent", "child")
    assert _dist_to_line(joint.origin_xyz, a) < 1e-9
    assert abs(abs(sum(joint.axis[i] * a[i] for i in range(3))) - 1) < 1e-9


def test_largest_cylinder_wins_over_bolt_holes_listed_first():
    holes, a = _ring(4, 0.05, 0.0025, (0.0, 0.0, 1.0))
    bore = (_cyl((0.0, 0.0, 0.3), (0, 0, 1), 0.03), _cyl((0.0, 0.0, 0.3), (0, 0, 1), 0.03))
    joint = classify_joint(holes + [bore], "parent", "child")
    assert joint.origin_xyz == (0.0, 0.0, 0.3)


def test_two_holes_or_holes_in_a_row_keep_the_first_hole():
    two, _ = _ring(2, 0.02, 0.0025, (0.0, 0.0, 1.0))
    assert classify_joint(two, "p", "c").origin_xyz == two[0][0].center
    row = [(_cyl((x, 0.0, 0.0), (0, 0, 1), 0.0025), _cyl((x, 0.0, 0.0), (0, 0, 1), 0.0025)) for x in (0.0, 0.01, 0.02)]
    assert classify_joint(row, "p", "c").origin_xyz == (0.0, 0.0, 0.0)
