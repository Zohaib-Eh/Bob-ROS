from app.models import Link, Joint
from app.validate.structural import validate_structure


def _link(id_, mass=1.0, inertia=(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)):
    return Link(
        id=id_, name=id_, body_ids=[], density_kg_m3=1000.0,
        mass_kg=mass, center_of_mass=(0, 0, 0), inertia=list(inertia),
    )


def _joint(id_, parent, child, type_="fixed"):
    return Joint(
        id=id_, parent_link_id=parent, child_link_id=child, type=type_,
        origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0), axis=(0, 0, 1),
        source="confirmed",
    )


def test_valid_two_link_tree_has_no_errors():
    links = [_link("base"), _link("arm")]
    joints = [_joint("j1", "base", "arm")]
    assert validate_structure(links, joints) == []


def test_two_roots_is_an_error():
    links = [_link("base"), _link("arm1"), _link("arm2")]
    joints = []  # arm1 and arm2 both have no parent -> two roots alongside base
    errors = validate_structure(links, joints)
    assert any("root" in e.lower() for e in errors)


def test_cycle_is_an_error():
    links = [_link("a"), _link("b")]
    joints = [_joint("j1", "a", "b"), _joint("j2", "b", "a")]
    errors = validate_structure(links, joints)
    assert any("cycle" in e.lower() for e in errors)


def test_non_leaf_link_with_zero_inertia_is_an_error():
    zero_inertia_parent = _link("base", inertia=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    child = _link("arm")
    links = [zero_inertia_parent, child]
    joints = [_joint("j1", "base", "arm")]
    errors = validate_structure(links, joints)
    assert any("inertia" in e.lower() for e in errors)


def test_leaf_link_with_zero_mass_is_an_error():
    links = [_link("base"), _link("arm", mass=0.0)]
    joints = [_joint("j1", "base", "arm")]
    errors = validate_structure(links, joints)
    assert any("mass" in e.lower() and "arm" in e for e in errors)


def test_link_with_missing_mass_is_an_error():
    links = [_link("base"), _link("arm", mass=None)]
    joints = [_joint("j1", "base", "arm")]
    errors = validate_structure(links, joints)
    assert any("mass" in e.lower() and "arm" in e for e in errors)


def test_child_with_two_parent_joints_is_an_error():
    links = [_link("base"), _link("a"), _link("b")]
    joints = [
        _joint("j1", "base", "a"),
        _joint("j2", "base", "b"),
        _joint("j3", "a", "b"),
    ]
    errors = validate_structure(links, joints)
    assert any("parent" in e.lower() and "'b'" in e for e in errors)
