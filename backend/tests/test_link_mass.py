import math
import pytest
import trimesh
from app.models import Body
from app.massprops.link_mass import compute_link_mass_properties


def _body_from_trimesh(mesh) -> Body:
    return Body(
        id="b1", name="test",
        vertices=[tuple(v) for v in mesh.vertices.tolist()],
        triangles=[tuple(t) for t in mesh.faces.tolist()],
        faces=[],
    )


def test_unit_box_mass_matches_volume_times_density():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    body = _body_from_trimesh(mesh)
    mass, com, inertia = compute_link_mass_properties([body], density_kg_m3=1000.0)
    assert math.isclose(mass, 1000.0, rel_tol=1e-3)  # 1 m^3 * 1000 kg/m^3
    assert all(math.isclose(c, 0.0, abs_tol=1e-6) for c in com)


def test_known_cube_inertia_matches_closed_form():
    # solid cube inertia about its own center: I = (1/6) * m * s^2 for each axis
    side = 2.0
    mesh = trimesh.creation.box(extents=(side, side, side))
    body = _body_from_trimesh(mesh)
    density = 500.0
    mass, com, inertia = compute_link_mass_properties([body], density_kg_m3=density)
    expected_i = (mass / 6.0) * side**2
    ixx, iyy, izz = inertia[0], inertia[1], inertia[2]
    assert math.isclose(ixx, expected_i, rel_tol=0.02)
    assert math.isclose(iyy, expected_i, rel_tol=0.02)
    assert math.isclose(izz, expected_i, rel_tol=0.02)


def test_degenerate_zero_volume_body_raises():
    # a flat, zero-thickness "box" has zero volume
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 0.0))
    body = _body_from_trimesh(mesh)
    with pytest.raises(ValueError, match="volume"):
        compute_link_mass_properties([body], density_kg_m3=1000.0)


def test_offset_body_center_of_mass_is_in_world_frame():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    mesh.apply_translation((1.0, 2.0, 3.0))
    body = _body_from_trimesh(mesh)
    _, com, _ = compute_link_mass_properties([body], density_kg_m3=1000.0)
    assert all(math.isclose(c, e, abs_tol=1e-6) for c, e in zip(com, (1.0, 2.0, 3.0)))


def test_inverted_box_raises_instead_of_negative_mass():
    # faces wound inward -> signed volume is -1 m^3
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    mesh.faces = mesh.faces[:, ::-1]
    body = _body_from_trimesh(mesh)
    with pytest.raises(ValueError, match="volume"):
        compute_link_mass_properties([body], density_kg_m3=1000.0)
