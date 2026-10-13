import zipfile

import numpy as np
import pytest
import trimesh

from app.ros.chain import parse_chain
from app.ros.collisions import disabled_pairs, link_meshes
from app.ros.fk import fk

# Four links turning about the same z axis through the origin; boxes offset along x.
_LINKS = {"base": (0.2, 0.0), "l1": (0.1, 0.5), "l2": (0.1, 2.0), "l3": (0.1, -0.5), "l4": (0.1, 0.1)}


def _urdf():
    links = "".join(
        f"<link name='{n}'><collision><origin xyz='{x} 0 0'/><geometry>"
        f"<mesh filename='package://r/meshes/{n}.stl'/></geometry></collision></link>"
        for n, (_, x) in _LINKS.items())
    names = list(_LINKS)
    joints = "".join(
        f"<joint name='j{i}' type='revolute'><parent link='{names[i - 1]}'/><child link='{names[i]}'/>"
        f"<axis xyz='0 0 1'/><limit lower='-3.14159' upper='3.14159' effort='1' velocity='1'/></joint>"
        for i in range(1, len(names)))
    return f"<robot name='r'>{links}{joints}</robot>"


@pytest.fixture
def robot(tmp_path):
    (tmp_path / "meshes").mkdir()
    for n, (size, _) in _LINKS.items():
        trimesh.creation.box(extents=(size, size, size)).export(tmp_path / "meshes" / f"{n}.stl")
    return parse_chain(_urdf()), link_meshes(_urdf(), tmp_path)


def test_link_meshes_apply_the_collision_origin(robot):
    _, meshes = robot
    v, f = meshes["l2"]
    assert v.mean(axis=0) == pytest.approx([2.0, 0, 0])
    assert len(f) == 12


def test_pairs_are_classified(robot):
    chain, meshes = robot
    pairs = disabled_pairs(chain, meshes, samples=2000)
    assert pairs[("base", "l1")] == "Adjacent"
    assert pairs[("base", "l4")] == "Default"      # l4 cuts through the base surface at home
    # (fcl tests triangles: a mesh fully inside another does not count as a collision)
    assert pairs[("base", "l2")] == "Never"        # l2 circles 2 m out
    assert pairs[("base", "l3")] == "Never"
    assert pairs[("l1", "l4")] == "Never"
    assert ("l1", "l3") not in pairs               # they meet when l3 swings round: keep checking


def test_irb1300_tip_frame_sits_on_link6(irb1300_zip, tmp_path):
    with zipfile.ZipFile(irb1300_zip) as z:
        z.extractall(tmp_path)
    urdf = (tmp_path / "irb1300" / "urdf" / "irb1300.urdf").read_text()
    chain = parse_chain(urdf)
    T = fk(chain, np.zeros(6))
    assert T[:3, :3] == pytest.approx(np.eye(3))   # exporter frames are world-aligned
    v, _ = link_meshes(urdf, tmp_path / "irb1300")["link6"]
    lo, hi = v.min(axis=0) - 0.01, v.max(axis=0) + 0.01
    assert np.all(lo <= 0) and np.all(0 <= hi)     # joint 6's origin lies on link6's geometry
