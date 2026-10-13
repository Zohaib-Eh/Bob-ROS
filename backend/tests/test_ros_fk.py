import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.ros.chain import parse_chain
from app.ros.fk import fk, link_poses
from tests.ros_helpers import two_joint_zip, unzip
from tests.test_ros_chain import ARM


def _rz(a):
    return Rotation.from_euler("z", a).as_matrix()


def test_fk_turns_the_arm_about_each_joint_axis():
    c = parse_chain(ARM)
    T = fk(c, [math.pi / 2, 0.5, 0.0])
    # j1 at (0,0,0.1) about z; j2 1 m out along l1's x; tip 0.5 m further out
    a = math.pi / 2 + 0.5
    expected = np.array([0, 1, 0.1]) + 0.5 * np.array([math.cos(a), math.sin(a), 0])
    assert T[:3, 3] == pytest.approx(expected)
    assert T[:3, :3] == pytest.approx(_rz(a))


def test_continuous_wrist_rotates_about_its_x_axis():
    c = parse_chain(ARM)
    T = fk(c, [0.0, 0.5, math.pi / 2])
    assert T[:3, :3] == pytest.approx(_rz(0.5) @ Rotation.from_euler("x", math.pi / 2).as_matrix())


def test_prismatic_joint_translates_along_its_axis():
    urdf = ("<robot name='p'><link name='base'/><link name='slide'/>"
            "<joint name='s' type='prismatic'><parent link='base'/><child link='slide'/>"
            "<axis xyz='1 0 0'/><limit lower='0' upper='1' effort='1' velocity='1'/></joint></robot>")
    assert fk(parse_chain(urdf), [0.3])[:3, 3] == pytest.approx([0.3, 0, 0])


def test_link_poses_place_fixed_links_on_their_segment():
    c = parse_chain(ARM)
    P = link_poses(c, [0.0, 0.5, 0.0])
    assert P["flange"][:3, 3] == pytest.approx(fk(c, [0.0, 0.5, 0.0])[:3, 3])
    assert P["camera"][:3, 3] == pytest.approx([1, 1, 1])


def test_fk_matches_the_exporters_world_frames(tmp_path):
    """Exported joints sit at world (0,0,0.2) about z and (0.5,0,0.2) about y."""
    d = unzip(two_joint_zip(tmp_path), tmp_path / "x")
    c = parse_chain((d / "bot" / "urdf" / "bot.urdf").read_text())
    assert fk(c, [0, 0])[:3, 3] == pytest.approx([0.5, 0, 0.2])
    T = fk(c, [math.pi / 2, 0.3])
    assert T[:3, 3] == pytest.approx([0, 0.5, 0.2])
    assert T[:3, :3] == pytest.approx(_rz(math.pi / 2) @ Rotation.from_euler("y", 0.3).as_matrix())




def _wrist_gaps(c, q):
    """Distances between the axis lines of the last three joints at pose q (all 0 = spherical wrist)."""
    from app.ros.fk import segment_poses
    seg = segment_poses(c, q)
    lines = []
    for i in (-3, -2, -1):
        T = seg[len(c.joints) + i] @ c.joints[i].origin  # joint frame in the base frame
        lines.append((T[:3, 3], T[:3, :3] @ c.joints[i].axis))
    gaps = []
    for (p, a), (r, b) in [(lines[0], lines[1]), (lines[1], lines[2]), (lines[0], lines[2])]:
        n = np.cross(a, b)
        gaps.append(abs((r - p) @ n) / np.linalg.norm(n) if np.linalg.norm(n) > 1e-9
                    else np.linalg.norm(np.cross(r - p, a)))
    return gaps


def test_six_axis_helper_has_an_exact_spherical_wrist(tmp_path):
    """Joints 4-6 must meet at one point at any pose, or EAIK has no closed form."""
    from tests.ros_helpers import six_axis_zip
    d = unzip(six_axis_zip(tmp_path), tmp_path / "x")
    c = parse_chain((d / "six" / "urdf" / "six.urdf").read_text())
    assert max(_wrist_gaps(c, [0.3, -0.2, 0.4, 0.5, -0.6, 0.7])) < 1e-9


def test_irb1300_export_has_a_spherical_wrist(irb1300_zip):
    """The real IRB 1300's wrist axes meet at one point; joint 6 is located by a ring of bolt holes."""
    import zipfile
    with zipfile.ZipFile(irb1300_zip) as z:
        c = parse_chain(z.read("irb1300/urdf/irb1300.urdf").decode())
    assert max(_wrist_gaps(c, [0.0] * 6)) < 1e-4
