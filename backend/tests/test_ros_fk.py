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
