import math
import zipfile

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET

from app.ros.chain import UnsupportedRobot, home, origin_matrix, parse_chain

ARM = """<robot name="arm">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="flange"/>
  <link name="tool"/><link name="camera"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="cam" type="fixed"><parent link="base"/><child link="camera"/><origin xyz="1 1 1"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 2"/><limit lower="0.5" upper="1" effort="1" velocity="1"/></joint>
  <joint name="fl" type="fixed"><parent link="l2"/><child link="flange"/><origin xyz="0.5 0 0"/></joint>
  <joint name="j3" type="continuous"><parent link="flange"/><child link="tool"/><axis xyz="1 0 0"/></joint>
</robot>"""


def _robot(joints_xml, links=("base", "a", "b")):
    return ("<robot name='r'>" + "".join(f"<link name='{l}'/>" for l in links)
            + joints_xml + "</robot>")


def _rev(name, parent, child, limit=True, type_="revolute"):
    lim = "<limit lower='-1' upper='1' effort='1' velocity='1'/>" if limit else ""
    return (f"<joint name='{name}' type='{type_}'><parent link='{parent}'/><child link='{child}'/>"
            f"<axis xyz='0 0 1'/>{lim}</joint>")


def test_chain_follows_moving_joints_and_folds_fixed_ones():
    c = parse_chain(ARM)
    assert c.robot == "arm" and c.base == "base" and c.tip == "tool"
    assert [j.name for j in c.joints] == ["j1", "j2", "j3"]
    assert c.joints[1].axis == pytest.approx([0, 0, 1])           # normalized
    assert c.joints[2].origin[:3, 3] == pytest.approx([0.5, 0, 0])  # fixed flange folded in
    assert c.joints[2].lower is None                                # continuous: no limits needed
    seg, T = c.links["flange"]
    assert seg == 2 and T[:3, 3] == pytest.approx([0.5, 0, 0])
    assert c.links["camera"][0] == 0                                # fixed side branch is fine


def test_home_is_zero_or_nearest_limit():
    assert home(parse_chain(ARM)) == [0.0, 0.5, 0.0]


def test_origin_matrix_uses_urdf_rpy_convention():
    el = ET.fromstring('<origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>')
    T = origin_matrix(el)
    assert T[:3, :3] == pytest.approx(Rotation.from_euler("xyz", [0.1, 0.2, 0.3]).as_matrix())
    assert T[:3, 3] == pytest.approx([1, 2, 3])
    assert origin_matrix(None) == pytest.approx(np.eye(4))


def test_two_wheels_are_declined():
    urdf = _robot(_rev("left", "base", "a", limit=False, type_="continuous")
                  + _rev("right", "base", "b", limit=False, type_="continuous"))
    with pytest.raises(UnsupportedRobot, match="Joint 'left' looks like a wheel; mobile robots"):
        parse_chain(urdf)


def test_branching_is_declined():
    urdf = _robot(_rev("j1", "base", "a") + _rev("j2", "base", "b"))
    with pytest.raises(UnsupportedRobot, match="Robot has 2 moving branches"):
        parse_chain(urdf)


def test_planar_joint_is_declined():
    urdf = _robot(_rev("p", "base", "a", type_="planar"), links=("base", "a"))
    with pytest.raises(UnsupportedRobot, match="Joint 'p' is planar"):
        parse_chain(urdf)


def test_revolute_without_limits_is_declined():
    urdf = _robot(_rev("j1", "base", "a", limit=False), links=("base", "a"))
    with pytest.raises(UnsupportedRobot, match="Joint 'j1' has no limits; set them in the app"):
        parse_chain(urdf)


def test_no_moving_joints_is_declined():
    urdf = _robot("<joint name='f' type='fixed'><parent link='base'/><child link='a'/></joint>",
                  links=("base", "a"))
    with pytest.raises(UnsupportedRobot, match="no moving joints"):
        parse_chain(urdf)


def test_irb1300_is_a_six_joint_chain(irb1300_zip):
    with zipfile.ZipFile(irb1300_zip) as z:
        c = parse_chain(z.read("irb1300/urdf/irb1300.urdf").decode())
    assert c.base == "base" and c.tip == "link6"
    assert [j.name for j in c.joints] == [f"link{i}_joint" for i in range(1, 7)]
