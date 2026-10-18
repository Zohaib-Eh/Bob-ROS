"""A mobile manipulator through the ROS side: the chain sees the arm (not the wheels), the MoveIt config
puts the base in the world, ros2_control drives the wheels."""
import zipfile

import pytest
import yaml

from app.ros.chain import UnsupportedRobot, parse_chain
from app.ros.moveit_config import generate
from tests.ros_helpers import mobile_arm_zip


@pytest.fixture(scope="module")
def ws(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("mobi")
    zipfile.ZipFile(mobile_arm_zip(tmp)).extractall(tmp / "x")
    generate("mobi", tmp / "x" / "mobi", tmp / "src", "mobi.zip", samples=300)
    return tmp


def test_the_chain_is_the_arm_and_the_base_drives(ws):
    c = parse_chain((ws / "x" / "mobi" / "urdf" / "mobi.urdf").read_text())
    assert c.base == "chassis" and c.tip == "tcp" and len(c.joints) == 6
    assert c.gripper is not None
    m = c.mobile
    assert (m.drive, m.left, m.right) == ("diff", ["wheel_l_joint"], ["wheel_r_joint"])
    assert m.separation == pytest.approx(0.5) and m.radius == pytest.approx(0.08, rel=0.01)
    assert m.center == pytest.approx((0.1, 0.0)) and m.height == pytest.approx(0.24, abs=1e-3)


def test_the_brief_teaches_driving_in_world_coordinates(ws):
    b = (ws / "src" / "mobi_program" / "brief.md").read_text(encoding="utf-8")
    assert "differential-drive mobile base" in b and "World frame: map" in b
    assert "r.drive_to(x, y, yaw=None)" in b and "PathBlocked" in b
    assert "## Gripper" in b                           # the gripper's own section is still there
    assert (ws / "src" / "mobi_program" / "drive_logic.py").exists()


def test_wheels_without_the_tag_are_still_refused():
    urdf = """<robot name="r"><link name="b"/><link name="w1"/><link name="w2"/>
      <joint name="a" type="continuous"><parent link="b"/><child link="w1"/></joint>
      <joint name="c" type="continuous"><parent link="b"/><child link="w2"/></joint></robot>"""
    with pytest.raises(UnsupportedRobot, match="wheel"):
        parse_chain(urdf)


def test_moveit_puts_the_base_in_the_map_and_ros2_control_drives_the_wheels(ws):
    cfg = ws / "src" / "mobi_moveit_config" / "config"
    srdf = (cfg / "mobi.srdf").read_text()
    assert '<virtual_joint name="world_joint" type="floating" parent_frame="map" child_link="chassis"/>' in srdf
    ctl = yaml.safe_load((cfg / "ros2_controllers.yaml").read_text())
    assert ctl["controller_manager"]["ros__parameters"]["base_controller"]["type"] == "diff_drive_controller/DiffDriveController"
    p = ctl["base_controller"]["ros__parameters"]
    assert p["left_wheel_names"] == ["wheel_l_joint"] and p["right_wheel_names"] == ["wheel_r_joint"]
    assert p["wheel_separation"] == pytest.approx(0.5) and p["base_frame_id"] == "chassis" and p["odom_frame_id"] == "odom"
    xacro = (cfg / "mobi.ros2_control.xacro").read_text()
    wheel = xacro[xacro.index('<joint name="wheel_l_joint">'):]
    assert '<command_interface name="velocity"/>' in wheel[:wheel.index("</joint>")]
    assert "base_yaw" not in xacro          # ros2_control only knows URDF joints: the truth comes from the glue plugin
    glue = (ws / "src" / "cad_to_urdf_glue" / "src" / "glue_plugin.cpp").read_text()
    assert "/sim/base_pose" in glue
    launch = (ws / "src" / "mobi_moveit_config" / "launch" / "demo.launch.py").read_text()
    assert '"base_controller"' in launch and "localization" in launch
    assert "diff_drive_controller" in (ws / "src" / "mobi_moveit_config" / "package.xml").read_text()
