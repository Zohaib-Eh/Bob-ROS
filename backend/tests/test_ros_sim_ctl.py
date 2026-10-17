import importlib
import sys
import zipfile
from pathlib import Path

import pytest
import yaml

from tests.ros_helpers import two_joint_zip

T = Path(__file__).resolve().parents[1] / "app" / "ros" / "templates"
sys.path.insert(0, str(T))  # sim_ctl imports scene as a sibling module, as in the container
sim_ctl = importlib.import_module("sim_ctl")


@pytest.fixture
def src(tmp_path):
    zipfile.ZipFile(two_joint_zip(tmp_path)).extractall(tmp_path / "src")
    (tmp_path / "src" / "bot_tasks").mkdir()
    return tmp_path / "src"


def test_plan_merges_workcell_and_program_scene(src):
    (src / "bot_tasks" / "workcell.yaml").write_text(
        "- {name: table, shape: box, size: [0.4, 0.4, 0.05], xyz: [2, 2, 0.025], fixed: true}\n")
    prog = "SCENE = [dict(name='part_a', shape='box', size=(0.05, 0.05, 0.05), xyz=(2, 2, 0.075))]\n"
    xml, info = sim_ctl.plan(src, "bot", "hand", prog)
    assert 'name="table"' in xml and 'name="part_a"' in xml and info["fixed"] == ["table"]
    assert [o["name"] for o in info["objects"]] == ["table", "part_a"]


def test_plan_without_workcell_or_scene_is_robot_only(src):
    xml, info = sim_ctl.plan(src, "bot", "hand", None)
    assert "glue_" not in xml and info["masses"] == {} and info["objects"] == []


def test_plan_errors_name_the_problem(src):
    with pytest.raises(sim_ctl.scene.SceneError, match="unknown field 'colour'"):
        sim_ctl.plan(src, "bot", "hand", "SCENE = [dict(name='a', shape='box', size=(1,1,1), xyz=(5,5,1), colour=1)]\n")


def test_plan_catches_overlap_with_the_robot(src):
    prog = "SCENE = [dict(name='a', shape='box', size=(0.1, 0.1, 0.1), xyz=(0.7, 0, 0.2))]\n"  # on the hand's cube
    with pytest.raises(sim_ctl.scene.SceneError, match="'a' and 'hand' overlap"):
        sim_ctl.plan(src, "bot", "hand", prog)


def test_decide_resets_only_for_the_identical_scene(tmp_path):
    p = tmp_path / "scene.xml"
    assert sim_ctl.decide("<mujoco/>", p) == "restart"
    p.write_text("<mujoco/>")
    assert sim_ctl.decide("<mujoco/>", p) == "reset" and sim_ctl.decide("<mujoco></mujoco>", p) == "restart"


def test_plugins_file_loads_pose_publisher_and_glue():
    cfg = yaml.safe_load(sim_ctl.plugins_yaml())["/**"]["ros__parameters"]["mujoco_plugins"]
    assert cfg["free_joint_state_publisher"]["type"] == "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin"
    assert cfg["free_joint_state_publisher"]["topic"] == "/sim/free_joint_states"
    assert cfg["glue"]["type"] == "cad_to_urdf_glue/GluePlugin"
