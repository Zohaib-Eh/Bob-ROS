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
    xml, info = sim_ctl.plan(src, "bot", prog)
    assert 'name="table"' in xml and 'name="part_a"' in xml and info["fixed"] == ["table"]
    assert [o["name"] for o in info["objects"]] == ["table", "part_a"]


def test_plan_brings_a_cad_fixture_in_as_its_pieces_and_tells_moveit_where_its_mesh_is(src):
    import trimesh
    meshes = src / "bot_tasks" / "meshes"
    meshes.mkdir()
    for f in ("table.stl", "table_c0.stl"):
        trimesh.creation.box([0.6, 0.4, 0.3]).export(meshes / f, file_type="stl")
    (src / "bot_tasks" / "workcell.yaml").write_text(
        "- {name: table, shape: mesh, file: table.stl, pieces: 1, size: [0.6, 0.4, 0.3], xyz: [2, 2, 0.15], fixed: true}\n")
    xml, info = sim_ctl.plan(src, "bot", None)
    assert 'type="mesh" mesh="wc_table_c0"' in xml
    assert info["objects"][0]["path"] == str(meshes / "table.stl") and info["fixed"] == ["table"]
    assert sim_ctl.objects_meta(src, "bot", None)["objects"][0]["path"] == str(meshes / "table.stl")


def test_plan_without_workcell_or_scene_is_robot_only(src):
    xml, info = sim_ctl.plan(src, "bot", None)
    assert "glue_" not in xml and info["masses"] == {} and info["objects"] == []


def test_plan_errors_name_the_problem(src):
    with pytest.raises(sim_ctl.scene.SceneError, match="unknown field 'colour'"):
        sim_ctl.plan(src, "bot", "SCENE = [dict(name='a', shape='box', size=(1,1,1), xyz=(5,5,1), colour=1)]\n")


def test_plan_catches_overlap_with_the_robot(src):
    prog = "SCENE = [dict(name='a', shape='box', size=(0.1, 0.1, 0.1), xyz=(0.7, 0, 0.2))]\n"  # on the hand's cube
    with pytest.raises(sim_ctl.scene.SceneError, match="'a' and 'hand' overlap"):
        sim_ctl.plan(src, "bot", prog)


def test_decide_resets_only_a_live_sim_on_the_identical_scene(tmp_path):  # review I-3
    p = tmp_path / "scene.xml"
    assert sim_ctl.decide("<mujoco/>", p, alive=True) == "restart"
    p.write_text("<mujoco/>")
    assert sim_ctl.decide("<mujoco/>", p, alive=True) == "reset"
    assert sim_ctl.decide("<mujoco></mujoco>", p, alive=True) == "restart"
    assert sim_ctl.decide("<mujoco/>", p, alive=False) == "restart"   # crashed or never started


def test_objects_without_an_mjcf_still_come_from_workcell_and_scene(src):  # review I-1
    (src / "bot" / "mjcf" / "bot.xml").unlink()
    (src / "bot_tasks" / "workcell.yaml").write_text(
        "- {name: table, shape: box, size: [0.4, 0.4, 0.05], xyz: [2, 2, 0.025], fixed: true}\n")
    info = sim_ctl.objects_meta(src, "bot", "SCENE = [dict(name='a', shape='box', size=(0.05, 0.05, 0.05), xyz=(2, 2, 0.1))]\n")
    assert [o["name"] for o in info["objects"]] == ["table", "a"] and info["fixed"] == ["table"]
    with pytest.raises(sim_ctl.scene.SceneError, match="SCENE must be plain values"):
        sim_ctl.objects_meta(src, "bot", "x = 1\nSCENE = [dict(name='a', shape='box', size=(x, x, x), xyz=(0, 0, 0))]\n")


def test_meta_is_written_atomically(tmp_path):  # review I-4: scene_sync reads it 10x a second
    p = tmp_path / "meta.json"
    sim_ctl.write_atomic(p, '{"a": 1}')
    assert p.read_text() == '{"a": 1}' and [f.name for f in tmp_path.iterdir()] == ["meta.json"]


def test_plugins_file_loads_only_the_pose_publisher():
    cfg = yaml.safe_load(sim_ctl.plugins_yaml())["/**"]["ros__parameters"]["mujoco_plugins"]
    assert cfg["free_joint_state_publisher"]["type"] == "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin"
    assert cfg["free_joint_state_publisher"]["topic"] == "/sim/free_joint_states"
    assert set(cfg) == {"free_joint_state_publisher"}


def test_controllers_include_the_gripper_only_when_there_is_one():
    assert sim_ctl.controller_names(False) == ["arm_controller"]
    assert sim_ctl.controller_names(True) == ["arm_controller", "gripper_controller"]


def _fake_sim(monkeypatch, tmp_path, waits):
    """sim_ctl's start/stop/wait replaced: waits = what each wait for object poses returns, in order."""
    calls = []
    monkeypatch.setattr(sim_ctl, "SIM", tmp_path)
    (tmp_path / "control.log").write_text("sim log")
    monkeypatch.setattr(sim_ctl, "_start_sim", lambda robot: calls.append("start"))
    monkeypatch.setattr(sim_ctl, "_stop_sim", lambda: calls.append("stop"))
    answers = iter(waits)
    monkeypatch.setattr(sim_ctl, "_wait_for_object_poses", lambda names, timeout=10.0: calls.append("wait") or next(answers))
    return calls


def test_a_sim_that_starts_without_reporting_its_objects_is_restarted_once(monkeypatch, tmp_path):
    # container check: after a restart the pose plugin sometimes published nothing, and a grip closed on air
    calls = _fake_sim(monkeypatch, tmp_path, [False, True])
    sim_ctl._start_with_object_poses("bot", {"box"})
    assert calls == ["start", "wait", "stop", "start", "wait"]
    assert (tmp_path / "silent_start_1.log").read_text() == "sim log"     # kept for diagnosis


def test_a_sim_that_stays_silent_fails_the_run_in_plain_words(monkeypatch, tmp_path):
    _fake_sim(monkeypatch, tmp_path, [False, False])
    with pytest.raises(sim_ctl.SimError, match="did not report where the objects are"):
        sim_ctl._start_with_object_poses("bot", {"box"})


def test_a_scene_without_loose_objects_needs_no_poses(monkeypatch, tmp_path):
    calls = _fake_sim(monkeypatch, tmp_path, [])
    sim_ctl._start_with_object_poses("bot", set())
    assert calls == ["start"]
