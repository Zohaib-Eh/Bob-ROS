import importlib.util
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import mujoco
import numpy as np
import pytest

from tests.ros_helpers import two_joint_zip

_spec = importlib.util.spec_from_file_location(
    "scene", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "scene.py")
scene = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scene)


@pytest.fixture
def bot(tmp_path):
    zipfile.ZipFile(two_joint_zip(tmp_path)).extractall(tmp_path / "x")
    root = tmp_path / "x" / "bot"
    return (root / "mjcf" / "bot.xml").read_text(), str(root / "meshes")


def _model(xml):
    return mujoco.MjModel.from_xml_string(xml)


def _robot_bodies(mjcf):  # what sim_ctl passes: every robot body plus the world (floor)
    return {b.get("name") for b in ET.fromstring(mjcf).iter("body")} | {"world"}


def _far_box(**kw):  # well away from the 2-joint arm
    return {"name": "part_a", "shape": "box", "size": [0.05, 0.05, 0.05], "xyz": [2.0, 2.0, 0.025], **kw}


def _settle(m, d, seconds):
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)


def test_no_objects_is_just_the_robot(bot):
    m = _model(scene.build_scene_xml(bot[0], [], "hand", bot[1]))
    assert m.neq == 0 and mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hand") >= 0


def test_box_has_its_mass_free_joint_and_an_inactive_glue(bot):
    objs = scene.validate([_far_box(mass=0.2)], "SCENE")
    m = _model(scene.build_scene_xml(bot[0], objs, "hand", bot[1]))
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "part_a")
    assert m.body_mass[b] == pytest.approx(0.2)
    assert m.jnt_type[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "part_a")] == mujoco.mjtJoint.mjJNT_FREE
    e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "glue_part_a")
    assert e >= 0 and m.eq_active0[e] == 0


def test_fixed_object_has_no_joint_and_no_glue(bot):
    objs = scene.validate([{"name": "table", "shape": "box", "size": [0.5, 0.5, 0.05],
                            "xyz": [2.0, 2.0, 0.025], "fixed": True}], "workcell.yaml")
    m = _model(scene.build_scene_xml(bot[0], objs, "hand", bot[1]))
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "table") == -1 and m.neq == 0


def test_dropped_box_lands_on_the_floor_and_settles(bot):
    objs = scene.validate([_far_box(xyz=[2.0, 2.0, 0.3])], "SCENE")
    m = _model(scene.build_scene_xml(bot[0], objs, "hand", bot[1]))
    d = mujoco.MjData(m)
    _settle(m, d, 2.0)
    assert d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "part_a")][2] == pytest.approx(0.025, abs=0.003)


def test_box_resting_on_table_is_not_an_overlap(bot):
    objs = scene.validate([
        {"name": "table", "shape": "box", "size": [0.5, 0.5, 0.05], "xyz": [2.0, 2.0, 0.025], "fixed": True},
        _far_box(xyz=[2.0, 2.0, 0.075])], "SCENE")
    scene.check_start(scene.build_scene_xml(bot[0], objs, "hand", bot[1]), None, _robot_bodies(bot[0]))


def test_box_inside_another_box_is_an_overlap(bot):
    objs = scene.validate([_far_box(), _far_box(name="part_b", xyz=[2.01, 2.0, 0.025])], "SCENE")
    with pytest.raises(scene.SceneError, match="'part_a' and 'part_b' overlap"):
        scene.check_start(scene.build_scene_xml(bot[0], objs, "hand", bot[1]), None, _robot_bodies(bot[0]))


def test_box_inside_the_floor_is_an_overlap(bot):
    objs = scene.validate([_far_box(xyz=[2.0, 2.0, 0.0])], "SCENE")
    with pytest.raises(scene.SceneError, match="'part_a' and 'world' overlap"):
        scene.check_start(scene.build_scene_xml(bot[0], objs, "hand", bot[1]), None, _robot_bodies(bot[0]))


def test_object_named_like_a_robot_link_is_rejected(bot):
    objs = scene.validate([_far_box(name="hand")], "SCENE")
    with pytest.raises(scene.SceneError, match="'hand' is the name of a robot link"):
        scene.build_scene_xml(bot[0], objs, "hand", bot[1])


def test_glued_box_follows_the_tool_and_released_box_falls(bot):
    # just under the hand (hand cube centre (0.5, 0, 0.2), half-size 0.05), as attach() requires
    box = _far_box(xyz=[0.5, 0.0, 0.12])
    m = _model(scene.build_scene_xml(bot[0], scene.validate([box], "SCENE"), "hand", bot[1]))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "glue_part_a")
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "part_a")
    scene.glue_on(m, d, e)                         # the same maths the C++ plugin uses
    start = d.xpos[b].copy()
    _settle(m, d, 0.5)
    assert np.linalg.norm(d.xpos[b] - start) < 0.01   # held where it was when glued: no jump, no fall
    d.eq_active[e] = 0
    _settle(m, d, 1.0)
    assert d.xpos[b][2] == pytest.approx(0.025, abs=0.003)


def test_reset_puts_objects_back_and_glue_off(bot):
    m = _model(scene.build_scene_xml(bot[0], scene.validate([_far_box(xyz=[2.0, 2.0, 0.3])], "SCENE"), "hand", bot[1]))
    d = mujoco.MjData(m)
    e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "glue_part_a")
    d.eq_active[e] = 1
    _settle(m, d, 0.4)
    mujoco.mj_resetData(m, d)
    assert d.eq_active[e] == 0
    assert d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "part_a")] + 2] == pytest.approx(0.3)


def test_meta_has_motor_limits_and_masses(bot):
    info = scene.meta(bot[0], scene.validate([_far_box(mass=0.2)], "SCENE"))
    assert info["masses"] == {"part_a": 0.2} and info["fixed"] == []
    assert set(info["efforts"]) == {"arm_joint", "hand_joint"} and all(v > 0 for v in info["efforts"].values())
