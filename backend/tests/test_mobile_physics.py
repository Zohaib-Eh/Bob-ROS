"""The exported mobile robot in MuJoCo: it stands on its wheels with its whole weight, holds still, drives
straight, turns in place, and a heavy reach can tip it."""
import io
import math
import xml.etree.ElementTree as ET
import zipfile

import mujoco
import numpy as np
import pytest

from app.export.urdf_writer import export_package
from tests.ros_helpers import mobile_arm


def _export(**kw):
    links, joints, bodies, g, mobile = mobile_arm(**kw)
    z = zipfile.ZipFile(io.BytesIO(export_package("mobi", links, joints, bodies, gripper=g, mobile=mobile)))
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if "/meshes/" in n}
    xml = z.read("mobi/mjcf/mobi.xml").decode()
    urdf = z.read("mobi/urdf/mobi.urdf").decode()
    return xml, urdf, meshes


@pytest.fixture(scope="module")
def mobi():
    return _export()


def _sim(xml, meshes):
    xml = xml.replace('meshdir="../meshes"', 'meshdir=""')
    root = ET.fromstring(xml)
    ET.SubElement(root.find("worldbody"), "geom", {"name": "floor", "type": "plane", "size": "0 0 1"})
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), meshes)
    return m, mujoco.MjData(m)


def _base(m, d):
    q = lambda n: d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]]
    return np.array([q("base_x"), q("base_y"), q("base_z"), q("base_yaw"), q("base_pitch"), q("base_roll")])


def _wheels(m, d, left, right):
    for n, v in (("wheel_l_joint_motor", left), ("wheel_r_joint_motor", right)):
        d.ctrl[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)] = v




def test_the_export_says_how_it_drives(mobi):
    xml, urdf, _ = mobi
    tag = ET.fromstring(urdf).find("mobile_base")
    assert tag.get("drive") == "diff" and tag.get("left") == "wheel_l_joint" and tag.get("right") == "wheel_r_joint"
    assert float(tag.get("wheel_radius")) == pytest.approx(0.08, rel=0.01) and float(tag.get("wheel_separation")) == pytest.approx(0.5)
    wheel = next(j for j in ET.fromstring(urdf).iter("joint") if j.get("name") == "wheel_l_joint")
    assert wheel.get("type") == "continuous" and float(wheel.find("limit").get("velocity")) > 0
    root = ET.fromstring(xml)
    acts = {a.get("joint"): a.tag for a in root.find("actuator")}
    assert acts["wheel_l_joint"] == "velocity" and acts["l1_joint"] == "position"


def test_it_stands_on_its_wheels_with_its_whole_weight_and_holds_still(mobi):
    xml, _, meshes = mobi
    m, d = _sim(xml, meshes)
    for _ in range(1000):   # 2 s
        mujoco.mj_step(m, d)
    total, f = 0.0, np.zeros(6)
    for i in range(d.ncon):
        mujoco.mj_contactForce(m, d, i, f)
        total += f[0]
    weight = sum(m.body_mass) * 9.81
    assert total == pytest.approx(weight, rel=0.02)          # no gravity compensation leaking into the base
    b = _base(m, d)
    assert np.abs(b[:2]).max() < 0.005 and np.abs(b[4:]).max() < 0.01
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    touching = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1 if c.geom2 == floor else c.geom2])
                for c in d.contact[:d.ncon]}
    assert touching == {"chassis", "wheel_l", "wheel_r"}     # chassis: only its caster sphere (its hull is off the floor)


def test_it_drives_straight_and_turns_in_place(mobi):
    xml, _, meshes = mobi
    m, d = _sim(xml, meshes)
    for _ in range(250):
        mujoco.mj_step(m, d)
    w = 0.2 / 0.08            # 0.2 m/s
    _wheels(m, d, w, w)
    start = _base(m, d)
    for _ in range(1000):     # 2 s
        mujoco.mj_step(m, d)
    moved = _base(m, d) - start
    assert moved[0] == pytest.approx(0.4, rel=0.1) and abs(moved[1]) < 0.02 and abs(moved[3]) < math.radians(3)
    _wheels(m, d, -w, w)      # spin left in place: yaw rate = 2 v / separation, once the left wheel has reversed
    for _ in range(250):      # 0.5 s: 27 kg, wheel torque capped at its motor strength
        mujoco.mj_step(m, d)
    start = _base(m, d)
    for _ in range(500):      # 1 s
        mujoco.mj_step(m, d)
    end = _base(m, d)
    axle = lambda b: b[:2] + 0.1 * np.array([math.cos(b[3]), math.sin(b[3])])   # it spins about its axle's middle (x 0.1)
    assert end[3] - start[3] == pytest.approx(2 * 0.2 / 0.5, rel=0.15) and np.linalg.norm(axle(end) - axle(start)) < 0.02


def test_a_heavy_reach_tips_a_base_with_no_caster_in_front():
    xml, _, meshes = _export()
    root = ET.fromstring(xml)
    for b in root.iter("body"):        # 40 kg bolted on, overhanging the front 0.5 m past the wheels
        if b.get("name") == "base":
            b.find("inertial").set("mass", "40")
            b.find("inertial").set("pos", "0.6 0 0.05")
    m, d = _sim(ET.tostring(root, encoding="unicode"), meshes)
    for _ in range(1500):
        mujoco.mj_step(m, d)
    assert _base(m, d)[4] > 0.1        # pitched forward onto its nose: physics, not a bolted-down base


def test_the_apps_physics_check_holds_still_and_drives_straight():
    from app.sim.check import physics_check
    links, joints, bodies, g, mobile = mobile_arm()
    r = physics_check("mobi", links, joints, bodies, g, mobile)
    assert r["ok"], r
    assert "Drives straight" in r["message"]
    weak = [j.model_copy(update={"max_effort": 0.05}) if j.id.startswith("j_wheel") else j for j in joints]
    r = physics_check("mobi", links, weak, bodies, g, mobile)
    assert not r["ok"] and r["kind"] == "drive" and "wheel" in r["message"]


def test_a_box_set_on_the_chassis_rests_on_it(tmp_path, mobi):   # the chassis hull is off the floor (contype 2), not off objects
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location("scene", Path(__file__).resolve().parents[1] / "app/ros/templates/scene.py")
    scene = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(scene)
    xml, _, meshes = mobi
    for n, b in meshes.items():
        (tmp_path / n).write_bytes(b)
    objs = scene.validate([{"name": "parcel", "shape": "box", "size": [0.05, 0.05, 0.05], "xyz": [-0.15, 0.1, 0.23], "mass": 0.2}], "t")
    m = mujoco.MjModel.from_xml_string(scene.build_scene_xml(xml, objs, "gbase", str(tmp_path)))
    d = mujoco.MjData(m)
    for _ in range(1000):
        mujoco.mj_step(m, d)
    z = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "parcel")][2]
    assert z == pytest.approx(0.18 + 0.025, abs=0.003)
