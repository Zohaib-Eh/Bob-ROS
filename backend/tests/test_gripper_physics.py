"""The exported gripper in MuJoCo (Python, no ROS): arm motors hold home, the leader finger is commanded."""
import io
import zipfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from app.export.urdf_writer import export_package
from app.sim.mjcf import compile_mjcf
from tests.ros_helpers import six_axis_with_gripper


WRIST = np.pi / 2   # j5 at 90°: the gripper points straight down and its fingers close along x


def _scene(box_mass=None, box=0.04):
    """Exported sixg MJCF with the wrist turned so the gripper points down (plate at z 0.77, fingers
    down to 0.70, hooks at 0.70..0.708, uprights' inner faces at x 0.575 / 0.625). Optionally a box
    between the uprights above the hooks, on a pedestal thin enough to pass between the hooks, so
    only the fingers can lift it."""
    links, joints, bodies, g = six_axis_with_gripper()
    z = zipfile.ZipFile(io.BytesIO(export_package("sixg", links, joints, bodies, gripper=g)))
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if "/meshes/" in n}
    root = ET.fromstring(z.read("sixg/mjcf/sixg.xml").decode())
    if box_mass is not None:
        wb = root.find("worldbody")
        bottom = 0.715
        ET.SubElement(wb, "geom", {"name": "pedestal", "type": "box", "size": f"0.005 0.005 {bottom / 2}",
                                   "pos": f"0.6 0 {bottom / 2}"})
        b = ET.SubElement(wb, "body", {"name": "box", "pos": f"0.6 0 {bottom + box / 2}"})
        ET.SubElement(b, "freejoint", {"name": "box"})
        ET.SubElement(b, "geom", {"type": "box", "size": f"{box / 2} {box / 2} {box / 2}", "mass": str(box_mass),
                                  "friction": "1 0.005 0.0001"})
    m = compile_mjcf(ET.tostring(root, encoding="unicode"), meshes)
    d = mujoco.MjData(m)
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "l5_joint")]] = WRIST
    mujoco.mj_forward(m, d)
    return m, d


def _act(m, joint):
    return next(a for a in range(m.nu) if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0]) == joint)


def _q(m, d, joint):
    return d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)]]


def _run(m, d, seconds, ctrl=None):
    """Step with the wrist held at WRIST, plus any other actuator targets."""
    targets = {_act(m, "l5_joint"): WRIST, **(ctrl or {})}
    for _ in range(int(seconds / m.opt.timestep)):
        for a, v in targets.items():
            d.ctrl[a] = v
        mujoco.mj_step(m, d)


def test_follower_has_no_motor_and_mirrors_the_leader():
    m, d = _scene()
    names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0]) for a in range(m.nu)}
    assert "gfl_joint" in names and "gfr_joint" not in names
    _run(m, d, 1.0, {_act(m, "gfl_joint"): 0.01})   # hooks would meet at 0.017
    assert _q(m, d, "gfl_joint") == pytest.approx(0.01, abs=1e-3)
    assert _q(m, d, "gfr_joint") == pytest.approx(_q(m, d, "gfl_joint"), abs=1e-3)


def test_fingers_stay_mirrored_while_squeezing_a_box():   # Task 1 finding: the coupling was soft
    m, d = _scene(box_mass=0.2)
    _run(m, d, 1.5, {_act(m, "gfl_joint"): 0.025})
    assert abs(_q(m, d, "gfl_joint") - _q(m, d, "gfr_joint")) < 1e-3


def test_fingers_dont_stall_on_a_hull_the_uprights_reach_the_box():
    m, d = _scene(box_mass=0.2)
    _run(m, d, 1.5, {_act(m, "gfl_joint"): 0.025})
    # open gap 0.05, box 0.04: each finger travels ~0.005 before the uprights touch it (a hull over the
    # hook corner would stop them earlier, at ~0.0 since the hooks are already within 0.034)
    assert _q(m, d, "gfl_joint") == pytest.approx(0.005, abs=1.5e-3)


def test_box_is_gripped_and_lifted_with_the_arm():
    m, d = _scene(box_mass=0.2)
    fl = _act(m, "gfl_joint")
    _run(m, d, 1.5, {fl: 0.025})
    box = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box")
    z0 = d.xpos[box][2]
    j2 = _act(m, "l2_joint")
    _run(m, d, 2.0, {fl: 0.025, j2: -0.1})         # shoulder back: the gripper rises ~5 cm
    assert d.xpos[box][2] > z0 + 0.03


def test_a_box_too_heavy_for_the_grip_slips():
    m, d = _scene(box_mass=12.0)                  # 2 fingers x 40 N x friction 1 = 80 N < 118 N weight
    fl = _act(m, "gfl_joint")
    _run(m, d, 1.5, {fl: 0.025})
    box = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box")
    z0 = d.xpos[box][2]
    _run(m, d, 2.0, {fl: 0.025, _act(m, "l2_joint"): -0.1})
    assert d.xpos[box][2] < z0 + 0.01               # stayed behind on the pedestal
