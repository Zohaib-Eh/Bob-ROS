import xml.etree.ElementTree as ET
import zipfile

import pytest

from app.sim.mjcf import build_mjcf, compile_mjcf
from app.sim.torque import required_efforts, suggest
from tests.ros_helpers import two_joint_zip


def _parts(tmp_path):
    z = zipfile.ZipFile(two_joint_zip(tmp_path))
    urdf = z.read("bot/urdf/bot.urdf").decode()
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if n.startswith("bot/meshes/")}
    return urdf, meshes


def test_mjcf_keeps_links_and_adds_one_motor_per_moving_joint(tmp_path):
    urdf, meshes = _parts(tmp_path)
    xml = build_mjcf(urdf, meshes, {"arm_joint": 50.0, "hand_joint": 20.0})
    root = ET.fromstring(xml)
    assert root.find("compiler").get("meshdir") == "../meshes"
    bodies = {b.get("name") for b in root.iter("body")}
    assert {"arm", "hand"} <= bodies
    acts = {a.get("joint"): a for a in root.find("actuator")}
    assert set(acts) == {"arm_joint", "hand_joint"}
    assert acts["arm_joint"].get("forcerange") == "-50 50" and float(acts["arm_joint"].get("kp")) == pytest.approx(5000)   # 50 N·m at 0.01 rad
    m = compile_mjcf(xml, meshes)
    assert m.nu == 2 and m.opt.timestep == pytest.approx(0.002)


def test_motors_hold_their_target_without_gravity_droop(tmp_path):
    # a real controller feeds gravity torque forward; proportional-only motors settled 1-2 cm low (Task 1)
    import mujoco
    urdf, meshes = _parts(tmp_path)
    req = required_efforts(compile_mjcf(build_mjcf(urdf, meshes, None), meshes))
    m = compile_mjcf(build_mjcf(urdf, meshes, suggest(req)), meshes)
    d = mujoco.MjData(m)
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    hand = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hand_joint")]
    assert abs(d.qpos[hand]) < 0.002


def test_a_vertical_axis_still_needs_torque_to_move_the_arm(tmp_path):
    # arm_joint turns about z: no gravity torque, but accelerating the arm at the joint limits takes torque
    import mujoco
    from app.ros.moveit_config import MAX_ACCELERATION
    urdf, meshes = _parts(tmp_path)
    m = compile_mjcf(build_mjcf(urdf, meshes, None), meshes)
    req = required_efforts(m)
    dof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "arm_joint")]
    assert req["arm_joint"] >= 0.5 * float(m.dof_M0[dof]) * MAX_ACCELERATION


def test_gravity_holding_counts_against_the_motor_limit(tmp_path):
    root = ET.fromstring(build_mjcf(*_parts(tmp_path), {"arm_joint": 50.0, "hand_joint": 20.0}))
    joints = {j.get("name"): j for j in root.iter("joint")}
    assert joints["hand_joint"].get("actuatorgravcomp") == "true"
    assert joints["hand_joint"].get("actuatorfrcrange") == "-20 20"
    assert all(b.get("gravcomp") == "1" for b in root.find("worldbody").iter("body"))


def test_no_efforts_means_no_motors(tmp_path):
    urdf, meshes = _parts(tmp_path)
    assert compile_mjcf(build_mjcf(urdf, meshes, None), meshes).nu == 0


def test_required_torque_is_larger_nearer_the_base(tmp_path):
    urdf, meshes = _parts(tmp_path)
    req = required_efforts(compile_mjcf(build_mjcf(urdf, meshes, None), meshes))
    # arm_joint turns about z (vertical): only inertia, no gravity; hand_joint about y also lifts the hand
    assert req["hand_joint"] > req["arm_joint"] > 0.1


def test_links_touching_at_home_do_not_collide(tmp_path):
    import io
    import mujoco
    from app.export.urdf_writer import export_package
    from tests.ros_helpers import _body, _joint, _link
    links = [_link("base"), _link("arm"), _link("hand")]
    joints = [_joint("j1", "base", "arm", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0)),
              _joint("j2", "arm", "hand", (0.5, 0.0, 0.2), (0.0, 1.0, 0.0))]
    bodies = {"base": [_body("base", (0.0, 0.0, 0.0))], "arm": [_body("arm", (0.25, 0.0, 0.2))],
              "hand": [_body("hand", (0.05, 0.0, 0.0))]}  # hand overlaps base, which is not its parent
    z = zipfile.ZipFile(io.BytesIO(export_package("bot", links, joints, bodies)))
    urdf = z.read("bot/urdf/bot.urdf").decode()
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if n.startswith("bot/meshes/")}
    xml = build_mjcf(urdf, meshes, None)
    assert any({e.get("body1"), e.get("body2")} == {"base", "hand"} for e in ET.fromstring(xml).iter("exclude"))
    m = compile_mjcf(xml, meshes)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert d.ncon == 0


def test_suggest_doubles_and_rounds_up_to_two_significant_figures():
    assert suggest({"a": 70.1, "b": 0.2, "c": 0.0}) == {"a": 150.0, "b": 1.0, "c": 1.0}


from app.export.urdf_writer import export_package, suggest_efforts  # noqa: E402
from tests.ros_helpers import _body, _joint, _link  # noqa: E402


def _arm(max_effort=None):
    links = [_link("base"), _link("arm"), _link("hand")]
    j1 = _joint("j1", "base", "arm", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0))
    j2 = _joint("j2", "arm", "hand", (0.5, 0.0, 0.2), (0.0, 1.0, 0.0))
    if max_effort is not None:
        j2 = j2.model_copy(update={"max_effort": max_effort})
    bodies = {"base": [_body("base", (0.0, 0.0, 0.0))], "arm": [_body("arm", (0.25, 0.0, 0.2))],
              "hand": [_body("hand", (0.7, 0.0, 0.2))]}
    return links, [j1, j2], bodies


def _zip(data):
    import io
    return zipfile.ZipFile(io.BytesIO(data))


def _effort(zip_bytes, joint_name):
    root = ET.fromstring(_zip(zip_bytes).read("bot/urdf/bot.urdf"))
    return float(next(j for j in root.iter("joint") if j.get("name") == joint_name).find("limit").get("effort"))


def test_export_uses_suggested_efforts_and_ships_an_mjcf():
    links, joints, bodies = _arm()
    data = export_package("bot", links, joints, bodies)
    sug = suggest_efforts("bot", links, joints, bodies)
    assert set(sug) == {"j1", "j2"} and _effort(data, "hand_joint") == sug["j2"]
    mjcf = _zip(data).read("bot/mjcf/bot.xml").decode()
    assert 'joint="hand_joint"' in mjcf and 'meshdir="../meshes"' in mjcf


def test_an_explicit_max_effort_wins():
    links, joints, bodies = _arm(max_effort=7.5)
    assert _effort(export_package("bot", links, joints, bodies), "hand_joint") == 7.5


def test_an_old_project_joint_without_max_effort_still_exports():
    from app.models import Joint
    raw = _arm()[1][1].model_dump()
    raw.pop("max_effort", None)
    assert Joint.model_validate(raw).max_effort is None


def test_max_effort_must_be_positive():
    from pydantic import ValidationError
    from app.models import Joint
    with pytest.raises(ValidationError):
        Joint.model_validate(_arm()[1][1].model_dump() | {"max_effort": -1})
