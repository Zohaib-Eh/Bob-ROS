import io
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import pytest
import trimesh

from app.export.gripper import convex_pieces, inward_axes, tcp_point
from app.export.urdf_writer import export_package, export_urdf
from app.models import Gripper
from tests.ros_helpers import _boxes_body, l_finger_body, six_axis_with_gripper, two_joint_zip


def test_inward_axes_point_at_each_other_and_flip_limits():
    la, fa, lflip, fflip = inward_axes((0, 0, 1), (0, 0, 1), (0, 0, -0.03), (0, 0, 0.03))
    assert la == pytest.approx((0, 0, 1)) and fa == pytest.approx((0, 0, -1))
    assert (lflip, fflip) == (False, True)
    la, fa, lflip, fflip = inward_axes((0, 0, -1), (0, 0, 1), (0, 0, -0.03), (0, 0, 0.03))
    assert la == pytest.approx((0, 0, 1)) and (lflip, fflip) == (True, True)


def test_tcp_point_is_between_the_fingertips():
    # fixture geometry, unmounted: plate z 0..0.01, fingers down to z = -0.06
    base = _boxes_body("b", [((-0.04, -0.02, 0.0), (0.04, 0.02, 0.01))]).vertices
    fl = _boxes_body("l", [((-0.035, -0.01, -0.06), (-0.025, 0.01, 0.0))]).vertices
    fr = _boxes_body("r", [((0.025, -0.01, -0.06), (0.035, 0.01, 0.0))]).vertices
    xyz, axis = tcp_point(base, fl, fr, 0.0)
    assert xyz == pytest.approx((0, 0, -0.06)) and axis == pytest.approx((0, 0, -1))
    assert tcp_point(base, fl, fr, 0.01)[0] == pytest.approx((0, 0, -0.07))


def test_l_shaped_finger_splits_into_convex_pieces():
    b = l_finger_body("l")
    whole = trimesh.Trimesh(np.array(b.vertices), np.array(b.triangles))
    assert whole.is_watertight and whole.is_winding_consistent and not whole.is_convex
    assert whole.volume == pytest.approx(0.01 * 0.02 * 0.06 + 0.004 * 0.02 * 0.008)
    pieces = convex_pieces(b.vertices, b.triangles)
    assert len(pieces) >= 2
    meshes = [trimesh.Trimesh(np.array(v), np.array(t)) for v, t in pieces]
    assert all(m.is_convex for m in meshes)
    assert sum(m.volume for m in meshes) == pytest.approx(whole.volume, rel=0.1)


def _urdf(zbytes, name="sixg"):
    return ET.fromstring(zipfile.ZipFile(io.BytesIO(zbytes)).read(f"{name}/urdf/{name}.urdf"))


def test_export_writes_mimic_inward_axes_tcp_and_convex_collisions():
    links, joints, bodies, g = six_axis_with_gripper()
    data = export_package("sixg", links, joints, bodies, gripper=g)
    root = _urdf(data)
    js = {j.get("name"): j for j in root.iter("joint")}
    fol = js["gfr_joint"]
    assert fol.find("mimic").get("joint") == "gfl_joint" and fol.find("mimic").get("multiplier") == "1"
    assert float(fol.find("limit").get("lower")) == pytest.approx(0) and float(fol.find("limit").get("upper")) == pytest.approx(0.025)
    assert float(js["gfl_joint"].find("limit").get("effort")) == pytest.approx(40)
    assert js["tcp_joint"].get("type") == "fixed" and js["tcp_joint"].find("parent").get("link") == "gbase"
    assert any(l.get("name") == "tcp" for l in root.iter("link"))
    gfl = next(l for l in root.iter("link") if l.get("name") == "gfl")
    assert len(gfl.findall("collision")) >= 2 and len(gfl.findall("visual")) == 1
    names = zipfile.ZipFile(io.BytesIO(data)).namelist()
    assert "sixg/meshes/gfl_c0.stl" in names and "sixg/meshes/gfl.stl" in names
    l6 = next(l for l in root.iter("link") if l.get("name") == "l6")
    assert len(l6.findall("collision")) == 1          # arm links stay one mesh


def test_a_gripper_that_is_not_two_sliding_joints_is_refused():
    links, joints, bodies, _ = six_axis_with_gripper()
    with pytest.raises(ValueError, match="Gripper fingers must be two sliding joints of this robot."):
        export_package("sixg", links, joints, bodies, gripper=Gripper(leader_joint_id="j6", follower_joint_id="finger_r"))
    with pytest.raises(ValueError, match="Gripper fingers must be two sliding joints of this robot."):
        export_package("sixg", links, joints, bodies, gripper=Gripper(leader_joint_id="nope", follower_joint_id="finger_r"))


def test_robot_without_gripper_exports_exactly_as_before(tmp_path):   # Review Focus 5
    z = zipfile.ZipFile(two_joint_zip(tmp_path))
    urdf = z.read("bot/urdf/bot.urdf").decode()
    assert "<mimic" not in urdf and "tcp" not in urdf
    assert all("_c0" not in n for n in z.namelist())


def test_export_route_rejects_a_bad_gripper():
    from fastapi.testclient import TestClient
    from app.main import app
    links, joints, bodies, _ = six_axis_with_gripper()
    body = {"robot_name": "sixg", "links": [l.model_dump() for l in links], "joints": [j.model_dump() for j in joints],
            "bodies_by_link": {k: [b.model_dump() for b in v] for k, v in bodies.items()},
            "gripper": {"leader_joint_id": "j6", "follower_joint_id": "finger_r", "tcp_offset_m": 0}}
    r = TestClient(app).post("/export", json=body)
    assert r.status_code == 422 and "two sliding joints" in r.json()["detail"]


def _merged_base():
    """The test gripper with its plate merged into l6 (bolted parts are one link), and l6 carrying an
    off-axis lump, so l6's centroid is not on the wrist axis."""
    links, joints, bodies, g = six_axis_with_gripper()
    bodies["l6"] = bodies["l6"] + bodies.pop("gbase") + [_boxes_body("lump", [((0.72, -0.02, 1.0), (0.76, 0.02, 1.08))])]
    links = [l for l in links if l.id != "gbase"]
    joints = [j.model_copy(update={"parent_link_id": "l6"}) if j.parent_link_id == "gbase" else j
              for j in joints if j.id != "gfix"]
    return links, joints, bodies, g


def test_a_gripper_bolted_into_the_last_arm_link_leaves_the_wrist_an_arm_joint():
    z = zipfile.ZipFile(io.BytesIO(export_package("sixm", *_merged_base())))
    mjcf = ET.fromstring(z.read("sixm/mjcf/sixm.xml"))
    motors = {m.get("joint"): float(m.get("kp")) for m in mjcf.iter("position")}
    armature = {j.get("name"): float(j.get("armature", 0)) for j in mjcf.iter("joint")}
    assert motors["gfl_joint"] == pytest.approx(40.0 / 0.002)
    e6 = next(float(m.get("forcerange").split()[1]) for m in mjcf.iter("position") if m.get("joint") == "l6_joint")
    assert motors["l6_joint"] <= e6 / 0.01 + 1e-9                # the wrist keeps the arm's servo rule (HOLD_ERROR)
    assert armature.get("l6_joint", 0.0) == 0.0
    urdf = ET.fromstring(z.read("sixm/urdf/sixm.urdf"))
    tcp = next(j for j in urdf.iter("joint") if j.get("name") == "tcp_joint")
    rpy = [float(x) for x in tcp.find("origin").get("rpy").split()]
    approach = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")[:3, 2]
    assert approach == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)   # along the wrist axis, not tilted by the lump


def test_only_the_fingers_are_split_into_convex_pieces():   # a real gripper body takes CoACD ~30-40 s
    links, joints, bodies, g = six_axis_with_gripper()
    names = zipfile.ZipFile(io.BytesIO(export_package("sixg", links, joints, bodies, gripper=g))).namelist()
    assert "sixg/meshes/gfl_c0.stl" in names and "sixg/meshes/gfr_c0.stl" in names
    assert not any(n.startswith("sixg/meshes/gbase_c") for n in names)


def test_the_same_finger_is_split_once(monkeypatch):   # Simulate and Program re-export the same fingers
    import coacd
    calls = []
    real = coacd.run_coacd
    monkeypatch.setattr(coacd, "run_coacd", lambda *a, **k: calls.append(1) or real(*a, **k))
    b = l_finger_body("cached", R=np.diag([1.0, -1.0, -1.0]))   # a mesh no other test has split
    first = convex_pieces(b.vertices, b.triangles)
    assert convex_pieces(b.vertices, b.triangles) == first and len(calls) == 1


def _bare_jaws():
    """The test arm with HFK40-like jaws (AirTAC air gripper from GrabCAD): modelled closed, their slides
    touch inside the plate, but the tips that stick out are 30 mm apart; 15 mm travel each (open 60)."""
    links, joints, bodies, g = six_axis_with_gripper()
    R = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    t = np.array([0.78, 0.0, 0.95]) - R @ [0.0, 0.0, 0.01]
    for name, s in (("gfl", -1.0), ("gfr", 1.0)):
        slide = ((min(0.0, s * 0.035), -0.01, 0.002), (max(0.0, s * 0.035), 0.01, 0.008))
        tip = ((min(s * 0.015, s * 0.025), -0.01, -0.06), (max(s * 0.015, s * 0.025), 0.01, 0.0))
        bodies[name] = [_boxes_body(name, [slide, tip], R, t)]
    up = tuple(float(x) for x in R @ [1.0, 0.0, 0.0])
    joints = [j.model_copy(update={"limit_lower": -0.015, "limit_upper": 0.0}) if j.id == "finger_l" else
              j.model_copy(update={"limit_lower": 0.0, "limit_upper": 0.015, "axis": up}) if j.id == "finger_r" else j
              for j in joints]
    return links, joints, bodies, g


def test_jaw_gap_is_measured_at_the_tips_not_at_slides_hidden_in_the_body():
    from app.ros.chain import parse_chain
    urdf = zipfile.ZipFile(io.BytesIO(export_package("sixj", *_bare_jaws()))).read("sixj/urdf/sixj.urdf").decode()
    g = parse_chain(urdf).gripper
    assert g.jaw_gap == pytest.approx(0.03, abs=5e-4)
    assert g.opening == pytest.approx(0.06, abs=5e-4) and g.width(g.upper) == pytest.approx(0.03, abs=5e-4)
