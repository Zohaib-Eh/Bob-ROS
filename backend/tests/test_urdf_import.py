import io
import zipfile

import numpy as np
import pytest
import trimesh
from fastapi.testclient import TestClient

from app.export.urdf_writer import export_package
from app.importers.urdf_importer import import_urdf
from app.main import app
from tests.ros_helpers import six_axis_with_gripper


def test_this_apps_export_comes_back_as_the_same_robot():
    links, joints, bodies, g = six_axis_with_gripper()
    r = import_urdf("sixg.zip", export_package("sixg", links, joints, bodies, gripper=g))
    names = {l.name for l in r["links"]}
    assert names == {l.name for l in links}                       # tcp (no shape, fixed) is left out
    assert any(w.code == "dropped_links" and "tcp" in w.message for w in r["warnings"])
    assert any(w.code == "no_faces" for w in r["warnings"])
    by_child = {next(l.name for l in r["links"] if l.id == j.child_link_id): j for j in r["joints"]}
    for j in joints:
        child = next(l.name for l in links if l.id == j.child_link_id)
        got = by_child[child]
        assert got.type == j.type and got.origin_xyz == pytest.approx(j.origin_xyz, abs=1e-6)
        if j.type == "revolute":
            assert got.axis == pytest.approx(j.axis, abs=1e-6)       # world axes, as the app keeps them
            assert (got.limit_lower, got.limit_upper) == pytest.approx((j.limit_lower, j.limit_upper))
    # the parts sit where they were (world coordinates at q = 0)
    for l in links:
        before = np.vstack([b.vertices for b in bodies[l.id]])
        after = np.array(next(b for b in r["bodies"] if b.name == l.name).vertices)
        assert after.min(0) == pytest.approx(before.min(0), abs=1e-5) and after.max(0) == pytest.approx(before.max(0), abs=1e-5)
    assert r["gripper"] is not None
    lead = next(j for j in r["joints"] if j.id == r["gripper"]["leader"])
    assert lead.type == "prismatic"
    assert all(l.mass_override_kg and l.mass_override_kg > 0 for l in r["links"])


def _zip(files: dict) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        for n, d in files.items():
            z.writestr(n, d)
    return buf.getvalue()


URDF = """<robot name="r">
  <link name="world"/>
  <joint name="fix" type="fixed"><parent link="world"/><child link="base"/></joint>
  <link name="base"><visual><geometry><box size="0.2 0.2 0.1"/></geometry></visual>
    <inertial><mass value="3"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0.05" rpy="0 1.5707963 0"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="20" velocity="1"/></joint>
  <link name="arm"><visual><origin xyz="0 0 0.1"/><geometry><mesh filename="package://r_description/meshes/arm.dae" scale="2 2 2"/></geometry></visual></link>
  <joint name="j2" type="continuous"><parent link="arm"/><child link="wrist"/><origin xyz="0 0 0.2"/></joint>
  <link name="wrist"/>
</robot>"""


def _dae(unit: float) -> bytes:
    d = trimesh.creation.box((0.01, 0.01, 0.1)).export(file_type="dae")
    d = d.decode() if isinstance(d, bytes) else d
    return d.replace("<asset>", f'<asset><unit name="u" meter="{unit}"/>', 1).encode()


def test_package_meshes_units_shapeless_root_and_placeholders():
    r = import_urdf("r.zip", _zip({"r_description/urdf/r.urdf": URDF, "r_description/meshes/arm.dae": _dae(1.0)}))
    names = [l.name for l in r["links"]]
    assert names == ["base", "arm", "wrist"]                       # world: shapeless root on a fixed joint
    assert r["root_link_id"] == "urdf_base"
    j1 = next(j for j in r["joints"] if j.id == "urdf_j1")
    assert j1.axis == pytest.approx([1, 0, 0], abs=1e-6)           # z turned 90 degrees about y: world x
    assert (j1.limit_lower, j1.limit_upper, j1.max_effort) == (-1, 1, 20)
    j2 = next(j for j in r["joints"] if j.id == "urdf_j2")
    assert j2.axis == pytest.approx([0, 0, -1], abs=1e-6)          # URDF's default axis, x, in the turned joint frame
    arm = np.array(next(b for b in r["bodies"] if b.name == "arm").vertices)
    assert np.ptp(arm, axis=0) == pytest.approx([0.2, 0.02, 0.02], abs=1e-6)   # scale 2, long along world x
    assert any(w.code == "placeholder" and "wrist" in w.message for w in r["warnings"])
    assert next(l for l in r["links"] if l.name == "base").mass_override_kg == 3
    mm = import_urdf("r.zip", _zip({"r_description/urdf/r.urdf": URDF, "r_description/meshes/arm.dae": _dae(0.001)}))
    arm = np.array(next(b for b in mm["bodies"] if b.name == "arm").vertices)
    assert np.ptp(arm, axis=0).max() == pytest.approx(0.0002, abs=1e-8)     # COLLADA unit honoured


def test_a_mesh_path_finds_its_own_file_among_same_named_ones():   # found on example-robot-data's UR: ur3/ ur5/ ur10/
    box = lambda s: trimesh.creation.box((s, s, s)).export(file_type="stl")
    urdf = URDF.replace("package://r_description/meshes/arm.dae", "package://data/robots/ur/meshes/ur5/arm.stl").replace('scale="2 2 2"', "")
    r = import_urdf("r.zip", _zip({"ur/urdf/r.urdf": urdf, "ur/meshes/ur3/arm.stl": box(0.03), "ur/meshes/ur5/arm.stl": box(0.05)}))
    arm = np.array(next(b for b in r["bodies"] if b.name == "arm").vertices)
    assert np.ptp(arm, axis=0) == pytest.approx([0.05] * 3, abs=1e-6)


def test_missing_meshes_and_bad_files_are_reported():
    r = import_urdf("r.urdf", URDF.encode())                      # no zip: the mesh can't be found
    assert any(w.code == "missing_mesh" for w in r["warnings"])
    with pytest.raises(ValueError, match="no .urdf"):
        import_urdf("x.zip", _zip({"a/robot.urdf.xacro": "<robot/>"}))
    with pytest.raises(ValueError, match="one root"):
        import_urdf("x.urdf", b'<robot name="x"><link name="a"/><link name="b"/></robot>')


def test_route():
    c = TestClient(app)
    res = c.post("/import/urdf", files={"file": ("r.zip", _zip({"r/urdf/r.urdf": URDF, "r/meshes/arm.dae": _dae(1.0)}))})
    assert res.status_code == 200, res.text
    assert [l["name"] for l in res.json()["links"]] == ["base", "arm", "wrist"]
    assert c.post("/import/urdf", files={"file": ("r.step", b"x")}).status_code == 400
    assert c.post("/import/urdf", files={"file": ("r.urdf", b"<nope")}).status_code == 422
