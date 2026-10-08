import io
import zipfile
import xml.etree.ElementTree as ET

import pytest
import trimesh
from fastapi.testclient import TestClient

from app.main import app
from app.export.urdf_writer import export_package
from app.importers.step_importer import import_step
from app.joints.face_matching import find_matching_face_pairs
from app.joints.ipv_img import classify_joint
from app.massprops.link_mass import compute_link_mass_properties
from app.models import Body, Link

client = TestClient(app)


def test_peg_in_hole_assembly_imports_detects_and_exports(peg_in_hole_step_path):
    bodies = import_step(peg_in_hole_step_path)
    assert len(bodies) == 2
    body_hole, body_peg = bodies

    pairs = find_matching_face_pairs(body_hole, body_peg)
    assert len(pairs) >= 1

    joint = classify_joint(pairs, parent_link_id="base", child_link_id="peg")
    assert joint.type == "cylindrical"
    # URDF cannot represent cylindrical; the user confirms it as continuous.
    joint.type = "continuous"
    joint.source = "confirmed"

    mass_a, com_a, inertia_a = compute_link_mass_properties([body_hole], 1000.0)
    mass_b, com_b, inertia_b = compute_link_mass_properties([body_peg], 1000.0)
    links = [
        Link(id="base", name="base", body_ids=[body_hole.id], density_kg_m3=1000.0,
             mass_kg=mass_a, center_of_mass=com_a, inertia=inertia_a),
        Link(id="peg", name="peg", body_ids=[body_peg.id], density_kg_m3=1000.0,
             mass_kg=mass_b, center_of_mass=com_b, inertia=inertia_b),
    ]
    data = export_package("peg_robot", links, [joint],
                          {"base": [body_hole], "peg": [body_peg]})
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        names = z.namelist()
        urdf = z.read("peg_robot/urdf/peg_robot.urdf").decode()
    assert "peg_robot/meshes/base.stl" in names and "peg_robot/meshes/peg.stl" in names
    assert ET.fromstring(urdf).tag == "robot"
    assert 'type="continuous"' in urdf


def test_health_route_still_works():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


def test_import_box_stl_segments_into_six_planes(box_stl_path):
    with open(box_stl_path, "rb") as f:
        r = client.post("/import", files={"file": ("box.stl", f.read())})
    assert r.status_code == 200
    j = r.json()
    assert j["applied_scale"] == 0.001
    assert len(j["bodies"]) == 1
    faces = j["bodies"][0]["faces"]
    assert len(faces) == 6
    assert all(f["kind"] == "plane" and f["origin"] == "segmented" for f in faces)


def test_import_unsupported_extension_is_400():
    r = client.post("/import", files={"file": ("notes.txt", b"hello")})
    assert r.status_code == 400


def _box_body(bid):
    box = trimesh.creation.box(extents=(1, 1, 1))  # closed unit cube at origin
    return Body(id=bid, name=bid, vertices=[tuple(v) for v in box.vertices.tolist()],
                triangles=[tuple(t) for t in box.faces.tolist()])


def _link(lid):
    return Link(id=lid, name=lid, body_ids=[lid], density_kg_m3=1000.0,
                mass_kg=1.0, center_of_mass=(0, 0, 0), inertia=[1, 1, 1, 0, 0, 0])


def _joint(parent, child, jid="j"):
    return dict(id=jid, parent_link_id=parent, child_link_id=child, type="fixed",
                origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0), source="confirmed")


def _payload(links, joints):
    return {"robot_name": "r",
            "links": [_link(i).model_dump() for i in links],
            "joints": joints,
            "bodies_by_link": {i: [_box_body(i).model_dump()] for i in links}}


def test_export_cycle_is_422():
    r = client.post("/export", json=_payload(
        ["a", "b"], [_joint("a", "b", "j1"), _joint("b", "a", "j2")]))
    assert r.status_code == 422


def test_export_valid_model_returns_zip():
    r = client.post("/export", json=_payload(["a", "b"], [_joint("a", "b")]))
    assert r.status_code == 200
    assert r.headers["content-type"] == "application/zip"
    assert 'r.zip' in r.headers["content-disposition"]
    assert zipfile.ZipFile(io.BytesIO(r.content)).testzip() is None


def test_import_nan_scale_is_422(box_stl_path):
    with open(box_stl_path, "rb") as f:
        r = client.post("/import?scale=nan", files={"file": ("box.stl", f.read())})
    assert r.status_code == 422


def test_import_scale_override(box_stl_path):
    with open(box_stl_path, "rb") as f:
        r = client.post("/import?scale=1.0", files={"file": ("box.stl", f.read())})
    assert r.status_code == 200
    assert r.json()["applied_scale"] == 1.0


def test_detect_joints_cylinder_without_axis_is_422():
    face = {"kind": "cylinder", "origin": "brep", "center": [0, 0, 0], "radius": 1.0,
            "residual": 0, "triangle_indices": []}
    body = {"id": "a", "name": "a", "vertices": [], "triangles": [], "faces": [face]}
    r = client.post("/detect-joints", json={"body_a": body, "body_b": body,
                                            "parent_link_id": "a", "child_link_id": "b"})
    assert r.status_code == 422


def test_export_zero_length_axis_is_422():
    joint = dict(_joint("a", "b"), type="continuous", axis=[0, 0, 0])
    r = client.post("/export", json=_payload(["a", "b"], [joint]))
    assert r.status_code == 422


def _urdf_link(r, name):
    root = ET.fromstring(zipfile.ZipFile(io.BytesIO(r.content)).read("r/urdf/r.urdf"))
    return next(el for el in root.findall("link") if el.attrib["name"] == name)


def test_export_ignores_client_mass_and_uses_geometry():
    payload = _payload(["a", "b"], [_joint("a", "b")])
    payload["links"][0].update(mass_kg=-1.0, center_of_mass=[9, 9, 9],
                               inertia=[-5, -5, -5, 0, 0, 0])
    r = client.post("/export", json=payload)
    assert r.status_code == 200
    inertial = _urdf_link(r, "a").find("inertial")
    assert float(inertial.find("mass").attrib["value"]) == pytest.approx(1000.0)
    i = inertial.find("inertia").attrib
    # solid unit cube, 1000 kg: I = m (1 + 1) / 12 about each axis
    for k in ("ixx", "iyy", "izz"):
        assert float(i[k]) == pytest.approx(1000.0 / 6.0, rel=1e-6)
    for k in ("ixy", "ixz", "iyz"):
        assert float(i[k]) == pytest.approx(0.0, abs=1e-9)
    assert [float(x) for x in inertial.find("origin").attrib["xyz"].split()] == pytest.approx([0, 0, 0], abs=1e-9)


@pytest.mark.parametrize("density", [0.0, -1.0])
def test_export_non_positive_density_is_422_naming_link(density):
    payload = _payload(["a", "b"], [_joint("a", "b")])
    payload["links"][1]["density_kg_m3"] = density
    r = client.post("/export", json=payload)
    assert r.status_code == 422
    assert "'b'" in r.json()["detail"] and "density" in r.json()["detail"]


def test_export_zero_volume_body_is_422_naming_link():
    payload = _payload(["a", "b"], [_joint("a", "b")])
    payload["bodies_by_link"]["b"] = [{"id": "t", "name": "t",
                                       "vertices": [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                                       "triangles": [[0, 1, 2]]}]
    r = client.post("/export", json=payload)
    assert r.status_code == 422
    assert "'b'" in r.json()["detail"] and "volume" in r.json()["detail"]


_raising_client = TestClient(app, raise_server_exceptions=False)


@pytest.mark.parametrize("name, data", [
    ("junk.iges", b"junk"), ("junk.step", b"garbage"), ("junk.brep", b"junk"),
    ("junk.stl", b"garbage"), ("junk.obj", b"v 0 0 0\n"),
])
def test_import_corrupt_file_is_422_naming_upload(name, data):
    r = _raising_client.post("/import", files={"file": (name, data)})
    assert r.status_code == 422
    detail = r.json()["detail"]
    assert name in detail
    assert "tmp" not in detail.lower() and "temp" not in detail.lower()


def test_import_empty_stl_is_422_could_not_read():
    r = _raising_client.post("/import", files={"file": ("empty.stl", b"")})
    assert r.status_code == 422
    assert "could not read" in r.json()["detail"].lower()
    assert "empty.stl" in r.json()["detail"]


@pytest.mark.xfail(reason="wired in Task 4", strict=True)
def test_import_non_watertight_is_approximate_not_422(non_watertight_stl_path):
    with open(non_watertight_stl_path, "rb") as f:
        r = client.post("/import", files={"file": ("broken.stl", f.read())})
    assert r.status_code == 200
    body = r.json()["bodies"][0]
    assert body["status"] == "approximate"
    assert body["warnings"][0]["code"] == "open_geometry"
    assert "tmp" not in body["warnings"][0]["message"].lower()


def _cyl_radius(r):
    return next(f["radius"] for f in r.json()["bodies"][0]["faces"] if f["kind"] == "cylinder")


def test_brep_import_reports_and_honors_scale(cylinder_brep_path):
    data = open(cylinder_brep_path, "rb").read()
    r = client.post("/import", files={"file": ("cyl.brep", data)})
    assert r.status_code == 200
    assert r.json()["applied_scale"] == 0.001
    assert _cyl_radius(r) == pytest.approx(0.5)
    r1 = client.post("/import?scale=1.0", files={"file": ("cyl.brep", data)})
    assert r1.status_code == 200
    assert r1.json()["applied_scale"] == 1.0
    assert _cyl_radius(r1) == pytest.approx(500.0)
    xs = [v[0] for v in r1.json()["bodies"][0]["vertices"]]
    assert max(xs) - min(xs) == pytest.approx(1000.0, rel=1e-6)


def test_step_import_reports_unit_scale_and_ignores_scale_param(cylinder_step_path):
    r = client.post("/import?scale=5.0",
                    files={"file": ("cyl.step", open(cylinder_step_path, "rb").read())})
    assert r.status_code == 200
    assert r.json()["applied_scale"] == 1.0
    assert _cyl_radius(r) == pytest.approx(0.5)


def _shifted(body: dict, dx: float) -> dict:
    out = dict(body, id=body["id"] + "_shifted")
    out["vertices"] = [[x + dx, y, z] for x, y, z in body["vertices"]]
    out["faces"] = [dict(f, center=[f["center"][0] + dx, *f["center"][1:]]) for f in body["faces"]]
    return out


def _detect(a, b):
    return client.post("/detect-joints", json={"body_a": a, "body_b": b,
                                               "parent_link_id": "a", "child_link_id": "b"})


def test_detect_joints_far_apart_bodies_returns_null(box_step_path):
    box = import_step(box_step_path)[0].model_dump()
    r = _detect(box, _shifted(box, 100.0))
    assert r.status_code == 200
    assert r.json() is None


def test_detect_joints_touching_bodies_returns_joint(box_step_path):
    box = import_step(box_step_path)[0].model_dump()  # 1 x 2 x 3, x in [0, 1]
    r = _detect(box, _shifted(box, 1.0))
    assert r.status_code == 200
    assert r.json()["type"] == "planar"


def _confirm_and_export(block: dict, peg: dict) -> zipfile.ZipFile:
    r = _detect(block, peg)
    assert r.status_code == 200
    joint = r.json()
    assert joint["type"] == "cylindrical"
    joint.update(type="continuous", source="confirmed")  # user confirms a URDF type
    links = [{"id": lid, "name": lid, "body_ids": [b["id"]], "density_kg_m3": 1000.0}
             for lid, b in (("a", block), ("b", peg))]
    r = client.post("/export", json={"robot_name": "r", "links": links, "joints": [joint],
                                     "bodies_by_link": {"a": [block], "b": [peg]}})
    assert r.status_code == 200
    z = zipfile.ZipFile(io.BytesIO(r.content))
    assert {"r/urdf/r.urdf", "r/meshes/a.stl", "r/meshes/b.stl"} <= set(z.namelist())
    root = ET.fromstring(z.read("r/urdf/r.urdf"))
    assert root.find("joint").attrib["type"] == "continuous"
    for link_el in root.findall("link"):
        assert float(link_el.find("inertial").find("mass").attrib["value"]) > 0
    return z


def test_http_step_peg_in_hole_import_detect_confirm_export(peg_in_hole_step_path):
    r = client.post("/import", files={"file": ("peg.step", open(peg_in_hole_step_path, "rb").read())})
    assert r.status_code == 200
    block, peg = r.json()["bodies"]
    _confirm_and_export(block, peg)


def test_http_stl_two_part_import_detect_confirm_export(peg_in_hole_step_path, tmp_path):
    # one fine mm-unit STL per part, from the STEP tessellation
    bodies = []
    for b in import_step(peg_in_hole_step_path):
        stl = trimesh.Trimesh(b.vertices, b.triangles).apply_scale(1000.0).export(file_type="stl")
        r = client.post("/import", files={"file": (f"{b.name}.stl", stl)})
        assert r.status_code == 200
        assert r.json()["applied_scale"] == 0.001
        bodies.append(r.json()["bodies"][0])
    _confirm_and_export(*bodies)
