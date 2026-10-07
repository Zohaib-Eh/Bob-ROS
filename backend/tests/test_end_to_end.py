import io
import zipfile
import xml.etree.ElementTree as ET

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


def _tri_body(bid):
    return Body(id=bid, name=bid, vertices=[(0, 0, 0), (1, 0, 0), (0, 1, 0)],
                triangles=[(0, 1, 2)])


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
            "bodies_by_link": {i: [_tri_body(i).model_dump()] for i in links}}


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
