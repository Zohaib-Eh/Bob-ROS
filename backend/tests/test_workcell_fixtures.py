"""CAD fixtures in the workcell (a table, a conveyor from GrabCAD): imported once, bolted down, in MuJoCo as
convex pieces and in MoveIt as the exact mesh."""
import importlib.util
import io
import zipfile
from pathlib import Path

import mujoco
import pytest
import trimesh
from fastapi.testclient import TestClient

from app.main import app
from app.program import workcell
from tests.ros_helpers import two_joint_zip

_spec = importlib.util.spec_from_file_location(
    "scene", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "scene.py")
scene = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scene)
client = TestClient(app)


def table_stl() -> bytes:
    """A table, metres: 0.6 × 0.4 top at 0.27..0.30 on four 40 mm legs. Not convex: a single hull would
    fill the space under the top."""
    parts = [trimesh.creation.box([0.6, 0.4, 0.03], trimesh.transformations.translation_matrix([0, 0, 0.285]))]
    parts += [trimesh.creation.box([0.04, 0.04, 0.27], trimesh.transformations.translation_matrix([x, y, 0.135]))
              for x in (-0.27, 0.27) for y in (-0.17, 0.17)]
    return trimesh.util.concatenate(parts).export(file_type="stl")


@pytest.fixture
def ws(tmp_path, monkeypatch):
    monkeypatch.setattr(workcell, "WORKSPACES", tmp_path)
    return tmp_path


def test_a_mesh_object_validates_and_is_always_bolted_down():
    ok = {"name": "table", "shape": "mesh", "file": "table.stl", "pieces": 3, "size": [0.6, 0.4, 0.3], "xyz": [0.8, 0, 0.15], "fixed": True}
    [o] = scene.validate([ok], "workcell")
    assert o["file"] == "table.stl" and o["pieces"] == 3 and o["mass"] is None
    with pytest.raises(scene.SceneError, match="CAD fixtures are bolted down"):
        scene.validate([{**ok, "fixed": False}], "workcell")
    with pytest.raises(scene.SceneError, match="file must be a .stl name"):
        scene.validate([{**ok, "file": "../x.stl"}], "workcell")


def test_importing_a_fixture_adds_it_on_the_floor_with_its_pieces(ws):
    r = client.post("/program/workcell/fixture", data={"robot": "bot", "scale": "1"},
                    files={"file": ("table.stl", table_stl(), "application/octet-stream")})
    assert r.status_code == 200, r.text
    [o] = r.json()["objects"]
    assert (o["name"], o["shape"], o["fixed"], o["file"]) == ("table", "mesh", True, "table.stl")
    assert o["size"] == pytest.approx([0.6, 0.4, 0.3], abs=1e-6)
    assert o["xyz"][2] == pytest.approx(0.15, abs=1e-6)      # standing on the floor
    assert o["pieces"] >= 2                                   # legs and top, not one solid block
    meshes = ws / "bot" / "src" / "bot_tasks" / "meshes"
    assert (meshes / "table.stl").exists() and all((meshes / f"table_c{i}.stl").exists() for i in range(o["pieces"]))
    again = client.post("/program/workcell/fixture", data={"robot": "bot", "scale": "1"},
                        files={"file": ("table.stl", table_stl(), "application/octet-stream")}).json()["objects"]
    assert [x["name"] for x in again] == ["table", "table_2"]


def test_a_box_set_on_an_imported_table_stays_on_it_in_mujoco(ws, tmp_path):
    client.post("/program/workcell/fixture", data={"robot": "bot", "scale": "1"},
                files={"file": ("table.stl", table_stl(), "application/octet-stream")})
    [table] = workcell.read("bot")["objects"]
    table = {**table, "xyz": [1.5, 1.5, 0.15]}                # well away from the 2-joint arm
    box = {"name": "part", "shape": "box", "size": [0.05, 0.05, 0.05], "xyz": [1.5, 1.5, 0.33], "mass": 0.2}
    objs = scene.validate([table, box], "workcell")
    zipfile.ZipFile(two_joint_zip(tmp_path)).extractall(tmp_path / "x")
    root = tmp_path / "x" / "bot"
    xml = scene.build_scene_xml((root / "mjcf" / "bot.xml").read_text(), objs, "hand", str(root / "meshes"),
                                fixture_dir=str(workcell.path("bot").parent / "meshes"))
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    z = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "part")][2]
    assert z == pytest.approx(0.30 + 0.025, abs=0.003)        # on the top, not fallen through or onto the floor


def test_the_editor_gets_a_fixtures_triangles_to_draw(ws):
    client.post("/program/workcell/fixture", data={"robot": "bot", "scale": "1"},
                files={"file": ("table.stl", table_stl(), "application/octet-stream")})
    r = client.get("/program/workcell/mesh", params={"robot": "bot", "file": "table.stl"})
    assert r.status_code == 200
    m = r.json()
    assert len(m["triangles"]) == 60 and max(max(t) for t in m["triangles"]) < len(m["vertices"])   # 5 boxes x 12
    assert client.get("/program/workcell/mesh", params={"robot": "bot", "file": "../x.stl"}).status_code == 422
    assert client.get("/program/workcell/mesh", params={"robot": "bot", "file": "nope.stl"}).status_code == 404


def test_read_stl_gives_the_mesh_back(tmp_path):
    p = tmp_path / "t.stl"
    p.write_bytes(table_stl())
    v, f = scene.read_stl(str(p))
    m = trimesh.load(io.BytesIO(table_stl()), file_type="stl")
    assert len(f) == len(m.faces) and max(max(t) for t in f) < len(v)


def test_the_brief_lists_a_fixture_by_its_bounding_box():
    from app.ros.program_pkg import _workcell_lines
    text = "- {name: table, shape: mesh, file: table.stl, pieces: 3, size: [0.6, 0.4, 0.3], xyz: [0.8, 0, 0.15], fixed: true}\n"
    assert _workcell_lines(text) == ["- table: CAD fixture, 0.6 × 0.4 × 0.3 m bounding box, at (0.8, 0, 0.15), fixed"]
