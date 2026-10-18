"""The workcell editor's backend: workcell.yaml as JSON objects, by robot name, with or without ROS running."""
import zipfile

import pytest
import yaml
from fastapi.testclient import TestClient

from app.main import app
from app.program import workcell

client = TestClient(app)
BOX = {"name": "table", "shape": "box", "size": [0.6, 0.4, 0.02], "xyz": [0.5, 0.0, 0.3], "fixed": True}
PART = {"name": "part", "shape": "cylinder", "size": [0.02, 0.05], "xyz": [0.5, 0.1, 0.335], "mass": 0.1}


@pytest.fixture
def ws(tmp_path, monkeypatch):
    monkeypatch.setattr(workcell, "WORKSPACES", tmp_path)
    return tmp_path


def test_a_robot_without_a_workspace_has_an_empty_workcell(ws):
    r = client.get("/program/workcell", params={"robot": "bot"})
    assert r.status_code == 200 and r.json()["objects"] == [] and r.json()["error"] is None


def test_put_writes_workcell_yaml_that_runs_read_back(ws):
    r = client.put("/program/workcell", json={"robot": "bot", "objects": [BOX, PART]})
    assert r.status_code == 200
    text = (ws / "bot" / "src" / "bot_tasks" / "workcell.yaml").read_text(encoding="utf-8")
    on_disk = yaml.safe_load(text)
    assert [o["name"] for o in on_disk] == ["table", "part"] and on_disk[0]["fixed"] is True
    got = client.get("/program/workcell", params={"robot": "bot"}).json()["objects"]
    assert got[1]["xyz"] == [0.5, 0.1, 0.335] and got[1]["mass"] == 0.1


def test_a_bad_object_is_refused_in_plain_words_and_nothing_is_written(ws):
    r = client.put("/program/workcell", json={"robot": "bot", "objects": [{**BOX, "size": [0.6, -1, 0.02]}]})
    assert r.status_code == 422 and "size must be 3 numbers > 0" in r.json()["detail"]
    assert not (ws / "bot" / "src" / "bot_tasks" / "workcell.yaml").exists()


def test_what_get_returns_can_be_put_back(ws):   # found in the browser: fixed objects come back with mass: null
    client.put("/program/workcell", json={"robot": "bot", "objects": [BOX, PART]})
    objs = client.get("/program/workcell", params={"robot": "bot"}).json()["objects"]
    assert objs[0]["mass"] is None
    r = client.put("/program/workcell", json={"robot": "bot", "objects": objs})
    assert r.status_code == 200, r.json()


def test_robot_names_are_checked_before_they_become_paths(ws):
    assert client.get("/program/workcell", params={"robot": "../etc"}).status_code == 422
    assert client.put("/program/workcell", json={"robot": "..", "objects": []}).status_code == 422


def test_reach_and_brief_come_from_the_workspace_when_there_is_one(ws, tmp_path):
    from app.ros.cli import workspace_from_zip
    from tests.ros_helpers import two_joint_zip
    (tmp_path / "zips").mkdir()
    robot = workspace_from_zip(two_joint_zip(tmp_path / "zips"), ws / "bot", samples=50)
    prof = client.get("/program/workcell", params={"robot": robot}).json()["reach"]
    assert prof and all(r > 0 for _, r in prof)
    assert [z for z, _ in prof] == sorted(z for z, _ in prof)    # one radius per height band, bottom up
    client.put("/program/workcell", json={"robot": robot, "objects": [BOX]})
    brief = (ws / "bot" / "src" / f"{robot}_program" / "brief.md").read_text(encoding="utf-8")
    assert "- table: box 0.6 × 0.4 × 0.02 m at (0.5, 0, 0.3), fixed" in brief   # the chat sees the new cell


def test_a_saved_workcell_loads_into_another_robot_with_its_fixture_meshes(ws):
    import io
    folder = workcell.meshes("bot")
    folder.mkdir(parents=True)
    for f in ("rack.stl", "rack_c0.stl", "rack_c1.stl"):
        (folder / f).write_bytes(b"solid " + f.encode())
    rack = {"name": "rack", "shape": "mesh", "file": "rack.stl", "pieces": 2, "size": [0.3, 0.3, 0.2],
            "xyz": [0.6, 0.0, 0.1], "fixed": True}
    assert client.put("/program/workcell", json={"robot": "bot", "objects": [BOX, rack]}).status_code == 200
    saved = client.get("/program/workcell/bundle", params={"robot": "bot"})
    assert saved.status_code == 200
    assert sorted(zipfile.ZipFile(io.BytesIO(saved.content)).namelist()) == [
        "meshes/rack.stl", "meshes/rack_c0.stl", "meshes/rack_c1.stl", "workcell.yaml"]
    # the yaml alone, on a robot without those meshes: refused, nothing written
    yaml_only = zipfile.ZipFile(io.BytesIO(saved.content)).read("workcell.yaml")
    r = client.post("/program/workcell/bundle", params={"robot": "other"}, files={"file": ("workcell.yaml", yaml_only)})
    assert r.status_code == 422 and "rack_c1.stl" in r.json()["detail"]
    assert not workcell.path("other").exists()
    r = client.post("/program/workcell/bundle", params={"robot": "other"}, files={"file": ("cell.zip", saved.content)})
    assert r.status_code == 200, r.text
    assert [o["name"] for o in r.json()["objects"]] == ["table", "rack"]
    assert (workcell.meshes("other") / "rack_c1.stl").read_bytes() == b"solid rack_c1.stl"
    bad = client.post("/program/workcell/bundle", params={"robot": "other"}, files={"file": ("w.yaml", b"- {name: x}")})
    assert bad.status_code == 422
    assert [o["name"] for o in workcell.read("other")["objects"]] == ["table", "rack"]   # a bad load changes nothing
