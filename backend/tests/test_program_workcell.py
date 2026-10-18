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


def test_robot_names_are_checked_before_they_become_paths(ws):
    assert client.get("/program/workcell", params={"robot": "../etc"}).status_code == 422
    assert client.put("/program/workcell", json={"robot": "..", "objects": []}).status_code == 422


def test_reach_and_brief_come_from_the_workspace_when_there_is_one(ws, tmp_path):
    from app.ros.cli import workspace_from_zip
    from tests.ros_helpers import two_joint_zip
    (tmp_path / "zips").mkdir()
    robot = workspace_from_zip(two_joint_zip(tmp_path / "zips"), ws / "bot", samples=50)
    assert client.get("/program/workcell", params={"robot": robot}).json()["reach_m"] > 0
    client.put("/program/workcell", json={"robot": robot, "objects": [BOX]})
    brief = (ws / "bot" / "src" / f"{robot}_program" / "brief.md").read_text(encoding="utf-8")
    assert "- table: box 0.6 × 0.4 × 0.02 m at (0.5, 0, 0.3), fixed" in brief   # the chat sees the new cell
