import json

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.program import conversations as conv, routes

client = TestClient(app)


@pytest.fixture
def src(tmp_path, monkeypatch):
    s = tmp_path / "ws" / "src"
    s.mkdir(parents=True)
    monkeypatch.setattr(routes.session, "current", lambda: {"robot": "bot", "src": s})
    return s


def test_create_list_rename_delete(src):
    a = client.post("/program/conversations", json={}).json()
    b = client.post("/program/conversations", json={"title": "Square"}).json()
    assert a["title"] == "New conversation" and len(a["id"]) == 12 and a["messages"] == []
    assert [c["id"] for c in client.get("/program/conversations").json()] == [b["id"], a["id"]]
    assert client.patch(f"/program/conversations/{a['id']}", json={"title": "Pick"}).json()["title"] == "Pick"
    assert client.get(f"/program/conversations/{a['id']}").json()["title"] == "Pick"
    assert client.delete(f"/program/conversations/{a['id']}").status_code == 200
    assert client.get(f"/program/conversations/{a['id']}").status_code == 404
    assert (src.parent / "conversations" / f"{b['id']}.json").exists()


def test_corrupt_files_are_listed_as_broken(src):
    good = client.post("/program/conversations", json={}).json()
    (src.parent / "conversations" / "0123456789ab.json").write_text("{not json")
    items = client.get("/program/conversations").json()
    assert {i["id"] for i in items} == {good["id"], "0123456789ab"}
    assert next(i for i in items if i["id"] == "0123456789ab") == {
        "id": "0123456789ab", "title": "Unreadable conversation", "broken": True}


def test_bad_ids_and_no_session(src, monkeypatch):
    assert client.get("/program/conversations/../x").status_code in (400, 404)
    assert client.get("/program/conversations/NOTHEX123456").status_code == 400
    monkeypatch.setattr(routes.session, "current", lambda: None)
    r = client.get("/program/conversations")
    assert r.status_code == 409 and r.json()["detail"] == "Start the ROS session first."


def test_title_from_and_fold():
    assert conv.title_from("  Put a 5 cm box on the floor at (0.5, 0.2) and more ") == "Put a 5 cm box on"
    events = [{"kind": "text", "text": "Planning"}, {"kind": "file", "path": "programs/task_001.py"},
              {"kind": "run", "n": 1}, {"kind": "run_result", "ok": False, "message": "failed: X", "output": "log1"},
              {"kind": "run", "n": 2}, {"kind": "run_result", "ok": True, "message": "ok", "output": "log2"},
              {"kind": "text", "text": "Done.\nAll good."},
              {"kind": "done", "summary": "Done.\nAll good.", "runs": 2, "ok": True, "session_id": "s1",
               "program": "programs/task_001.py", "code": "x"}]
    m = conv.fold(events)
    assert m == {"role": "claude", "items": [
        {"kind": "text", "text": "Planning"}, {"kind": "file", "path": "programs/task_001.py"},
        {"kind": "run", "n": 1, "ok": False, "message": "failed: X", "output": "log1"},
        {"kind": "run", "n": 2, "ok": True, "message": "ok", "output": "log2"}],
        "summary": "Done.\nAll good.", "result": {"ok": True, "runs": 2}, "error": None,
        "program": "programs/task_001.py"}
    assert conv.fold([{"kind": "error", "message": "boom"}])["error"] == "boom"
