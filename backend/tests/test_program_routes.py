import json

from fastapi.testclient import TestClient

from app.main import app
from app.program import routes

client = TestClient(app)


def test_run_streams_agent_events(monkeypatch, tmp_path):
    monkeypatch.setattr(routes.session, "current", lambda: {"robot": "bot", "src": tmp_path})
    monkeypatch.setattr(routes.session, "status", lambda: {"state": "ready", "robot": "bot"})
    seen = {}

    def fake_agent(robot, src, task, resume=None):
        seen.update(robot=robot, task=task, resume=resume)
        yield {"kind": "text", "text": "hi"}
        yield {"kind": "done", "summary": "ok", "runs": 1, "ok": True, "session_id": "s", "program": None, "code": None}
    monkeypatch.setattr(routes.agent, "run_agent", fake_agent)
    r = client.post("/program/run", json={"task": "move", "resume": "s0"})
    assert r.status_code == 200 and r.headers["content-type"].startswith("text/event-stream")
    events = [json.loads(chunk[6:]) for chunk in r.text.split("\n\n") if chunk.startswith("data: ")]
    assert [e["kind"] for e in events] == ["text", "done"]
    assert seen == {"robot": "bot", "task": "move", "resume": "s0"}


def test_run_without_a_ready_session_is_409(monkeypatch):
    monkeypatch.setattr(routes.session, "current", lambda: None)
    assert client.post("/program/run", json={"task": "x"}).status_code == 409


def test_session_errors_are_503(monkeypatch):
    def boom(*a):
        raise routes.session.SessionError("Start Docker Desktop.")
    monkeypatch.setattr(routes.session, "start", boom)
    body = {"robot_name": "bot", "links": [], "joints": [], "bodies_by_link": {}}
    r = client.post("/program/session", json=body)
    assert r.status_code == 503 and r.json()["detail"] == "Start Docker Desktop."


def test_rerun_bad_path_is_400(monkeypatch):
    monkeypatch.setattr(routes.session, "run_program", lambda p: (_ for _ in ()).throw(ValueError("Program path must look like programs/task_001.py.")))
    assert client.post("/program/rerun", json={"program": "../x"}).status_code == 400
