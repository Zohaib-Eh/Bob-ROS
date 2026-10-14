import json

from fastapi.testclient import TestClient

from app.main import app
from app.program import routes

client = TestClient(app)


def test_session_errors_are_503(monkeypatch):
    def boom(*a):
        raise routes.session.SessionError("Start Docker Desktop.")
    monkeypatch.setattr(routes.session, "start", boom)
    body = {"robot_name": "bot", "links": [], "joints": [], "bodies_by_link": {}}
    r = client.post("/program/session", json=body)
    assert r.status_code == 503 and r.json()["detail"] == "Start Docker Desktop."


def test_delete_session_when_docker_fails_is_503(monkeypatch):
    def boom():
        raise routes.session.SessionError("Start Docker Desktop.")
    monkeypatch.setattr(routes.session, "stop", boom)
    r = client.delete("/program/session")
    assert r.status_code == 503 and r.json()["detail"] == "Start Docker Desktop."


def test_start_while_busy_is_409(monkeypatch):
    def busy(*a):
        raise routes.session.SessionBusy("Startup is still finishing; try again in a moment.")
    monkeypatch.setattr(routes.session, "start", busy)
    body = {"robot_name": "bot", "links": [], "joints": [], "bodies_by_link": {}}
    r = client.post("/program/session", json=body)
    assert r.status_code == 409 and "still finishing" in r.json()["detail"]
