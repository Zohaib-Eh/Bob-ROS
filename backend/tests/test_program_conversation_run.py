import json

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.program import conversations, routes

client = TestClient(app)


@pytest.fixture
def setup(tmp_path, monkeypatch):
    s = tmp_path / "ws" / "src"
    s.mkdir(parents=True)
    monkeypatch.setattr(routes.session, "current", lambda: {"robot": "bot", "src": s})
    monkeypatch.setattr(routes.session, "status", lambda: {"state": "ready", "robot": "bot"})
    seen = {}

    def fake_agent(robot, src, task, resume=None, cancel=None):
        seen.update(task=task, resume=resume, cancel=cancel)
        yield {"kind": "text", "text": "Writing"}
        yield {"kind": "run", "n": 1}
        yield {"kind": "run_result", "ok": True, "message": "ok", "output": "RESULT: ok"}
        yield {"kind": "done", "summary": "Moved.", "runs": 1, "ok": True, "session_id": "s1",
               "program": "programs/task_001.py", "code": "x"}
    monkeypatch.setattr(routes.agent, "run_agent", fake_agent)
    return s, seen


def _events(r):
    return [json.loads(c[6:]) for c in r.text.split("\n\n") if c.startswith("data: ")]


def test_run_streams_saves_and_resumes(setup):
    src, seen = setup
    cid = conversations.create(src)["id"]
    r = client.post(f"/program/conversations/{cid}/run", json={"task": "Move the tool up please"})
    assert [e["kind"] for e in _events(r)] == ["text", "run", "run_result", "done"]
    c = conversations.load(src, cid)
    assert c["title"] == "Move the tool up please" and c["claude_session_id"] == "s1"
    assert c["messages"][0] == {"role": "user", "text": "Move the tool up please"}
    assert c["messages"][1]["summary"] == "Moved." and c["messages"][1]["items"][-1]["output"] == "RESULT: ok"
    client.post(f"/program/conversations/{cid}/run", json={"task": "again"})
    assert seen["resume"] == "s1" and seen["cancel"].is_set()


def test_user_edits_are_announced_once(setup):
    src, seen = setup
    c = conversations.create(src)
    c["edited"] = ["bot_tasks/programs/task_001.py"]
    conversations.save(src, c)
    client.post(f"/program/conversations/{c['id']}/run", json={"task": "go"})
    assert seen["task"] == "The user edited bot_tasks/programs/task_001.py since your last turn.\n\ngo"
    assert conversations.load(src, c["id"])["edited"] == []


def test_a_broken_stream_still_saves_the_partial_message(setup, monkeypatch):
    src, _ = setup

    def failing(robot, src_, task, resume=None, cancel=None):
        yield {"kind": "text", "text": "Starting"}
        raise RuntimeError("stream died")
    monkeypatch.setattr(routes.agent, "run_agent", failing)
    cid = conversations.create(src)["id"]
    with pytest.raises(RuntimeError):
        client.post(f"/program/conversations/{cid}/run", json={"task": "go"})
    msgs = conversations.load(src, cid)["messages"]
    assert msgs[-1]["role"] == "claude" and msgs[-1]["items"] == [{"kind": "text", "text": "Starting"}]


def test_deleting_the_conversation_mid_run_skips_the_save(setup, monkeypatch):
    src, _ = setup
    cid = conversations.create(src)["id"]

    def deleting(robot, src_, task, resume=None, cancel=None):
        yield {"kind": "text", "text": "hi"}
        conversations.delete(src, cid)
    monkeypatch.setattr(routes.agent, "run_agent", deleting)
    assert client.post(f"/program/conversations/{cid}/run", json={"task": "go"}).status_code == 200
    with pytest.raises(KeyError):
        conversations.load(src, cid)


def test_not_ready_is_409_with_log_and_docker_error_is_503(setup, monkeypatch):
    src, _ = setup
    cid = conversations.create(src)["id"]
    monkeypatch.setattr(routes.session, "status", lambda: {"state": "failed", "robot": "bot", "log": "boom"})
    r = client.post(f"/program/conversations/{cid}/run", json={"task": "x"})
    assert r.status_code == 409 and "boom" in r.json()["detail"]

    def boom():
        raise routes.session.SessionError("Start Docker Desktop.")
    monkeypatch.setattr(routes.session, "status", boom)
    r = client.post(f"/program/conversations/{cid}/run", json={"task": "x"})
    assert r.status_code == 503


def test_old_endpoints_are_gone():
    assert client.post("/program/run", json={"task": "x"}).status_code in (404, 405)
    assert client.post("/program/rerun", json={"program": "x"}).status_code in (404, 405)


def _start(cid):
    resp = routes.run_conversation(cid, routes.ConvRun(task="go"))
    return resp.body_iterator


def test_disconnect_cancels_and_saves_partial(setup, monkeypatch):
    import asyncio
    src, _ = setup
    seen = {}

    def slow(robot, src_, task, resume=None, cancel=None):
        seen["cancel"] = cancel
        yield {"kind": "text", "text": "Part"}
        yield {"kind": "text", "text": "More"}
    monkeypatch.setattr(routes.agent, "run_agent", slow)
    cid = conversations.create(src)["id"]

    async def go():
        it = _start(cid)
        await it.__anext__()
        await it.aclose()
    asyncio.run(go())
    assert seen["cancel"].is_set()
    last = conversations.load(src, cid)["messages"][-1]
    assert last["role"] == "claude" and last["items"][0] == {"kind": "text", "text": "Part"}


def test_one_run_at_a_time(setup):
    import asyncio
    src, _ = setup
    cid = conversations.create(src)["id"]
    (src / "bot_tasks" / "programs").mkdir(parents=True)
    (src / "bot_tasks" / "programs" / "task_001.py").write_text("x")

    async def go():
        it = _start(cid)
        await it.__anext__()
        r = client.post(f"/program/conversations/{cid}/run", json={"task": "again"})
        assert r.status_code == 409 and "already running" in r.json()["detail"]
        r = client.post("/program/files/run", json={"path": "bot_tasks/programs/task_001.py"})
        assert r.status_code == 409 and "already running" in r.json()["detail"]
        await it.aclose()
        it2 = _start(cid)
        await it2.__anext__()
        await it2.aclose()
    asyncio.run(go())


def test_done_without_session_id_keeps_the_old_one(setup, monkeypatch):
    src, _ = setup

    def agent_(robot, src_, task, resume=None, cancel=None):
        yield {"kind": "done", "summary": "s", "runs": 0, "ok": True, "program": None, "code": None}
    monkeypatch.setattr(routes.agent, "run_agent", agent_)
    c = conversations.create(src)
    c["claude_session_id"] = "old"
    conversations.save(src, c)
    client.post(f"/program/conversations/{c['id']}/run", json={"task": "go"})
    assert conversations.load(src, c["id"])["claude_session_id"] == "old"
