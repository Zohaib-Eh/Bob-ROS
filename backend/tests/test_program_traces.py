"""Run replay, backend side: the TRACE line a run prints, carried to the chat, and the trace file served."""
import json

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.program import agent, conversations, traces
from tests.test_program_agent import FIXTURE

client = TestClient(app)


def test_trace_of_reads_the_last_trace_line():
    assert traces.trace_of("x\nRESULT: ok\n") is None
    assert traces.trace_of("TRACE: runs/task_001-20261018T120000Z.json\nRESULT: ok") == "runs/task_001-20261018T120000Z.json"
    assert traces.trace_of("TRACE: none (recorder failed: boom)\nRESULT: ok") is None
    assert traces.trace_of("TRACE: ../../etc.json\n") is None                         # never a path out of runs/


def test_a_run_in_the_chat_carries_its_trace(tmp_path):
    (tmp_path / "programs").mkdir()
    (tmp_path / "programs" / "task_001.py").write_text('print("hello")\n', encoding="utf-8")
    (tmp_path / ".run_count").write_text("1")
    lines = FIXTURE.read_text(encoding="utf-8").replace("RESULT: ok", "TRACE: runs/task_001-1.json\\nRESULT: ok").splitlines()
    rr = next(e for e in agent.parse_stream(lines, tmp_path) if e["kind"] == "run_result")
    assert rr["trace"] == "runs/task_001-1.json"
    m = conversations.fold([{"kind": "run", "n": 1}, rr])
    assert m["items"][0]["trace"] == "runs/task_001-1.json"


@pytest.fixture
def ws(tmp_path, monkeypatch):
    monkeypatch.setattr(traces, "WORKSPACES", tmp_path)
    runs = tmp_path / "bot" / "src" / "bot_tasks" / "runs"
    runs.mkdir(parents=True)
    (runs / "task_001-1.json").write_text(json.dumps({"version": 1, "robot": "bot", "frames": []}))
    return tmp_path


def test_the_trace_is_served_by_robot_without_a_session(ws):
    r = client.get("/program/traces/task_001-1.json", params={"robot": "bot"})
    assert r.status_code == 200 and r.json()["robot"] == "bot"
    assert client.get("/program/traces/nope.json", params={"robot": "bot"}).status_code == 404
    for bad in ("..%2Fx.json", "x.txt"):
        assert client.get(f"/program/traces/{bad}", params={"robot": "bot"}).status_code in (400, 404)
    assert client.get("/program/traces/task_001-1.json", params={"robot": "../x"}).status_code == 400
