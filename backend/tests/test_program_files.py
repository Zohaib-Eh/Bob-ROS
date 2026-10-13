import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.program import conversations, routes

client = TestClient(app)


@pytest.fixture
def src(tmp_path, monkeypatch):
    s = tmp_path / "ws" / "src"
    (s / "bot_tasks" / "programs").mkdir(parents=True)
    (s / "bot_program").mkdir()
    (s / "bot" / "meshes").mkdir(parents=True)
    (s / "bot_tasks" / "programs" / "task_001.py").write_text("print(1)\n")
    (s / "bot_program" / "robot.py").write_text("# lib\n")
    (s / "bot" / "meshes" / "base.stl").write_bytes(b"\0\1")
    monkeypatch.setattr(routes.session, "current", lambda: {"robot": "bot", "src": s})
    return s


def test_tree_lists_text_files_and_marks_programs_editable(src):
    assert client.get("/program/files").json() == [
        {"path": "bot_program/robot.py", "editable": False},
        {"path": "bot_tasks/programs/task_001.py", "editable": True}]


def test_read_and_save_a_program(src):
    r = client.get("/program/files/content", params={"path": "bot_tasks/programs/task_001.py"}).json()
    assert r == {"path": "bot_tasks/programs/task_001.py", "content": "print(1)\n", "editable": True}
    cid = conversations.create(src)["id"]
    assert client.put("/program/files/content", json={
        "path": "bot_tasks/programs/task_001.py", "content": "print(2)\n", "conversation": cid}).status_code == 200
    assert (src / "bot_tasks/programs/task_001.py").read_text() == "print(2)\n"
    assert conversations.load(src, cid)["edited"] == ["bot_tasks/programs/task_001.py"]


@pytest.mark.parametrize("bad", ["../x.py", "/etc/passwd", "C:/x.py", "bot_tasks/../../x.py", "bot_tasks/programs/../../x"])
def test_paths_outside_the_workspace_are_refused(src, bad):
    assert client.get("/program/files/content", params={"path": bad}).status_code == 400
    assert client.put("/program/files/content", json={"path": bad, "content": "x"}).status_code == 400


def test_only_programs_are_editable(src):
    r = client.put("/program/files/content", json={"path": "bot_program/robot.py", "content": "x"})
    assert r.status_code == 400 and r.json()["detail"] == "Only bot_tasks/programs/*.py can be edited."
    assert (src / "bot_program/robot.py").read_text() == "# lib\n"


def test_run_a_program_and_record_it(src, monkeypatch):
    seen = {}
    monkeypatch.setattr(routes.session, "run_program", lambda p: seen.setdefault("p", p) and {"ok": True, "output": "RESULT: ok"})
    cid = conversations.create(src)["id"]
    r = client.post("/program/files/run", json={"path": "bot_tasks/programs/task_001.py", "conversation": cid}).json()
    assert r == {"ok": True, "output": "RESULT: ok"} and seen["p"] == "programs/task_001.py"
    assert conversations.load(src, cid)["messages"][-1] == {
        "role": "run", "path": "bot_tasks/programs/task_001.py", "ok": True, "output": "RESULT: ok"}
