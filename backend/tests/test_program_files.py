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


@pytest.fixture
def runs(monkeypatch):
    calls = []

    def fake_run(program):
        calls.append(program)
        return {"ok": True, "output": "RESULT: ok"}

    monkeypatch.setattr(routes.session, "run_program", fake_run)
    return calls


def test_run_a_program_and_record_it(src, runs):
    cid = conversations.create(src)["id"]
    r = client.post("/program/files/run", json={"path": "bot_tasks/programs/task_001.py", "conversation": cid}).json()
    assert r == {"ok": True, "output": "RESULT: ok"} and runs == ["programs/task_001.py"]
    assert conversations.load(src, cid)["messages"][-1] == {
        "role": "run", "path": "bot_tasks/programs/task_001.py", "ok": True, "output": "RESULT: ok"}


@pytest.mark.parametrize("bad", ["../x.py", "/etc/passwd", "C:/x.py", "bot_tasks/../../x.py",
                                 "bot_tasks/programs/../../x", "bot_program/robot.py"])
def test_run_refuses_bad_paths(src, runs, bad):
    assert client.post("/program/files/run", json={"path": bad}).status_code == 400
    assert runs == []


def test_run_missing_program_is_404(src, runs):
    r = client.post("/program/files/run", json={"path": "bot_tasks/programs/missing.py"})
    assert r.status_code == 404 and r.json()["detail"] == "No such file."
    assert runs == []


def test_unknown_conversation_on_save_writes_nothing(src):
    r = client.put("/program/files/content", json={
        "path": "bot_tasks/programs/task_001.py", "content": "print(2)\n", "conversation": "000000000000"})
    assert r.status_code == 404
    assert (src / "bot_tasks/programs/task_001.py").read_text() == "print(1)\n"


def test_unknown_conversation_on_run_does_not_run(src, runs):
    r = client.post("/program/files/run", json={
        "path": "bot_tasks/programs/task_001.py", "conversation": "000000000000"})
    assert r.status_code == 404 and runs == []


def test_bad_conversation_id_on_save_writes_nothing(src):
    r = client.put("/program/files/content", json={
        "path": "bot_tasks/programs/task_001.py", "content": "print(2)\n", "conversation": "../x"})
    assert r.status_code == 400
    assert (src / "bot_tasks/programs/task_001.py").read_text() == "print(1)\n"


def test_tree_skips_hidden_folders(src):
    (src / ".bot_kinematics.old-1234abcd").mkdir()
    (src / ".bot_kinematics.old-1234abcd" / "setup.py").write_text("x")
    assert all(not f["path"].startswith(".") for f in client.get("/program/files").json())
