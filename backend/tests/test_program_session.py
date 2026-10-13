import subprocess

import pytest

from app.program import session
from tests.ros_helpers import _body, _joint, _link


class FakeDocker:
    def __init__(self, running="true", info_ok=True, image=True):
        self.calls, self.running, self.info_ok, self.image = [], running, info_ok, image

    def __call__(self, args, capture_output=True, text=True, check=True):
        self.calls.append(args)
        cmd = args[1]
        if cmd == "info" and not self.info_ok:
            raise subprocess.CalledProcessError(1, args, "", "failed to connect to the docker API at npipe:////./pipe/dockerDesktopLinuxEngine")
        rc, out = 0, ""
        if cmd == "image":
            rc = 0 if self.image else 1
        if cmd == "inspect":
            out = "running" if self.running == "true" else "exited"
        if cmd == "exec" and args[-1:] == ["/tmp/ready"]:
            rc = 0
        if cmd == "exec" and "runner" in " ".join(args):
            out = "hello\nRESULT: ok\n"
        cp = subprocess.CompletedProcess(args, rc, out, "")
        if check and rc:
            raise subprocess.CalledProcessError(rc, args, out, "")
        return cp


def _model():
    links = [_link("base"), _link("arm"), _link("hand")]
    joints = [_joint("j1", "base", "arm", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0)),
              _joint("j2", "arm", "hand", (0.5, 0.0, 0.2), (0.0, 1.0, 0.0))]
    bodies = {"base": [_body("base", (0.0, 0.0, 0.0))], "arm": [_body("arm", (0.25, 0.0, 0.2))],
              "hand": [_body("hand", (0.7, 0.0, 0.2))]}
    return links, joints, bodies


@pytest.fixture
def fake(monkeypatch, tmp_path):
    d = FakeDocker()
    monkeypatch.setattr(session.subprocess, "run", d)
    monkeypatch.setattr(session, "WORKSPACES", tmp_path / "workspaces")
    monkeypatch.setattr(session, "SAMPLES", 50)
    return d


def test_start_generates_the_workspace_and_runs_the_container(fake, tmp_path):
    out = session.start("bot", *_model())
    assert out == {"state": "starting", "robot": "bot"}
    src = tmp_path / "workspaces" / "bot" / "src"
    assert (src / "bot_program" / "robot.py").exists() and (src / "bot_tasks" / "programs").is_dir()
    run = next(c for c in fake.calls if c[1] == "run")
    assert ["--name", "cad-to-urdf-session"] == run[3:5] and "9090:9090" in run
    assert any(str(tmp_path / "workspaces" / "bot") + ":/ws" == a for a in run)
    assert run[-2:] == ["bash", "/session/start.sh"]
    assert ["docker", "rm", "-f", "cad-to-urdf-session"] in fake.calls


def test_docker_not_running_says_start_docker_desktop(fake):
    fake.info_ok = False
    with pytest.raises(session.SessionError, match="Start Docker Desktop."):
        session.start("bot", *_model())


def test_status_reports_ready_and_failed(fake):
    session.start("bot", *_model())
    assert session.status()["state"] == "ready"
    fake.running = "false"
    st = session.status()
    assert st["state"] == "failed" and "log" in st


def test_run_program_rejects_bad_paths_and_runs_good_ones(fake):
    session.start("bot", *_model())
    with pytest.raises(ValueError):
        session.run_program("../x.py")
    assert session.run_program("programs/task_001.py") == {"ok": True, "output": "hello\nRESULT: ok\n"}
