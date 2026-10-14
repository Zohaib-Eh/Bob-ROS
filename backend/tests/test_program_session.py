import subprocess
import threading

import pytest

from app.program import session
from tests.ros_helpers import _body, _joint, _link


class FakeDocker:
    def __init__(self, running="true", info_ok=True, image=True):
        self.calls, self.running, self.info_ok, self.image = [], running, info_ok, image
        self.run_args = None
        self.stage = "ready"

    def __call__(self, args, capture_output=True, text=True, check=True):
        self.calls.append(args)
        cmd = args[1]
        if cmd == "run":
            self.run_args = args
        if "Labels" in " ".join(args):
            if self.run_args is None:
                return subprocess.CompletedProcess(args, 1, "", "No such container")
            labels = dict(a.split("=", 1) for a in self.run_args if a.startswith("cad-to-urdf."))
            return subprocess.CompletedProcess(
                args, 0, f"{labels['cad-to-urdf.robot']}|{labels['cad-to-urdf.src']}\n", "")
        if cmd == "info" and not self.info_ok:
            raise subprocess.CalledProcessError(1, args, "", "failed to connect to the docker API at npipe:////./pipe/dockerDesktopLinuxEngine")
        rc, out = 0, ""
        if cmd == "image":
            rc = 0 if self.image else 1
        if cmd == "inspect":
            out = "running" if self.running == "true" else "exited"
        if cmd == "exec" and args[-2:] == ["cat", "/tmp/stage"]:
            out = self.stage
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
    monkeypatch.setattr(session, "_prep", None)
    monkeypatch.setattr(session, "_thread", None)
    monkeypatch.setattr(session, "_current", None)
    return d


def _started(*model):
    out = session.start("bot", *(model or _model()))
    session._thread.join(timeout=60)
    return out


def test_start_generates_the_workspace_and_runs_the_container(fake, tmp_path):
    out = _started()
    assert out["state"] == "starting" and out["robot"] == "bot"
    src = tmp_path / "workspaces" / "bot" / "src"
    assert (src / "bot_program" / "robot.py").exists() and (src / "bot_tasks" / "programs").is_dir()
    run = next(c for c in fake.calls if c[1] == "run")
    assert ["--name", "cad-to-urdf-session"] == run[3:5] and "127.0.0.1:9090:9090" in run
    assert any(str(tmp_path / "workspaces" / "bot") + ":/ws" == a for a in run)
    assert run[-2:] == ["bash", "/session/start.sh"]
    assert ["docker", "rm", "-f", "cad-to-urdf-session"] in fake.calls


def test_current_recovers_robot_and_src_from_container_labels(fake, tmp_path):
    _started()
    session._current = None  # as after a backend restart
    assert session.current() == {"robot": "bot", "src": tmp_path / "workspaces" / "bot" / "src"}


def test_docker_not_running_says_start_docker_desktop(fake):
    fake.info_ok = False
    with pytest.raises(session.SessionError, match="Start Docker Desktop."):
        session.start("bot", *_model())


def test_status_reports_ready_and_failed(fake):
    _started()
    assert session.status()["state"] == "ready"
    fake.running = "false"
    st = session.status()
    assert st["state"] == "failed" and "log" in st


def test_run_program_rejects_bad_paths_and_runs_good_ones(fake):
    _started()
    with pytest.raises(ValueError):
        session.run_program("../x.py")
    assert session.run_program("programs/task_001.py") == {"ok": True, "output": "hello\nRESULT: ok\n"}


def test_start_returns_at_once_and_status_walks_the_stages(fake, monkeypatch):
    gate = threading.Event()
    real = session.workspace_from_zip

    def slow(data, ws, samples, on_progress):
        on_progress(250, samples)
        gate.wait(10)
        return real(data, ws, samples=samples, on_progress=on_progress)
    monkeypatch.setattr(session, "workspace_from_zip", slow)
    out = session.start("bot", *_model())
    assert out["state"] == "starting" and out["stage"] == "collisions"
    st = session.status()
    assert st["state"] == "starting" and st["stage"] == "collisions" and st["done"] == 250
    gate.set()
    session._thread.join(60)
    fake.stage = "moveit"
    assert session.status()["stage"] == "moveit" and session.status()["state"] == "starting"
    fake.stage = "ready"
    assert session.status()["state"] == "ready"


def test_missing_image_shows_the_image_stage(fake, monkeypatch):
    fake.image = False
    seen = []
    real = session._docker

    def spy(*args, **kw):
        if args[0] == "build":
            seen.append(session.status()["stage"])
        return real(*args, **kw)
    monkeypatch.setattr(session, "_docker", spy)
    _started()
    assert seen == ["image"]


def test_a_failure_in_the_thread_is_failed_with_stage_and_message(fake, monkeypatch):
    def boom(*a, **k):
        raise ValueError("Robot name 'Bot' is not a valid ROS package name.")
    monkeypatch.setattr(session, "workspace_from_zip", boom)
    _started()
    st = session.status()
    assert st["state"] == "failed" and st["stage"] == "collisions" and "not a valid ROS package name" in st["error"]


def test_stop_during_prep_never_runs_the_container(fake, monkeypatch):
    gate = threading.Event()
    real = session.workspace_from_zip

    def slow(data, ws, samples, on_progress):
        gate.wait(10)
        return real(data, ws, samples=samples)
    monkeypatch.setattr(session, "workspace_from_zip", slow)
    session.start("bot", *_model())
    session.stop()
    gate.set()
    session._thread.join(60)
    assert not any(c[1] == "run" for c in fake.calls)
    assert session.status()["state"] == "stopped"


def test_second_start_during_prep_is_busy(fake, monkeypatch):
    gate = threading.Event()
    monkeypatch.setattr(session, "workspace_from_zip", lambda *a, **k: gate.wait(10) and "bot")
    session.start("bot", *_model())
    with pytest.raises(session.SessionBusy):
        session.start("bot", *_model())
    gate.set()
    session._thread.join(60)


def test_status_while_starting_carries_a_log_tail(fake):
    _started()
    fake.stage = "build"
    st = session.status()
    assert st["state"] == "starting" and st["stage"] == "build" and "log" in st


def test_a_container_from_before_stage_files_still_reports_ready(fake, monkeypatch):
    _started()
    real = fake.__call__

    def old_container(args, **kw):
        if args[1] == "exec" and args[-2:] == ["cat", "/tmp/stage"]:
            return subprocess.CompletedProcess(args, 1, "", "No such file")
        if args[1] == "exec" and args[-1:] == ["/tmp/ready"]:
            return subprocess.CompletedProcess(args, 0, "", "")
        return real(args, **kw)
    monkeypatch.setattr(session.subprocess, "run", old_container)
    assert session.status()["state"] == "ready"
