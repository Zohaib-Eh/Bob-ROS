import os
import shutil
import subprocess
from pathlib import Path

import pytest

from app.ros.cli import workspace_from_zip
from tests.ros_helpers import two_joint_zip


def _bash():
    for p in (os.environ.get("CLAUDE_CODE_GIT_BASH_PATH"), r"C:\Program Files\Git\bin\bash.exe", shutil.which("bash")):
        if p and Path(p).exists() and "system32" not in p.lower():
            return p
    pytest.skip("Git Bash not found")


@pytest.fixture
def ws(tmp_path):
    out = tmp_path / "ws"
    assert workspace_from_zip(two_joint_zip(tmp_path), out, samples=50) == "bot"
    return out / "src"


def test_program_folder_and_tasks_folder_are_generated(ws):
    prog = ws / "bot_program"
    assert {p.name for p in prog.iterdir()} >= {"brief.md", "run_task.sh", "robot.py", "runner.py"}
    assert (ws / "bot_tasks" / "programs").is_dir()
    text = (prog / "brief.md").read_text()
    assert "arm_joint" in text and "hand_joint" in text and "Tool link: hand" in text
    assert "at most 3 runs" in text and "../bot_program/run_task.sh programs/task_NNN.py" in text


def test_regenerating_keeps_user_programs(ws, tmp_path):
    (ws / "bot_tasks" / "programs" / "task_001.py").write_text("print('mine')")
    (ws / "bot_program" / "stale.txt").write_text("old")
    workspace_from_zip(two_joint_zip(tmp_path, name="bot"), ws.parent, samples=50)
    assert (ws / "bot_tasks" / "programs" / "task_001.py").read_text() == "print('mine')"
    assert not (ws / "bot_program" / "stale.txt").exists()


def _run_task(ws, tmp_path, prog, docker_exit=0, running="true"):
    fake = tmp_path / "bin"
    fake.mkdir(exist_ok=True)
    (fake / "docker").write_text(
        "#!/usr/bin/env bash\n"
        f'if [ "$1" = inspect ]; then echo {running}; exit 0; fi\n'
        'echo "docker $*" >> "$(dirname "$0")/calls.log"\n'
        f"echo RESULT: ok; exit {docker_exit}\n", newline="\n")
    env = {**os.environ, "PATH": str(fake) + os.pathsep + os.environ.get("PATH", "")}  # Git Bash converts a Windows PATH
    tasks = ws / "bot_tasks"
    r = subprocess.run([_bash(), "../bot_program/run_task.sh", prog], cwd=tasks, env=env,
                       capture_output=True, text=True)
    calls = (fake / "calls.log").read_text() if (fake / "calls.log").exists() else ""
    return r, calls


def test_run_task_counts_runs_and_refuses_the_fourth(ws, tmp_path):
    (ws / "bot_tasks" / "programs" / "task_001.py").write_text("pass")
    (ws / "bot_tasks" / ".run_count").write_text("0")
    for n in (1, 2, 3):
        r, _ = _run_task(ws, tmp_path, "programs/task_001.py")
        assert f"run {n}/3" in r.stdout and "RESULT: ok" in r.stdout
    r, _ = _run_task(ws, tmp_path, "programs/task_001.py")
    assert "RESULT: failed: run limit reached" in r.stdout and r.returncode != 0


@pytest.mark.parametrize("bad", ["../../x.py", "programs/a.py; rm -rf ~", "programs/../x.py", "task.py"])
def test_run_task_refuses_paths_outside_programs(ws, tmp_path, bad):
    r, calls = _run_task(ws, tmp_path, bad)
    assert r.stdout.startswith("RESULT: failed:") and "docker exec" not in calls


def test_run_task_says_when_the_session_is_down(ws, tmp_path):
    (ws / "bot_tasks" / "programs" / "task_001.py").write_text("pass")
    r, calls = _run_task(ws, tmp_path, "programs/task_001.py", running="false")
    assert "RESULT: failed: the ROS session is not running" in r.stdout and "docker exec" not in calls
