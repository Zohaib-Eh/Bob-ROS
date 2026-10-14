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


def test_generated_library_compiles_and_names_the_robot(ws):
    for name in ("robot.py", "runner.py"):
        src = (ws / "bot_program" / name).read_text()
        compile(src, name, "exec")
    lib = (ws / "bot_program" / "robot.py").read_text()
    assert 'ROBOT = "bot"' in lib
    for api in ("def add_box", "def add_cylinder", "def move_to", "def move_line", "def move_joints",
                "def home", "def attach", "def detach", "def solutions", "def tool_pose", "def reset",
                "class Unreachable", "class Collision", "class PlanningFailed", "class UnknownObject"):
        assert api in lib, api


def test_run_task_keeps_the_ros_pythonpath(ws):
    assert "_program:\$PYTHONPATH" in (ws / "bot_program" / "run_task.sh").read_text()


from app.ros.chain import parse_chain  # noqa: E402
from app.ros.program_pkg import bearing, brief  # noqa: E402
from tests.ros_helpers import four_axis_zip, unzip  # noqa: E402


def test_brief_states_joints_facing_and_the_no_probing_rule(ws, tmp_path):
    text = (ws / "bot_program" / "brief.md").read_text()
    assert "The arm has 2 moving joints." in text
    assert "It cannot hold arbitrary tool orientations: call r.move_to(xyz) without rpy" in text
    assert '"In front of the robot" means toward bearing' in text
    assert "Never search for joint angles by moving the robot" in text
    assert "r.move_to(xyz, rpy=None)" in text and "r.tool_down()" in text and "r.tool_facing(direction)" in text


def test_bearing_is_the_home_reach_direction(tmp_path):
    d = unzip(four_axis_zip(tmp_path), tmp_path / "x")
    c = parse_chain((d / "four" / "urdf" / "four.urdf").read_text())
    assert abs(bearing(c) - 0.0) < 1e-6          # the test arm reaches along +x at home
    assert "The arm has 4 moving joints." in brief(c, "four")


def test_library_has_position_only_moves_and_tool_down(ws):
    lib = (ws / "bot_program" / "robot.py").read_text()
    assert "def move_to(self, xyz, rpy=None)" in lib and "def move_line(self, xyz, rpy=None)" in lib
    assert "def tool_down(self)" in lib and "def _position_ik(self" in lib and "def tool_facing(self, direction)" in lib
