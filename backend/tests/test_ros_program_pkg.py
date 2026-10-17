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
    assert {p.name for p in prog.iterdir()} >= {"brief.md", "run_task.sh", "robot.py", "runner.py",
                                                "scene.py", "sim_ctl.py", "sim_errors.py", "scene_sync.py"}
    assert (ws / "bot_tasks" / "programs").is_dir()
    assert (ws / "bot_tasks" / "workcell.yaml").read_text().startswith("# Objects every program runs with")
    text = (prog / "brief.md").read_text()
    assert "arm_joint" in text and "hand_joint" in text and "Tool link: hand" in text
    assert "at most 3 runs" in text and "../bot_program/run_task.sh programs/task_NNN_<name>.py" in text
    assert "task_007_stack_two_boxes.py" in text


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


def test_glue_package_is_generated(ws):
    glue = ws / "cad_to_urdf_glue"
    assert (glue / "src" / "glue_plugin.cpp").is_file() and (glue / "plugins.xml").is_file()
    assert (glue / "package.xml").is_file() and (glue / "CMakeLists.txt").is_file()
    cpp = (glue / "src" / "glue_plugin.cpp").read_text()
    assert "MuJoCoROS2ControlPluginBase" in cpp and '"/sim/glue"' in cpp and "eq_active" in cpp
    assert "cad_to_urdf_glue" in (ws / "bot_moveit_config" / "package.xml").read_text()


def test_regenerating_keeps_the_users_workcell(ws, tmp_path):
    (ws / "bot_tasks" / "workcell.yaml").write_text("- {name: t, shape: box, size: [1, 1, 1], xyz: [3, 3, 0.5], fixed: true}\n")
    workspace_from_zip(two_joint_zip(tmp_path, name="bot"), ws.parent, samples=50)
    assert (ws / "bot_tasks" / "workcell.yaml").read_text().startswith("- {name: t")


def test_session_picks_hardware_before_moveit_and_boots_the_sim_after():
    # the MuJoCo node reads /robot_description, which MoveIt's robot_state_publisher publishes
    start = (Path(__file__).resolve().parents[2] / "docker" / "jazzy" / "session" / "start.sh").read_text()
    hw, launch = start.index("python3 -m sim_ctl hardware"), start.index("demo.launch.py")
    assert hw < launch < start.index("python3 -m sim_ctl boot") < start.index("python3 -m scene_sync")


def test_sim_node_listens_on_the_robot_description_topic():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app" / "ros" / "templates"))
    import sim_ctl
    assert "~/robot_description:=/robot_description" in sim_ctl.control_node_cmd("bot")


def test_robot_library_speaks_physics(ws):
    lib = (ws / "bot_program" / "robot.py").read_text()
    assert "class MotionError(RobotError)" in lib and "def pose(self, name)" in lib
    assert "Objects can't appear mid-run. Declare '{name}' in SCENE at the top of the program (or in workcell.yaml)." in lib
    assert '"/sim/glue"' in lib and "sim_errors.lag_message(" in lib and "sim_errors.blow_up(" in lib
    assert "_meta()[\"objects\"]" in lib  # reset loads the run's objects into MoveIt
    compile(lib, "robot.py", "exec")
    runner = (ws / "bot_program" / "runner.py").read_text()
    assert "sim_ctl.prepare(path)" in runner and "except scene.SceneError" in runner
    compile(runner, "runner.py", "exec")


def test_brief_teaches_scene_and_pose(ws):
    text = (ws / "bot_program" / "brief.md").read_text()
    assert "SCENE = [" in text and "workcell.yaml" in text and "r.pose(" in text
    assert "r.add_box(" not in text and "Physics is real" in text
    assert "Workcell (always present; see workcell.yaml):" in text and "- (empty)" in text


def test_brief_lists_the_workcell_objects(ws, tmp_path):
    (ws / "bot_tasks" / "workcell.yaml").write_text(
        "- {name: table, shape: box, size: [1, 0.5, 0.05], xyz: [0.6, 0, 0.3], fixed: true}\n")
    workspace_from_zip(two_joint_zip(tmp_path, name="bot"), ws.parent, samples=50)
    text = (ws / "bot_program" / "brief.md").read_text(encoding="utf-8")
    assert "- table: box 1 × 0.5 × 0.05 m at (0.6, 0, 0.3), fixed" in text
