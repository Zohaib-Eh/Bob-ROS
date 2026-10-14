# Prompt to Program Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A "Program" step in the app where a typed task becomes a MoveIt Python program written by Claude Code (`claude -p`), run in the ROS 2 Jazzy container on mock hardware, with the robot and objects moving live in the app's viewer.

**Architecture:** The ROS generator gains a `<robot>_program` folder (task library `robot.py`, container-side `runner.py`, host-side `run_task.sh`, agent `brief.md`). A backend module `app/program/` starts a long-running session container (MoveIt + rosbridge), runs `claude -p` restricted to the tasks folder and `run_task.sh`, and streams its events to the browser over SSE. The frontend Program step shows the chat and program, and animates the viewer from rosbridge (`/tf` for links, `/get_planning_scene` for objects).

**Tech Stack:** Python 3.11 backend (FastAPI), ROS 2 Jazzy + MoveIt 2 + rosbridge_suite in Docker, Claude Code CLI 2.1.296 headless, React + three.js + vitest frontend.

**Spec:** `docs/superpowers/specs/2026-10-13-prompt-to-program-design.md`

## Global Constraints

- Backend tests: from `backend/`, `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest` (Git Bash). `PY` below means `~/miniforge3/envs/cad-to-urdf/python.exe`. Frontend: from `frontend/`, `npm test`.
- No new backend or frontend dependencies (rosbridge client uses the browser `WebSocket`; Claude is a subprocess).
- Container image `cad-to-urdf-jazzy`; session container name `cad-to-urdf-session`; rosbridge on host port `9090`.
- Run cap: `MAX_RUNS = 3` per task (one constant); `RUN_TIMEOUT_S = 120` per run; every run starts with `Robot().reset()` (no objects, robot at home).
- Generated folders: `<robot>_program/` is regenerated freely; `<robot>_tasks/` belongs to the user and regeneration never deletes it; programs live in `<robot>_tasks/programs/task_NNN.py`.
- Claude backend: `claude -p` with output `stream-json`, allowed only Read, Write/Edit under `programs/`, and Bash for `../<robot>_program/run_task.sh`; working directory `<robot>_tasks/`; follow-ups use `--resume <session_id>`.
- `robot.py` API exactly (metres, radians, base frame): `Robot()`, `add_box(name, size_xyz, xyz, rpy=(0,0,0))`, `add_cylinder(name, radius, height, xyz, rpy=(0,0,0))`, `move_to(xyz, rpy)`, `move_line(xyz, rpy)`, `move_joints(q)`, `home()`, `attach(name)`, `detach(name)`, `solutions(xyz, rpy)`, `tool_pose()`, `reset()`; errors `RobotError` ⊃ `Unreachable`, `Collision`, `PlanningFailed`, `UnknownObject`, each with one plain sentence.
- Chat error texts verbatim: "Run `claude` once in a terminal to sign in." / "Start Docker Desktop." / usage-limit text passed through as is.
- Commits end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **A program path that isn't a plain `programs/<name>.py`** (e.g. `../../x.py` or `programs/a.py; rm -rf ~`) handed to `run_task.sh`: refused with a `RESULT: failed` line, nothing executed. Test in Task 2.
2. **The session container not running when Claude calls `run_task.sh`** (Docker stopped, session crashed): one `RESULT: failed: the ROS session is not running` line, not a docker error dump. Test in Task 2.
3. **Regenerating the workspace after the user has programs**: `<robot>_tasks/programs/*.py` survive, `<robot>_program/` is replaced. Test in Task 2.
4. **`claude` missing from PATH or not signed in**: one `error` event with the spec's sentence, never a Python traceback in the chat. Test in Task 5.
5. **SSE events split across network chunks** in the browser: every event still parsed exactly once. Test in Task 8.

---

## File Structure

| File | Responsibility |
|---|---|
| `docker/jazzy/Dockerfile` (modify) | add `ros-jazzy-rosbridge-suite` |
| `docker/jazzy/session/start.sh` | session container entry: build, MoveIt demo, rosbridge, `/tmp/ready` |
| `backend/app/ros/program_pkg.py` | files of `<robot>_program/`: brief, run_task.sh, robot.py, runner.py |
| `backend/app/ros/templates/robot.py` | task library template (container side) |
| `backend/app/ros/templates/runner.py` | reset + run one program + print `RESULT:` |
| `backend/app/ros/templates/run_task.sh` | host side: validate, count runs, `docker exec` the runner |
| `backend/app/ros/moveit_config.py` (modify) | `generate()` writes `<robot>_program`, ensures `<robot>_tasks/programs/` |
| `backend/app/ros/cli.py` (modify) | expose `workspace_from_zip(zip_source, out_dir, samples)` |
| `backend/app/program/__init__.py` | empty |
| `backend/app/program/session.py` | export → workspace → session container start/status/stop/run program |
| `backend/app/program/agent.py` | `run_agent(...)` over `claude -p` stream-json |
| `backend/app/program/routes.py` | `/program/*` endpoints, SSE |
| `backend/app/main.py` (modify) | include the program router |
| `docker/jazzy/check/programs/*.py`, `docker/jazzy/check/run.sh` (modify) | container checks of `robot.py` |
| `frontend/src/api/client.ts` (modify) | export `send`, `json` |
| `frontend/src/api/program.ts` | session + run + rerun client, SSE parsing |
| `frontend/src/ros/rosbridge.ts` | minimal rosbridge client over WebSocket |
| `frontend/src/ros/liveRobot.ts` | TF tree, body poses, scene objects (pure) |
| `frontend/src/viewer/Viewer.ts` (modify) | `setBodyPoses`, `setSceneObjects` |
| `frontend/src/ui/steps/ProgramStep.tsx` | chat, program view, live viewer wiring |
| `frontend/src/model/checklist.ts`, `ui/Stepper.tsx`, `ui/steps/StepPanel.tsx` (modify) | the new step |

---

### Task 1: Probe the container and the Claude CLI (gate, controller inline)

Answers the facts later tasks depend on before code is written. No product code except the Dockerfile line.

**Files:**
- Modify: `docker/jazzy/Dockerfile`

**Interfaces:**
- Produces (recorded in the SDD ledger and, if different from this plan, patched into Tasks 3–6 before they start): service/action names and types; whether `/monitored_planning_scene` is published; whether `GetCartesianPath` returns a time-parameterized trajectory; whether attaching an existing world object with no geometry works; the exact working `claude -p` permission flags; a recorded stream-json fixture.

- [ ] **Step 1: Add rosbridge to the image and rebuild**

In `docker/jazzy/Dockerfile`, add `ros-jazzy-rosbridge-suite \` after `ros-jazzy-xacro \`. Run `docker build -t cad-to-urdf-jazzy docker/jazzy` (timeout 600000). Expected: success.

- [ ] **Step 2: Probe ROS interfaces**

Generate the IRB workspace and start MoveIt + rosbridge in a throwaway container, then list interfaces:

```bash
WS=$(mktemp -d); (cd backend && OPENBLAS_NUM_THREADS=1 $PY -m app.ros.cli "$(cygpath -w ../corpus/out/irb1300.zip)" -o "$(cygpath -w "$WS")")
cat > "$WS/probe.sh" <<'EOF'
source /opt/ros/jazzy/setup.bash; cd /ws && colcon build >/dev/null 2>&1; source install/setup.bash
ros2 launch irb1300_moveit_config demo.launch.py >/tmp/l.log 2>&1 &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml >/tmp/rb.log 2>&1 &
sleep 30
ros2 action list; ros2 service list | grep -E "planning_scene|cartesian|compute_ik"
ros2 topic list | grep -E "planning_scene|tf"
ros2 interface show moveit_msgs/srv/GetCartesianPath | grep -E "^[a-z_]+ |max_"
grep -iE "rosbridge.*started|9090" /tmp/rb.log | head -3
EOF
MSYS_NO_PATHCONV=1 docker run --rm -v "$(cygpath -w "$WS"):/ws" cad-to-urdf-jazzy bash /ws/probe.sh
```

Expected: actions `/move_action`, `/execute_trajectory`; services `/apply_planning_scene`, `/get_planning_scene`, `/compute_cartesian_path`; topics `/monitored_planning_scene`, `/tf`, `/tf_static`; rosbridge listening on 9090. Record the `GetCartesianPath` request fields (whether `max_velocity_scaling_factor` exists).

- [ ] **Step 3: Probe the Claude CLI**

In a scratch dir containing `programs/` and a dummy `../x_program/run_task.sh` that echoes `RESULT: ok`, run:

```bash
echo "Write programs/task_001.py printing hello, then run ../x_program/run_task.sh programs/task_001.py, then try running 'ls /' with Bash" | \
claude -p --output-format stream-json --verbose --tools Read,Write,Edit,Bash \
  --allowedTools "Read" "Write(./programs/**)" "Edit(./programs/**)" "Bash(../x_program/run_task.sh:*)" \
  --append-system-prompt-file brief.md > fixture.jsonl
```

Confirm: the file is written, `run_task.sh` runs, `ls /` is denied without a prompt hang. If `--append-system-prompt-file` or `--tools` is rejected, find the working equivalent with `claude --help` and record it. Copy `fixture.jsonl` to `backend/tests/fixtures/claude_stream.jsonl` (trim to under 60 lines, keep at least: the `system` init line with `session_id`, an assistant `text` block, a `Write` tool_use, a `Bash` tool_use for `run_task.sh` with its `tool_result` containing `RESULT: ok`, and the final `result` line).

- [ ] **Step 4: Commit**

```bash
git add docker/jazzy/Dockerfile backend/tests/fixtures/claude_stream.jsonl
git commit -m "build: rosbridge in the Jazzy image; recorded claude stream-json fixture

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

**Gate:** if `claude -p` cannot be restricted to these tools, stop and tell the user before Task 5.

---

### Task 2: Generate the `<robot>_program` folder (brief, run_task.sh)

**Files:**
- Create: `backend/app/ros/program_pkg.py`, `backend/app/ros/templates/run_task.sh`
- Create (placeholders filled in Task 3): `backend/app/ros/templates/robot.py`, `backend/app/ros/templates/runner.py` with the single line `"""Filled in by Task 3."""`
- Modify: `backend/app/ros/moveit_config.py` (`generate`), `backend/app/ros/cli.py`
- Test: `backend/tests/test_ros_program_pkg.py`

**Interfaces:**
- Consumes: `generate(robot, description_dir, out_src, source, samples)`, `write_files`, `hash_note` (moveit_config); `Chain`, `home`, `fk`.
- Produces: `program_files(chain, robot, source) -> dict[str, str]` (keys `brief.md`, `run_task.sh`, `robot.py`, `runner.py`); `brief(chain, robot) -> str`; constants `MAX_RUNS = 3`, `RUN_TIMEOUT_S = 120`, `SESSION_CONTAINER = "cad-to-urdf-session"`; `workspace_from_zip(zip_source: Path | bytes, out_dir: Path, samples: int = 10000) -> str` (returns robot name) in `app/ros/cli.py`. After `generate`, `out_src/<robot>_program/` exists with those four files and `out_src/<robot>_tasks/programs/` exists.

- [ ] **Step 1: Write the failing tests** — `backend/tests/test_ros_program_pkg.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_ros_program_pkg.py -q`
Expected: FAIL with `ImportError: cannot import name 'workspace_from_zip'`.

- [ ] **Step 3: Implement**

`backend/app/ros/templates/run_task.sh` (copied with `__ROBOT__`, `__MAX_RUNS__`, `__TIMEOUT__`, `__CONTAINER__` replaced; LF endings):

```bash
#!/usr/bin/env bash
# Generated by cad-to-urdf. Runs one task program in the ROS session container.
# Usage, from __ROBOT___tasks/:  ../__ROBOT___program/run_task.sh programs/task_001.py
set -u
MAX_RUNS=__MAX_RUNS__
HERE=$(cd "$(dirname "$0")" && pwd)
TASKS="$HERE/../__ROBOT___tasks"
PROG="${1:-}"
if ! [[ "$PROG" =~ ^programs/[A-Za-z0-9_]+\.py$ ]]; then
  echo "RESULT: failed: give a program path like programs/task_001.py"; exit 2
fi
if [ ! -f "$TASKS/$PROG" ]; then echo "RESULT: failed: $PROG does not exist"; exit 2; fi
if [ "$(docker inspect -f '{{.State.Running}}' __CONTAINER__ 2>/dev/null)" != "true" ]; then
  echo "RESULT: failed: the ROS session is not running (restart it in the Program step)"; exit 4
fi
N=$(( $(cat "$TASKS/.run_count" 2>/dev/null || echo 0) + 1 ))
if [ "$N" -gt "$MAX_RUNS" ]; then
  echo "RESULT: failed: run limit reached ($MAX_RUNS runs for this task); stop and explain what is still wrong"; exit 3
fi
echo "$N" > "$TASKS/.run_count"
echo "run $N/$MAX_RUNS"
MSYS_NO_PATHCONV=1 docker exec __CONTAINER__ timeout __TIMEOUT__ bash -c \
  "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && cd /ws/src/__ROBOT___tasks && PYTHONPATH=/ws/src/__ROBOT___program python3 -u -m runner $PROG"
CODE=$?
if [ "$CODE" -eq 124 ]; then echo "RESULT: failed: timed out after __TIMEOUT__ s"; fi
exit $CODE
```

`backend/app/ros/program_pkg.py`:

```python
"""Files of the generated <robot>_program folder: the task library LLM-written programs call
(robot.py), the container-side runner, run_task.sh for the agent, and the agent's brief."""
import math
from pathlib import Path

import numpy as np

from .chain import Chain, home
from .fk import fk
from .moveit_config import hash_note

HERE = Path(__file__).parent
MAX_RUNS = 3
RUN_TIMEOUT_S = 120
SESSION_CONTAINER = "cad-to-urdf-session"

API_DOC = """## Library

```python
from robot import Robot   # errors: Unreachable, Collision, PlanningFailed, UnknownObject (all RobotError)
r = Robot()                                    # connected; robot at home, scene empty
r.add_box(name, size_xyz, xyz, rpy=(0, 0, 0))  # box centred at xyz
r.add_cylinder(name, radius, height, xyz, rpy=(0, 0, 0))  # axis along its local z
r.move_to(xyz, rpy)       # tool link to a pose, planned around obstacles
r.move_line(xyz, rpy)     # tool link along a straight line
r.move_joints(q)          # joint values in joint order
r.home()
r.attach(name)            # the object rides on the tool link (pick)
r.detach(name)            # the object stays where it is (place)
r.solutions(xyz, rpy)     # every exact joint solution, nearest first; [] = unreachable
r.tool_pose()             # current (xyz, rpy) of the tool link
```

Example (pick and place):

```python
from robot import Robot
r = Robot()
r.add_box("box", (0.05, 0.05, 0.05), (0.5, 0.2, 0.025))
xyz, down = r.tool_pose()                 # keep the home orientation of the tool
r.move_to((0.5, 0.2, 0.15), down)
r.move_line((0.5, 0.2, 0.06), down)
r.attach("box")
r.move_line((0.5, 0.2, 0.15), down)
r.move_to((0.4, -0.3, 0.15), down)
r.move_line((0.4, -0.3, 0.06), down)
r.detach("box")
r.home()
```"""

RULES = """## Rules
- Write exactly one program per task: `programs/task_NNN.py` (next free number). For a follow-up, write a new file starting from the previous one.
- Test it with `../__ROBOT___program/run_task.sh programs/task_NNN.py`. Every run starts clean (robot at home, no objects) and ends with `RESULT: ok` or `RESULT: failed: <reason>`.
- Fix and re-run until `RESULT: ok`. You have at most __MAX_RUNS__ runs per task; after that, stop and explain what is still wrong.
- Positions are in metres in the base frame, z up; keep objects above the floor (z > 0) and inside the reach.
- Only edit files under `programs/`. Do not run any other command.
- Finish with a two-line summary: what the program does, and the last run result."""


def reach(chain: Chain, samples: int = 2000, seed: int = 0) -> float:
    rng = np.random.default_rng(seed)
    best = 0.0
    for _ in range(samples):
        q = [rng.uniform(j.lower, j.upper) if j.lower is not None else rng.uniform(-math.pi, math.pi)
             for j in chain.joints]
        best = max(best, float(np.linalg.norm(fk(chain, q)[:3, 3])))
    return best


def brief(chain: Chain, robot: str) -> str:
    T = fk(chain, home(chain))
    lines = [f"# Robot: {robot}", "",
             f"You write Python programs that move the {robot} robot arm in a ROS 2 simulation.",
             f"Base frame: {chain.base}. Tool link: {chain.tip}. Units: metres and radians.",
             f"Approximate reach from the base origin: {reach(chain):.2f} m.",
             f"At home the tool link is at xyz ({T[0, 3]:.3f}, {T[1, 3]:.3f}, {T[2, 3]:.3f}).",
             "", "Joints, in order:"]
    for j, h in zip(chain.joints, home(chain)):
        unit = "m" if j.type == "prismatic" else "rad"
        span = "continuous" if j.lower is None else f"{j.lower:.3f} .. {j.upper:.3f} {unit}"
        lines.append(f"- {j.name}: {j.type}, {span}, home {h:.3f}")
    rules = RULES.replace("__MAX_RUNS__", str(MAX_RUNS)).replace("__ROBOT__", robot)
    return "\n".join(lines + ["", API_DOC, "", rules]) + "\n"


def _template(name: str, robot: str, source: str) -> str:
    text = (HERE / "templates" / name).read_text(encoding="utf-8")
    return (text.replace("__ROBOT__", robot).replace("__SOURCE__", source)
            .replace("__MAX_RUNS__", str(MAX_RUNS)).replace("__TIMEOUT__", str(RUN_TIMEOUT_S))
            .replace("__CONTAINER__", SESSION_CONTAINER))


def program_files(chain: Chain, robot: str, source: str) -> dict[str, str]:
    return {
        "brief.md": brief(chain, robot),
        "run_task.sh": _template("run_task.sh", robot, source),
        "robot.py": hash_note(source, robot) + _template("robot.py", robot, source),
        "runner.py": hash_note(source, robot) + _template("runner.py", robot, source),
    }
```

In `backend/app/ros/moveit_config.py` `generate()`: after the line writing `<robot>_kinematics` into `tmp`, add

```python
        from .program_pkg import program_files  # local: program_pkg imports this module
        write_files(tmp / f"{robot}_program", program_files(chain, robot, source))
```

and after `_swap_in(...)` returns (still inside the `try`), add

```python
        (out_src / f"{robot}_tasks" / "programs").mkdir(parents=True, exist_ok=True)
```

(`out_src` is already a `Path` there; if the local variable differs, use the same one `_swap_in` received.) `_swap_in` only touches names generated into `tmp`, so `<robot>_tasks` is never replaced.

In `backend/app/ros/cli.py`, move the body of the `with zipfile.ZipFile(...)` block into:

```python
def workspace_from_zip(zip_source: Path | bytes, out_dir: Path, samples: int = 10000) -> str:
    """Generate <out_dir>/src from a cad-to-urdf export zip (path or bytes). Returns the robot name."""
    import io
    src = io.BytesIO(zip_source) if isinstance(zip_source, bytes) else zip_source
    with zipfile.ZipFile(src) as z, tempfile.TemporaryDirectory() as tmp:
        robot = _robot_name(z)
        z.extractall(tmp)
        name = "upload.zip" if isinstance(zip_source, bytes) else Path(zip_source).name
        generate(robot, Path(tmp) / robot, Path(out_dir) / "src", source=name, samples=samples)
    return robot
```

and make `main` call `robot = workspace_from_zip(a.zip, Path(a.out), a.samples)` inside its existing `try` (keep its error handling unchanged).

Write the two placeholder templates (`robot.py`, `runner.py`) containing `"""Filled in by Task 3."""`.

- [ ] **Step 4: Run tests, then the full suite**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_ros_program_pkg.py -q && OPENBLAS_NUM_THREADS=1 $PY -m pytest -q`
Expected: all PASS. If `test_every_moveit_config_file_has_the_note` or package-count assertions in older tests now see the extra folder, update only the expected package set (`{"bot", "bot_moveit_config", "bot_kinematics", "bot_program", "bot_tasks"}`) in `tests/test_ros_cli.py`.

- [ ] **Step 5: Commit**

```bash
git add backend/app/ros/program_pkg.py backend/app/ros/templates backend/app/ros/moveit_config.py backend/app/ros/cli.py backend/tests/test_ros_program_pkg.py backend/tests/test_ros_cli.py
git commit -m "feat: generated program folder with agent brief and guarded run_task.sh

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: Task library `robot.py` and `runner.py`, checked in the container

**Files:**
- Modify: `backend/app/ros/templates/robot.py`, `backend/app/ros/templates/runner.py`
- Create: `docker/jazzy/check/programs/pick_place.py`, `docker/jazzy/check/programs/unreachable.py`, `docker/jazzy/check/programs/blocked.py`
- Modify: `docker/jazzy/check/run.sh`
- Test: `backend/tests/test_ros_program_pkg.py` (append one test)

**Interfaces:**
- Consumes: generated `<robot>_kinematics` (`load()`, `._chain.home`), MoveIt interfaces confirmed in Task 1.
- Produces: the `robot.py` API in Global Constraints; `runner.py` usage `python3 -m runner programs/x.py` printing a final `RESULT: ok` or `RESULT: failed: <Class>: <sentence>`.

- [ ] **Step 1: Write the failing test** — append to `backend/tests/test_ros_program_pkg.py`:

```python
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
```

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_ros_program_pkg.py -q` — Expected: FAIL (placeholder has no `ROBOT =`).

- [ ] **Step 2: Implement `templates/runner.py`**

```python
"""Container side of run_task.sh: reset the simulation, run one program, print RESULT."""
import runpy
import sys
import traceback

from robot import Robot, RobotError


def main(path: str) -> int:
    try:
        Robot().reset()
    except Exception as exc:  # the session itself is broken; nothing the program can fix
        print(f"RESULT: failed: could not reset the simulation ({exc})", flush=True)
        return 1
    try:
        runpy.run_path(path, run_name="__main__")
    except RobotError as exc:
        print(f"RESULT: failed: {type(exc).__name__}: {exc}", flush=True)
        return 1
    except SystemExit as exc:
        if exc.code not in (None, 0):
            print(f"RESULT: failed: the program exited with {exc.code}", flush=True)
            return 1
    except Exception as exc:
        traceback.print_exc()
        print(f"RESULT: failed: {type(exc).__name__}: {exc}", flush=True)
        return 1
    print("RESULT: ok", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
```

- [ ] **Step 3: Implement `templates/robot.py`**

```python
"""Task library for __ROBOT__: the functions LLM-written programs call. MoveIt plans every motion.
Units: metres and radians, in the robot's base frame."""
import importlib
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint, PlanningScene,
                             PlanningSceneComponents, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ROBOT = "__ROBOT__"
GROUP = "arm"
_kin = importlib.import_module(f"{ROBOT}_kinematics")
_home = importlib.import_module(f"{ROBOT}_kinematics._chain").home
_ERRORS = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}
_UNREACHABLE = {"NO_IK_SOLUTION", "INVALID_GOAL_CONSTRAINTS", "GOAL_CONSTRAINTS_VIOLATED"}
_COLLISION = {"GOAL_IN_COLLISION", "START_STATE_IN_COLLISION", "INVALID_MOTION_PLAN",
              "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"}


class RobotError(Exception):
    """A plain-language problem the program can fix."""


class Unreachable(RobotError):
    pass


class Collision(RobotError):
    pass


class PlanningFailed(RobotError):
    pass


class UnknownObject(RobotError):
    pass


def _matrix(xyz, rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = xyz
    return T


def _rpy(R) -> tuple:
    p = math.asin(-max(-1.0, min(1.0, R[2, 0])))
    if abs(math.cos(p)) < 1e-9:
        return (math.atan2(-R[1, 2], R[1, 1]), p, 0.0)
    return (math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0]))


def _quat(R) -> tuple:
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def _pose(xyz, rpy) -> Pose:
    T = _matrix(xyz, rpy)
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = _quat(T[:3, :3])
    return p


def _fmt(xyz) -> str:
    return "(" + ", ".join(f"{v:.3f}" for v in xyz) + ")"


_NODE = None


def _node():
    global _NODE
    if _NODE is None:
        if not rclpy.ok():
            rclpy.init()
        _NODE = rclpy.create_node("cad_to_urdf_task")
    return _NODE


def _wait(future, timeout: float):
    end = time.time() + timeout
    while not future.done() and time.time() < end:
        rclpy.spin_once(_node(), timeout_sec=0.05)
    if not future.done():
        raise PlanningFailed("MoveIt did not answer in time")
    return future.result()


class Robot:
    def __init__(self):
        n = _node()
        self._kin = _kin.load()
        self.chain = self._kin.chain
        self.joint_names = [j.name for j in self.chain.joints]
        self._move = ActionClient(n, MoveGroup, "/move_action")
        self._exec = ActionClient(n, ExecuteTrajectory, "/execute_trajectory")
        self._cartesian = n.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._apply = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._get = n.create_client(GetPlanningScene, "/get_planning_scene")
        self._joints = {}
        n.create_subscription(JointState, "/joint_states",
                              lambda m: self._joints.update(zip(m.name, m.position)), 10)
        for c in (self._cartesian, self._apply, self._get):
            if not c.wait_for_service(timeout_sec=30):
                raise PlanningFailed(f"MoveIt service {c.srv_name} is not available")
        if not (self._move.wait_for_server(timeout_sec=30) and self._exec.wait_for_server(timeout_sec=30)):
            raise PlanningFailed("MoveIt is not running")

    # --- scene -------------------------------------------------------------------------------
    def _diff(self, world=(), attached=()):
        s = PlanningScene()
        s.is_diff = True
        s.robot_state.is_diff = True
        s.world.collision_objects = list(world)
        s.robot_state.attached_collision_objects = list(attached)
        if not _wait(self._apply.call_async(ApplyPlanningScene.Request(scene=s)), 10).success:
            raise PlanningFailed("MoveIt rejected the scene change")

    def _scene(self) -> tuple[list[str], list[str]]:
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                     | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        sc = _wait(self._get.call_async(req), 10).scene
        return ([o.id for o in sc.world.collision_objects],
                [a.object.id for a in sc.robot_state.attached_collision_objects])

    def _add(self, name, shape, dims, xyz, rpy):
        co = CollisionObject()
        co.id = name
        co.header.frame_id = self.chain.base
        co.pose.orientation.w = 1.0
        co.primitives = [SolidPrimitive(type=shape, dimensions=[float(d) for d in dims])]
        co.primitive_poses = [_pose(xyz, rpy)]
        co.operation = CollisionObject.ADD
        self._diff(world=[co])

    def add_box(self, name, size_xyz, xyz, rpy=(0, 0, 0)):
        self._add(name, SolidPrimitive.BOX, size_xyz, xyz, rpy)

    def add_cylinder(self, name, radius, height, xyz, rpy=(0, 0, 0)):
        self._add(name, SolidPrimitive.CYLINDER, (height, radius), xyz, rpy)

    def _attached(self, name, op) -> AttachedCollisionObject:
        a = AttachedCollisionObject()
        a.link_name = self.chain.tip
        a.object.id = name
        a.object.operation = op
        a.touch_links = [self.chain.tip]
        return a

    def attach(self, name):
        world, attached = self._scene()
        if name not in world:
            raise UnknownObject(f"'{name}' is not a free object in the scene "
                                f"(objects: {', '.join(world + attached) or 'none'})")
        self._diff(attached=[self._attached(name, CollisionObject.ADD)])

    def detach(self, name):
        _, attached = self._scene()
        if name not in attached:
            raise UnknownObject(f"'{name}' is not attached to the tool (attached: {', '.join(attached) or 'none'})")
        self._diff(attached=[self._attached(name, CollisionObject.REMOVE)])

    def reset(self):
        _, attached = self._scene()
        if attached:
            self._diff(attached=[self._attached(n, CollisionObject.REMOVE) for n in attached])
        world, _ = self._scene()
        removals = []
        for n in world:
            co = CollisionObject()
            co.id, co.operation = n, CollisionObject.REMOVE
            removals.append(co)
        if removals:
            self._diff(world=removals)
        self.home()

    # --- state -------------------------------------------------------------------------------
    def _current(self) -> np.ndarray:
        end = time.time() + 5
        while any(n not in self._joints for n in self.joint_names) and time.time() < end:
            rclpy.spin_once(_node(), timeout_sec=0.05)
        missing = [n for n in self.joint_names if n not in self._joints]
        if missing:
            raise PlanningFailed(f"no joint states received for {', '.join(missing)}")
        return np.array([self._joints[n] for n in self.joint_names])

    def tool_pose(self) -> tuple:
        T = self._kin.fk(self._current())
        return tuple(float(v) for v in T[:3, 3]), _rpy(T[:3, :3])

    def solutions(self, xyz, rpy) -> list:
        try:
            sols = self._kin.solve_all(_matrix(xyz, rpy), self._current())
        except NotImplementedError:
            raise PlanningFailed("exact IK is not available for this robot; use move_to and let MoveIt solve it")
        return [[float(v) for v in q] for q in sols]

    # --- motion ------------------------------------------------------------------------------
    def _check_reach(self, xyz, rpy):
        try:
            if not self._kin.solve_all(_matrix(xyz, rpy), self._current()):
                raise Unreachable(f"no joint solution puts the tool at {_fmt(xyz)} with orientation {_fmt(rpy)} "
                                  "(outside the reach or the joint limits)")
        except NotImplementedError:
            pass  # no closed form for this robot: MoveIt's own IK decides

    def _run(self, client, goal, timeout):
        handle = _wait(client.send_goal_async(goal), 30)
        if not handle.accepted:
            raise PlanningFailed("MoveIt refused the motion request")
        return _wait(handle.get_result_async(), timeout).result

    def _error(self, code: int, what: str) -> RobotError:
        name = _ERRORS.get(code, str(code))
        if name in _UNREACHABLE:
            return Unreachable(f"{what} cannot be reached ({name})")
        if name in _COLLISION:
            world, attached = self._scene()
            return Collision(f"{what} collides with the robot or the scene ({name}); "
                             f"objects: {', '.join(world + attached) or 'none'}")
        return PlanningFailed(f"{what}: MoveIt could not plan it ({name})")

    def _plan_execute(self, constraints: Constraints, what: str):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        req.start_state.is_diff = True
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        res = self._run(self._move, goal, 120)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise self._error(res.error_code.val, what)

    def move_to(self, xyz, rpy):
        self._check_reach(xyz, rpy)
        target = _pose(xyz, rpy)
        pc = PositionConstraint()
        pc.header.frame_id, pc.link_name, pc.weight = self.chain.base, self.chain.tip, 1.0
        pc.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-3])]
        centre = Pose()
        centre.position = target.position
        centre.orientation.w = 1.0
        pc.constraint_region.primitive_poses = [centre]
        oc = OrientationConstraint()
        oc.header.frame_id, oc.link_name, oc.weight = self.chain.base, self.chain.tip, 1.0
        oc.orientation = target.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 1e-3
        self._plan_execute(Constraints(position_constraints=[pc], orientation_constraints=[oc]),
                           f"pose {_fmt(xyz)}")

    def move_joints(self, q):
        q = [float(v) for v in q]
        if len(q) != len(self.joint_names):
            raise PlanningFailed(f"move_joints needs {len(self.joint_names)} values ({', '.join(self.joint_names)})")
        for v, j in zip(q, self.chain.joints):
            if j.lower is not None and not j.lower - 1e-6 <= v <= j.upper + 1e-6:
                raise Unreachable(f"{j.name} = {v:.3f} is outside its limits {j.lower:.3f} .. {j.upper:.3f}")
        cs = []
        for name, v in zip(self.joint_names, q):
            jc = JointConstraint()
            jc.joint_name, jc.position, jc.weight = name, v, 1.0
            jc.tolerance_above = jc.tolerance_below = 1e-3
            cs.append(jc)
        self._plan_execute(Constraints(joint_constraints=cs), f"joint target {_fmt(q)}")

    def home(self):
        self.move_joints(_home(self.chain))

    def move_line(self, xyz, rpy):
        self._check_reach(xyz, rpy)
        req = GetCartesianPath.Request()
        req.header.frame_id = self.chain.base
        req.group_name = GROUP
        req.link_name = self.chain.tip
        req.waypoints = [_pose(xyz, rpy)]
        req.max_step = 0.005
        req.avoid_collisions = True
        req.start_state.is_diff = True
        for field in ("max_velocity_scaling_factor", "max_acceleration_scaling_factor"):
            if hasattr(req, field):
                setattr(req, field, 0.5)
        res = _wait(self._cartesian.call_async(req), 30)
        if res.fraction < 0.999:
            if res.error_code.val != MoveItErrorCodes.SUCCESS and res.fraction == 0.0:
                raise self._error(res.error_code.val, f"straight line to {_fmt(xyz)}")
            raise PlanningFailed(f"a straight line to {_fmt(xyz)} is only possible for {res.fraction * 100:.0f}% "
                                 "of the way (an obstacle, a joint limit or the edge of the reach is in the way); "
                                 "try move_to or a different path")
        out = self._run(self._exec, ExecuteTrajectory.Goal(trajectory=res.solution), 120)
        if out.error_code.val != MoveItErrorCodes.SUCCESS:
            raise self._error(out.error_code.val, f"straight line to {_fmt(xyz)}")
```

If Task 1 found that `GetCartesianPath` returns an untimed trajectory, add before `self._run(self._exec, ...)` a time-parameterization step using the approach Task 1 recorded, and say so in the report.

- [ ] **Step 4: Container check programs**

`docker/jazzy/check/programs/pick_place.py`:

```python
"""robot.py round trip: add, attach, carry, detach, solutions, home."""
from robot import Robot

r = Robot()
(x, y, z), rpy = r.tool_pose()
r.add_box("box", (0.04, 0.04, 0.04), (x, y, z - 0.10))
r.move_to((x, y, z - 0.05), rpy)
r.attach("box")
r.move_line((x, y + 0.05, z - 0.05), rpy)
r.detach("box")
(x2, y2, z2), _ = r.tool_pose()
assert abs(y2 - (y + 0.05)) < 2e-3, (y, y2)
assert r.solutions((x, y, z - 0.05), rpy), "expected at least one exact solution"
r.home()
print("pick_place ok")
```

`docker/jazzy/check/programs/unreachable.py`:

```python
"""A pose far outside the reach raises Unreachable."""
from robot import Robot, Unreachable

r = Robot()
_, rpy = r.tool_pose()
try:
    r.move_to((10.0, 0.0, 0.5), rpy)
except Unreachable as exc:
    print(f"unreachable ok: {exc}")
else:
    raise AssertionError("expected Unreachable")
```

`docker/jazzy/check/programs/blocked.py`:

```python
"""A target inside an obstacle raises Collision (or PlanningFailed)."""
from robot import Collision, PlanningFailed, Robot

r = Robot()
(x, y, z), rpy = r.tool_pose()
r.add_box("wall", (0.3, 0.3, 0.3), (x, y, z - 0.05))
try:
    r.move_to((x, y, z - 0.05), rpy)
except (Collision, PlanningFailed) as exc:
    print(f"blocked ok: {type(exc).__name__}: {exc}")
else:
    raise AssertionError("expected Collision or PlanningFailed")
```

In `docker/jazzy/check/run.sh`, after `python3 /check/check_ik.py "$ROBOT" || status=1`, add:

```bash
for p in /check/programs/*.py; do
  out=$(cd /ws/src/${ROBOT}_tasks && PYTHONPATH=/ws/src/${ROBOT}_program timeout 180 python3 -u -m runner "$p" 2>&1) || true
  if echo "$out" | grep -q "^RESULT: ok"; then echo "PASS robot.py $(basename "$p")"
  else echo "FAIL robot.py $(basename "$p")"; echo "$out" | tail -n 15; status=1; fi
done
```

- [ ] **Step 5: Run the unit test, then the container check on both robots**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_ros_program_pkg.py -q` — Expected: PASS.
Run: `for z in irb1300 six; do bash scripts/ros_check.sh corpus/out/$z.zip | grep -E "^(PASS|FAIL|SKIP)"; done` (timeout 600000) — Expected: every line PASS, including `PASS robot.py pick_place.py`, `unreachable.py`, `blocked.py`. On FAIL, fix `robot.py` (not the check programs, unless a check program's own geometry is wrong for the robot — then say why) and re-run.

- [ ] **Step 6: Commit**

```bash
git add backend/app/ros/templates backend/tests/test_ros_program_pkg.py docker/jazzy/check
git commit -m "feat: robot task library and runner, checked in the container

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: Session container

**Files:**
- Create: `docker/jazzy/session/start.sh`, `backend/app/program/__init__.py` (empty), `backend/app/program/session.py`
- Test: `backend/tests/test_program_session.py`

**Interfaces:**
- Consumes: `export_package` (urdf_writer), `workspace_from_zip` (cli), constants from `program_pkg`.
- Produces: `class SessionError(Exception)`; `start(robot_name, links, joints, bodies_by_link) -> dict` (`{"state": "starting", "robot": str}`); `status() -> dict` (`state` in `stopped|starting|ready|failed`, plus `robot`, and `log` when failed); `stop() -> None`; `current() -> dict | None` (`{"robot": str, "src": Path}`); `run_program(program: str) -> dict` (`{"ok": bool, "output": str}`); module constants `IMAGE`, `WORKSPACES`.

- [ ] **Step 1: Write the failing tests** — `backend/tests/test_program_session.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_program_session.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.program'`.

- [ ] **Step 3: Implement**

`docker/jazzy/session/start.sh`:

```bash
#!/usr/bin/env bash
# Session container entry: build the workspace, run MoveIt on mock hardware and rosbridge.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /ws && colcon build 2>&1 | tail -n 5
source /ws/install/setup.bash
ROBOT=$(basename "$(ls -d src/*_moveit_config)" _moveit_config)
ros2 launch "${ROBOT}_moveit_config" demo.launch.py &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
until ros2 action list 2>/dev/null | grep -q '^/move_action$'; do sleep 1; done
touch /tmp/ready
echo "session ready: ${ROBOT}"
wait
```

`backend/app/program/session.py`:

```python
"""The ROS session behind the Program step: one container running MoveIt (mock hardware) and
rosbridge on the robot's generated workspace."""
import re
import subprocess
from pathlib import Path

from app.export.urdf_writer import export_package
from app.ros.cli import workspace_from_zip
from app.ros.program_pkg import RUN_TIMEOUT_S, SESSION_CONTAINER

IMAGE = "cad-to-urdf-jazzy"
REPO = Path(__file__).resolve().parents[3]
DOCKER_DIR = REPO / "docker" / "jazzy"
WORKSPACES = Path.home() / ".cad-to-urdf" / "workspaces"
SAMPLES = 10000
PROGRAM_RE = re.compile(r"^programs/[A-Za-z0-9_]+\.py$")

_current: dict | None = None


class SessionError(Exception):
    pass


def _docker(*args: str, check: bool = True) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(["docker", *args], capture_output=True, text=True, check=check)
    except FileNotFoundError:
        raise SessionError("Docker is not installed or not on PATH.")
    except subprocess.CalledProcessError as exc:
        err = (exc.stderr or exc.stdout or "").strip()
        if "docker API" in err or "daemon" in err.lower():
            raise SessionError("Start Docker Desktop.")
        raise SessionError(err.splitlines()[-1] if err else f"docker {args[0]} failed")


def current() -> dict | None:
    return _current


def start(robot_name: str, links, joints, bodies_by_link) -> dict:
    """Export the model, generate its workspace and (re)start the session container."""
    global _current
    data = export_package(robot_name, links, joints, bodies_by_link)  # ValueError -> caller
    ws = WORKSPACES / robot_name
    robot = workspace_from_zip(data, ws, samples=SAMPLES)
    _docker("info")
    if _docker("image", "inspect", IMAGE, check=False).returncode != 0:
        _docker("build", "-t", IMAGE, str(DOCKER_DIR))
    _docker("rm", "-f", SESSION_CONTAINER, check=False)
    _docker("run", "-d", "--name", SESSION_CONTAINER, "-p", "9090:9090",
            "-v", f"{ws}:/ws", "-v", f"{DOCKER_DIR / 'session'}:/session:ro",
            IMAGE, "bash", "/session/start.sh")
    _current = {"robot": robot, "src": ws / "src"}
    return {"state": "starting", "robot": robot}


def logs(lines: int = 30) -> str:
    r = _docker("logs", "--tail", str(lines), SESSION_CONTAINER, check=False)
    return (r.stdout or "") + (r.stderr or "")


def status() -> dict:
    robot = _current["robot"] if _current else None
    r = _docker("inspect", "-f", "{{.State.Status}}", SESSION_CONTAINER, check=False)
    if r.returncode != 0 or not _current:
        return {"state": "stopped", "robot": robot}
    if r.stdout.strip() != "running":
        return {"state": "failed", "robot": robot, "log": logs()}
    ready = _docker("exec", SESSION_CONTAINER, "test", "-f", "/tmp/ready", check=False).returncode == 0
    return {"state": "ready" if ready else "starting", "robot": robot}


def stop() -> None:
    global _current
    _docker("rm", "-f", SESSION_CONTAINER, check=False)
    _current = None


def run_program(program: str) -> dict:
    """Run a program again without the agent (no run count)."""
    if not PROGRAM_RE.match(program):
        raise ValueError("Program path must look like programs/task_001.py.")
    if not _current:
        raise SessionError("Start a session first.")
    robot = _current["robot"]
    cmd = (f"source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && "
           f"cd /ws/src/{robot}_tasks && PYTHONPATH=/ws/src/{robot}_program python3 -u -m runner {program}")
    r = _docker("exec", SESSION_CONTAINER, "timeout", str(RUN_TIMEOUT_S), "bash", "-c", cmd, check=False)
    out = (r.stdout or "") + (r.stderr or "")
    return {"ok": "RESULT: ok" in out, "output": out}
```

- [ ] **Step 4: Run tests, then the full suite**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_program_session.py -q && OPENBLAS_NUM_THREADS=1 $PY -m pytest -q` — Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add docker/jazzy/session/start.sh backend/app/program backend/tests/test_program_session.py
git commit -m "feat: ROS session container for the Program step

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: Agent adapter over `claude -p`

**Files:**
- Create: `backend/app/program/agent.py`
- Test: `backend/tests/test_program_agent.py` (uses `backend/tests/fixtures/claude_stream.jsonl` from Task 1)

**Interfaces:**
- Consumes: the flags confirmed in Task 1; `<robot>_program/brief.md`; `MAX_RUNS`.
- Produces: `claude_command(exe: str, robot: str, brief_file: Path, resume: str | None) -> list[str]`; `parse_stream(lines: Iterable[str], tasks_dir: Path) -> Iterator[dict]`; `run_agent(robot: str, src: Path, task: str, resume: str | None = None, exe: str | None = None) -> Iterator[dict]`. Event dicts: `{"kind": "text", "text"}`, `{"kind": "file", "path"}`, `{"kind": "run", "n"}`, `{"kind": "run_result", "ok", "message"}`, `{"kind": "done", "summary", "runs", "ok", "session_id", "program", "code"}`, `{"kind": "error", "message"}`.

- [ ] **Step 1: Write the failing tests** — `backend/tests/test_program_agent.py`:

```python
import json
import sys
from pathlib import Path

from app.program import agent

FIXTURE = Path(__file__).parent / "fixtures" / "claude_stream.jsonl"


def test_command_restricts_claude_to_programs_and_run_task(tmp_path):
    cmd = agent.claude_command("claude", "bot", tmp_path / "brief.md", None)
    assert cmd[:2] == ["claude", "-p"] and "stream-json" in cmd
    assert "Bash(../bot_program/run_task.sh:*)" in cmd and "Write(./programs/**)" in cmd
    assert "--resume" not in cmd
    assert agent.claude_command("claude", "bot", tmp_path / "b.md", "abc")[-2:] == ["--resume", "abc"]


def test_parse_stream_turns_the_recorded_session_into_events(tmp_path):
    (tmp_path / "programs").mkdir()
    (tmp_path / ".run_count").write_text("1")
    events = list(agent.parse_stream(FIXTURE.read_text(encoding="utf-8").splitlines(), tmp_path))
    kinds = [e["kind"] for e in events]
    assert "text" in kinds and "file" in kinds and "run" in kinds and "run_result" in kinds
    done = events[-1]
    assert done["kind"] == "done" and done["session_id"] and done["runs"] == 1 and done["ok"] is True
    assert done["summary"]


def test_missing_claude_is_one_error_event(tmp_path):
    events = list(agent.run_agent("bot", tmp_path, "do it", exe=str(tmp_path / "nope.exe")))
    assert events == [{"kind": "error", "message": "Claude Code was not found. Install it, then run `claude` once in a terminal to sign in."}]


def test_signed_out_claude_maps_to_sign_in_message(tmp_path):
    (tmp_path / "bot_tasks").mkdir()
    fake = tmp_path / "fake_claude.py"
    fake.write_text("import sys; sys.stdin.read(); print('Invalid API key · Please run /login', file=sys.stderr); sys.exit(1)")
    events = list(agent.run_agent("bot", tmp_path, "do it", exe=sys.executable, extra=[str(fake)]))
    assert events[-1] == {"kind": "error", "message": "Run `claude` once in a terminal to sign in."}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_program_agent.py -q` — Expected: FAIL (`No module named 'app.program.agent'`).

- [ ] **Step 3: Implement** — `backend/app/program/agent.py`:

```python
"""The LLM side of the Program step: run_agent() is the only entry point. Its one backend today
is Claude Code in headless mode (`claude -p`) on the user's own subscription; an API backend
goes here when other people use the app."""
import json
import re
import shutil
import subprocess
from pathlib import Path
from typing import Iterable, Iterator

SIGN_IN = "Run `claude` once in a terminal to sign in."
NOT_FOUND = "Claude Code was not found. Install it, then run `claude` once in a terminal to sign in."
_SIGNED_OUT = re.compile(r"(invalid api key|/login|not logged in|authentication)", re.I)


def claude_command(exe: str, robot: str, brief_file: Path, resume: str | None) -> list[str]:
    cmd = [exe, "-p", "--output-format", "stream-json", "--verbose",
           "--append-system-prompt-file", str(brief_file),
           "--tools", "Read,Write,Edit,Bash",
           "--allowedTools", "Read", "Write(./programs/**)", "Edit(./programs/**)",
           f"Bash(../{robot}_program/run_task.sh:*)"]
    if resume:
        cmd += ["--resume", resume]
    return cmd


def _blocks(msg: dict) -> list[dict]:
    content = (msg.get("message") or {}).get("content") or []
    return content if isinstance(content, list) else [{"type": "text", "text": str(content)}]


def _result_text(block: dict) -> str:
    c = block.get("content")
    if isinstance(c, list):
        return "\n".join(b.get("text", "") for b in c if isinstance(b, dict))
    return str(c or "")


def parse_stream(lines: Iterable[str], tasks_dir: Path) -> Iterator[dict]:
    session_id, last_ok, last_program, runs_seen = None, False, None, set()
    for line in lines:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        kind = msg.get("type")
        session_id = msg.get("session_id") or session_id
        if kind == "assistant":
            for b in _blocks(msg):
                if b.get("type") == "text" and b.get("text", "").strip():
                    yield {"kind": "text", "text": b["text"]}
                elif b.get("type") == "tool_use":
                    inp = b.get("input") or {}
                    if b.get("name") in ("Write", "Edit") and inp.get("file_path"):
                        path = Path(inp["file_path"])
                        rel = path.relative_to(tasks_dir) if path.is_absolute() and tasks_dir in path.parents else path
                        last_program = Path(str(rel).replace("\\", "/")).as_posix()
                        yield {"kind": "file", "path": last_program}
                    elif b.get("name") == "Bash" and "run_task.sh" in inp.get("command", ""):
                        runs_seen.add(b.get("id"))
                        yield {"kind": "run", "n": len(runs_seen)}
        elif kind == "user":
            for b in _blocks(msg):
                if b.get("type") == "tool_result" and b.get("tool_use_id") in runs_seen:
                    text = _result_text(b)
                    m = re.findall(r"^RESULT: (ok|failed: .*)$", text, re.M)
                    if m:
                        last_ok = m[-1] == "ok"
                        yield {"kind": "run_result", "ok": last_ok, "message": m[-1]}
        elif kind == "result":
            if msg.get("is_error"):
                text = str(msg.get("result") or msg.get("subtype") or "Claude stopped with an error.")
                yield {"kind": "error", "message": SIGN_IN if _SIGNED_OUT.search(text) else text}
                return
            count = tasks_dir / ".run_count"
            runs = int(count.read_text().strip() or 0) if count.exists() else len(runs_seen)
            code = None
            if last_program and (tasks_dir / last_program).exists():
                code = (tasks_dir / last_program).read_text(encoding="utf-8")
            yield {"kind": "done", "summary": str(msg.get("result") or "").strip(), "runs": runs,
                   "ok": last_ok, "session_id": session_id, "program": last_program, "code": code}
            return


def run_agent(robot: str, src: Path, task: str, resume: str | None = None,
              exe: str | None = None, extra: list[str] | None = None) -> Iterator[dict]:
    """Stream events for one task. `extra` is for tests (arguments placed before the claude flags)."""
    exe = exe or shutil.which("claude")
    if not exe or not Path(exe).exists():
        yield {"kind": "error", "message": NOT_FOUND}
        return
    tasks_dir = Path(src) / f"{robot}_tasks"
    tasks_dir.mkdir(parents=True, exist_ok=True)
    (tasks_dir / ".run_count").write_text("0")
    brief = Path(src) / f"{robot}_program" / "brief.md"
    cmd = claude_command(exe, robot, brief, resume)
    if extra:
        cmd = [exe, *extra]
    proc = subprocess.Popen(cmd, cwd=tasks_dir, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, encoding="utf-8")
    proc.stdin.write(task)
    proc.stdin.close()
    finished = False
    for ev in parse_stream(proc.stdout, tasks_dir):
        finished = finished or ev["kind"] in ("done", "error")
        yield ev
    err = proc.stderr.read()
    code = proc.wait()
    if not finished:
        if _SIGNED_OUT.search(err):
            yield {"kind": "error", "message": SIGN_IN}
        else:
            last = err.strip().splitlines()[-1] if err.strip() else f"Claude exited with code {code}."
            yield {"kind": "error", "message": last}
```

If the Task 1 fixture uses different field names (e.g. tool results under another key), adjust `parse_stream` to the fixture — the fixture is the source of truth — and note it in the report. If Task 1 found a different flag for the brief file, use it in `claude_command`.

- [ ] **Step 4: Run tests, then the full suite**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_program_agent.py -q && OPENBLAS_NUM_THREADS=1 $PY -m pytest -q` — Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add backend/app/program/agent.py backend/tests/test_program_agent.py
git commit -m "feat: agent adapter over claude -p with stream-json events

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: Program routes

**Files:**
- Create: `backend/app/program/routes.py`
- Modify: `backend/app/main.py`
- Test: `backend/tests/test_program_routes.py`

**Interfaces:**
- Consumes: `session.start/status/stop/current/run_program/SessionError`, `agent.run_agent`, `ExportRequest` (export_routes).
- Produces: `POST /program/session` (ExportRequest body → status dict; 422 on model errors; 503 on SessionError), `GET /program/session`, `DELETE /program/session`, `POST /program/run` (`{"task": str, "resume": str | null}` → `text/event-stream` of `data: <event json>\n\n`; 409 when no ready session), `POST /program/rerun` (`{"program": str}` → `{"ok", "output"}`; 400 on bad path).

- [ ] **Step 1: Write the failing tests** — `backend/tests/test_program_routes.py`:

```python
import json

from fastapi.testclient import TestClient

from app.main import app
from app.program import routes

client = TestClient(app)


def test_run_streams_agent_events(monkeypatch, tmp_path):
    monkeypatch.setattr(routes.session, "current", lambda: {"robot": "bot", "src": tmp_path})
    monkeypatch.setattr(routes.session, "status", lambda: {"state": "ready", "robot": "bot"})
    seen = {}

    def fake_agent(robot, src, task, resume=None):
        seen.update(robot=robot, task=task, resume=resume)
        yield {"kind": "text", "text": "hi"}
        yield {"kind": "done", "summary": "ok", "runs": 1, "ok": True, "session_id": "s", "program": None, "code": None}
    monkeypatch.setattr(routes.agent, "run_agent", fake_agent)
    r = client.post("/program/run", json={"task": "move", "resume": "s0"})
    assert r.status_code == 200 and r.headers["content-type"].startswith("text/event-stream")
    events = [json.loads(chunk[6:]) for chunk in r.text.split("\n\n") if chunk.startswith("data: ")]
    assert [e["kind"] for e in events] == ["text", "done"]
    assert seen == {"robot": "bot", "task": "move", "resume": "s0"}


def test_run_without_a_ready_session_is_409(monkeypatch):
    monkeypatch.setattr(routes.session, "current", lambda: None)
    assert client.post("/program/run", json={"task": "x"}).status_code == 409


def test_session_errors_are_503(monkeypatch):
    def boom(*a):
        raise routes.session.SessionError("Start Docker Desktop.")
    monkeypatch.setattr(routes.session, "start", boom)
    body = {"robot_name": "bot", "links": [], "joints": [], "bodies_by_link": {}}
    r = client.post("/program/session", json=body)
    assert r.status_code == 503 and r.json()["detail"] == "Start Docker Desktop."


def test_rerun_bad_path_is_400(monkeypatch):
    monkeypatch.setattr(routes.session, "run_program", lambda p: (_ for _ in ()).throw(ValueError("Program path must look like programs/task_001.py.")))
    assert client.post("/program/rerun", json={"program": "../x"}).status_code == 400
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_program_routes.py -q` — Expected: FAIL (404s / import error).

- [ ] **Step 3: Implement** — `backend/app/program/routes.py`:

```python
import json

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from app.program import agent, session
from app.routes.export_routes import ExportRequest

router = APIRouter(prefix="/program")


class RunRequest(BaseModel):
    task: str
    resume: str | None = None


class RerunRequest(BaseModel):
    program: str


@router.post("/session")
def start_session(req: ExportRequest) -> dict:
    try:
        return session.start(req.robot_name, req.links, req.joints, req.bodies_by_link)
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    except ValueError as exc:
        raise HTTPException(422, str(exc))


@router.get("/session")
def get_session() -> dict:
    try:
        return session.status()
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))


@router.delete("/session")
def stop_session() -> dict:
    session.stop()
    return {"state": "stopped"}


@router.post("/run")
def run(req: RunRequest) -> StreamingResponse:
    cur = session.current()
    if not cur or session.status()["state"] != "ready":
        raise HTTPException(409, "Start the ROS session first and wait until it is ready.")
    events = agent.run_agent(cur["robot"], cur["src"], req.task, req.resume)
    return StreamingResponse((f"data: {json.dumps(e)}\n\n" for e in events), media_type="text/event-stream")


@router.post("/rerun")
def rerun(req: RerunRequest) -> dict:
    try:
        return session.run_program(req.program)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
```

In `backend/app/main.py`: import `from app.program import routes as program_routes` and add `app.include_router(program_routes.router)`.

- [ ] **Step 4: Run tests, then the full suite**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_program_routes.py -q && OPENBLAS_NUM_THREADS=1 $PY -m pytest -q` — Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add backend/app/program/routes.py backend/app/main.py backend/tests/test_program_routes.py
git commit -m "feat: program session, run (SSE) and rerun endpoints

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 7: Live robot in the viewer (rosbridge client, TF math, viewer methods)

**Files:**
- Create: `frontend/src/ros/rosbridge.ts`, `frontend/src/ros/liveRobot.ts`
- Modify: `frontend/src/viewer/Viewer.ts`
- Test: `frontend/src/ros/rosbridge.test.ts`, `frontend/src/ros/liveRobot.test.ts`

**Interfaces:**
- Consumes: `Link`, `Joint` types (`api/types`), three.js.
- Produces:
  - `class RosBridge { constructor(url: string, Socket?: typeof WebSocket); ready: Promise<void>; subscribe(topic: string, type: string, cb: (msg: any) => void, throttleMs?: number): void; call<T>(service: string, type: string, args: object): Promise<T>; close(): void }`
  - `class FrameTree { update(ts: TfTransform[]): void; world(frame: string, root: string): Matrix4 | null }`
  - `homeFrames(links: Link[], joints: Joint[], rootId: string): Map<string, Matrix4>` (key = link name)
  - `bodyPoses(links: Link[], tree: FrameTree, home: Map<string, Matrix4>, base: string): Record<string, Matrix4>` (key = body id)
  - `interface SceneObject { id: string; shape: "box" | "cylinder"; dims: number[]; matrix: Matrix4; attached: boolean }`
  - `sceneObjects(scene: any, tree: FrameTree, base: string): SceneObject[]`
  - `SCENE_COMPONENTS = 20` (attached objects | world geometry)
  - Viewer: `setBodyPoses(poses: Record<string, Matrix4>): void`, `setSceneObjects(objs: SceneObject[]): void`

- [ ] **Step 1: Write the failing tests**

`frontend/src/ros/liveRobot.test.ts`:

```ts
import { Matrix4, Vector3 } from "three";
import { describe, expect, it } from "vitest";
import type { Joint, Link } from "../api/types";
import { FrameTree, bodyPoses, homeFrames, sceneObjects } from "./liveRobot";

const tf = (parent: string, child: string, t: [number, number, number], q: [number, number, number, number] = [0, 0, 0, 1]) =>
  ({ header: { frame_id: parent }, child_frame_id: child,
     transform: { translation: { x: t[0], y: t[1], z: t[2] }, rotation: { x: q[0], y: q[1], z: q[2], w: q[3] } } });
const link = (id: string, bodies: string[]): Link => ({ id, name: id, body_ids: bodies } as unknown as Link);
const joint = (child: string, xyz: [number, number, number]): Joint =>
  ({ id: child, parent_link_id: "base", child_link_id: child, type: "revolute", origin_xyz: xyz, origin_rpy: [0, 0, 0],
     axis: [0, 0, 1], limit_lower: -1, limit_upper: 1, source: "manual" } as Joint);
const at = (m: Matrix4, p: [number, number, number]) => new Vector3(...p).applyMatrix4(m).toArray();

describe("liveRobot", () => {
  it("composes TF frames up to the base", () => {
    const tree = new FrameTree();
    tree.update([tf("base", "arm", [0, 0, 1]), tf("arm", "hand", [1, 0, 0])]);
    expect(at(tree.world("hand", "base")!, [0, 0, 0])).toEqual([1, 0, 1]);
    expect(tree.world("ghost", "base")).toBeNull();
  });

  it("moves each link's bodies by its motion since home", () => {
    const links = [link("base", ["b0"]), link("arm", ["b1"])];
    const home = homeFrames(links, [joint("arm", [0, 0, 1])], "base");
    const tree = new FrameTree();
    tree.update([tf("base", "arm", [0, 0, 1])]);
    const still = bodyPoses(links, tree, home, "base");
    expect(at(still.b1, [2, 0, 1])).toEqual([2, 0, 1]);
    const s = Math.SQRT1_2;
    tree.update([tf("base", "arm", [0, 0, 1], [0, 0, s, s])]);   // arm turned 90° about z
    const p = at(bodyPoses(links, tree, home, "base").b1, [2, 0, 1]);
    expect(p[0]).toBeCloseTo(0); expect(p[1]).toBeCloseTo(2); expect(p[2]).toBeCloseTo(1);
  });

  it("places world boxes and attached cylinders", () => {
    const tree = new FrameTree();
    tree.update([tf("base", "hand", [1, 0, 0])]);
    const pose = (x: number) => ({ position: { x, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } });
    const scene = {
      world: { collision_objects: [{ id: "box", header: { frame_id: "base" }, pose: pose(0),
        primitives: [{ type: 1, dimensions: [0.1, 0.2, 0.3] }], primitive_poses: [pose(2)] }] },
      robot_state: { attached_collision_objects: [{ link_name: "hand", object: { id: "can", header: { frame_id: "hand" },
        pose: pose(0), primitives: [{ type: 3, dimensions: [0.4, 0.05] }], primitive_poses: [pose(0.5)] } }] },
    };
    const objs = sceneObjects(scene, tree, "base");
    expect(objs.map((o) => [o.id, o.shape, o.attached])).toEqual([["box", "box", false], ["can", "cylinder", true]]);
    expect(at(objs[0].matrix, [0, 0, 0])).toEqual([2, 0, 0]);
    expect(at(objs[1].matrix, [0, 0, 0])).toEqual([1.5, 0, 0]);
    expect(objs[1].dims).toEqual([0.4, 0.05]);
  });
});
```

`frontend/src/ros/rosbridge.test.ts`:

```ts
import { describe, expect, it } from "vitest";
import { RosBridge } from "./rosbridge";

class FakeSocket {
  static last: FakeSocket;
  sent: any[] = [];
  onopen?: () => void; onerror?: () => void; onmessage?: (e: { data: string }) => void;
  constructor(public url: string) { FakeSocket.last = this; setTimeout(() => this.onopen?.(), 0); }
  send(s: string) { this.sent.push(JSON.parse(s)); }
  close() {}
  emit(o: object) { this.onmessage?.({ data: JSON.stringify(o) }); }
}

describe("RosBridge", () => {
  it("subscribes, delivers messages and answers service calls", async () => {
    const ros = new RosBridge("ws://x", FakeSocket as unknown as typeof WebSocket);
    await ros.ready;
    const got: any[] = [];
    ros.subscribe("/tf", "tf2_msgs/msg/TFMessage", (m) => got.push(m), 50);
    expect(FakeSocket.last.sent[0]).toEqual({ op: "subscribe", topic: "/tf", type: "tf2_msgs/msg/TFMessage", throttle_rate: 50 });
    FakeSocket.last.emit({ op: "publish", topic: "/tf", msg: { a: 1 } });
    expect(got).toEqual([{ a: 1 }]);
    const p = ros.call<{ scene: number }>("/get_planning_scene", "moveit_msgs/srv/GetPlanningScene", { components: { components: 20 } });
    const req = FakeSocket.last.sent[1];
    expect(req.op).toBe("call_service");
    FakeSocket.last.emit({ op: "service_response", id: req.id, result: true, values: { scene: 7 } });
    expect(await p).toEqual({ scene: 7 });
  });
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd frontend && npx vitest run src/ros` — Expected: FAIL (modules missing).

- [ ] **Step 3: Implement**

`frontend/src/ros/rosbridge.ts`:

```ts
/** Minimal rosbridge v2 client over the browser WebSocket: subscribe and call services. */
export class RosBridge {
  readonly ready: Promise<void>;
  private ws: WebSocket;
  private nextId = 0;
  private subs = new Map<string, (msg: any) => void>();
  private calls = new Map<string, (m: any) => void>();

  constructor(url: string, Socket: typeof WebSocket = WebSocket) {
    this.ws = new Socket(url);
    this.ready = new Promise((resolve, reject) => {
      this.ws.onopen = () => resolve();
      this.ws.onerror = () => reject(new Error(`Cannot connect to ROS at ${url}`));
    });
    this.ws.onmessage = (e: MessageEvent) => {
      const m = JSON.parse(e.data as string);
      if (m.op === "publish") this.subs.get(m.topic)?.(m.msg);
      else if (m.op === "service_response") { this.calls.get(m.id)?.(m); this.calls.delete(m.id); }
    };
  }

  subscribe(topic: string, type: string, cb: (msg: any) => void, throttleMs = 0) {
    this.subs.set(topic, cb);
    this.send({ op: "subscribe", topic, type, throttle_rate: throttleMs });
  }

  call<T>(service: string, type: string, args: object): Promise<T> {
    const id = `call_${this.nextId++}`;
    return new Promise((resolve, reject) => {
      this.calls.set(id, (m) => (m.result === false ? reject(new Error(`${service} failed`)) : resolve(m.values as T)));
      this.send({ op: "call_service", id, service, type, args });
    });
  }

  close() { this.ws.close(); }

  private send(o: object) { this.ws.send(JSON.stringify(o)); }
}
```

`frontend/src/ros/liveRobot.ts`:

```ts
import { Euler, Matrix4, Quaternion, Vector3 } from "three";
import type { Joint, Link } from "../api/types";

export interface TfTransform {
  header: { frame_id: string };
  child_frame_id: string;
  transform: { translation: { x: number; y: number; z: number }; rotation: { x: number; y: number; z: number; w: number } };
}
export interface SceneObject { id: string; shape: "box" | "cylinder"; dims: number[]; matrix: Matrix4; attached: boolean }
/** GetPlanningScene components: ROBOT_STATE_ATTACHED_OBJECTS (4) | WORLD_OBJECT_GEOMETRY (16). */
export const SCENE_COMPONENTS = 20;

const clean = (f: string) => f.replace(/^\//, "");
const poseMatrix = (p: { position: { x: number; y: number; z: number }; orientation: { x: number; y: number; z: number; w: number } }) =>
  new Matrix4().compose(new Vector3(p.position.x, p.position.y, p.position.z),
    new Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), new Vector3(1, 1, 1));

/** Parent links and transforms from /tf and /tf_static. */
export class FrameTree {
  private edges = new Map<string, { parent: string; m: Matrix4 }>();

  update(ts: TfTransform[]) {
    for (const t of ts) {
      const { translation: p, rotation: q } = t.transform;
      this.edges.set(clean(t.child_frame_id), { parent: clean(t.header.frame_id),
        m: new Matrix4().compose(new Vector3(p.x, p.y, p.z), new Quaternion(q.x, q.y, q.z, q.w), new Vector3(1, 1, 1)) });
    }
  }

  world(frame: string, root: string): Matrix4 | null {
    const m = new Matrix4();
    let f = clean(frame);
    for (let guard = 0; f !== root; guard++) {
      const e = this.edges.get(f);
      if (!e || guard > 100) return null;
      m.premultiply(e.m);
      f = e.parent;
    }
    return m;
  }
}

/** Each link's frame at the CAD pose (the exported URDF's zero configuration), by link name. */
export function homeFrames(links: Link[], joints: Joint[], rootId: string): Map<string, Matrix4> {
  const byChild = new Map(joints.map((j) => [j.child_link_id, j]));
  const out = new Map<string, Matrix4>();
  for (const l of links) {
    const j = byChild.get(l.id);
    if (l.id === rootId || !j) { out.set(l.name, new Matrix4()); continue; }
    const [r, p, y] = j.origin_rpy;
    out.set(l.name, new Matrix4().compose(new Vector3(...j.origin_xyz),
      new Quaternion().setFromEuler(new Euler(r, p, y, "ZYX")), new Vector3(1, 1, 1)));
  }
  return out;
}

/** Body meshes are in the CAD world frame: move each by its link's motion since the CAD pose. */
export function bodyPoses(links: Link[], tree: FrameTree, home: Map<string, Matrix4>, base: string): Record<string, Matrix4> {
  const out: Record<string, Matrix4> = {};
  for (const l of links) {
    const now = l.name === base ? new Matrix4() : tree.world(l.name, base);
    const h = home.get(l.name);
    if (!now || !h) continue;
    const delta = now.clone().multiply(h.clone().invert());
    for (const id of l.body_ids) out[id] = delta;
  }
  return out;
}

function objectsOf(o: any, frameM: Matrix4, attached: boolean): SceneObject[] {
  const base = frameM.clone().multiply(o.pose ? poseMatrix(o.pose) : new Matrix4());
  return (o.primitives ?? []).flatMap((p: any, i: number) => {
    const shape = p.type === 1 ? "box" : p.type === 3 ? "cylinder" : null;
    if (!shape) return [];
    const local = o.primitive_poses?.[i] ? poseMatrix(o.primitive_poses[i]) : new Matrix4();
    return [{ id: o.id, shape, dims: [...p.dimensions], matrix: base.clone().multiply(local), attached }];
  });
}

export function sceneObjects(scene: any, tree: FrameTree, base: string): SceneObject[] {
  const frame = (f: string) => (clean(f || base) === base ? new Matrix4() : tree.world(f, base));
  const out: SceneObject[] = [];
  for (const o of scene?.world?.collision_objects ?? []) {
    const m = frame(o.header?.frame_id);
    if (m) out.push(...objectsOf(o, m, false));
  }
  for (const a of scene?.robot_state?.attached_collision_objects ?? []) {
    const m = frame(a.object?.header?.frame_id || a.link_name);
    if (m) out.push(...objectsOf(a.object, m, true));
  }
  return out;
}
```

In `frontend/src/viewer/Viewer.ts` add (near `setPreview`; reuse its imports, adding `BoxGeometry`, `CylinderGeometry`, `Group`, `MeshStandardMaterial` from three and `SceneObject` type from `../ros/liveRobot` if not already imported):

```ts
  private objects = new Group();

  /** Per-body poses (e.g. a live robot); bodies not listed stay at their CAD pose. */
  setBodyPoses(poses: Record<string, Matrix4>) {
    for (const [id, mesh] of this.meshes) {
      mesh.matrixAutoUpdate = false;
      mesh.matrix.copy(poses[id] ?? new Matrix4());
      mesh.matrixWorldNeedsUpdate = true;
    }
    this.requestRender();
  }

  setSceneObjects(objs: SceneObject[]) {
    if (!this.objects.parent) this.scene.add(this.objects);
    for (const c of [...this.objects.children] as Mesh[]) {
      this.objects.remove(c); c.geometry.dispose(); (c.material as Material).dispose();
    }
    for (const o of objs) {
      const geo = o.shape === "box" ? new BoxGeometry(o.dims[0], o.dims[1], o.dims[2])
        : new CylinderGeometry(o.dims[1], o.dims[1], o.dims[0], 32).rotateX(Math.PI / 2); // ROS cylinders stand on z
      const mesh = new Mesh(geo, new MeshStandardMaterial({ color: o.attached ? 0xf59e0b : 0x60a5fa }));
      mesh.matrixAutoUpdate = false;
      mesh.matrix.copy(o.matrix);
      this.objects.add(mesh);
    }
    this.requestRender();
  }
```

- [ ] **Step 4: Run tests, then the full frontend suite and type-check**

Run: `cd frontend && npx vitest run src/ros && npm test && npx tsc --noEmit` — Expected: all PASS, no type errors.

- [ ] **Step 5: Commit**

```bash
git add frontend/src/ros frontend/src/viewer/Viewer.ts
git commit -m "feat: rosbridge client and live robot poses for the viewer

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: Program step UI

**Files:**
- Create: `frontend/src/api/program.ts`, `frontend/src/ui/steps/ProgramStep.tsx`
- Modify: `frontend/src/api/client.ts` (export `send` and `json`), `frontend/src/model/checklist.ts` (`StepId`), `frontend/src/ui/Stepper.tsx` (`STEPS`, `stepBadges`), `frontend/src/ui/steps/StepPanel.tsx`
- Test: `frontend/src/api/program.test.ts`, `frontend/src/ui/steps/program.test.tsx`

**Interfaces:**
- Consumes: Task 6 endpoints; Task 7 `RosBridge`, `FrameTree`, `homeFrames`, `bodyPoses`, `sceneObjects`, `SCENE_COMPONENTS`, Viewer methods; `buildExportRequest`, `withNames` as used by ExportStep; `Ctx`.
- Produces: `type ProgramEvent` (union matching Task 5's events); `startSession(robotName, links, joints, bodiesByLink): Promise<SessionStatus>`; `sessionStatus(): Promise<SessionStatus>`; `stopSession(): Promise<void>`; `runTask(task: string, resume: string | null, onEvent: (e: ProgramEvent) => void, signal?: AbortSignal): Promise<void>`; `rerun(program: string): Promise<{ ok: boolean; output: string }>`; `parseSse(buffer: string): { events: ProgramEvent[]; rest: string }`.

- [ ] **Step 1: Write the failing tests**

`frontend/src/api/program.test.ts`:

```ts
import { afterEach, describe, expect, it, vi } from "vitest";
import { parseSse, runTask, type ProgramEvent } from "./program";

afterEach(() => vi.restoreAllMocks());

describe("program api", () => {
  it("parses SSE events split across chunks exactly once", async () => {
    const text = 'data: {"kind":"text","text":"a"}\n\ndata: {"kind":"run","n":1}\n\ndata: {"kind":"done","summary":"s","runs":1,"ok":true,"session_id":"x","program":null,"code":null}\n\n';
    const chunks = [text.slice(0, 17), text.slice(17, 50), text.slice(50)];
    const body = new ReadableStream({ start(c) { for (const k of chunks) c.enqueue(new TextEncoder().encode(k)); c.close(); } });
    vi.spyOn(globalThis, "fetch").mockResolvedValue(new Response(body, { status: 200 }));
    const got: ProgramEvent[] = [];
    await runTask("go", null, (e) => got.push(e));
    expect(got.map((e) => e.kind)).toEqual(["text", "run", "done"]);
  });

  it("keeps an unfinished event for the next chunk", () => {
    const { events, rest } = parseSse('data: {"kind":"run","n":1}\n\ndata: {"kind"');
    expect(events).toEqual([{ kind: "run", n: 1 }]);
    expect(rest).toBe('data: {"kind"');
  });
});
```

`frontend/src/ui/steps/program.test.tsx`:

```tsx
import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as program from "../../api/program";
import { boxBody } from "../../model/testBodies";
import { Harness, imported } from "../testing";
import ProgramStep from "./ProgramStep";

vi.mock("../../ros/rosbridge", () => ({
  RosBridge: class { ready = Promise.resolve(); subscribe() {} call() { return Promise.resolve({ scene: {} }); } close() {} },
}));
afterEach(() => vi.restoreAllMocks());

const ui = () => render(<Harness start={imported([boxBody("base", [0, 0, 0], [1, 1, 1])])} step="program"
  ui={(ctx) => <ProgramStep ctx={ctx} />} />);

describe("ProgramStep", () => {
  it("starts a session, runs a task and shows the result, runs used and the program", async () => {
    vi.spyOn(program, "startSession").mockResolvedValue({ state: "starting", robot: "bot" });
    vi.spyOn(program, "sessionStatus").mockResolvedValue({ state: "ready", robot: "bot" });
    vi.spyOn(program, "runTask").mockImplementation(async (_t, _r, on) => {
      on({ kind: "text", text: "Writing the program" });
      on({ kind: "run", n: 1 });
      on({ kind: "run_result", ok: false, message: "failed: Unreachable: too far" });
      on({ kind: "run", n: 2 });
      on({ kind: "run_result", ok: true, message: "ok" });
      on({ kind: "done", summary: "Moves the tool.\nRun ok.", runs: 2, ok: true, session_id: "s1",
           program: "programs/task_001.py", code: "from robot import Robot" });
    });
    const rerun = vi.spyOn(program, "rerun").mockResolvedValue({ ok: true, output: "RESULT: ok" });
    ui();
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /start ros/i })); });
    await screen.findByText(/ROS ready/i);
    fireEvent.change(screen.getByLabelText(/task/i), { target: { value: "move the tool" } });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /^send$/i })); });
    expect(screen.getByText(/worked on run 2\/3/i)).toBeInTheDocument();
    expect(screen.getByText(/Moves the tool\./)).toBeInTheDocument();
    expect(screen.getByText("from robot import Robot")).toBeInTheDocument();
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /run again/i })); });
    expect(rerun).toHaveBeenCalledWith("programs/task_001.py");
  });

  it("shows backend errors in the chat", async () => {
    vi.spyOn(program, "startSession").mockRejectedValue(new Error("Start Docker Desktop."));
    ui();
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /start ros/i })); });
    expect(await screen.findByText("Start Docker Desktop.")).toBeInTheDocument();
  });
});
```

(If `Harness` requires the step id to be a known `StepId`, this test also proves the union was extended.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd frontend && npx vitest run src/api/program.test.ts src/ui/steps/program.test.tsx` — Expected: FAIL (modules missing).

- [ ] **Step 3: Implement**

In `frontend/src/api/client.ts`, change `async function send(` to `export async function send(` and `const json =` to `export const json =`.

`frontend/src/api/program.ts`:

```ts
import { json, send } from "./client";
import type { Body, Joint, Link } from "./types";

export type ProgramEvent =
  | { kind: "text"; text: string }
  | { kind: "file"; path: string }
  | { kind: "run"; n: number }
  | { kind: "run_result"; ok: boolean; message: string }
  | { kind: "done"; summary: string; runs: number; ok: boolean; session_id: string | null; program: string | null; code: string | null }
  | { kind: "error"; message: string };
export interface SessionStatus { state: "stopped" | "starting" | "ready" | "failed"; robot?: string | null; log?: string }

export async function startSession(robotName: string, links: Link[], joints: Joint[],
  bodiesByLink: Record<string, Body[]>): Promise<SessionStatus> {
  return (await send("/program/session", json({ robot_name: robotName, links, joints, bodies_by_link: bodiesByLink }))).json();
}
export async function sessionStatus(): Promise<SessionStatus> {
  return (await send("/program/session", { method: "GET" })).json();
}
export async function stopSession(): Promise<void> {
  await send("/program/session", { method: "DELETE" });
}
export async function rerun(program: string): Promise<{ ok: boolean; output: string }> {
  return (await send("/program/rerun", json({ program }))).json();
}

export function parseSse(buffer: string): { events: ProgramEvent[]; rest: string } {
  const parts = buffer.split("\n\n");
  const rest = parts.pop() ?? "";
  const events = parts.filter((p) => p.startsWith("data: ")).map((p) => JSON.parse(p.slice(6)) as ProgramEvent);
  return { events, rest };
}

export async function runTask(task: string, resume: string | null, onEvent: (e: ProgramEvent) => void,
  signal?: AbortSignal): Promise<void> {
  const res = await send("/program/run", { ...json({ task, resume }), signal });
  const reader = res.body!.getReader();
  const decoder = new TextDecoder();
  let buffer = "";
  for (;;) {
    const { done, value } = await reader.read();
    if (value) buffer += decoder.decode(value, { stream: true });
    const { events, rest } = parseSse(buffer);
    buffer = rest;
    events.forEach(onEvent);
    if (done) break;
  }
}
```

`frontend/src/model/checklist.ts`: `export type StepId = "parts" | "group" | "connect" | "export" | "program";`

`frontend/src/ui/Stepper.tsx`: add `{ id: "program", label: "Program" }` at the end of `STEPS`; in `stepBadges`, add `program: 0` to both returned objects.

`frontend/src/ui/steps/StepPanel.tsx`: import `ProgramStep` and add `case "program": return <ProgramStep ctx={ctx} />;`.

`frontend/src/ui/steps/ProgramStep.tsx`:

```tsx
import { Play, RotateCcw, Send, Square } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import * as api from "../../api/program";
import type { ProgramEvent, SessionStatus } from "../../api/program";
import { buildExportRequest } from "../../model/exportRequest";
import { FrameTree, SCENE_COMPONENTS, bodyPoses, homeFrames, sceneObjects } from "../../ros/liveRobot";
import { RosBridge } from "../../ros/rosbridge";
import type { Ctx } from "../ctx";
import { withNames } from "../useLinkMass";

const MAX_RUNS = 3;
interface TaskCard { task: string; lines: string[]; runs: number; done?: Extract<ProgramEvent, { kind: "done" }>; error?: string }

export default function ProgramStep({ ctx }: { ctx: Ctx }) {
  const { model, robotName, viewer } = ctx;
  const [status, setStatus] = useState<SessionStatus>({ state: "stopped" });
  const [error, setError] = useState<string | null>(null);
  const [cards, setCards] = useState<TaskCard[]>([]);
  const [task, setTask] = useState("");
  const [busy, setBusy] = useState(false);
  const [resume, setResume] = useState<string | null>(null);
  const [rerunOut, setRerunOut] = useState<string | null>(null);
  const ros = useRef<RosBridge | null>(null);
  const last = [...cards].reverse().find((c) => c.done?.code);

  async function start() {
    setError(null);
    try {
      const req = buildExportRequest(model);
      setStatus(await api.startSession(robotName, req.links, req.joints, req.bodiesByLink));
    } catch (e) { setError(withNames((e as Error).message, model.links)); }
  }

  useEffect(() => {  // poll until ready or failed
    if (status.state !== "starting") return;
    const t = setInterval(async () => {
      try { setStatus(await api.sessionStatus()); } catch (e) { setError((e as Error).message); }
    }, 1000);
    return () => clearInterval(t);
  }, [status.state]);

  useEffect(() => {  // live robot while ready
    if (status.state !== "ready" || !viewer) return;
    const req = buildExportRequest(model);
    const root = model.rootLinkId ?? req.links[0]?.id;
    const base = req.links.find((l) => l.id === root)?.name ?? "";
    const home = homeFrames(req.links, req.joints, root ?? "");
    const tree = new FrameTree();
    const bridge = new RosBridge(`ws://${location.hostname || "localhost"}:9090`);
    ros.current = bridge;
    let frame = 0;
    const redraw = () => { frame = 0; viewer.setBodyPoses(bodyPoses(req.links, tree, home, base)); };
    const onTf = (m: { transforms: never[] }) => { tree.update(m.transforms); if (!frame) frame = requestAnimationFrame(redraw); };
    const refreshScene = () => bridge.call<{ scene: unknown }>("/get_planning_scene", "moveit_msgs/srv/GetPlanningScene",
      { components: { components: SCENE_COMPONENTS } }).then((r) => viewer.setSceneObjects(sceneObjects(r.scene, tree, base))).catch(() => {});
    bridge.ready.then(() => {
      bridge.subscribe("/tf", "tf2_msgs/msg/TFMessage", onTf);
      bridge.subscribe("/tf_static", "tf2_msgs/msg/TFMessage", onTf);
      bridge.subscribe("/monitored_planning_scene", "moveit_msgs/msg/PlanningScene", refreshScene, 300);
      refreshScene();
    }).catch((e) => setError(e.message));
    return () => { cancelAnimationFrame(frame); bridge.close(); ros.current = null; viewer.setBodyPoses({}); viewer.setSceneObjects([]); };
  }, [status.state, viewer, model]);

  async function send() {
    const text = task.trim();
    if (!text || busy) return;
    setTask(""); setBusy(true); setRerunOut(null);
    setCards((cs) => [...cs, { task: text, lines: [], runs: 0 }]);
    const update = (f: (c: TaskCard) => TaskCard) => setCards((cs) => cs.map((c, i) => (i === cs.length - 1 ? f(c) : c)));
    try {
      await api.runTask(text, resume, (e) => {
        if (e.kind === "text") update((c) => ({ ...c, lines: [...c.lines, e.text] }));
        else if (e.kind === "run") update((c) => ({ ...c, runs: e.n }));
        else if (e.kind === "run_result") update((c) => ({ ...c, lines: [...c.lines, `Run ${c.runs}: ${e.message}`] }));
        else if (e.kind === "done") { update((c) => ({ ...c, done: e })); if (e.session_id) setResume(e.session_id); }
        else if (e.kind === "error") update((c) => ({ ...c, error: e.message }));
      });
    } catch (e) { update((c) => ({ ...c, error: (e as Error).message })); }
    setBusy(false);
  }

  async function runAgain() {
    if (!last?.done?.program) return;
    try { const r = await api.rerun(last.done.program); setRerunOut(r.ok ? "Run again: ok" : r.output.trim().split("\n").pop() ?? "failed"); }
    catch (e) { setRerunOut((e as Error).message); }
  }

  const ready = status.state === "ready";
  return (
    <div className="program-step">
      <div className="row">
        {status.state === "stopped" || status.state === "failed"
          ? <button onClick={start}><Play size={14} /> Start ROS</button>
          : <button onClick={async () => { await api.stopSession(); setStatus({ state: "stopped" }); }}><Square size={14} /> Stop ROS</button>}
        <span className="muted">
          {status.state === "starting" && "Starting ROS… (the first start builds the workspace)"}
          {ready && "ROS ready"}
          {status.state === "failed" && "ROS session stopped"}
        </span>
      </div>
      {status.state === "failed" && status.log && <pre className="log">{status.log}</pre>}
      {error && <p className="error">{error}</p>}
      <div className="chat">
        {cards.map((c, i) => (
          <div key={i} className="task-card">
            <p><b>{c.task}</b></p>
            {c.lines.map((l, k) => <p key={k} className="muted">{l}</p>)}
            {c.done && <p>{c.done.ok ? `Worked on run ${c.done.runs}/${MAX_RUNS}` : `Failed after ${c.done.runs} run${c.done.runs === 1 ? "" : "s"}`}</p>}
            {c.done?.summary && <p style={{ whiteSpace: "pre-wrap" }}>{c.done.summary}</p>}
            {c.error && <p className="error">{c.error}</p>}
          </div>
        ))}
      </div>
      <label>Task<textarea value={task} disabled={!ready || busy} onChange={(e) => setTask(e.target.value)}
        placeholder="e.g. Put a 5 cm box at (0.5, 0.2) on the floor, pick it up and place it at (0.4, -0.3)" /></label>
      <div className="row">
        <button onClick={send} disabled={!ready || busy || !task.trim()}><Send size={14} /> Send</button>
        <button onClick={() => setResume(null)} disabled={!resume || busy}>New conversation</button>
        <button onClick={runAgain} disabled={!ready || busy || !last?.done?.program}><RotateCcw size={14} /> Run again</button>
      </div>
      {rerunOut && <p className="muted">{rerunOut}</p>}
      {last?.done?.code && (<><p className="muted">{last.done.program}</p><pre className="code">{last.done.code}</pre></>)}
    </div>
  );
}
```

`withNames(message, links)` turns link ids in backend error messages into link names, as in `ExportStep.tsx`. `model.rootLinkId` is the root link id (`ModelState`).

- [ ] **Step 4: Run tests, the full frontend suite and the type-check**

Run: `cd frontend && npx vitest run src/api/program.test.ts src/ui/steps/program.test.tsx && npm test && npx tsc --noEmit`
Expected: all PASS. Existing tests asserting the step list or badge record (e.g. `shell.test.tsx`) may need `program` added to their expected values — update only those expectations.

- [ ] **Step 5: Commit**

```bash
git add frontend/src/api frontend/src/model/checklist.ts frontend/src/ui
git commit -m "feat: Program step with chat, program view and live robot

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 9: End-to-end run and benchmark (controller inline)

**Files:** none new (fixes go to the owning module with a test).

- [ ] **Step 1: Full automated suites**

Run backend `pytest -q`, frontend `npm test && npx tsc --noEmit && npm run e2e`, and `bash scripts/ros_check.sh corpus/out/irb1300.zip`. Expected: all pass.

- [ ] **Step 2: Live session check (no LLM)**

Start the app (`scripts/dev.ps1` or backend + `npm run dev`), import `corpus/irb1300_asm.stp`, group and connect as in the corpus expectations (or load a saved project), open Program, click Start ROS. Expected: "ROS ready" within ~30 s after the first build; robot shown at home. Write `programs/task_000.py` by hand (the pick_place check program), click Run again: the robot moves and the box rides on the tool in the viewer.

- [ ] **Step 3: Benchmark with Claude (uses the user's Claude subscription: ask before starting)**

Run the five tasks from spec §8 in the Program step, one each, with "New conversation" between them. Record per task: success (RESULT ok + motion looks right), runs needed, any error text. Pass: ≥ 4/5 within 3 runs. Report the table to the user. Failures from the library (wrong error text, missing API) are fixed in `robot.py` / `brief.md` with a test, then the failed task is re-run once.

- [ ] **Step 4: Commit fixes, if any, per owning task's conventions.**
