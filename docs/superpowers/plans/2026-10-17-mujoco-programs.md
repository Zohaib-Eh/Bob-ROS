# MuJoCo Programs (part C) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Program-mode runs drive a physically simulated arm and physical objects in MuJoCo through `mujoco_ros2_control`, instead of mock hardware.

**Architecture:**
- The session container runs `ros2_control_node` with the `mujoco_ros2_control` hardware plugin on a generated `scene.xml`. That file holds the exported robot MJCF plus the objects from `workcell.yaml` and the program's `SCENE`.
- MoveIt and the JointTrajectoryController are unchanged.
- A small C++ plugin toggles one weld per object for `attach`/`detach`.
- A Python node mirrors object poses into MoveIt's planning scene.
- Every run resets the world. It uses `reset_world` when the scene is unchanged and restarts the sim node when it changed.

**Tech Stack:**
- ROS 2 Jazzy, MoveIt 2, ros2_control, `ros-jazzy-mujoco-ros2-control`
- MuJoCo 3.x (Python `mujoco` on host and in the container)
- C++17 (plugin), Python 3.11 (host) / 3.12 (container)
- pytest; React + vitest (frontend)

**Spec:** `docs/superpowers/specs/2026-10-17-mujoco-programs-design.md`

## Global Constraints

- Branch `mujoco-programs`. The backend test command, from `backend/`: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`.
- In a worktree, ad-hoc scripts need `PYTHONPATH=.` in `backend/`, or they import the main checkout's `app`.
- Object format fields, exactly: `name`, `shape` (`box` | `cylinder`), `size` (box `[x, y, z]` m; cylinder `[radius, height]`), `xyz`, `rpy` (optional), `fixed` (optional), `mass` (kg, optional, default `700 kg/m³ × volume`), `friction` (optional).
- `ATTACH_REACH = 0.02` m stays.
- Sim speed: real time (`sim_speed_factor` 1.0), headless.
- The sim restart timeout is 30 s. On timeout, the message carries the sim node's last 20 log lines.
- Exact message texts:
  - "Objects can't appear mid-run. Declare '<name>' in SCENE at the top of the program (or in workcell.yaml)."
  - "SCENE must be plain values (line N)"
  - "Simulation blew up: check <link>'s mass and inertia."
  - Top bar: "Physics: MuJoCo" / "Physics off: <reason>"
- No MJCF in the export means mock hardware, exactly as today.
- Every new container-side file is generated into the workspace by `program_pkg.py` or `moveit_config.py`. Nothing is hand-placed in the image except the apt and pip packages.
- Container checks: irb1300 in the foreground, other robots in the background (~3 min each). Say the duration before starting.

## Review Focus

1. **Resting contact isn't overlap:** a box sitting exactly on a table (gap 0 to 1 mm) must pass the overlap check. Only penetration deeper than 1 mm is an error. Test in Task 3.
2. **No objects at all:** no `SCENE`, and `workcell.yaml` missing or empty, must give an arm-only scene and a normal run. Tests in Tasks 2 and 3.
3. **`SCENE` written as `dict(...)` calls** (the form the brief shows) must parse; `ast.literal_eval` alone rejects calls. Test in Task 2.
4. **Object names that clash with robot links,** or aren't identifiers, must be rejected before MuJoCo sees them, with a clear message. Tests in Tasks 2 and 3.
5. **The same program run twice** (the `reset_world` path) must start with every weld off and every object back at its declared pose. Test in Task 3 (`on_reset` equivalent in MuJoCo Python) and Task 11 (container).

---

## File Structure

| File | Responsibility |
|---|---|
| `docker/jazzy/Dockerfile` | + `ros-jazzy-mujoco-ros2-control`, + pip `mujoco` (pinned to the Debian numpy) |
| `backend/app/ros/templates/scene.py` | Pure Python (+ `mujoco`): object validation, workcell + `SCENE` merge, `SCENE` reading via AST, `scene.xml` generation, home-overlap check, meta for error messages. Runs on host (tests) and in the container. |
| `backend/app/ros/templates/sim_errors.py` | Pure Python: turns a tracking abort or a blow-up into the plain-language message. |
| `backend/app/ros/templates/sim_ctl.py` | Container: start, restart and reset the sim node for a scene; writes `/tmp/physics`. |
| `backend/app/ros/templates/scene_sync.py` | Container node: MuJoCo free-joint poses into MoveIt planning-scene moves. |
| `backend/app/ros/templates/glue/` (`package.xml`, `CMakeLists.txt`, `glue_plugin.cpp`, `plugins.xml`) | Generated `cad_to_urdf_glue` package: `MuJoCoROS2ControlPluginBase` that toggles welds. |
| `backend/app/ros/moveit_config.py` | `hardware == 'mujoco'` xacro branch, JTC tolerances, launch without `ros2_control_node` when physics is on, deps |
| `backend/app/ros/program_pkg.py` | Writes the new templates and the glue package; brief/API_DOC update; creates an empty `workcell.yaml` |
| `backend/app/ros/templates/robot.py` | `add_*` raise; `attach`/`detach` via the glue; `pose()`; `MotionError`; blow-up check |
| `backend/app/ros/templates/runner.py` | Per-run reset through `sim_ctl.prepare(program)` |
| `docker/jazzy/session/start.sh` | Boots the sim (or mock) before MoveIt; starts `scene_sync` |
| `backend/app/program/session.py`, `routes.py` | Status gains `physics` |
| `backend/app/program/files.py` | `<robot>_tasks/workcell.yaml` is editable but not deletable |
| `frontend/src/ui/program/ProgramMode.tsx` | Shows the physics status in the top bar |
| `docker/jazzy/check/programs/pick_place_physics.py` | Container acceptance program |

---

### Task 1: Verify `mujoco_ros2_control` before building on it (gate)

This task produces findings, not product code. **If any check below fails, stop and report to the user.** Do not switch approach on your own.

**Files:**
- Modify: `docker/jazzy/Dockerfile`
- Modify: `docs/superpowers/specs/2026-10-17-mujoco-programs-design.md` (append a "Task 1 findings" section)

- [ ] **Step 1: Add the packages to the image.**

In `docker/jazzy/Dockerfile`, add `ros-jazzy-mujoco-ros2-control \` to the apt list, after `ros-jazzy-ros2-controllers \`. Then extend the pip line so `mujoco` installs against the pinned numpy:

```dockerfile
RUN python3 -c "import numpy; print('numpy==' + numpy.__version__)" > /tmp/pin.txt \
    && pip3 install --no-cache-dir --break-system-packages -c /tmp/pin.txt EAIK mujoco
```

- [ ] **Step 2: Build the image (~5–10 min; say so first).**

Run (from the repo root): `docker build -t cad-to-urdf-jazzy docker/jazzy`
Expected: success. If apt can't find `ros-jazzy-mujoco-ros2-control`: STOP and report. The fallback, a source build with `mujoco_vendor`, is the user's call.

- [ ] **Step 3: Record the plugin facts we depend on.**

Run:

```bash
docker run --rm cad-to-urdf-jazzy bash -c "source /opt/ros/jazzy/setup.bash && \
  cat \$(ros2 pkg prefix mujoco_ros2_control)/share/mujoco_ros2_control/*.xml; \
  ls \$(ros2 pkg prefix mujoco_ros2_control_plugins)/share/mujoco_ros2_control_plugins; \
  cat \$(ros2 pkg prefix mujoco_ros2_control_plugins)/share/mujoco_ros2_control_plugins/*.xml; \
  ros2 interface show mujoco_ros2_control_msgs/msg/FreeJointStateArray; \
  ros2 interface show mujoco_ros2_control_msgs/msg/FreeJointState; \
  python3 -c 'import mujoco, numpy; print(mujoco.__version__, numpy.__version__)'"
```

Write down:
- (a) the hardware plugin class name, e.g. `mujoco_ros2_control/MujocoSystemInterface`
- (b) the free-joint publisher plugin class name and its parameters (topic name, rate)
- (c) the `FreeJointState` fields, and whether they carry the joint name or the body name
- (d) how plugins are listed in the URDF `<hardware>` params
- (e) the MuJoCo Python version

- [ ] **Step 4: Run the irb1300 MJCF under the plugin.**

Use the irb1300 workspace from an earlier session (`~/.cad-to-urdf/workspaces/irb1300`). If it's missing, start a session for irb1300 in the app once.

Make a probe xacro by copying `config/irb1300.ros2_control.xacro` and replacing the `<hardware>` block with:

```xml
<plugin>(a)</plugin>
<param name="mujoco_model">/ws/src/irb1300/mjcf/irb1300.xml</param>
<param name="headless">true</param>
```

Each joint keeps `<command_interface name="position"/>` and state interfaces `position`, `velocity` and `effort`.

Launch `ros2_control_node` and spawn `joint_state_broadcaster` and `arm_controller` by hand in a container with `/ws` mounted. Then send one `FollowJointTrajectory` goal (joint2 +0.3 rad over 2 s) with `ros2 action send_goal`.

Expected:
- the controllers go active
- `/joint_states` follows the goal within 0.02 rad

If startup errors on the `position` actuators (the docs hint that drivers may need motor-type actuators), STOP and report the exact error.

- [ ] **Step 5: Measure the restart and check MoveIt reconnects.**

With `move_group` running (`ros2 launch irb1300_moveit_config demo.launch.py` with the probe hardware):
- kill `ros2_control_node`, start it again and respawn the controllers; time it with `date +%s.%N`
- then plan and execute one MoveIt goal, e.g. `ros2 action send_goal /move_action` with a joint goal, without restarting `move_group`

Record:
- (f) the restart time
- (g) whether MoveIt executes after the restart, and whether it needs a pause
- (h) whether publishing a new MJCF on `/mujoco_robot_description` reloads without a restart

If (g) fails, STOP and report.

- [ ] **Step 6: Build a no-op plugin to prove the toolchain.**

Create a temporary `/ws/src/probe_plugin` package containing a class that derives from `mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase` and logs once in `init`. Export it with pluginlib (use the base class name from Step 3's plugin XML) and list it in the probe hardware params as in (d). Run `colcon build`, launch, and confirm the log line appears. Delete the package afterwards.

- [ ] **Step 7: Write the findings into the spec and commit.**

Append `## Task 1 findings (YYYY-MM-DD)` to the spec with (a)–(h), and set the Task 4–7 placeholders named `<<from Task 1 …>>` below to the recorded values.

```bash
git add docker/jazzy/Dockerfile docs/superpowers/specs/2026-10-17-mujoco-programs-design.md
git commit -m "chore: mujoco_ros2_control in the session image; Task 1 findings"
```

---

### Task 2: Objects: validation, merge, and reading `SCENE`

**Files:**
- Create: `backend/app/ros/templates/scene.py`
- Test: `backend/tests/test_ros_scene.py`

**Interfaces:**
- Produces:
  - `class SceneError(Exception)`
  - `validate(objs: list[dict], where: str) -> list[dict]`: normalized copies with every field present (`rpy`, `fixed`, `mass`, `friction`, where `friction` is `None` if unset; `mass` is `None` for fixed objects)
  - `merge(workcell: list[dict], program: list[dict]) -> list[dict]`
  - `read_scene(source: str) -> list[dict]`
  - `load_workcell(text: str | None) -> list[dict]`

The template has no `__ROBOT__` placeholders, so tests import it directly from `app/ros/templates`.

- [ ] **Step 1: Write the failing tests.**

```python
# backend/tests/test_ros_scene.py
import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    "scene", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "scene.py")
scene = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scene)


def box(**kw):
    return {"name": "part_a", "shape": "box", "size": [0.05, 0.05, 0.05], "xyz": [0.5, 0, 0.025], **kw}


def test_defaults_fill_in_and_mass_comes_from_wood_density():
    (o,) = scene.validate([box()], "SCENE")
    assert o["rpy"] == [0.0, 0.0, 0.0] and o["fixed"] is False and o["friction"] is None
    assert o["mass"] == pytest.approx(700 * 0.05 ** 3)


def test_cylinder_volume_uses_radius_and_height():
    (o,) = scene.validate([{"name": "can", "shape": "cylinder", "size": [0.03, 0.1], "xyz": [0, 0, 0.05]}], "SCENE")
    assert o["mass"] == pytest.approx(700 * 3.141592653589793 * 0.03 ** 2 * 0.1)


@pytest.mark.parametrize("bad, words", [
    (box(colour="red"), "unknown field 'colour'"),
    (box(shape="sphere"), "shape must be box or cylinder"),
    (box(size=[0.05, 0, 0.05]), "size must be 3 numbers > 0"),
    (box(mass=0), "mass must be > 0"),
    (box(fixed=True, mass=1.0), "fixed objects have no mass"),
    (box(name="2fast"), "name must start with a letter"),
    (box(name="a b"), "name must start with a letter"),
    ({"shape": "box"}, "missing 'name'"),
])
def test_bad_objects_say_what_is_wrong(bad, words):
    with pytest.raises(scene.SceneError, match=words):
        scene.validate([bad], "SCENE")


def test_merge_rejects_duplicate_names_and_keeps_workcell_first():
    table = {"name": "table", "shape": "box", "size": [1, 1, 0.05], "xyz": [0.6, 0, 0.3], "fixed": True}
    out = scene.merge(scene.validate([table], "workcell.yaml"), scene.validate([box()], "SCENE"))
    assert [o["name"] for o in out] == ["table", "part_a"]
    with pytest.raises(scene.SceneError, match="'part_a' is in both workcell.yaml and SCENE"):
        scene.merge(scene.validate([box()], "workcell.yaml"), scene.validate([box()], "SCENE"))


def test_scene_reads_dict_calls_and_literals_without_running_code():
    src = ("import os\nos.system('boom')\n"
           "SCENE = [dict(name='a', shape='box', size=(0.05, 0.05, 0.05), xyz=(0.5, 0, 0.025), mass=0.2),\n"
           "         {'name': 'b', 'shape': 'cylinder', 'size': [0.02, 0.1], 'xyz': [0.4, 0.1, 0.05]}]\n")
    objs = scene.read_scene(src)
    assert [o["name"] for o in objs] == ["a", "b"] and objs[0]["size"] == [0.05, 0.05, 0.05]


def test_scene_with_computed_values_names_the_line():
    with pytest.raises(scene.SceneError, match=r"SCENE must be plain values \(line 2\)"):
        scene.read_scene("x = 0.5\nSCENE = [dict(name='a', shape='box', size=(x, x, x), xyz=(0, 0, 0))]\n")


def test_no_scene_and_empty_workcell_mean_no_objects():
    assert scene.read_scene("from robot import Robot\nRobot().home()\n") == []
    assert scene.load_workcell(None) == [] and scene.load_workcell("") == [] and scene.load_workcell("# none\n") == []


def test_workcell_must_be_a_list():
    with pytest.raises(scene.SceneError, match="workcell.yaml must be a list of objects"):
        scene.load_workcell("table: {}\n")
```

- [ ] **Step 2: Run the tests and check they fail.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_scene.py`
Expected: FAIL, because `scene.py` doesn't exist.

- [ ] **Step 3: Implement.**

```python
# backend/app/ros/templates/scene.py
"""Objects for a run: workcell.yaml plus the program's SCENE, validated, merged and turned into
the MuJoCo scene. Pure Python + mujoco, so the backend tests run it without ROS."""
import ast
import math
import re

import yaml

FIELDS = {"name", "shape", "size", "xyz", "rpy", "fixed", "mass", "friction"}
DENSITY = 700.0  # kg/m³ (wood) when an object has no mass
NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


class SceneError(Exception):
    """A problem with the declared objects; the run fails before the sim starts."""


def _numbers(v, n, positive=False) -> bool:
    return (isinstance(v, (list, tuple)) and len(v) == n
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)
            and (not positive or all(x > 0 for x in v)))


def _one(o, where: str) -> dict:
    if not isinstance(o, dict):
        raise SceneError(f"{where}: every object must be a mapping of fields")
    if "name" not in o:
        raise SceneError(f"{where}: an object is missing 'name'")
    name = o["name"]
    if not isinstance(name, str) or not NAME.match(name):
        raise SceneError(f"{where}: '{name}': name must start with a letter and use only letters, digits and _")
    for k in o:
        if k not in FIELDS:
            raise SceneError(f"{where}: '{name}': unknown field '{k}'")
    shape = o.get("shape")
    if shape not in ("box", "cylinder"):
        raise SceneError(f"{where}: '{name}': shape must be box or cylinder")
    n = 3 if shape == "box" else 2
    if not _numbers(o.get("size"), n, positive=True):
        raise SceneError(f"{where}: '{name}': size must be {n} numbers > 0"
                         + ("" if shape == "box" else " (radius, height)"))
    if not _numbers(o.get("xyz"), 3):
        raise SceneError(f"{where}: '{name}': xyz must be 3 numbers")
    rpy = o.get("rpy", [0, 0, 0])
    if not _numbers(rpy, 3):
        raise SceneError(f"{where}: '{name}': rpy must be 3 numbers")
    fixed = o.get("fixed", False)
    if not isinstance(fixed, bool):
        raise SceneError(f"{where}: '{name}': fixed must be true or false")
    size = [float(x) for x in o["size"]]
    if fixed:
        if "mass" in o:
            raise SceneError(f"{where}: '{name}': fixed objects have no mass")
        mass = None
    elif "mass" in o:
        m = o["mass"]
        if not (isinstance(m, (int, float)) and not isinstance(m, bool) and math.isfinite(m) and m > 0):
            raise SceneError(f"{where}: '{name}': mass must be > 0 (kg)")
        mass = float(m)
    else:
        vol = size[0] * size[1] * size[2] if shape == "box" else math.pi * size[0] ** 2 * size[1]
        mass = DENSITY * vol
    fr = o.get("friction")
    if fr is not None and not (isinstance(fr, (int, float)) and not isinstance(fr, bool) and fr > 0):
        raise SceneError(f"{where}: '{name}': friction must be > 0")
    return {"name": name, "shape": shape, "size": size, "xyz": [float(x) for x in o["xyz"]],
            "rpy": [float(x) for x in rpy], "fixed": fixed, "mass": mass,
            "friction": None if fr is None else float(fr)}


def validate(objs, where: str) -> list[dict]:
    if not isinstance(objs, list):
        raise SceneError(f"{where} must be a list of objects")
    out = [_one(o, where) for o in objs]
    seen = set()
    for o in out:
        if o["name"] in seen:
            raise SceneError(f"{where}: '{o['name']}' is declared twice")
        seen.add(o["name"])
    return out


def merge(workcell: list[dict], program: list[dict]) -> list[dict]:
    names = {o["name"] for o in workcell}
    for o in program:
        if o["name"] in names:
            raise SceneError(f"'{o['name']}' is in both workcell.yaml and SCENE; rename one")
    return workcell + program


def _plain(node):
    """Literal values, plus dict(key=literal, ...) calls; anything computed raises ValueError."""
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "dict" and not node.args:
        return {k.arg: _plain(k.value) for k in node.keywords if k.arg is not None}
    if isinstance(node, (ast.List, ast.Tuple)):
        return [_plain(e) for e in node.elts]
    if isinstance(node, ast.Dict):
        return {_plain(k): _plain(v) for k, v in zip(node.keys, node.values)}
    return ast.literal_eval(node)


def read_scene(source: str) -> list[dict]:
    """The program's top-level SCENE = [...] as raw objects (validate separately); [] if absent."""
    for stmt in ast.parse(source).body:
        if (isinstance(stmt, ast.Assign) and len(stmt.targets) == 1
                and isinstance(stmt.targets[0], ast.Name) and stmt.targets[0].id == "SCENE"):
            try:
                return _plain(stmt.value)
            except (ValueError, TypeError, SyntaxError):
                raise SceneError(f"SCENE must be plain values (line {stmt.lineno})")
    return []


def load_workcell(text: str | None) -> list[dict]:
    data = yaml.safe_load(text) if text else None
    if data is None:
        return []
    if not isinstance(data, list):
        raise SceneError("workcell.yaml must be a list of objects (each starting with '- name: ...')")
    return data
```

- [ ] **Step 4: Run the tests and check they pass.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_scene.py`
Expected: PASS (all).

- [ ] **Step 5: Commit.**

```bash
git add backend/app/ros/templates/scene.py backend/tests/test_ros_scene.py
git commit -m "feat: run objects: validation, workcell + SCENE merge, SCENE read without running code"
```

---

### Task 3: `scene.xml` generation and the physics it must give

**Files:**
- Modify: `backend/app/ros/templates/scene.py` (append)
- Test: `backend/tests/test_ros_scene_physics.py`

**Interfaces:**
- Consumes: `validate`, `merge` (Task 2); the exported MJCF text (`<robot>/mjcf/<robot>.xml`, meshes under `<robot>/meshes/`).
- Produces:
  - `build_scene_xml(robot_mjcf: str, objs: list[dict], tool: str, meshdir: str) -> str`. The objects are bodies named after the object, with a free joint named after the object (fixed objects get none), and each movable one has a weld `glue_<name>` from `tool` to it, inactive.
  - `check_start(xml: str, meshes: dict[str, bytes] | None, robot_bodies: set[str]) -> None`: raises `SceneError` on penetration deeper than 1 mm (object–robot or object–object) and on names that clash with robot bodies.
  - `meta(robot_mjcf: str, objs) -> dict`, returning `{"efforts": {joint: N·m}, "masses": {name: kg}, "fixed": [names]}`.

- [ ] **Step 1: Write the failing tests.**

They use the existing two-joint test robot: `tests.ros_helpers.two_joint_zip`, robot `bot`, tool link `hand`. Its export includes `bot/mjcf/bot.xml` when MuJoCo accepts it.

```python
# backend/tests/test_ros_scene_physics.py
import importlib.util
import zipfile
from pathlib import Path

import mujoco
import numpy as np
import pytest

from tests.ros_helpers import two_joint_zip

_spec = importlib.util.spec_from_file_location(
    "scene", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "scene.py")
scene = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scene)


@pytest.fixture
def bot(tmp_path):
    z = zipfile.ZipFile(two_joint_zip(tmp_path))
    z.extractall(tmp_path / "x")
    root = tmp_path / "x" / "bot"
    return (root / "mjcf" / "bot.xml").read_text(), str(root / "meshes")


def _model(xml):
    return mujoco.MjModel.from_xml_string(xml)


def _robot_bodies(mjcf):  # what sim_ctl passes: every robot body plus the world (floor)
    import xml.etree.ElementTree as ET
    return {b.get("name") for b in ET.fromstring(mjcf).iter("body")} | {"world"}


def _far_box(**kw):  # well away from the 2-joint arm
    return {"name": "part_a", "shape": "box", "size": [0.05, 0.05, 0.05], "xyz": [2.0, 2.0, 0.025], **kw}


def test_no_objects_is_just_the_robot(bot):
    m = _model(scene.build_scene_xml(bot[0], [], "hand", bot[1]))
    assert m.neq == 0 and mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hand") >= 0


def test_box_has_its_mass_free_joint_and_an_inactive_glue(bot):
    objs = scene.validate([_far_box(mass=0.2)], "SCENE")
    m = _model(scene.build_scene_xml(bot[0], objs, "hand", bot[1]))
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "part_a")
    assert m.body_mass[b] == pytest.approx(0.2)
    assert m.jnt_type[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "part_a")] == mujoco.mjtJoint.mjJNT_FREE
    e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "glue_part_a")
    assert e >= 0 and m.eq_active0[e] == 0


def test_fixed_object_has_no_joint_and_no_glue(bot):
    objs = scene.validate([{"name": "table", "shape": "box", "size": [0.5, 0.5, 0.05],
                            "xyz": [2.0, 2.0, 0.025], "fixed": True}], "workcell.yaml")
    m = _model(scene.build_scene_xml(bot[0], objs, "hand", bot[1]))
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "table") == -1 and m.neq == 0


def test_dropped_box_lands_on_the_floor_and_settles(bot):
    objs = scene.validate([_far_box(xyz=[2.0, 2.0, 0.3])], "SCENE")
    m = _model(scene.build_scene_xml(bot[0], objs, "hand", bot[1]))
    d = mujoco.MjData(m)
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    z = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "part_a")][2]
    assert z == pytest.approx(0.025, abs=0.003)


def test_box_resting_on_table_is_not_an_overlap(bot):
    objs = scene.validate([
        {"name": "table", "shape": "box", "size": [0.5, 0.5, 0.05], "xyz": [2.0, 2.0, 0.025], "fixed": True},
        _far_box(xyz=[2.0, 2.0, 0.075])], "SCENE")
    scene.check_start(scene.build_scene_xml(bot[0], objs, "hand", bot[1]), None, _robot_bodies(bot[0]))


def test_box_inside_another_box_is_an_overlap(bot):
    objs = scene.validate([_far_box(), _far_box(name="part_b", xyz=[2.01, 2.0, 0.025])], "SCENE")
    with pytest.raises(scene.SceneError, match="'part_a' and 'part_b' overlap"):
        scene.check_start(scene.build_scene_xml(bot[0], objs, "hand", bot[1]), None, _robot_bodies(bot[0]))


def test_box_inside_the_floor_is_an_overlap(bot):
    objs = scene.validate([_far_box(xyz=[2.0, 2.0, 0.0])], "SCENE")
    with pytest.raises(scene.SceneError, match="'part_a' and 'world' overlap"):
        scene.check_start(scene.build_scene_xml(bot[0], objs, "hand", bot[1]), None, _robot_bodies(bot[0]))


def test_object_named_like_a_robot_link_is_rejected(bot):
    objs = scene.validate([_far_box(name="hand")], "SCENE")
    with pytest.raises(scene.SceneError, match="'hand' is the name of a robot link"):
        scene.build_scene_xml(bot[0], objs, "hand", bot[1])


def test_glued_box_follows_the_tool_and_released_box_falls(bot):
    m_xml = scene.build_scene_xml(bot[0], scene.validate([_far_box()], "SCENE"), "hand", bot[1])
    m = _model(m_xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "glue_part_a")
    scene.glue_on(m, d, e)                         # the same maths the C++ plugin uses
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "part_a")
    start = d.xpos[b].copy()
    for _ in range(int(0.5 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    assert np.linalg.norm(d.xpos[b] - start) < 0.01   # held where it was when glued: no jump, no fall
    d.eq_active[e] = 0
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    assert d.xpos[b][2] == pytest.approx(0.025, abs=0.003)


def test_reset_puts_objects_back_and_glue_off(bot):
    m = _model(scene.build_scene_xml(bot[0], scene.validate([_far_box(xyz=[2.0, 2.0, 0.3])], "SCENE"), "hand", bot[1]))
    d = mujoco.MjData(m)
    e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "glue_part_a")
    d.eq_active[e] = 1
    for _ in range(200):
        mujoco.mj_step(m, d)
    mujoco.mj_resetData(m, d)
    assert d.eq_active[e] == 0
    assert d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "part_a")] + 2] == pytest.approx(0.3)


def test_meta_has_motor_limits_and_masses(bot):
    objs = scene.validate([_far_box(mass=0.2)], "SCENE")
    info = scene.meta(bot[0], objs)
    assert info["masses"] == {"part_a": 0.2} and all(v > 0 for v in info["efforts"].values())
```

- [ ] **Step 2: Run the tests and check they fail.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_scene_physics.py`
Expected: FAIL, with `AttributeError: module 'scene' has no attribute 'build_scene_xml'`.

- [ ] **Step 3: Implement (append to `scene.py`).**

```python
import xml.etree.ElementTree as ET

import numpy as np

PENETRATION = 0.001  # m: resting contact is fine, deeper overlap is a setup mistake


def _quat(rpy) -> list[float]:
    r, p, y = (v / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def _fmt(v) -> str:
    return " ".join(repr(float(x)) for x in v)


def build_scene_xml(robot_mjcf: str, objs: list[dict], tool: str, meshdir: str) -> str:
    root = ET.fromstring(robot_mjcf)
    robot_bodies = {b.get("name") for b in root.iter("body")}
    for o in objs:
        if o["name"] in robot_bodies:
            raise SceneError(f"'{o['name']}' is the name of a robot link; rename the object")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", meshdir)
    world = root.find("worldbody")
    if world.find("geom[@name='floor']") is None:
        ET.SubElement(world, "geom", {"name": "floor", "type": "plane", "size": "0 0 1"})
    eq = None
    for o in objs:
        body = ET.SubElement(world, "body", {"name": o["name"], "pos": _fmt(o["xyz"]), "quat": _fmt(_quat(o["rpy"]))})
        if not o["fixed"]:
            ET.SubElement(body, "freejoint", {"name": o["name"]})
        half = [s / 2 for s in o["size"]] if o["shape"] == "box" else [o["size"][0], o["size"][1] / 2]
        g = {"type": o["shape"], "size": _fmt(half)}
        if o["mass"] is not None:
            g["mass"] = repr(o["mass"])
        if o["friction"] is not None:
            g["friction"] = f"{o['friction']!r} 0.005 0.0001"
        ET.SubElement(body, "geom", g)
        if not o["fixed"]:
            if eq is None:  # not `find() or ...`: an Element with no children is falsy
                eq = root.find("equality")
                if eq is None:
                    eq = ET.SubElement(root, "equality")
            ET.SubElement(eq, "weld", {"name": f"glue_{o['name']}", "body1": tool, "body2": o["name"],
                                       "active": "false"})
    return ET.tostring(root, encoding="unicode")


def glue_on(m, d, e: int) -> None:
    """Switch weld e on, holding body2 where it is now relative to body1 (no jump).
    Mirrors glue_plugin.cpp: eq_data = [anchor(3), relpos(3), relquat(4), torquescale]."""
    import mujoco
    b1, b2 = m.eq_obj1id[e], m.eq_obj2id[e]
    q1inv = np.zeros(4)
    mujoco.mju_negQuat(q1inv, d.xquat[b1])
    relpos = np.zeros(3)
    mujoco.mju_rotVecQuat(relpos, d.xpos[b2] - d.xpos[b1], q1inv)
    relquat = np.zeros(4)
    mujoco.mju_mulQuat(relquat, q1inv, d.xquat[b2])
    m.eq_data[e, 0:3] = 0.0
    m.eq_data[e, 3:6] = relpos
    m.eq_data[e, 6:10] = relquat
    d.eq_active[e] = 1


def check_start(xml: str, meshes, robot_bodies: set[str]) -> None:
    import mujoco
    m = mujoco.MjModel.from_xml_string(xml, meshes or {})
    d = mujoco.MjData(m)
    for j in range(m.njnt):  # home: 0 clamped into the limits, like the hold check
        if m.jnt_limited[j] and m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            a = m.jnt_qposadr[j]
            d.qpos[a] = min(max(0.0, m.jnt_range[j][0]), m.jnt_range[j][1])
    mujoco.mj_forward(m, d)
    for i in range(d.ncon):
        c = d.contact[i]
        if c.dist >= -PENETRATION:
            continue
        a, b = (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "world" for g in (c.geom1, c.geom2))
        if a in robot_bodies and b in robot_bodies:
            continue  # built into the robot; the export already excludes these pairs
        a, b = sorted((a, b))
        raise SceneError(f"'{a}' and '{b}' overlap by {-c.dist * 1000:.0f} mm at the start; move one of them")


def meta(robot_mjcf: str, objs) -> dict:
    root = ET.fromstring(robot_mjcf)
    efforts = {a.get("joint"): float(a.get("forcerange").split()[1])
               for a in root.iter("position") if a.get("forcerange")}
    return {"efforts": efforts, "masses": {o["name"]: o["mass"] for o in objs if not o["fixed"]},
            "fixed": [o["name"] for o in objs if o["fixed"]]}
```

Notes for the implementer:
- If the robot MJCF lacks a `floor`, one is added. Objects at z ≈ half-height rest on it.
- `glue_on` writes `m.eq_data` from Python. The C++ plugin does the same through a non-const model pointer (Task 5).
- `robot_bodies` is every robot body plus `"world"`. A contact is skipped only when both sides are in it (robot vs robot, or robot vs floor), so object vs floor and object vs robot are still checked.
- `sorted((a, b))` puts the names in alphabetical order, so a box sunk into the floor reports `'part_a' and 'world' overlap`.

- [ ] **Step 4: Run the tests and check they pass.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_scene.py tests/test_ros_scene_physics.py`
Expected: PASS. If `test_glued_box_follows_the_tool…` fails because the box sags under the weld, raise the weld's `solref` stiffness (`"solref": "0.005 1"` on the `<weld>`). Record that the same value goes into the C++ plugin's scene.

- [ ] **Step 5: Commit.**

```bash
git add backend/app/ros/templates/scene.py backend/tests/test_ros_scene_physics.py
git commit -m "feat: scene.xml with physical objects, inactive tool glue, start overlap check"
```

---

### Task 4: MoveIt config: `mujoco` hardware, tracking tolerances, launch without the control node

**Files:**
- Modify: `backend/app/ros/moveit_config.py` (`_controllers_yaml`, `_ros2_control_xacro`, `_urdf_xacro`, `_launch`, `_package_xml`)
- Test: `backend/tests/test_ros_moveit_config.py` (append)

**Interfaces:**
- Produces:
  - xacro args `hardware` (`mock` | `mujoco`) and `mujoco_model` (path).
  - `demo.launch.py` reads `/tmp/sim/hardware` (written by `sim_ctl`). With `mujoco`, it does not start `ros2_control_node` or the spawners; `sim_ctl` owns them.
  - The `arm_controller` gains tolerances.

- [ ] **Step 1: Write the failing tests.** Append:

```python
def test_mujoco_hardware_branch_and_tolerances(tmp_path):
    from app.ros.moveit_config import moveit_files
    from app.ros.chain import parse_chain
    from tests.ros_helpers import two_joint_urdf
    files = moveit_files(parse_chain(two_joint_urdf()), "bot", "src", [])
    xacro = files["config/bot.ros2_control.xacro"]
    assert "hardware == 'mujoco'" in xacro and "<<from Task 1 (a)>>" not in xacro
    assert '<param name="mujoco_model">${mujoco_model}</param>' in xacro
    assert '<state_interface name="effort"/>' in xacro
    ctl = files["config/ros2_controllers.yaml"]
    assert "constraints:" in ctl and "trajectory: 0.1" in ctl and "goal: 0.02" in ctl
    launch = files["launch/demo.launch.py"]
    assert "/tmp/sim/hardware" in launch and 'hardware:=' not in launch
```

If `tests/ros_helpers.py` has no `two_joint_urdf()`, add one returning the URDF text inside `two_joint_zip` (read it from the zip, so there's no new fixture data).

- [ ] **Step 2: Run the tests and check they fail.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_moveit_config.py -k mujoco`
Expected: FAIL.

- [ ] **Step 3: Implement.**

In `_controllers_yaml`, add under `arm_controller: ros__parameters:`:

```yaml
    state_interfaces: [position, velocity]
    constraints:
      stopped_velocity_tolerance: 0.05
      goal_time: 1.0
{per_joint}
```

where `per_joint` is built in Python, one block per joint name:

```python
per_joint = "\n".join(f"      {n}:\n        trajectory: 0.1\n        goal: 0.02" for n in names.split(", "))
```

`_controllers_yaml` gets `names` as `", ".join(...)`; split it back as above. 0.1 rad is about 6° of lag before an abort, and 0.02 rad is the final accuracy.

In `_ros2_control_xacro`:
- the macro params become `name initial_positions_file hardware mujoco_model`
- each joint adds `<state_interface name="effort"/>`
- add the branch:

```xml
        <xacro:if value="${{hardware == 'mujoco'}}">
          <plugin><<from Task 1 (a)>></plugin>
          <param name="mujoco_model">${{mujoco_model}}</param>
          <param name="headless">true</param>
          <param name="sim_speed_factor">1.0</param>
          <<from Task 1 (d): free-joint publisher plugin + cad_to_urdf_glue/GluePlugin entries>>
        </xacro:if>
```

Delete the old `<!-- hardware:=mock for now … -->` comment.

In `_urdf_xacro`, add `<xacro:arg name="mujoco_model" default="/tmp/sim/scene.xml"/>`, and pass `mujoco_model="$(arg mujoco_model)"` to the macro.

`_launch` becomes:

```python
def _launch(robot: str, note: str) -> str:
    return note + f'''"""MoveIt headless. Hardware comes from /tmp/sim/hardware (written by sim_ctl):
mock = ros2_control runs here; mujoco = sim_ctl runs ros2_control so it can restart the sim alone."""
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    flag = Path("/tmp/sim/hardware")
    hardware = flag.read_text().strip() if flag.exists() else "mock"
    cfg = (MoveItConfigsBuilder("{robot}", package_name="{robot}_moveit_config")
           .robot_description(mappings={{"hardware": hardware}}).to_moveit_configs())
    controllers = str(cfg.package_path / "config" / "ros2_controllers.yaml")
    nodes = [Node(package="robot_state_publisher", executable="robot_state_publisher",
                  parameters=[cfg.robot_description]),
             Node(package="moveit_ros_move_group", executable="move_group", parameters=[cfg.to_dict()])]
    if hardware == "mock":
        nodes += [Node(package="controller_manager", executable="ros2_control_node",
                       parameters=[controllers], remappings=[("~/robot_description", "/robot_description")]),
                  Node(package="controller_manager", executable="spawner",
                       arguments=["joint_state_broadcaster", "arm_controller"])]
    return LaunchDescription(nodes)
'''
```

In `_package_xml`, add `mujoco_ros2_control` and `cad_to_urdf_glue` to `deps`. They're `exec_depend` only, so mock-only images still build, because rosdep isn't used.

- [ ] **Step 4: Run the tests and check they pass.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_moveit_config.py`
Expected: PASS, the existing tests included. Update any assertion that pinned the old launch text so it checks the new behaviour, not the old string.

- [ ] **Step 5: Commit.**

```bash
git add backend/app/ros/moveit_config.py backend/tests/test_ros_moveit_config.py backend/tests/ros_helpers.py
git commit -m "feat: MoveIt config gains mujoco hardware, tracking tolerances, sim-owned control node"
```

---

### Task 5: Glue plugin package (`cad_to_urdf_glue`)

**Files:**
- Create: `backend/app/ros/templates/glue/package.xml`, `CMakeLists.txt`, `plugins.xml`, `src/glue_plugin.cpp`
- Modify: `backend/app/ros/program_pkg.py`: `program_files` gains a sibling `glue_files() -> dict[str, str]`, and `moveit_config.generate` writes them to `cad_to_urdf_glue/`
- Test: `backend/tests/test_ros_program_pkg.py` (append)

**Interfaces:**
- Produces:
  - Topic `/sim/glue` (`std_msgs/String`, `"attach <name>"` | `"detach <name>"`).
  - Topic `/sim/glued` (`std_msgs/String`, comma-joined glued names, transient-local QoS), published after each change and after reset.
  - Weld names are `glue_<name>` (Task 3).

- [ ] **Step 1: Write the failing test.**

```python
def test_glue_package_is_generated(ws):
    glue = ws / "cad_to_urdf_glue"
    assert (glue / "src" / "glue_plugin.cpp").is_file() and (glue / "plugins.xml").is_file()
    cpp = (glue / "src" / "glue_plugin.cpp").read_text()
    assert "MuJoCoROS2ControlPluginBase" in cpp and "/sim/glue" in cpp and "eq_active" in cpp
```

- [ ] **Step 2: Run the test and check it fails.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_program_pkg.py -k glue`
Expected: FAIL.

- [ ] **Step 3: Implement.**

`src/glue_plugin.cpp`:

```cpp
// Generated by cad-to-urdf. attach/detach for objects until grippers exist: one weld per object
// ("glue_<name>", inactive in scene.xml), switched on holding the object where it is (no jump).
// Threads: pre_step/on_reset run on the physics thread, update/publish on the control thread;
// mu_ guards pending_ and glued_.
#include <atomic>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace cad_to_urdf_glue {

class GluePlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
 public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData*) override {
    // ponytail: writes eq_data through a cast-away const model; MuJoCo reads it every step.
    // Upgrade if the plugin API ever hands out a mutable model.
    model_ = const_cast<mjModel*>(model);
    node_ = node;
    auto qos = rclcpp::QoS(1).transient_local();
    glued_pub_ = node->create_publisher<std_msgs::msg::String>("/sim/glued", qos);
    sub_ = node->create_subscription<std_msgs::msg::String>(
        "/sim/glue", 10, [this](std_msgs::msg::String::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mu_);
          pending_.push_back(m->data);
        });
    publish();
    return true;
  }

  void pre_step(mjData* d) override {
    std::vector<std::string> todo;
    {
      std::lock_guard<std::mutex> lk(mu_);
      todo.swap(pending_);
    }
    for (const auto& cmd : todo) {
      auto sp = cmd.find(' ');
      if (sp == std::string::npos) continue;
      std::string op = cmd.substr(0, sp), name = cmd.substr(sp + 1);
      int e = mj_name2id(model_, mjOBJ_EQUALITY, ("glue_" + name).c_str());
      if (e < 0) continue;
      if (op == "attach") {
        int b1 = model_->eq_obj1id[e], b2 = model_->eq_obj2id[e];
        mjtNum q1inv[4], rel[3], dp[3], rq[4];
        mju_negQuat(q1inv, d->xquat + 4 * b1);
        mju_sub3(dp, d->xpos + 3 * b2, d->xpos + 3 * b1);
        mju_rotVecQuat(rel, dp, q1inv);
        mju_mulQuat(rq, q1inv, d->xquat + 4 * b2);
        mjtNum* ed = model_->eq_data + mjNEQDATA * e;
        mju_zero3(ed);
        mju_copy3(ed + 3, rel);
        mju_copy4(ed + 6, rq);
        d->eq_active[e] = 1;
        std::lock_guard<std::mutex> lk(mu_);
        glued_.insert(name);
      } else if (op == "detach") {
        d->eq_active[e] = 0;
        std::lock_guard<std::mutex> lk(mu_);
        glued_.erase(name);
      }
      dirty_ = true;
    }
  }

  void update(const mjModel*, mjData*) override {
    if (dirty_.exchange(false)) publish();
  }

  void on_reset(mjData* d) override {
    for (int e = 0; e < model_->neq; ++e) d->eq_active[e] = model_->eq_active0[e];
    {
      std::lock_guard<std::mutex> lk(mu_);
      glued_.clear();
      pending_.clear();
    }
    dirty_ = true;
  }

  void cleanup() override {}

 private:
  void publish() {
    std_msgs::msg::String m;
    {
      std::lock_guard<std::mutex> lk(mu_);
      for (const auto& n : glued_) m.data += (m.data.empty() ? "" : ",") + n;
    }
    glued_pub_->publish(m);
  }
  mjModel* model_ = nullptr;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr glued_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::mutex mu_;
  std::vector<std::string> pending_;
  std::set<std::string> glued_;
  std::atomic<bool> dirty_{false};
};

}  // namespace cad_to_urdf_glue

PLUGINLIB_EXPORT_CLASS(cad_to_urdf_glue::GluePlugin, mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
```

`CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(cad_to_urdf_glue)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(mujoco_vendor REQUIRED)  # <<from Task 1: the package that provides mujoco/mujoco.h>>
add_library(${PROJECT_NAME} SHARED src/glue_plugin.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs pluginlib mujoco_ros2_control_plugins mujoco_vendor)
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugins.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
ament_package()
```

`plugins.xml`:

```xml
<library path="cad_to_urdf_glue">
  <class type="cad_to_urdf_glue::GluePlugin"
         base_class_type="mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase">
    <description>attach/detach objects by toggling welds</description>
  </class>
</library>
```

`package.xml`: format 3, name `cad_to_urdf_glue`, with `build_depend`/`exec_depend` on `rclcpp`, `std_msgs`, `pluginlib`, `mujoco_ros2_control_plugins`, `mujoco_vendor`, and `buildtool_depend` on `ament_cmake`.

In `program_pkg.py`:

```python
def glue_files() -> dict[str, str]:
    d = HERE / "templates" / "glue"
    return {str(p.relative_to(d)).replace("\\", "/"): p.read_text(encoding="utf-8")
            for p in d.rglob("*") if p.is_file()}
```

In `moveit_config.generate`, next to the program package:

```python
        from .program_pkg import glue_files
        write_files(tmp / "cad_to_urdf_glue", glue_files())
```

- [ ] **Step 4: Run the test and check it passes; check the C++ builds in the container.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_program_pkg.py`
Expected: PASS.

Then regenerate the irb1300 workspace (start a session in the app, or `python -m app.ros.cli` as the existing tests do), and run `docker run --rm -v ~/.cad-to-urdf/workspaces/irb1300:/ws cad-to-urdf-jazzy bash -c "source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --packages-select cad_to_urdf_glue"`.
Expected: `Finished <<< cad_to_urdf_glue`.

- [ ] **Step 5: Commit.**

```bash
git add backend/app/ros/templates/glue backend/app/ros/program_pkg.py backend/app/ros/moveit_config.py backend/tests/test_ros_program_pkg.py
git commit -m "feat: cad_to_urdf_glue MuJoCo plugin: attach/detach toggles a weld without a jump"
```

---

### Task 6: `sim_ctl`: boot, restart and reset the sim per scene

**Files:**
- Create: `backend/app/ros/templates/sim_ctl.py`
- Modify: `backend/app/ros/program_pkg.py` (`program_files` writes `scene.py`, `sim_ctl.py`, `sim_errors.py`, `scene_sync.py`; `generate` creates an empty `workcell.yaml` if missing)
- Modify: `docker/jazzy/session/start.sh`
- Test: `backend/tests/test_ros_sim_ctl.py`

**Interfaces:**
- Consumes: `scene.load_workcell`, `read_scene`, `validate`, `merge`, `build_scene_xml`, `check_start`, `meta` (Tasks 2–3).
- Produces:
  - `plan(ws_src: Path, robot: str, tool: str, program_source: str | None) -> tuple[str, dict]`, returning `(scene_xml, meta)`. Pure; raises `SceneError`.
  - `decide(new_xml: str, loaded_path: Path) -> str`: `"reset"` if `loaded_path` holds identical text, else `"restart"`.
  - `prepare(program: str | None) -> None`: container only. Writes `/tmp/sim/scene.xml` and `/tmp/sim/meta.json`, then resets or restarts. Raises `SimError(message)` on timeout, with the last 20 log lines.
  - CLI: `python3 -m sim_ctl boot`, run by `start.sh`. It writes `/tmp/sim/hardware` (`mujoco` or `mock`) and `/tmp/physics` (`mujoco` or `off: <reason>`), then starts the sim when the hardware is `mujoco`.

- [ ] **Step 1: Write the failing tests (the pure part).**

```python
# backend/tests/test_ros_sim_ctl.py
import importlib.util
import sys
import zipfile
from pathlib import Path

import pytest

from tests.ros_helpers import two_joint_zip

T = Path(__file__).resolve().parents[1] / "app" / "ros" / "templates"
sys.path.insert(0, str(T))  # sim_ctl imports scene as a sibling module, as in the container
sim_ctl = importlib.import_module("sim_ctl")


@pytest.fixture
def src(tmp_path):
    zipfile.ZipFile(two_joint_zip(tmp_path)).extractall(tmp_path / "src")
    (tmp_path / "src" / "bot_tasks").mkdir()
    return tmp_path / "src"


def test_plan_merges_workcell_and_program_scene(src):
    (src / "bot_tasks" / "workcell.yaml").write_text(
        "- {name: table, shape: box, size: [0.4, 0.4, 0.05], xyz: [2, 2, 0.025], fixed: true}\n")
    prog = "SCENE = [dict(name='part_a', shape='box', size=(0.05, 0.05, 0.05), xyz=(2, 2, 0.075))]\n"
    xml, info = sim_ctl.plan(src, "bot", "hand", prog)
    assert 'name="table"' in xml and 'name="part_a"' in xml and info["fixed"] == ["table"]


def test_plan_without_workcell_or_scene_is_robot_only(src):
    xml, info = sim_ctl.plan(src, "bot", "hand", None)
    assert "glue_" not in xml and info["masses"] == {}


def test_plan_errors_name_the_problem(src):
    with pytest.raises(sim_ctl.scene.SceneError, match="unknown field 'colour'"):
        sim_ctl.plan(src, "bot", "hand", "SCENE = [dict(name='a', shape='box', size=(1,1,1), xyz=(5,5,1), colour=1)]\n")


def test_decide_resets_only_for_the_identical_scene(tmp_path):
    p = tmp_path / "scene.xml"
    assert sim_ctl.decide("<mujoco/>", p) == "restart"
    p.write_text("<mujoco/>")
    assert sim_ctl.decide("<mujoco/>", p) == "reset" and sim_ctl.decide("<mujoco></mujoco>", p) == "restart"
```

- [ ] **Step 2: Run the tests and check they fail.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_sim_ctl.py`
Expected: FAIL, because `sim_ctl` doesn't exist.

- [ ] **Step 3: Implement.**

```python
# backend/app/ros/templates/sim_ctl.py
"""Container side: one MuJoCo scene per run. Boots the sim at session start, then before every run
resets it (same scene) or restarts only ros2_control_node (new scene). MoveIt stays up."""
import json
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import scene

SIM = Path("/tmp/sim")
WS_SRC = Path("/ws/src")
RESTART_TIMEOUT = 30.0


class SimError(Exception):
    pass


def plan(ws_src: Path, robot: str, tool: str, program_source):
    robot_mjcf = (ws_src / robot / "mjcf" / f"{robot}.xml").read_text()
    wc_path = ws_src / f"{robot}_tasks" / "workcell.yaml"
    workcell = scene.validate(scene.load_workcell(wc_path.read_text() if wc_path.exists() else None), "workcell.yaml")
    program = scene.validate(scene.read_scene(program_source), "SCENE") if program_source else []
    objs = scene.merge(workcell, program)
    xml = scene.build_scene_xml(robot_mjcf, objs, tool, str(ws_src / robot / "meshes"))
    scene.check_start(xml, None, {b.get("name") for b in ET.fromstring(robot_mjcf).iter("body")} | {"world"})
    return xml, scene.meta(robot_mjcf, objs)


def decide(new_xml: str, loaded_path: Path) -> str:
    return "reset" if loaded_path.exists() and loaded_path.read_text() == new_xml else "restart"


def _robot_and_tool():
    robot = next(p.name[:-len("_moveit_config")] for p in WS_SRC.glob("*_moveit_config"))
    kin = __import__(f"{robot}_kinematics")
    return robot, kin.load().chain.tip


def _ros(cmd: str, timeout: float = RESTART_TIMEOUT) -> subprocess.CompletedProcess:
    return subprocess.run(["bash", "-c", f"source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && {cmd}"],
                          capture_output=True, text=True, timeout=timeout)


def _stop_sim():
    pid = SIM / "control.pid"
    if pid.exists():
        try:
            os.killpg(int(pid.read_text()), signal.SIGINT)
            for _ in range(50):
                os.killpg(int(pid.read_text()), 0)
                time.sleep(0.1)
            os.killpg(int(pid.read_text()), signal.SIGKILL)
        except (ProcessLookupError, ValueError):
            pass
        pid.unlink(missing_ok=True)


def _start_sim(robot: str):
    log = open(SIM / "control.log", "w")
    desc = (f"xacro $(ros2 pkg prefix {robot}_moveit_config)/share/{robot}_moveit_config/config/{robot}.urdf.xacro "
            f"hardware:=mujoco mujoco_model:={SIM / 'scene.xml'}")
    ctl = f"$(ros2 pkg prefix {robot}_moveit_config)/share/{robot}_moveit_config/config/ros2_controllers.yaml"
    p = subprocess.Popen(["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && "
                          f"exec ros2 run controller_manager ros2_control_node --ros-args --params-file {ctl} "
                          f"-p robot_description:=\"$({desc})\""],
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    (SIM / "control.pid").write_text(str(p.pid))
    end = time.time() + RESTART_TIMEOUT
    r = _ros("ros2 run controller_manager spawner joint_state_broadcaster arm_controller "
             f"--controller-manager-timeout {int(RESTART_TIMEOUT)}", timeout=RESTART_TIMEOUT + 5)
    if r.returncode != 0 or time.time() > end:
        tail = "\n".join((SIM / "control.log").read_text().splitlines()[-20:])
        raise SimError(f"The simulation did not start within {RESTART_TIMEOUT:.0f} s:\n{tail}")


def _reset_sim():
    r = _ros("ros2 control switch_controllers --deactivate arm_controller && "
             "ros2 service call /ros2_control_node/reset_world mujoco_ros2_control_msgs/srv/ResetWorld '{}' && "
             "ros2 control switch_controllers --activate arm_controller")
    if r.returncode != 0:
        raise SimError(f"Could not reset the simulation: {(r.stderr or r.stdout).strip().splitlines()[-1:]}")


def prepare(program):
    robot, tool = _robot_and_tool()
    source = (WS_SRC / f"{robot}_tasks" / program).read_text() if program else None
    xml, info = plan(WS_SRC, robot, tool, source)
    (SIM / "meta.json").write_text(json.dumps(info))
    if decide(xml, SIM / "scene.xml") == "reset":
        _reset_sim()
        return
    (SIM / "scene.xml").write_text(xml)
    _stop_sim()
    _start_sim(robot)


def boot():
    SIM.mkdir(parents=True, exist_ok=True)
    robot, _ = _robot_and_tool()
    if not (WS_SRC / robot / "mjcf" / f"{robot}.xml").exists():
        (SIM / "hardware").write_text("mock")
        Path("/tmp/physics").write_text("off: the export has no MuJoCo model (see the Physics card in Simulate)")
        return
    (SIM / "hardware").write_text("mujoco")
    try:
        prepare(None)
        Path("/tmp/physics").write_text("mujoco")
    except (scene.SceneError, SimError) as exc:
        (SIM / "hardware").write_text("mock")
        Path("/tmp/physics").write_text(f"off: {exc}".splitlines()[0])


if __name__ == "__main__":
    {"boot": boot}[sys.argv[1]]()
```

Task 1 (f) and (g) decide whether `_reset_sim` needs the deactivate/activate pair. Keep it unless Task 1 shows `reset_world` is safe without it.

In `program_pkg.program_files`, add:

```python
        "scene.py": _template("scene.py", robot, source),
        "sim_ctl.py": _template("sim_ctl.py", robot, source),
        "sim_errors.py": _template("sim_errors.py", robot, source),
        "scene_sync.py": _template("scene_sync.py", robot, source),
```

In `moveit_config.generate`, after the `programs` mkdir:

```python
        wc = Path(out_src) / f"{robot}_tasks" / "workcell.yaml"
        if not wc.exists():
            wc.write_text("# Objects every program runs with (table, fixtures, parts). Same format as SCENE.\n"
                          "# - {name: table, shape: box, size: [1.2, 0.8, 0.05], xyz: [0.6, 0, 0.375], fixed: true}\n")
```

`start.sh`: after `source /ws/install/setup.bash` and the `ROBOT=` line, insert:

```bash
echo physics > /tmp/stage
export PYTHONPATH=/ws/src/${ROBOT}_program:$PYTHONPATH
python3 -m sim_ctl boot || echo "off: sim boot crashed" > /tmp/physics
```

After the MoveIt launch line, add `python3 -m scene_sync &` (Task 7).

- [ ] **Step 4: Run the tests and check they pass.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_sim_ctl.py tests/test_ros_program_pkg.py`
Expected: PASS. The test for the generated file list in `test_program_folder_and_tasks_folder_are_generated` still passes; extend it to also expect `scene.py` and `sim_ctl.py`, and `bot_tasks/workcell.yaml`.

- [ ] **Step 5: Commit.**

```bash
git add backend/app/ros/templates/sim_ctl.py backend/app/ros/program_pkg.py backend/app/ros/moveit_config.py docker/jazzy/session/start.sh backend/tests/test_ros_sim_ctl.py backend/tests/test_ros_program_pkg.py
git commit -m "feat: sim_ctl boots the MuJoCo sim and resets or restarts it per run's scene"
```

---

### Task 7: `scene_sync`: real object poses into MoveIt

**Files:**
- Create: `backend/app/ros/templates/scene_sync.py`
- Test: `backend/tests/test_ros_scene_sync.py` (pure core only)

**Interfaces:**
- Consumes: the free-joint pose topic and message (`<<from Task 1 (b), (c)>>`), `/tmp/sim/meta.json` (Task 6), and `/sim/glued` (Task 5).
- Produces: `moved(poses: dict[str, tuple], last: dict[str, tuple], skip: set[str], tol: float = 0.002) -> dict[str, tuple]`. Poses are `(x, y, z, qx, qy, qz, qw)`; the result is the objects to update.

- [ ] **Step 1: Write the failing test.**

```python
# backend/tests/test_ros_scene_sync.py
import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "scene_sync_core", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "scene_sync.py")
core = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(core)   # scene_sync imports rclpy lazily, inside main()


def test_only_moved_and_not_carried_objects_update():
    last = {"a": (0, 0, 0, 0, 0, 0, 1), "b": (1, 0, 0, 0, 0, 0, 1)}
    now = {"a": (0, 0, 0.001, 0, 0, 0, 1), "b": (1, 0.05, 0, 0, 0, 0, 1), "c": (2, 0, 0, 0, 0, 0, 1)}
    assert core.moved(now, last, skip=set()) == {"b": now["b"], "c": now["c"]}
    assert core.moved(now, last, skip={"b"}) == {"c": now["c"]}


def test_rotation_counts_as_moved():
    last = {"a": (0, 0, 0, 0, 0, 0, 1)}
    assert "a" in core.moved({"a": (0, 0, 0, 0, 0, 0.0998, 0.995)}, last, skip=set())
```

- [ ] **Step 2: Run the test and check it fails.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_scene_sync.py`
Expected: FAIL.

- [ ] **Step 3: Implement.**

```python
# backend/app/ros/templates/scene_sync.py
"""Container node: keeps MoveIt's planning scene on the simulated object poses, so MoveIt plans
around where objects really are and the 3D view shows them. Carried (glued) objects are skipped:
MoveIt already moves them with the tool."""
import math


def moved(poses, last, skip, tol=0.002):
    out = {}
    for name, p in poses.items():
        if name in skip:
            continue
        q = last.get(name)
        dot = abs(sum(a * b for a, b in zip(p[3:], q[3:]))) if q else 0.0
        if q is None or math.dist(p[:3], q[:3]) > tol or 2 * math.acos(min(1.0, dot)) > 0.01:
            out[name] = p
    return out


def main():
    import json
    from pathlib import Path

    import rclpy
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    from std_msgs.msg import String
    from <<from Task 1 (b): msg module>> import <<FreeJointStateArray>>

    rclpy.init()
    n = rclpy.create_node("cad_to_urdf_scene_sync")
    apply = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    state = {"last": {}, "glued": set(), "poses": {}}

    def on_glued(m):
        state["glued"] = set(filter(None, m.data.split(",")))

    def on_poses(m):
        state["poses"] = {<<per (c): name>>: (<<x, y, z, qx, qy, qz, qw from (c)>>) for s in m.<<array field>>}

    def tick():
        meta = Path("/tmp/sim/meta.json")
        movable = set(json.loads(meta.read_text())["masses"]) if meta.exists() else set()
        poses = {k: v for k, v in state["poses"].items() if k in movable}
        todo = moved(poses, state["last"], state["glued"])
        if not todo or not apply.service_is_ready():
            return
        s = PlanningScene(is_diff=True)
        for name, p in todo.items():
            co = CollisionObject(id=name, operation=CollisionObject.MOVE)
            co.header.frame_id = "world"
            (co.pose.position.x, co.pose.position.y, co.pose.position.z,
             co.pose.orientation.x, co.pose.orientation.y, co.pose.orientation.z, co.pose.orientation.w) = p
            s.world.collision_objects.append(co)
        apply.call_async(ApplyPlanningScene.Request(scene=s))
        state["last"].update(todo)

    n.create_subscription(String, "/sim/glued", on_glued, rclpy.qos.QoSProfile(
        depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(<<FreeJointStateArray>>, "<<topic from (b)>>", on_poses, 10)
    n.create_timer(0.1, tick)
    rclpy.spin(n)


if __name__ == "__main__":
    main()
```

Fill every `<<…>>` with the exact names recorded in Task 1 (b) and (c), then search the file to confirm no `<<` remains. Also check the frame: `robot.py` adds objects in `self.chain.base`. If the base isn't at the MuJoCo world origin, use `chain.base` as `frame_id` and compose the base pose. Task 1 notes whether the exported MJCF puts the base at the origin (exports so far do).

- [ ] **Step 4: Run the test and check it passes.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_scene_sync.py`
Expected: PASS.

- [ ] **Step 5: Commit.**

```bash
git add backend/app/ros/templates/scene_sync.py backend/tests/test_ros_scene_sync.py
git commit -m "feat: scene_sync mirrors simulated object poses into MoveIt"
```

---

### Task 8: Physics error messages (`sim_errors.py`)

**Files:**
- Create: `backend/app/ros/templates/sim_errors.py`
- Test: `backend/tests/test_ros_sim_errors.py`

**Interfaces:**
- Produces:
  - `lag_message(names: list[str], planned: list[float], actual: list[float], efforts: dict[str, float], carrying: dict[str, float], hinge: dict[str, bool]) -> str`
  - `blow_up(names: list[str], positions: list[float], velocities: list[float], child_link: dict[str, str], limit: float = 50.0) -> str | None`

- [ ] **Step 1: Write the failing tests.**

```python
# backend/tests/test_ros_sim_errors.py
import importlib.util
import math
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "sim_errors", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "sim_errors.py")
se = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(se)


def test_lag_names_worst_joint_load_and_limit():
    msg = se.lag_message(["j1", "j2"], [0.0, 1.0], [0.01, 1.0 - math.radians(12)],
                         {"j1": 40, "j2": 150}, {"part_a": 0.2}, {"j1": True, "j2": True})
    assert msg == ("j2 fell 12° behind while carrying part_a (0.2 kg). Motor limit 150 N·m: "
                   "raise Max torque in Simulate, or use a lighter load.")


def test_lag_without_load_and_prismatic_units():
    msg = se.lag_message(["s"], [0.5], [0.47], {"s": 80}, {}, {"s": False})
    assert msg == ("s fell 30 mm behind. Motor limit 80 N: raise Max torque in Simulate, "
                   "or check what it hit.")


def test_blow_up_names_the_link_and_ignores_normal_motion():
    assert se.blow_up(["j1", "j2"], [0.1, 0.2], [1.0, -2.0], {"j1": "l1", "j2": "l2"}) is None
    assert se.blow_up(["j1", "j2"], [0.1, float("nan")], [1.0, 0.0], {"j1": "l1", "j2": "l2"}) == \
        "Simulation blew up: check l2's mass and inertia."
    assert se.blow_up(["j1", "j2"], [0.1, 0.2], [900.0, 1.0], {"j1": "l1", "j2": "l2"}) == \
        "Simulation blew up: check l1's mass and inertia."
```

- [ ] **Step 2: Run the tests and check they fail.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_sim_errors.py`
Expected: FAIL.

- [ ] **Step 3: Implement.**

```python
# backend/app/ros/templates/sim_errors.py
"""Plain-language messages for physics failures (pure Python, tested on the host)."""
import math


def lag_message(names, planned, actual, efforts, carrying, hinge) -> str:
    i = max(range(len(names)), key=lambda k: abs(planned[k] - actual[k]))
    n, err = names[i], abs(planned[i] - actual[i])
    amount = f"{math.degrees(err):.0f}°" if hinge.get(n, True) else f"{err * 1000:.0f} mm"
    unit = "N·m" if hinge.get(n, True) else "N"
    load = "".join(f" while carrying {k} ({v:g} kg)" for k, v in carrying.items())
    fix = "or use a lighter load." if carrying else "or check what it hit."
    return f"{n} fell {amount} behind{load}. Motor limit {efforts.get(n, 0):g} {unit}: raise Max torque in Simulate, {fix}"


def blow_up(names, positions, velocities, child_link, limit=50.0):
    bad = [k for k in range(len(names)) if not math.isfinite(positions[k]) or not math.isfinite(velocities[k])]
    if not bad:
        bad = [k for k in range(len(names)) if abs(velocities[k]) > limit]
    if not bad:
        return None
    k = max(bad, key=lambda j: abs(velocities[j]) if math.isfinite(velocities[j]) else math.inf)
    return f"Simulation blew up: check {child_link.get(names[k], names[k])}'s mass and inertia."
```

- [ ] **Step 4: Run the tests and check they pass.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_sim_errors.py`
Expected: PASS.

- [ ] **Step 5: Commit.**

```bash
git add backend/app/ros/templates/sim_errors.py backend/tests/test_ros_sim_errors.py
git commit -m "feat: plain-language messages for a lagging joint and a blown-up sim"
```

---

### Task 9: `robot.py` and `runner.py`: objects, glue, `pose()`, physics errors

**Files:**
- Modify: `backend/app/ros/templates/robot.py`
- Modify: `backend/app/ros/templates/runner.py`
- Modify: `backend/app/ros/program_pkg.py` (`API_DOC`, `RULES`, `brief`)
- Test: `backend/tests/test_ros_program_pkg.py` (append)

**Interfaces:**
- Consumes: `/sim/glue` and `/sim/glued` (Task 5), `/tmp/sim/meta.json` (Task 6), `sim_errors` (Task 8), `sim_ctl.prepare` and `SimError` (Task 6), `scene.SceneError` (Task 2).
- Produces:
  - `MotionError(RobotError)`
  - `Robot.pose(name) -> (xyz, rpy)`
  - `runner.main` resets through `sim_ctl.prepare(path)` when `/tmp/sim/hardware` is `mujoco`

- [ ] **Step 1: Write the failing tests.** These are text checks on the generated files; the behaviour is covered in Task 11.

```python
def test_robot_library_speaks_physics(ws):
    lib = (ws / "bot_program" / "robot.py").read_text()
    assert "class MotionError(RobotError)" in lib and "def pose(self, name)" in lib
    assert "Objects can't appear mid-run. Declare '{name}' in SCENE at the top of the program (or in workcell.yaml)." in lib
    assert '"/sim/glue"' in lib and "lag_message(" in lib and "blow_up(" in lib
    runner = (ws / "bot_program" / "runner.py").read_text()
    assert "sim_ctl.prepare(path)" in runner


def test_brief_teaches_scene_and_pose(ws):
    text = (ws / "bot_program" / "brief.md").read_text()
    assert "SCENE = [" in text and "workcell.yaml" in text and "r.pose(" in text
    assert "r.add_box(" not in text
```

- [ ] **Step 2: Run the tests and check they fail.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_program_pkg.py -k "physics or scene_and_pose"`
Expected: FAIL.

- [ ] **Step 3: Implement `robot.py`.**

Add near the other errors:

```python
class MotionError(RobotError):
    pass
```

Add module helpers:

```python
import json
from pathlib import Path

from std_msgs.msg import String

import sim_errors

_PHYSICS = Path("/tmp/sim/hardware").exists() and Path("/tmp/sim/hardware").read_text().strip() == "mujoco"


def _meta() -> dict:
    p = Path("/tmp/sim/meta.json")
    return json.loads(p.read_text()) if p.exists() else {"efforts": {}, "masses": {}, "fixed": []}
```

In `Robot.__init__`:

```python
        self._glued = set()
        self._glue = n.create_publisher(String, "/sim/glue", 10)
        n.create_subscription(String, "/sim/glued", self._on_glued, rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self._velocities = {}
```

`_on_joints` also stores `m.velocity` into `self._velocities`.

```python
    def _on_glued(self, m):
        self._glued = set(filter(None, m.data.split(",")))

    def _glue_cmd(self, op, name):
        want = (lambda: name in self._glued) if op == "attach" else (lambda: name not in self._glued)
        self._glue.publish(String(data=f"{op} {name}"))
        end = time.time() + 2.0
        while not want() and time.time() < end:
            rclpy.spin_once(_node(), timeout_sec=0.05)
        if not want():
            raise MotionError(f"The simulation did not confirm {op} of '{name}'")
```

Replace `add_box` and `add_cylinder` when physics is on, keeping the mock behaviour otherwise:

```python
    def add_box(self, name, size_xyz, xyz, rpy=(0, 0, 0)):
        if _PHYSICS:
            raise RobotError(f"Objects can't appear mid-run. Declare '{name}' in SCENE at the top of the program "
                             "(or in workcell.yaml).")
        self._add(name, SolidPrimitive.BOX, size_xyz, xyz, rpy)
```

`add_cylinder` gets the same guard.

`attach(name)`: after the existing reach check, refuse fixed objects, then glue:

```python
        if _PHYSICS:
            if name in _meta()["fixed"]:
                raise UnknownObject(f"'{name}' is fixed (bolted down in the workcell) and can't be picked up")
            self._glue_cmd("attach", name)
        self._diff(attached=[self._attached(name, CollisionObject.ADD)])
```

`detach(name)`: before the MoveIt removal, `if _PHYSICS: self._glue_cmd("detach", name)`.

`pose(name)`:

```python
    def pose(self, name):
        """Where object `name` really is now: (xyz, rpy) in the base frame."""
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                     | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        sc = _wait(self._get.call_async(req), 10).scene
        objs = {o.id: o for o in sc.world.collision_objects}
        for a in sc.robot_state.attached_collision_objects:
            objs[a.object.id] = a.object
        if name not in objs:
            raise UnknownObject(f"No object '{name}' (objects: {', '.join(sorted(objs)) or 'none'})")
        o = objs[name]
        M = _pose_matrix(o.pose) @ _pose_matrix(o.primitive_poses[0])
        if o.header.frame_id != self.chain.base:  # attached objects are in the tool frame
            M = self._kin.fk(self._current()) @ M
        return tuple(float(v) for v in M[:3, 3]), _rpy(M[:3, :3])
```

In the execution path (around the existing `self._run(self._exec, ExecuteTrajectory.Goal(...), 120)`), after the result:

```python
        if _PHYSICS:
            names = self.joint_names
            pos = [self._joints.get(n, 0.0) for n in names]
            vel = [self._velocities.get(n, 0.0) for n in names]
            child = {j.name: j.child for j in self.chain.joints}
            boom = sim_errors.blow_up(names, pos, vel, child)
            if boom:
                raise MotionError(boom)
            if code_name == "CONTROL_FAILED":   # JointTrajectoryController tolerance tripped
                planned = list(res.solution.joint_trajectory.points[-1].positions)
                m = _meta()
                carrying = {k: m["masses"][k] for k in self._glued if k in m["masses"]}
                hinge = {j.name: j.type != "prismatic" for j in self.chain.joints}
                raise MotionError(sim_errors.lag_message(names, planned, pos, m["efforts"], carrying, hinge))
```

Use the error-code variable the surrounding code already has (it maps codes via `_ERRORS`); name it `code_name` if it isn't already. `planned` uses the last waypoint. That's a simplification: the joint that lags most at abort is also the one furthest from the goal. Mark it with `# ponytail: last waypoint, not the sample at abort time; use the timed sample if messages name the wrong joint`.

`reset()`: when `_PHYSICS`, only call `self.home()`, because the world was already reset by `sim_ctl.prepare`. Otherwise keep the current body.

- [ ] **Step 4: Implement `runner.py`.**

```python
from pathlib import Path

import scene
import sim_ctl


def main(path: str) -> int:
    try:
        if Path("/tmp/sim/hardware").exists() and Path("/tmp/sim/hardware").read_text().strip() == "mujoco":
            sim_ctl.prepare(path)
        Robot().reset()
    except scene.SceneError as exc:
        print(f"RESULT: failed: {exc}", flush=True)
        return 1
    except Exception as exc:  # the session itself is broken; nothing the program can fix
        print(f"RESULT: failed: could not reset the simulation ({exc})", flush=True)
        return 1
    ...  # rest unchanged
```

`sim_ctl.prepare` expects a path relative to `<robot>_tasks/`; the runner already runs with that cwd and gets `programs/...`.

- [ ] **Step 5: Update the brief (`program_pkg.py`).**

In `API_DOC`:
- replace the `add_box`/`add_cylinder` lines with:

```
SCENE = [dict(name, shape="box"|"cylinder", size=(x, y, z) or (radius, height), xyz, rpy=(0, 0, 0), mass=kg, fixed=False)]
                                 # at the top of the program: objects that exist when the run starts
r.pose(name)              # where an object really is now: (xyz, rpy)
```

- `detach` becomes: `# the object is released and falls under gravity`
- the pick-and-place example starts with `SCENE = [dict(name="box", shape="box", size=(0.05, 0.05, 0.05), xyz=(0.5, 0.2, 0.025), mass=0.2)]` instead of `r.add_box(...)`
- after `r.detach("box")`, add `assert abs(r.pose("box")[0][0] - 0.4) < 0.01`

In `RULES`, replace "Every run starts clean (robot at home, no objects)" with "Every run starts clean: robot at home, the workcell's objects plus your SCENE at their declared places". Then add the line: "- Physics is real: motors have limits, objects have weight, and released objects fall. Check results with r.pose() instead of assuming them."

In `brief()`, after the joints list, append the workcell objects:

```python
    lines += ["", "Workcell (always present; see workcell.yaml):"]
```

then add one line per object, `- name: shape size at xyz (fixed | mass kg)`, or `- (empty)`. `brief()` gets an extra `workcell: list[dict]` parameter. `program_files` passes `scene.validate(scene.load_workcell(text), "workcell.yaml")`, reading the file from the tasks folder when it exists. Import the template module the same way the tests do (`importlib` from `HERE / "templates" / "scene.py"`).

- [ ] **Step 6: Run the tests and check they pass.**

Run: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_ros_program_pkg.py`
Expected: PASS. Update the existing brief assertions that pinned `add_box` text.

- [ ] **Step 7: Commit.**

```bash
git add backend/app/ros/templates/robot.py backend/app/ros/templates/runner.py backend/app/ros/program_pkg.py backend/tests/test_ros_program_pkg.py
git commit -m "feat: programs declare SCENE; attach/detach glue in MuJoCo; pose(); physics errors"
```

---

### Task 10: Physics status, and `workcell.yaml` editable in the app

**Files:**
- Modify: `backend/app/program/session.py` (`status`)
- Modify: `backend/app/program/files.py` (`editable`, delete rule)
- Modify: `frontend/src/ui/program/ProgramMode.tsx` (top bar), plus the status type wherever `status.state` is typed (find it with `grep -rn "state: \"ready\"" frontend/src`)
- Test: `backend/tests/test_program_session.py`, `backend/tests/test_program_files.py`, `frontend/src/ui/program/programMode.test.tsx`

**Interfaces:**
- Produces: `status()` on `ready` returns `{"state": "ready", "robot": ..., "physics": "mujoco" | "off: <reason>"}`.

- [ ] **Step 1: Write the failing tests.**

Backend files:

```python
def test_workcell_is_editable_but_not_deletable(tmp_path):
    from app.program import files
    assert files.editable("bot", "bot_tasks/workcell.yaml")
    assert not files.deletable("bot", "bot_tasks/workcell.yaml")
    assert files.deletable("bot", "bot_tasks/programs/task_001.py")
```

Backend session: follow the existing `test_program_session.py` pattern of faking `_docker`. Add a case where `cat /tmp/physics` returns `mujoco`, and assert `status()["physics"] == "mujoco"`. Add another where the file is missing (non-zero return code), and assert `"physics": "off: unknown (session started before physics support)"`.

Frontend (`programMode.test.tsx`, using the existing render helpers): with status `{state: "ready", robot: "bot", physics: "mujoco"}`, the top bar contains `Physics: MuJoCo`. With `physics: "off: no MuJoCo model"`, it contains `Physics off: no MuJoCo model`.

- [ ] **Step 2: Run the tests and check they fail.**

Run, from `backend/`: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_program_files.py tests/test_program_session.py`
Run, from `frontend/`: `npx vitest run src/ui/program/programMode.test.tsx`
Expected: FAIL.

- [ ] **Step 3: Implement.**

`files.py`:

```python
def editable(robot: str, rel: str) -> bool:
    return (re.fullmatch(rf"{re.escape(robot)}_tasks/programs/[A-Za-z0-9_]+\.py", rel) is not None
            or rel == f"{robot}_tasks/workcell.yaml")


def deletable(robot: str, rel: str) -> bool:
    return re.fullmatch(rf"{re.escape(robot)}_tasks/programs/[A-Za-z0-9_]+\.py", rel) is not None
```

The delete function uses `deletable` instead of `editable`. Its message stays "Only {robot}_tasks/programs/*.py can be deleted.". The save message becomes "Only {robot}_tasks/programs/*.py and workcell.yaml can be edited.".

`session.status()`, in the `stage == "ready"` branch:

```python
    if stage == "ready":
        r = _docker("exec", SESSION_CONTAINER, "cat", "/tmp/physics", check=False)
        physics = r.stdout.strip() if r.returncode == 0 and r.stdout.strip() else \
            "off: unknown (session started before physics support)"
        return {"state": "ready", "robot": robot, "physics": physics}
```

`ProgramMode.tsx`, next to the `ROS ready` text:

```tsx
          <span className="muted">{status.state === "ready" ? "ROS ready" : "ROS not running"}</span>
          {status.state === "ready" && status.physics && (
            <span className="muted"> · {status.physics === "mujoco" ? "Physics: MuJoCo" : `Physics ${status.physics}`}</span>
          )}
```

`"off: x"` renders as "Physics off: x". Add `physics?: string` to the status type.

The Code tab saves `workcell.yaml` through the same save route. The Editor's language is chosen by extension, and YAML falls back to plain text, which is fine. After a save of `workcell.yaml`, nothing restarts: the next run's reset picks it up. The 3D Preview shows workcell objects after the first run, or after session boot (`sim_ctl boot` → `prepare(None)` → `scene_sync`).

- [ ] **Step 4: Run the tests and check they pass.**

Run the commands from Step 2. Expected: PASS. Also run the full backend suite and `npx vitest run` in `frontend/`. Expected: all pass.

- [ ] **Step 5: Commit.**

```bash
git add backend/app/program backend/tests/test_program_files.py backend/tests/test_program_session.py frontend/src
git commit -m "feat: top bar shows physics status; workcell.yaml editable in the Code tab"
```

---

### Task 11: Container acceptance on the irb1300

**Files:**
- Create: `docker/jazzy/check/programs/pick_place_physics.py`
- Modify: `docker/jazzy/check/check_ros.py`, so it runs the new program when `/tmp/sim/hardware` is `mujoco` (follow how the existing programs are listed there)

- [ ] **Step 1: Write the acceptance program.**

```python
# docker/jazzy/check/programs/pick_place_physics.py
"""Physics acceptance: pick a 0.2 kg box, carry it, release it above the target; it must land there."""
from robot import Robot

SCENE = [dict(name="box", shape="box", size=(0.05, 0.05, 0.05), xyz=(0.5, 0.2, 0.025), mass=0.2)]

r = Robot()
down = r.tool_down()
r.move_to((0.5, 0.2, 0.15), down)
r.move_line((0.5, 0.2, 0.055), down)
r.attach("box")
r.move_line((0.5, 0.2, 0.15), down)
r.move_to((0.4, -0.3, 0.15), down)
r.move_line((0.4, -0.3, 0.06), down)
r.detach("box")
r.move_line((0.4, -0.3, 0.15), down)
(x, y, z), _ = r.pose("box")
assert abs(x - 0.4) < 0.01 and abs(y + 0.3) < 0.01 and abs(z - 0.025) < 0.005, f"box ended at {(x, y, z)}"
r.home()
```

The positions are for the irb1300's reach. If `check_ros.py` runs programs on other robots too, scale them the way the existing check programs do, or limit this program to robots with ≥ 6 joints.

- [ ] **Step 2: Run the container check: irb1300 in the foreground (~3 min; say so first).**

Run: `bash scripts/ros_check.sh irb1300` (use the existing script's argument form).
Expected:
- `pick_place_physics: RESULT: ok`
- the existing check programs still pass under physics
- running it twice in a row passes twice (the `reset_world` path)

If a move fails with a `MotionError`, read its message. If it names a too-weak motor, compare with the Simulate card's suggested efforts before changing tolerances.

- [ ] **Step 3: Run the other robots in the background.**

Run `bash scripts/ros_check.sh six sixin four` in the background, ~9 min. Say so first. Expected: all pass. A robot without an MJCF runs on mock and must pass exactly as before.

- [ ] **Step 4: Run the whole suite.**

Run, from `backend/`: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q` (with `corpus/irb1300_asm.stp` present)
Run, from `frontend/`: `npx vitest run`
Expected: all pass, 0 failed.

- [ ] **Step 5: Commit.**

```bash
git add docker/jazzy/check
git commit -m "test: container acceptance: pick-and-place under MuJoCo physics on the irb1300"
```

- [ ] **Step 6: Hand the live acceptance to the user.**

Ask the user to run the 5 benchmark tasks (prompt-to-program spec §8) in Program mode on the irb1300, each in its own conversation, and report how many work within 3 runs. The target is ≥ 4. This is the spec's live criterion and can't be automated here.
