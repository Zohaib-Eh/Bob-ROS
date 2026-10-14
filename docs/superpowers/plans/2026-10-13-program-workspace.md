# Program Workspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the cramped Program step with a full-window, bolt.new-style workspace (conversations sidebar, formatted chat, Preview / Code tabs with an editable file tree) and fix the task-library gaps the first live test exposed.

**Architecture:** Backend gains a conversation store (one JSON file per conversation), a sandboxed file API over the robot's ROS workspace, and a conversation-scoped run endpoint that streams the existing agent events and persists them. The frontend's Program step becomes a `ProgramMode` shell that takes over the window and composes `ConversationList`, `Chat`, and `Workspace` (`Preview` = the existing viewer, `Code` = `FileTree` + CodeMirror `Editor`). The task library gets position-only motion and a `tool_down()` helper; the agent brief states joint count and facing direction.

**Tech Stack:** FastAPI (Python 3.11), React 18 + three.js + vitest, new deps `react-markdown@9.0.1`, `codemirror@6.0.1`, `@codemirror/lang-python@6.1.6`; ROS 2 Jazzy container (unchanged).

**Spec:** `docs/superpowers/specs/2026-10-13-program-workspace-design.md`

## Global Constraints

- Backend tests: from `backend/`, `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q` (`PY` below). Frontend: from `frontend/`, `timeout 300 npx vitest run <files>` (never watch mode), `timeout 600 npm test`, `timeout 300 npx tsc --noEmit`. `@testing-library/jest-dom` is not installed (use `toBeTruthy()` / `getBy*`).
- New frontend dependencies, exact versions: `react-markdown@9.0.1`, `codemirror@6.0.1`, `@codemirror/lang-python@6.1.6`. No other new dependencies.
- Conversations live at `<workspace>/conversations/<id>.json` where `<workspace>` is the parent of the session's `src` folder; ids are 12 lowercase hex chars.
- Editable files: only `<robot>_tasks/programs/<name>.py` with `<name>` matching `[A-Za-z0-9_]+`. Every other workspace text file is read-only. Paths are relative to `src`, use `/`, and never contain `..` or start with `/` or a drive letter.
- One run at a time (agent task or manual program run); Send, Save and Run are disabled while anything runs.
- "User edited" note text, one line per file: `The user edited <robot>_tasks/programs/<file> since your last turn.`
- Brief additions (exact phrases): `The arm has N moving joints.`; when N < 6 also `It cannot hold arbitrary tool orientations: call r.move_to(xyz) without rpy and let the library choose the orientation.`; `"In front of the robot" means toward bearing B° in the base x-y plane (x axis = 0°, y axis = 90°), where the arm reaches at home.`; rule `Never search for joint angles by moving the robot; use move_to (with or without rpy), solutions() or known joint values.`
- Commits end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **A file path that escapes the workspace** (`../x`, `C:/x`, `/etc/passwd`, `src/../../x`) on read, save or run: refused with 400, nothing read or written. Test in Task 3.
2. **A corrupt conversation JSON file** next to good ones: listed as broken, never crashes the list or the app. Test in Task 2.
3. **Client disconnect mid-run of a conversation** (closing the tab): the partial Claude message is still saved, and the agent is cancelled. Test in Task 4.
4. **Robot with fewer than 6 joints asked for an orientation it can't hold**: `move_to(xyz)` without rpy reaches the position (numeric IK on the model, never by moving the robot). Test in Task 1 (container check).
5. **Switching conversations while a run streams**: the stream keeps writing into its own conversation, not the newly selected one. Test in Task 6.

---

## File Structure

| File | Responsibility |
|---|---|
| `backend/app/ros/templates/robot.py` (modify) | `move_to/move_line(xyz, rpy=None)`, `_position_ik`, `tool_down()` |
| `backend/app/ros/program_pkg.py` (modify) | brief: joint count, facing bearing, no-probing rule, API doc |
| `backend/tests/ros_helpers.py` (modify) | `four_axis_zip` |
| `docker/jazzy/check/programs/*.py` (modify/add) | <6-joint aware checks, `position_only.py`, `tool_down.py` |
| `backend/app/program/conversations.py` | conversation files: list/create/load/save/rename/delete, `fold(events)`, `title_from` |
| `backend/app/program/files.py` | workspace file tree, read, write (programs only), path validation |
| `backend/app/program/routes.py` (modify) | conversation + file endpoints; removes `/program/run` and `/program/rerun` |
| `backend/app/program/agent.py` (modify) | `run_result` carries `output` |
| `frontend/src/api/program.ts` (modify) | conversation + file client functions |
| `frontend/src/model/chat.ts` | message types + `applyEvent` fold (mirror of backend fold) |
| `frontend/src/ui/program/useRosSession.ts` | session status/polling/start/stop/restart + live robot wiring (moved from ProgramStep) |
| `frontend/src/ui/program/ProgramMode.tsx` | full-window shell |
| `frontend/src/ui/program/ConversationList.tsx` | sidebar |
| `frontend/src/ui/program/Chat.tsx` | messages + prompt |
| `frontend/src/ui/program/Workspace.tsx`, `FileTree.tsx`, `Editor.tsx` | Preview / Code |
| `frontend/src/App.tsx` (modify) | renders `ProgramMode` instead of the step layout when `step === "program"` |
| `frontend/src/ui/steps/ProgramStep.tsx` + its test (delete) | replaced |

---

### Task 1: Task library — position-only moves, tool_down, better brief

**Files:**
- Modify: `backend/app/ros/templates/robot.py`, `backend/app/ros/program_pkg.py`, `backend/tests/ros_helpers.py`, `docker/jazzy/check/programs/pick_place.py`, `unreachable.py`, `blocked.py`
- Create: `docker/jazzy/check/programs/position_only.py`, `docker/jazzy/check/programs/tool_down.py`
- Test: `backend/tests/test_ros_program_pkg.py` (append)

**Interfaces:**
- Produces: `Robot.move_to(xyz, rpy=None)`, `Robot.move_line(xyz, rpy=None)`, `Robot.tool_down() -> tuple[float, float, float]`; `program_pkg.bearing(chain) -> float` (degrees); `ros_helpers.four_axis_zip(tmp_path, name="four") -> Path`.

- [ ] **Step 1: Write the failing tests** — append to `backend/tests/test_ros_program_pkg.py`:

```python
from app.ros.chain import parse_chain
from app.ros.program_pkg import bearing, brief
from tests.ros_helpers import four_axis_zip, unzip


def test_brief_states_joints_facing_and_the_no_probing_rule(ws, tmp_path):
    text = (ws / "bot_program" / "brief.md").read_text()
    assert "The arm has 2 moving joints." in text
    assert "It cannot hold arbitrary tool orientations: call r.move_to(xyz) without rpy" in text
    assert '"In front of the robot" means toward bearing' in text
    assert "Never search for joint angles by moving the robot" in text
    assert "r.move_to(xyz, rpy=None)" in text and "r.tool_down()" in text


def test_bearing_is_the_home_reach_direction(tmp_path):
    d = unzip(four_axis_zip(tmp_path), tmp_path / "x")
    c = parse_chain((d / "four" / "urdf" / "four.urdf").read_text())
    assert abs(bearing(c) - 0.0) < 1e-6          # the test arm reaches along +x at home
    assert "The arm has 4 moving joints." in brief(c, "four")


def test_library_has_position_only_moves_and_tool_down(ws):
    lib = (ws / "bot_program" / "robot.py").read_text()
    assert "def move_to(self, xyz, rpy=None)" in lib and "def move_line(self, xyz, rpy=None)" in lib
    assert "def tool_down(self)" in lib and "def _position_ik(self" in lib
```

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_ros_program_pkg.py -q` — Expected: FAIL (`ImportError: cannot import name 'bearing'`).

- [ ] **Step 2: `four_axis_zip`** — add to `backend/tests/ros_helpers.py` after `six_axis_zip`:

```python
def four_axis_zip(tmp_path: Path, name: str = "four") -> Path:
    """A 4-joint arm (the six-axis arm without its last two wrist joints): it can reach positions but
    cannot hold arbitrary tool orientations. Limits ±3 rad; reaches along +x at home."""
    names = ["base", "l1", "l2", "l3", "l4"]
    joints = [_joint("j1", "base", "l1", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0), 3.0),
              _joint("j2", "l1", "l2", (0.1, 0.0, 0.4), (0.0, 1.0, 0.0), 3.0),
              _joint("j3", "l2", "l3", (0.1, 0.0, 0.9), (0.0, 1.0, 0.0), 3.0),
              _joint("j4", "l3", "l4", (0.3, 0.0, 0.95), (1.0, 0.0, 0.0), 3.0)]
    centers = [(0.0, 0.0, 0.1), (0.05, 0.0, 0.3), (0.1, 0.0, 0.65), (0.2, 0.0, 0.93), (0.45, 0.0, 0.95)]
    bodies = {n: [_body(n, c, size=0.06)] for n, c in zip(names, centers)}
    path = tmp_path / f"{name}.zip"
    path.write_bytes(export_package(name, [_link(n) for n in names], joints, bodies))
    return path
```

- [ ] **Step 3: Brief changes** — in `backend/app/ros/program_pkg.py`:

Add after `reach()`:

```python
def bearing(chain: Chain) -> float:
    """Direction (degrees, x axis = 0°, y axis = 90°) the arm reaches toward at home, in the base x-y plane."""
    T = fk(chain, home(chain))
    return math.degrees(math.atan2(T[1, 3], T[0, 3])) if math.hypot(T[0, 3], T[1, 3]) > 1e-9 else 0.0
```

In `brief()`, replace the line building `"At home the tool link is at xyz ..."` with:

```python
             f"At home the tool link is at xyz ({T[0, 3]:.3f}, {T[1, 3]:.3f}, {T[2, 3]:.3f}).",
             f'"In front of the robot" means toward bearing {bearing(chain):.0f}° in the base x-y plane '
             "(x axis = 0°, y axis = 90°), where the arm reaches at home.",
             f"The arm has {len(chain.joints)} moving joints."
             + ("" if len(chain.joints) >= 6 else " It cannot hold arbitrary tool orientations: call "
                "r.move_to(xyz) without rpy and let the library choose the orientation."),
```

In `API_DOC`, replace the `move_to` / `move_line` lines and add `tool_down`:

```
r.move_to(xyz, rpy=None)  # tool link to a position; with rpy also that orientation; planned around obstacles
r.move_line(xyz, rpy=None)  # straight line; without rpy keeps the current orientation
r.tool_down()             # an orientation (rpy) with the tool pointing straight down, for this robot
```

and change the example's orientation lines to:

```python
down = r.tool_down()                       # tool pointing straight down
r.move_to((0.5, 0.2, 0.15), down)
```

(keep the rest of the example, replacing every later use of the old variable with `down`).

In `RULES`, add as the second-to-last bullet:

```
- Never search for joint angles by moving the robot; use move_to (with or without rpy), solutions() or known joint values.
```

- [ ] **Step 4: Library changes** — in `backend/app/ros/templates/robot.py`:

Add inside `class Robot` (after `_check_reach`):

```python
    def _position_ik(self, xyz, tries: int = 8) -> list:
        """Joint solutions that put the tool link at xyz with any orientation, computed on the model
        (damped least squares from the current state and random seeds); nearest to the current state first."""
        target = np.asarray(xyz, float)
        lo = np.array([j.lower if j.lower is not None else -math.pi for j in self.chain.joints])
        hi = np.array([j.upper if j.upper is not None else math.pi for j in self.chain.joints])
        rng = np.random.default_rng(0)
        cur = self._current()
        sols = []
        for q in [cur] + [rng.uniform(lo, hi) for _ in range(tries - 1)]:
            q = np.clip(np.asarray(q, float), lo, hi)
            for _ in range(100):
                p = self._kin.fk(q)[:3, 3]
                err = target - p
                if np.linalg.norm(err) < 1e-4:
                    sols.append(q)
                    break
                J = np.empty((3, len(q)))
                for i in range(len(q)):
                    dq = np.zeros(len(q))
                    dq[i] = 1e-6
                    J[:, i] = (self._kin.fk(q + dq)[:3, 3] - p) / 1e-6
                step = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), err)
                q = np.clip(q + step * min(1.0, 0.3 / max(1e-9, float(np.abs(step).max()))), lo, hi)
        return sorted(sols, key=lambda s: float(np.linalg.norm(s - cur)))

    def tool_down(self) -> tuple:
        """rpy that points the tool's approach axis (the last joint's axis) straight down, turning the
        home orientation as little as possible."""
        R = self._kin.fk(_home(self.chain))[:3, :3]
        a = R @ np.asarray(self.chain.joints[-1].axis, float)          # approach axis in the base frame
        if a[2] > 0:
            a = -a                                                     # either direction of the axis may point outward
        target = np.array([0.0, 0.0, -1.0])
        v, c = np.cross(a, target), float(a @ target)
        if np.linalg.norm(v) < 1e-9:
            turn = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
        else:
            vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            turn = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
        return _rpy(turn @ R)
```

Replace `move_to` with:

```python
    def move_to(self, xyz, rpy=None):
        if rpy is None:
            cands = self._position_ik(xyz)
            if not cands:
                raise Unreachable(f"no joint solution puts the tool at {_fmt(xyz)} "
                                  "(outside the reach or the joint limits)")
            last = None
            for q in cands[:4]:
                try:
                    self.move_joints(list(q))
                    return
                except (Collision, PlanningFailed) as exc:
                    last = exc
            raise last
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
```

In `move_line`, change the signature to `def move_line(self, xyz, rpy=None):` and make its first lines:

```python
        if rpy is None:
            rpy = self.tool_pose()[1]                    # keep the current orientation along the line
        self._check_reach(xyz, rpy)
```

- [ ] **Step 5: Container check programs** — make the existing checks work on robots with fewer than 6 joints and add two:

`docker/jazzy/check/programs/pick_place.py`:

```python
"""robot.py round trip: add, attach, carry, detach, (solutions on 6-joint arms), home."""
from robot import Robot

r = Robot()
six = len(r.joint_names) >= 6
(x, y, z), rpy = r.tool_pose()
keep = rpy if six else None                       # <6 joints: let the library choose the orientation
r.add_box("box", (0.04, 0.04, 0.04), (x, y, z - 0.20))
r.move_to((x, y, z - 0.10), keep)
r.attach("box")
r.move_line((x, y + 0.05, z - 0.10), keep)
r.detach("box")
(x2, y2, z2), _ = r.tool_pose()
assert abs(y2 - (y + 0.05)) < 2e-3, (y, y2)
if six:
    assert r.solutions((x, y, z - 0.10), rpy), "expected at least one exact solution"
r.home()
print("pick_place ok")
```

`docker/jazzy/check/programs/unreachable.py` — replace the `move_to` call with `r.move_to((10.0, 0.0, 0.5))` (position only, so it works on any arm).

`docker/jazzy/check/programs/blocked.py` — replace the `move_to` call with `r.move_to(target, rpy if len(r.joint_names) >= 6 else None)`, keeping whatever variable names the file already uses for the target and the home orientation.

`docker/jazzy/check/programs/position_only.py`:

```python
"""move_to without rpy reaches a reachable position on any arm, using model IK (no probing moves)."""
import numpy as np
from robot import Robot

r = Robot()
q = np.array([0.4 * (-1) ** i for i in range(len(r.joint_names))])
lo = np.array([j.lower if j.lower is not None else -3.0 for j in r.chain.joints])
hi = np.array([j.upper if j.upper is not None else 3.0 for j in r.chain.joints])
target = r._kin.fk(np.clip(q, lo, hi))[:3, 3]
r.move_to(tuple(target))
(x, y, z), _ = r.tool_pose()
assert np.linalg.norm(np.array([x, y, z]) - target) < 2e-3, ((x, y, z), target)
print("position_only ok")
```

`docker/jazzy/check/programs/tool_down.py`:

```python
"""tool_down() points the approach axis straight down; on 6-joint arms the tool can get there."""
import numpy as np
from robot import Robot, _matrix

r = Robot()
rpy = r.tool_down()
a = _matrix((0, 0, 0), rpy)[:3, :3] @ np.asarray(r.chain.joints[-1].axis, float)
assert abs(abs(a[2]) - 1) < 1e-9, a                  # the approach axis is vertical
if len(r.joint_names) >= 6:
    (x, y, z), _ = r.tool_pose()
    r.move_to((x, y, z - 0.1), rpy)
print("tool_down ok")
```

- [ ] **Step 6: Run tests**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_ros_program_pkg.py -q && OPENBLAS_NUM_THREADS=1 $PY -m pytest -q` — Expected: PASS.
Run (Docker Desktop must be running): generate `four.zip` with `OPENBLAS_NUM_THREADS=1 $PY -c "from pathlib import Path; from tests.ros_helpers import four_axis_zip; print(four_axis_zip(Path(r'../corpus/out')))"` from `backend/`, then for `z in irb1300 six four`: `bash scripts/ros_check.sh corpus/out/$z.zip | grep -E "^(PASS|FAIL|SKIP)"` (timeout 600000 each). Expected: every `robot.py` line PASS on all three (EAIK lines may SKIP on `four`). If `tool_down.py`'s move fails on the IRB 1300, report the error (do not loosen the check).

- [ ] **Step 7: Commit**

```bash
git add backend/app/ros backend/tests docker/jazzy/check/programs
git commit -m "feat: position-only moves, tool_down, and a brief that says where the robot faces

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: Conversation store and CRUD endpoints

**Files:**
- Create: `backend/app/program/conversations.py`
- Modify: `backend/app/program/routes.py`
- Test: `backend/tests/test_program_conversations.py`

**Interfaces:**
- Consumes: `session.current() -> {"robot": str, "src": Path | str} | None`.
- Produces (`conversations.py`): `folder(src) -> Path`; `list_all(src) -> list[dict]` (each `{id, title, created, updated}` or `{id, title: "Unreadable conversation", broken: True}`; newest `updated` first); `create(src, title="New conversation") -> dict`; `load(src, cid) -> dict` (raises `KeyError`); `save(src, conv) -> None` (sets `updated`); `rename(src, cid, title) -> dict`; `delete(src, cid) -> None`; `title_from(task) -> str`; `fold(events: list[dict]) -> dict`.
- Conversation dict: `{id, title, created, updated, claude_session_id: str | None, edited: list[str], messages: list}`; message shapes: `{"role": "user", "text"}`, `{"role": "claude", "items": [...], "summary": str, "result": {"ok", "runs"} | None, "error": str | None, "program": str | None}`, `{"role": "run", "path", "ok", "output"}`; items: `{"kind": "text", "text"}`, `{"kind": "file", "path"}`, `{"kind": "run", "n", "ok"?, "message"?, "output"?}`.
- Routes: `GET /program/conversations`, `POST /program/conversations` (`{title?}`), `GET /program/conversations/{cid}`, `PATCH /program/conversations/{cid}` (`{title}`), `DELETE /program/conversations/{cid}`. 409 when no session (`"Start the ROS session first."`), 404 unknown id, 400 bad id.

- [ ] **Step 1: Write the failing tests** — `backend/tests/test_program_conversations.py`:

```python
import json

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.program import conversations as conv, routes

client = TestClient(app)


@pytest.fixture
def src(tmp_path, monkeypatch):
    s = tmp_path / "ws" / "src"
    s.mkdir(parents=True)
    monkeypatch.setattr(routes.session, "current", lambda: {"robot": "bot", "src": s})
    return s


def test_create_list_rename_delete(src):
    a = client.post("/program/conversations", json={}).json()
    b = client.post("/program/conversations", json={"title": "Square"}).json()
    assert a["title"] == "New conversation" and len(a["id"]) == 12 and a["messages"] == []
    assert [c["id"] for c in client.get("/program/conversations").json()] == [b["id"], a["id"]]
    assert client.patch(f"/program/conversations/{a['id']}", json={"title": "Pick"}).json()["title"] == "Pick"
    assert client.get(f"/program/conversations/{a['id']}").json()["title"] == "Pick"
    assert client.delete(f"/program/conversations/{a['id']}").status_code == 200
    assert client.get(f"/program/conversations/{a['id']}").status_code == 404
    assert (src.parent / "conversations" / f"{b['id']}.json").exists()


def test_corrupt_files_are_listed_as_broken(src):
    good = client.post("/program/conversations", json={}).json()
    (src.parent / "conversations" / "0123456789ab.json").write_text("{not json")
    items = client.get("/program/conversations").json()
    assert {i["id"] for i in items} == {good["id"], "0123456789ab"}
    assert next(i for i in items if i["id"] == "0123456789ab") == {
        "id": "0123456789ab", "title": "Unreadable conversation", "broken": True}


def test_bad_ids_and_no_session(src, monkeypatch):
    assert client.get("/program/conversations/../x").status_code in (400, 404)
    assert client.get("/program/conversations/NOTHEX123456").status_code == 400
    monkeypatch.setattr(routes.session, "current", lambda: None)
    r = client.get("/program/conversations")
    assert r.status_code == 409 and r.json()["detail"] == "Start the ROS session first."


def test_title_from_and_fold():
    assert conv.title_from("  Put a 5 cm box on the floor at (0.5, 0.2) and more ") == "Put a 5 cm box on"
    events = [{"kind": "text", "text": "Planning"}, {"kind": "file", "path": "programs/task_001.py"},
              {"kind": "run", "n": 1}, {"kind": "run_result", "ok": False, "message": "failed: X", "output": "log1"},
              {"kind": "run", "n": 2}, {"kind": "run_result", "ok": True, "message": "ok", "output": "log2"},
              {"kind": "text", "text": "Done.\nAll good."},
              {"kind": "done", "summary": "Done.\nAll good.", "runs": 2, "ok": True, "session_id": "s1",
               "program": "programs/task_001.py", "code": "x"}]
    m = conv.fold(events)
    assert m == {"role": "claude", "items": [
        {"kind": "text", "text": "Planning"}, {"kind": "file", "path": "programs/task_001.py"},
        {"kind": "run", "n": 1, "ok": False, "message": "failed: X", "output": "log1"},
        {"kind": "run", "n": 2, "ok": True, "message": "ok", "output": "log2"}],
        "summary": "Done.\nAll good.", "result": {"ok": True, "runs": 2}, "error": None,
        "program": "programs/task_001.py"}
    assert conv.fold([{"kind": "error", "message": "boom"}])["error"] == "boom"
```

Run: `cd backend && OPENBLAS_NUM_THREADS=1 $PY -m pytest tests/test_program_conversations.py -q` — Expected: FAIL (import error).

- [ ] **Step 2: Implement** — `backend/app/program/conversations.py`:

```python
"""Conversations of the Program workspace: one JSON file per conversation in the robot's workspace
(<workspace>/conversations/<id>.json). The message format is mirrored by frontend/src/model/chat.ts."""
import json
import re
import time
import uuid
from pathlib import Path

ID_RE = re.compile(r"^[0-9a-f]{12}$")


def folder(src) -> Path:
    return Path(src).parent / "conversations"


def _path(src, cid: str) -> Path:
    if not ID_RE.match(cid):
        raise ValueError("Bad conversation id.")
    return folder(src) / f"{cid}.json"


def title_from(task: str) -> str:
    return " ".join(task.split()[:6])[:60] or "New conversation"


def list_all(src) -> list[dict]:
    out = []
    d = folder(src)
    for p in sorted(d.glob("*.json")) if d.exists() else []:
        if not ID_RE.match(p.stem):
            continue
        try:
            c = json.loads(p.read_text(encoding="utf-8"))
            out.append({k: c[k] for k in ("id", "title", "created", "updated")})
        except (ValueError, KeyError, TypeError):
            out.append({"id": p.stem, "title": "Unreadable conversation", "broken": True})
    return sorted(out, key=lambda c: c.get("updated", 0), reverse=True)


def save(src, conv: dict) -> None:
    conv["updated"] = time.time()
    p = _path(src, conv["id"])
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(conv, indent=1), encoding="utf-8")
    tmp.replace(p)


def create(src, title: str = "New conversation") -> dict:
    now = time.time()
    conv = {"id": uuid.uuid4().hex[:12], "title": title, "created": now, "updated": now,
            "claude_session_id": None, "edited": [], "messages": []}
    save(src, conv)
    return conv


def load(src, cid: str) -> dict:
    p = _path(src, cid)
    if not p.exists():
        raise KeyError(cid)
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except ValueError:
        raise KeyError(cid)


def rename(src, cid: str, title: str) -> dict:
    conv = load(src, cid)
    conv["title"] = title.strip()[:80] or conv["title"]
    save(src, conv)
    return conv


def delete(src, cid: str) -> None:
    p = _path(src, cid)
    if not p.exists():
        raise KeyError(cid)
    p.unlink()


def fold(events: list[dict]) -> dict:
    """Agent events -> one Claude message. The final answer appears once: a trailing narration
    text equal to the summary is dropped."""
    m = {"role": "claude", "items": [], "summary": "", "result": None, "error": None, "program": None}
    for e in events:
        k = e.get("kind")
        if k == "text":
            m["items"].append({"kind": "text", "text": e["text"]})
        elif k == "file":
            if not (m["items"] and m["items"][-1] == {"kind": "file", "path": e["path"]}):
                m["items"].append({"kind": "file", "path": e["path"]})
        elif k == "run":
            m["items"].append({"kind": "run", "n": e["n"]})
        elif k == "run_result":
            for it in reversed(m["items"]):
                if it["kind"] == "run":
                    it.update(ok=e["ok"], message=e["message"], output=e.get("output", ""))
                    break
        elif k == "done":
            m["summary"] = e.get("summary", "")
            m["result"] = {"ok": e.get("ok", False), "runs": e.get("runs", 0)}
            m["program"] = e.get("program")
            while (m["items"] and m["items"][-1]["kind"] == "text"
                   and m["items"][-1]["text"].strip() == m["summary"].strip()):
                m["items"].pop()
        elif k == "error":
            m["error"] = e.get("message", "")
    return m
```

In `backend/app/program/routes.py` add (keep the existing endpoints for now; Task 4 removes `/run` and `/rerun`):

```python
from pathlib import Path
from app.program import conversations


class NewConversation(BaseModel):
    title: str | None = None


class Rename(BaseModel):
    title: str


def _src() -> Path:
    try:
        cur = session.current()
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    if not cur:
        raise HTTPException(409, "Start the ROS session first.")
    return Path(cur["src"])


def _conv(cid: str) -> tuple[Path, dict]:
    src = _src()
    try:
        return src, conversations.load(src, cid)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except KeyError:
        raise HTTPException(404, "No such conversation.")


@router.get("/conversations")
def list_conversations() -> list:
    return conversations.list_all(_src())


@router.post("/conversations")
def new_conversation(req: NewConversation) -> dict:
    return conversations.create(_src(), req.title or "New conversation")


@router.get("/conversations/{cid}")
def get_conversation(cid: str) -> dict:
    return _conv(cid)[1]


@router.patch("/conversations/{cid}")
def rename_conversation(cid: str, req: Rename) -> dict:
    src, _ = _conv(cid)
    return conversations.rename(src, cid, req.title)


@router.delete("/conversations/{cid}")
def delete_conversation(cid: str) -> dict:
    src, _ = _conv(cid)
    conversations.delete(src, cid)
    return {"deleted": cid}
```

- [ ] **Step 3: Run tests, then the full backend suite** — Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add backend/app/program/conversations.py backend/app/program/routes.py backend/tests/test_program_conversations.py
git commit -m "feat: conversation store with list, create, rename and delete endpoints

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: Workspace file API

**Files:**
- Create: `backend/app/program/files.py`
- Modify: `backend/app/program/routes.py`
- Test: `backend/tests/test_program_files.py`

**Interfaces:**
- Consumes: `_src()` (Task 2), `session.current()`, `session.run_program(program)` (takes `programs/<name>.py` relative to the tasks folder, returns `{ok, output}`), `conversations.load/save`.
- Produces (`files.py`): `TEXT_EXT`; `safe(src, rel) -> Path` (raises `ValueError`); `editable(robot, rel) -> bool`; `tree(src, robot) -> list[{"path", "editable"}]` (sorted); `read(src, rel) -> str`; `write(src, robot, rel, content) -> None`.
- Routes: `GET /program/files` → tree; `GET /program/files/content?path=` → `{path, content, editable}`; `PUT /program/files/content` (`{path, content, conversation?}`) → `{saved: path}` (records the path in the conversation's `edited` list); `POST /program/files/run` (`{path, conversation?}`) → `{ok, output}` (appends `{"role": "run", path, ok, output}` to the conversation). 400 on bad or non-editable paths (`"Only <robot>_tasks/programs/*.py can be edited."`), 404 missing file.

- [ ] **Step 1: Write the failing tests** — `backend/tests/test_program_files.py`:

```python
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
```

Run — Expected: FAIL (404s).

- [ ] **Step 2: Implement** — `backend/app/program/files.py`:

```python
"""The robot's ROS workspace as seen by the Program workspace's Code tab: list and read text files,
save only the user's programs."""
import re
from pathlib import Path, PurePosixPath

TEXT_EXT = {".py", ".yaml", ".yml", ".xml", ".xacro", ".srdf", ".urdf", ".md", ".sh", ".txt", ".cfg", ".json"}
SKIP_DIRS = {"build", "install", "log", "__pycache__", ".git"}


def safe(src, rel: str) -> Path:
    """Resolve a workspace-relative path; anything that could leave the workspace is refused."""
    if not rel or "\\" in rel or rel.startswith("/") or re.match(r"^[A-Za-z]:", rel):
        raise ValueError("Bad path.")
    parts = PurePosixPath(rel).parts
    if any(p in ("..", ".") for p in parts):
        raise ValueError("Bad path.")
    root = Path(src).resolve()
    p = (root / Path(*parts)).resolve()
    if root not in p.parents:
        raise ValueError("Bad path.")
    return p


def editable(robot: str, rel: str) -> bool:
    return re.fullmatch(rf"{re.escape(robot)}_tasks/programs/[A-Za-z0-9_]+\.py", rel) is not None


def tree(src, robot: str) -> list[dict]:
    root = Path(src)
    out = []
    for p in root.rglob("*"):
        rel = p.relative_to(root)
        if p.is_file() and p.suffix in TEXT_EXT and not SKIP_DIRS.intersection(rel.parts):
            r = rel.as_posix()
            out.append({"path": r, "editable": editable(robot, r)})
    return sorted(out, key=lambda f: f["path"])


def read(src, rel: str) -> str:
    p = safe(src, rel)
    if not p.is_file():
        raise FileNotFoundError(rel)
    return p.read_text(encoding="utf-8", errors="replace")


def write(src, robot: str, rel: str, content: str) -> None:
    p = safe(src, rel)
    if not editable(robot, rel):
        raise PermissionError(f"Only {robot}_tasks/programs/*.py can be edited.")
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(content, encoding="utf-8", newline="\n")
```

In `routes.py` add:

```python
from app.program import files


class SaveFile(BaseModel):
    path: str
    content: str
    conversation: str | None = None


class RunFile(BaseModel):
    path: str
    conversation: str | None = None


def _robot() -> str:
    _src()
    return session.current()["robot"]


@router.get("/files")
def list_files() -> list:
    return files.tree(_src(), _robot())


@router.get("/files/content")
def read_file(path: str) -> dict:
    src, robot = _src(), _robot()
    try:
        return {"path": path, "content": files.read(src, path), "editable": files.editable(robot, path)}
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except FileNotFoundError:
        raise HTTPException(404, "No such file.")


@router.put("/files/content")
def save_file(req: SaveFile) -> dict:
    src, robot = _src(), _robot()
    try:
        files.write(src, robot, req.path, req.content)
    except (ValueError, PermissionError) as exc:
        raise HTTPException(400, str(exc))
    if req.conversation:
        _, conv = _conv(req.conversation)
        if req.path not in conv["edited"]:
            conv["edited"].append(req.path)
        conversations.save(src, conv)
    return {"saved": req.path}


@router.post("/files/run")
def run_file(req: RunFile) -> dict:
    src, robot = _src(), _robot()
    try:
        files.safe(src, req.path)
        if not files.editable(robot, req.path):
            raise ValueError(f"Only {robot}_tasks/programs/*.py can be run.")
        result = session.run_program(req.path.split("/", 1)[1])
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    if req.conversation:
        _, conv = _conv(req.conversation)
        conv["messages"].append({"role": "run", "path": req.path, **result})
        conversations.save(src, conv)
    return result
```

- [ ] **Step 3: Run tests, then the full backend suite** — Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add backend/app/program/files.py backend/app/program/routes.py backend/tests/test_program_files.py
git commit -m "feat: workspace file API (tree, read, save programs, run programs)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: Conversation runs (stream + persist + edited note)

**Files:**
- Modify: `backend/app/program/routes.py`, `backend/app/program/agent.py`
- Modify: `backend/tests/test_program_routes.py` (drop `/program/run` and `/program/rerun` tests, keep session tests), `backend/tests/test_program_agent.py` (one assertion)
- Test: `backend/tests/test_program_conversation_run.py`

**Interfaces:**
- Consumes: `agent.run_agent(robot, src, task, resume=None, *, cancel=None, ...)`, `conversations.load/save/fold/title_from`.
- Produces: `POST /program/conversations/{cid}/run` (`{task}`) → `text/event-stream` of the same events as before; `run_result` events gain `"output": str`. Removes `POST /program/run` and `POST /program/rerun`.

Behaviour: on start, append `{"role": "user", "text": task}` and save; if the title is still "New conversation", set it with `title_from(task)`. The agent receives the task prefixed by one note line per `edited` path, then `edited` is cleared. Events are collected; in the stream's `finally` (normal end, error or disconnect) the folded Claude message is appended and saved, and `claude_session_id` is set from a `done` event's `session_id`. 409 when the session is not ready (same text as before, plus log).

- [ ] **Step 1: Write the failing tests** — `backend/tests/test_program_conversation_run.py`:

```python
import json

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.program import conversations, routes

client = TestClient(app)


@pytest.fixture
def setup(tmp_path, monkeypatch):
    s = tmp_path / "ws" / "src"
    s.mkdir(parents=True)
    monkeypatch.setattr(routes.session, "current", lambda: {"robot": "bot", "src": s})
    monkeypatch.setattr(routes.session, "status", lambda: {"state": "ready", "robot": "bot"})
    seen = {}

    def fake_agent(robot, src, task, resume=None, cancel=None):
        seen.update(task=task, resume=resume, cancel=cancel)
        yield {"kind": "text", "text": "Writing"}
        yield {"kind": "run", "n": 1}
        yield {"kind": "run_result", "ok": True, "message": "ok", "output": "RESULT: ok"}
        yield {"kind": "done", "summary": "Moved.", "runs": 1, "ok": True, "session_id": "s1",
               "program": "programs/task_001.py", "code": "x"}
    monkeypatch.setattr(routes.agent, "run_agent", fake_agent)
    return s, seen


def _events(r):
    return [json.loads(c[6:]) for c in r.text.split("\n\n") if c.startswith("data: ")]


def test_run_streams_saves_and_resumes(setup):
    src, seen = setup
    cid = conversations.create(src)["id"]
    r = client.post(f"/program/conversations/{cid}/run", json={"task": "Move the tool up please"})
    assert [e["kind"] for e in _events(r)] == ["text", "run", "run_result", "done"]
    c = conversations.load(src, cid)
    assert c["title"] == "Move the tool up please" and c["claude_session_id"] == "s1"
    assert c["messages"][0] == {"role": "user", "text": "Move the tool up please"}
    assert c["messages"][1]["summary"] == "Moved." and c["messages"][1]["items"][-1]["output"] == "RESULT: ok"
    client.post(f"/program/conversations/{cid}/run", json={"task": "again"})
    assert seen["resume"] == "s1" and seen["cancel"].is_set()


def test_user_edits_are_announced_once(setup):
    src, seen = setup
    c = conversations.create(src)
    c["edited"] = ["bot_tasks/programs/task_001.py"]
    conversations.save(src, c)
    client.post(f"/program/conversations/{c['id']}/run", json={"task": "go"})
    assert seen["task"] == "The user edited bot_tasks/programs/task_001.py since your last turn.\n\ngo"
    assert conversations.load(src, c["id"])["edited"] == []


def test_a_broken_stream_still_saves_the_partial_message(setup, monkeypatch):
    src, _ = setup

    def failing(robot, src_, task, resume=None, cancel=None):
        yield {"kind": "text", "text": "Starting"}
        raise RuntimeError("stream died")
    monkeypatch.setattr(routes.agent, "run_agent", failing)
    cid = conversations.create(src)["id"]
    with pytest.raises(RuntimeError):
        client.post(f"/program/conversations/{cid}/run", json={"task": "go"})
    msgs = conversations.load(src, cid)["messages"]
    assert msgs[-1]["role"] == "claude" and msgs[-1]["items"] == [{"kind": "text", "text": "Starting"}]


def test_old_endpoints_are_gone():
    assert client.post("/program/run", json={"task": "x"}).status_code in (404, 405)
    assert client.post("/program/rerun", json={"program": "x"}).status_code in (404, 405)
```

Run — Expected: FAIL.

- [ ] **Step 2: Implement**

In `agent.py` `parse_stream`, change the `run_result` yield to include the full tool result text:

```python
                        yield {"kind": "run_result", "ok": last_ok, "message": m[-1], "output": text}
```

and update `tests/test_program_agent.py`'s fixture assertion if it compares `run_result` events exactly (add the `output` key).

In `routes.py`: delete `RunRequest`, `RerunRequest`, the `@router.post("/run")` and `@router.post("/rerun")` functions; add:

```python
class ConvRun(BaseModel):
    task: str


@router.post("/conversations/{cid}/run")
def run_conversation(cid: str, req: ConvRun) -> StreamingResponse:
    try:
        st = session.status()
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    src, conv = _conv(cid)
    if st["state"] != "ready":
        detail = "Start the ROS session first and wait until it is ready."
        if st.get("log"):
            detail += "\n" + st["log"]
        raise HTTPException(409, detail)
    robot = session.current()["robot"]
    notes = [f"The user edited {p} since your last turn." for p in conv["edited"]]
    prompt = ("\n".join(notes) + "\n\n" + req.task) if notes else req.task
    conv["messages"].append({"role": "user", "text": req.task})
    if conv["title"] == "New conversation":
        conv["title"] = conversations.title_from(req.task)
    conv["edited"] = []
    conversations.save(src, conv)
    cancel = threading.Event()
    events = agent.run_agent(robot, src, prompt, conv["claude_session_id"], cancel=cancel)
    collected: list[dict] = []

    async def stream():
        try:
            async for e in iterate_in_threadpool(events):
                collected.append(e)
                yield f"data: {json.dumps(e)}\n\n"
        finally:
            cancel.set()  # client gone or stream done: stop the agent's child process
            latest = conversations.load(src, cid)
            latest["messages"].append(conversations.fold(collected))
            done = next((e for e in reversed(collected) if e.get("kind") == "done"), None)
            if done and done.get("session_id"):
                latest["claude_session_id"] = done["session_id"]
            conversations.save(src, latest)

    return StreamingResponse(stream(), media_type="text/event-stream")
```

In `tests/test_program_routes.py`, delete the tests that call `/program/run` or `/program/rerun` (they are replaced by the new file); keep the session start/status/stop tests.

- [ ] **Step 3: Run tests, then the full backend suite** — Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add backend/app/program backend/tests
git commit -m "feat: conversation runs stream, persist and announce user edits

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: Frontend client and chat model

**Files:**
- Modify: `frontend/src/api/program.ts`
- Create: `frontend/src/model/chat.ts`
- Test: `frontend/src/model/chat.test.ts`, `frontend/src/api/program.test.ts` (append)

**Interfaces:**
- Consumes: Task 2–4 endpoints; existing `send`, `json`, `parseSse`, `ProgramEvent` (add `output?: string` to the `run_result` variant).
- Produces (`api/program.ts`): `interface ConversationInfo { id: string; title: string; created?: number; updated?: number; broken?: boolean }`; `interface Conversation { id; title; created; updated; claude_session_id: string | null; edited: string[]; messages: Message[] }`; `listConversations(): Promise<ConversationInfo[]>`; `createConversation(title?: string): Promise<Conversation>`; `getConversation(id): Promise<Conversation>`; `renameConversation(id, title): Promise<Conversation>`; `deleteConversation(id): Promise<void>`; `runConversation(id, task, onEvent, signal?): Promise<void>`; `interface FileEntry { path: string; editable: boolean }`; `listFiles(): Promise<FileEntry[]>`; `readFile(path): Promise<{ path: string; content: string; editable: boolean }>`; `saveFile(path, content, conversation: string | null): Promise<void>`; `runFile(path, conversation: string | null): Promise<{ ok: boolean; output: string }>`. Remove `runTask` and `rerun`.
- Produces (`model/chat.ts`): `type Item = { kind: "text"; text: string } | { kind: "file"; path: string } | { kind: "run"; n: number; ok?: boolean; message?: string; output?: string }`; `interface ClaudeMessage { role: "claude"; items: Item[]; summary: string; result: { ok: boolean; runs: number } | null; error: string | null; program: string | null }`; `type Message = { role: "user"; text: string } | ClaudeMessage | { role: "run"; path: string; ok: boolean; output: string }`; `emptyClaude(): ClaudeMessage`; `applyEvent(m: ClaudeMessage, e: ProgramEvent): ClaudeMessage` (same rules as backend `fold`).

- [ ] **Step 1: Write the failing tests** — `frontend/src/model/chat.test.ts`:

```ts
import { describe, expect, it } from "vitest";
import type { ProgramEvent } from "../api/program";
import { applyEvent, emptyClaude } from "./chat";

const fold = (es: ProgramEvent[]) => es.reduce(applyEvent, emptyClaude());

describe("chat fold", () => {
  it("builds items, attaches run results and shows the final answer once", () => {
    const m = fold([
      { kind: "text", text: "Planning" }, { kind: "file", path: "programs/task_001.py" },
      { kind: "file", path: "programs/task_001.py" },
      { kind: "run", n: 1 }, { kind: "run_result", ok: false, message: "failed: X", output: "log1" },
      { kind: "text", text: "Done." },
      { kind: "done", summary: "Done.", runs: 1, ok: false, session_id: "s", program: "programs/task_001.py", code: null },
    ]);
    expect(m.items).toEqual([{ kind: "text", text: "Planning" }, { kind: "file", path: "programs/task_001.py" },
      { kind: "run", n: 1, ok: false, message: "failed: X", output: "log1" }]);
    expect(m).toMatchObject({ summary: "Done.", result: { ok: false, runs: 1 }, program: "programs/task_001.py", error: null });
  });
  it("records errors and does not mutate its input", () => {
    const start = emptyClaude();
    const m = applyEvent(start, { kind: "error", message: "boom" });
    expect(m.error).toBe("boom");
    expect(start.error).toBeNull();
  });
});
```

Append to `frontend/src/api/program.test.ts`:

```ts
import { runConversation, saveFile } from "./program";

describe("conversation client", () => {
  it("runs a conversation over SSE", async () => {
    const body = new ReadableStream({ start(c) { c.enqueue(new TextEncoder().encode('data: {"kind":"run","n":1}\n\n')); c.close(); } });
    const f = vi.spyOn(globalThis, "fetch").mockResolvedValue(new Response(body, { status: 200 }));
    const got: unknown[] = [];
    await runConversation("abc123abc123", "go", (e) => got.push(e));
    expect(got).toEqual([{ kind: "run", n: 1 }]);
    expect(f.mock.calls[0][0]).toBe("/api/program/conversations/abc123abc123/run");
  });
  it("saves a file with its conversation", async () => {
    const f = vi.spyOn(globalThis, "fetch").mockResolvedValue(new Response("{}", { status: 200 }));
    await saveFile("bot_tasks/programs/a.py", "x", "abc123abc123");
    expect(JSON.parse(f.mock.calls[0][1]!.body as string)).toEqual({ path: "bot_tasks/programs/a.py", content: "x", conversation: "abc123abc123" });
    expect(f.mock.calls[0][1]!.method).toBe("PUT");
  });
});
```

(Remove the existing `runTask` tests from `program.test.ts`; keep the `parseSse` tests.)

Run: `cd frontend && timeout 300 npx vitest run src/model/chat.test.ts src/api/program.test.ts` — Expected: FAIL.

- [ ] **Step 2: Implement** — `frontend/src/model/chat.ts`:

```ts
import type { ProgramEvent } from "../api/program";

export type Item =
  | { kind: "text"; text: string }
  | { kind: "file"; path: string }
  | { kind: "run"; n: number; ok?: boolean; message?: string; output?: string };
export interface ClaudeMessage {
  role: "claude"; items: Item[]; summary: string; result: { ok: boolean; runs: number } | null; error: string | null; program: string | null;
}
export type Message = { role: "user"; text: string } | ClaudeMessage | { role: "run"; path: string; ok: boolean; output: string };

export const emptyClaude = (): ClaudeMessage => ({ role: "claude", items: [], summary: "", result: null, error: null, program: null });

/** One agent event folded into the Claude message (mirror of backend conversations.fold). */
export function applyEvent(m: ClaudeMessage, e: ProgramEvent): ClaudeMessage {
  const items = [...m.items];
  switch (e.kind) {
    case "text": return { ...m, items: [...items, { kind: "text", text: e.text }] };
    case "file": {
      const last = items[items.length - 1];
      return last?.kind === "file" && last.path === e.path ? m : { ...m, items: [...items, { kind: "file", path: e.path }] };
    }
    case "run": return { ...m, items: [...items, { kind: "run", n: e.n }] };
    case "run_result": {
      for (let i = items.length - 1; i >= 0; i--) {
        const it = items[i];
        if (it.kind === "run") { items[i] = { ...it, ok: e.ok, message: e.message, output: e.output ?? "" }; break; }
      }
      return { ...m, items };
    }
    case "done": {
      while (items.length) {
        const last = items[items.length - 1];
        if (last.kind !== "text" || last.text.trim() !== e.summary.trim()) break;
        items.pop();
      }
      return { ...m, items, summary: e.summary, result: { ok: e.ok, runs: e.runs }, program: e.program };
    }
    case "error": return { ...m, error: e.message };
  }
}
```

In `frontend/src/api/program.ts`: add `output?: string` to the `run_result` variant of `ProgramEvent`; delete `runTask` and `rerun`; add (reusing the SSE loop the old `runTask` used — move its body into a private `streamEvents(path, body, onEvent, signal)` helper):

```ts
import type { Message } from "../model/chat";

export interface ConversationInfo { id: string; title: string; created?: number; updated?: number; broken?: boolean }
export interface Conversation { id: string; title: string; created: number; updated: number;
  claude_session_id: string | null; edited: string[]; messages: Message[] }
export interface FileEntry { path: string; editable: boolean }

const put = (body: unknown): RequestInit => ({ ...json(body), method: "PUT" });
const patch = (body: unknown): RequestInit => ({ ...json(body), method: "PATCH" });

export async function listConversations(): Promise<ConversationInfo[]> {
  return (await send("/program/conversations", { method: "GET" })).json();
}
export async function createConversation(title?: string): Promise<Conversation> {
  return (await send("/program/conversations", json(title ? { title } : {}))).json();
}
export async function getConversation(id: string): Promise<Conversation> {
  return (await send(`/program/conversations/${id}`, { method: "GET" })).json();
}
export async function renameConversation(id: string, title: string): Promise<Conversation> {
  return (await send(`/program/conversations/${id}`, patch({ title }))).json();
}
export async function deleteConversation(id: string): Promise<void> {
  await send(`/program/conversations/${id}`, { method: "DELETE" });
}
export async function runConversation(id: string, task: string, onEvent: (e: ProgramEvent) => void, signal?: AbortSignal): Promise<void> {
  await streamEvents(`/program/conversations/${id}/run`, { task }, onEvent, signal);
}
export async function listFiles(): Promise<FileEntry[]> {
  return (await send("/program/files", { method: "GET" })).json();
}
export async function readFile(path: string): Promise<{ path: string; content: string; editable: boolean }> {
  return (await send(`/program/files/content?path=${encodeURIComponent(path)}`, { method: "GET" })).json();
}
export async function saveFile(path: string, content: string, conversation: string | null): Promise<void> {
  await send("/program/files/content", put({ path, content, conversation }));
}
export async function runFile(path: string, conversation: string | null): Promise<{ ok: boolean; output: string }> {
  return (await send("/program/files/run", json({ path, conversation }))).json();
}
```

- [ ] **Step 3: Run tests and type-check** — `timeout 300 npx vitest run src/model/chat.test.ts src/api/program.test.ts` PASS. `npx tsc --noEmit` will fail only where `ProgramStep.tsx` still uses `runTask`/`rerun`; that file is deleted in Task 6, so in this task leave tsc failures limited to `ProgramStep.tsx` and say so in the report.

- [ ] **Step 4: Commit**

```bash
git add frontend/src/api/program.ts frontend/src/api/program.test.ts frontend/src/model/chat.ts frontend/src/model/chat.test.ts
git commit -m "feat: conversation and file client, chat message fold

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: Program mode shell, session hook, conversations sidebar

**Files:**
- Create: `frontend/src/ui/program/useRosSession.ts`, `frontend/src/ui/program/ProgramMode.tsx`, `frontend/src/ui/program/ConversationList.tsx`
- Modify: `frontend/src/App.tsx`, `frontend/src/ui/steps/StepPanel.tsx` (remove the `program` case), `frontend/src/ui/theme.css`
- Delete: `frontend/src/ui/steps/ProgramStep.tsx`, `frontend/src/ui/steps/program.test.tsx`
- Test: `frontend/src/ui/program/programMode.test.tsx`

**Interfaces:**
- Consumes: Task 5 client; existing `RosBridge`, `FrameTree`, `homeFrames`, `bodyPoses`, `sceneObjects`, `SCENE_COMPONENTS`; `buildExportRequest`, `withNames`; `Viewer.setBodyPoses/setSceneObjects`.
- Produces:
  - `useRosSession(ctx: Ctx): { status: SessionStatus; error: string | null; setError(e: string | null): void; start(): Promise<void>; stop(): Promise<void>; restart(): Promise<void>; refresh(): Promise<void>; busy: boolean }` — exactly the logic now in `ProgramStep.tsx` (`start`, `restart`, `stop`, `refresh`, the polling effect with `POLL`, and the live-robot effect), moved verbatim with its `mounted` guard; export `POLL` from this file.
  - `ProgramMode({ ctx, preview }: { ctx: Ctx; preview: ReactNode })` — the full-window shell.
  - `ConversationList({ items, active, onSelect, onNew, onRename, onDelete, disabled })`.
  - Chat and Workspace slots are filled in Tasks 7–8; until then `ProgramMode` renders placeholders with `data-testid="chat-slot"` and `data-testid="workspace-slot"`, and `preview` inside the workspace slot.

- [ ] **Step 1: Write the failing tests** — `frontend/src/ui/program/programMode.test.tsx`:

```tsx
import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import * as api from "../../api/program";
import { boxBody } from "../../model/testBodies";
import { Harness, imported, state } from "../testing";
import ProgramMode from "./ProgramMode";
import { POLL } from "./useRosSession";

vi.mock("../../ros/rosbridge", () => ({
  RosBridge: class { ready = Promise.resolve(); subscribe() {} call() { return Promise.resolve({ scene: {} }); } close() {} },
}));
beforeEach(() => { POLL.starting = 10; POLL.ready = 10_000; });
afterEach(() => vi.restoreAllMocks());

const conv = (id: string, title: string) => ({ id, title, created: 1, updated: 1, claude_session_id: null, edited: [], messages: [] });
function ui() {
  return render(<Harness start={imported([boxBody("base", [0, 0, 0], [1, 1, 1])])} step="program"
    ui={(ctx) => <ProgramMode ctx={ctx} preview={<div data-testid="viewer" />} />} />);
}
async function ready() {
  vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "ready", robot: "bot" });
  vi.spyOn(api, "startSession").mockResolvedValue({ state: "starting", robot: "bot" });
  ui();
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /start ros/i })); });
  await screen.findByText(/ROS ready/);
}

describe("ProgramMode", () => {
  it("fills the window, shows the preview and goes back to the robot", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([]);
    await ready();
    expect(screen.getByTestId("viewer")).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: /back to robot/i }));
    expect(state().step).toBe("export");
  });

  it("lists conversations, creates, switches and renames", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "Square" }]);
    vi.spyOn(api, "getConversation").mockImplementation(async (id) => conv(id, id === "aaaaaaaaaaaa" ? "Square" : "New conversation"));
    const create = vi.spyOn(api, "createConversation").mockResolvedValue(conv("bbbbbbbbbbbb", "New conversation"));
    const rename = vi.spyOn(api, "renameConversation").mockResolvedValue(conv("aaaaaaaaaaaa", "Square path"));
    await ready();
    await screen.findByText("Square");
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /new/i })); });
    expect(create).toHaveBeenCalled();
    expect(screen.getByTestId("conv-bbbbbbbbbbbb").className).toMatch(/active/);
    await act(async () => { fireEvent.doubleClick(screen.getByText("Square")); });
    fireEvent.change(screen.getByDisplayValue("Square"), { target: { value: "Square path" } });
    await act(async () => { fireEvent.keyDown(screen.getByDisplayValue("Square path"), { key: "Enter" }); });
    expect(rename).toHaveBeenCalledWith("aaaaaaaaaaaa", "Square path");
  });

  it("shows broken conversations without crashing", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "cccccccccccc", title: "Unreadable conversation", broken: true }]);
    await ready();
    expect((await screen.findByText("Unreadable conversation")).closest("li")!.className).toMatch(/broken/);
  });
});
```

Run — Expected: FAIL (module missing).

- [ ] **Step 2: Implement**

`frontend/src/ui/program/useRosSession.ts`: create the hook by moving, unchanged, from `ProgramStep.tsx`: the `status`, `error`, `busy` state and `mounted` ref; `start`, `restart`, `stop`, `refresh`; the polling effect; the live-robot effect (the one that creates `RosBridge` and calls `viewer.setBodyPoses`/`setSceneObjects`); export `POLL`. It returns `{ status, error, setError, start, stop, restart, refresh, busy }`.

`frontend/src/ui/program/ConversationList.tsx`:

```tsx
import { Plus, Trash2 } from "lucide-react";
import { useState } from "react";
import type { ConversationInfo } from "../../api/program";

interface Props {
  items: ConversationInfo[]; active: string | null; disabled: boolean;
  onSelect(id: string): void; onNew(): void; onRename(id: string, title: string): void; onDelete(id: string): void;
}

export default function ConversationList({ items, active, disabled, onSelect, onNew, onRename, onDelete }: Props) {
  const [editing, setEditing] = useState<string | null>(null);
  const [draft, setDraft] = useState("");
  return (
    <nav className="conversations">
      <button className="btn" onClick={onNew} disabled={disabled}><Plus size={14} />New</button>
      <ul>
        {items.map((c) => (
          <li key={c.id} data-testid={`conv-${c.id}`} className={`${c.id === active ? "active" : ""}${c.broken ? " broken" : ""}`}>
            {editing === c.id ? (
              <input autoFocus value={draft} onChange={(e) => setDraft(e.target.value)}
                onBlur={() => setEditing(null)}
                onKeyDown={(e) => {
                  if (e.key === "Enter" && draft.trim()) { onRename(c.id, draft.trim()); setEditing(null); }
                  if (e.key === "Escape") setEditing(null);
                }} />
            ) : (
              <button className="conv-title" disabled={c.broken || disabled} onClick={() => onSelect(c.id)}
                onDoubleClick={() => { if (!c.broken) { setEditing(c.id); setDraft(c.title); } }}>{c.title}</button>
            )}
            <button className="icon" aria-label={`Delete ${c.title}`} disabled={disabled}
              onClick={() => { if (window.confirm(`Delete "${c.title}"?`)) onDelete(c.id); }}><Trash2 size={12} /></button>
          </li>
        ))}
      </ul>
    </nav>
  );
}
```

`frontend/src/ui/program/ProgramMode.tsx`:

```tsx
import { ArrowLeft, Play, RotateCcw, Square } from "lucide-react";
import { useEffect, useState, type ReactNode } from "react";
import * as api from "../../api/program";
import type { Conversation, ConversationInfo } from "../../api/program";
import type { Ctx } from "../ctx";
import ConversationList from "./ConversationList";
import { useRosSession } from "./useRosSession";

export default function ProgramMode({ ctx, preview }: { ctx: Ctx; preview: ReactNode }) {
  const ros = useRosSession(ctx);
  const { status } = ros;
  const [convs, setConvs] = useState<ConversationInfo[]>([]);
  const [active, setActive] = useState<Conversation | null>(null);
  const [running, setRunning] = useState(false);
  const known = status.state === "ready" || status.state === "starting";

  async function reload(select?: string) {
    try {
      const list = await api.listConversations();
      setConvs(list);
      const id = select ?? active?.id ?? list.find((c) => !c.broken)?.id;
      if (id) setActive(await api.getConversation(id));
    } catch (e) { ros.setError((e as Error).message); }
  }
  useEffect(() => { if (known) void reload(); }, [known]);  // eslint-disable-line react-hooks/exhaustive-deps

  async function newConversation() {
    const c = await api.createConversation();
    setActive(c);
    setConvs((cs) => [{ id: c.id, title: c.title, created: c.created, updated: c.updated }, ...cs]);
  }

  return (
    <div className="program-mode">
      <header className="program-bar">
        <button className="btn" onClick={() => ctx.goTo("export")}><ArrowLeft size={14} />Back to robot</button>
        <strong>{ctx.robotName}</strong>
        <span className={`dot ${status.state}`} />
        <span className="muted">
          {status.state === "starting" && "Starting ROS… (first start builds the workspace)"}
          {status.state === "ready" && "ROS ready"}
          {status.state === "failed" && "ROS session stopped"}
          {status.state === "stopped" && "ROS not running"}
        </span>
        <span className="spacer" />
        {status.state === "stopped" || status.state === "failed"
          ? <button className="btn" onClick={() => void ros.start()}><Play size={14} />Start ROS</button>
          : <button className="btn" onClick={() => void ros.stop()} disabled={ros.busy || running}><Square size={14} />Stop ROS</button>}
        {(status.state === "ready" || status.state === "failed") &&
          <button className="btn" onClick={() => void ros.restart()} disabled={ros.busy || running}><RotateCcw size={14} />Restart session</button>}
      </header>
      {ros.error && <p className="error program-error">{ros.error}</p>}
      {status.state === "failed" && status.log && <pre className="log program-error">{status.log}</pre>}
      <div className="program-columns">
        <ConversationList items={convs} active={active?.id ?? null} disabled={running || !known}
          onSelect={(id) => void reload(id)} onNew={() => void newConversation()}
          onRename={async (id, title) => { await api.renameConversation(id, title); await reload(id); }}
          onDelete={async (id) => { await api.deleteConversation(id); if (active?.id === id) setActive(null); await reload(); }} />
        <section className="chat-column" data-testid="chat-slot" />
        <section className="workspace-column" data-testid="workspace-slot">{preview}</section>
      </div>
    </div>
  );
}
```

(`running`/`setRunning` are wired by Task 7.)

In `frontend/src/App.tsx`, after all hooks and just before the existing `return (`, add:

```tsx
  if (step === "program") {
    return (
      <ProgramMode ctx={ctx} preview={(
        <div className="viewer-pane">
          <ViewerPane bodies={bodies} ignored={model.ignored} selected={selected} tints={tints}
            approximate={approximate} glyphs={glyphs} loose={loose} onPick={onPick} onViewer={setViewer} />
          <ViewCube viewer={viewer} />
        </div>
      )} />
    );
  }
```

(import `ProgramMode` from `./ui/program/ProgramMode`; use the same props the normal layout passes to `ViewerPane`).

Delete `frontend/src/ui/steps/ProgramStep.tsx` and `frontend/src/ui/steps/program.test.tsx`; remove the `program` case and import from `StepPanel.tsx` (return `null` for it).

Append to `frontend/src/ui/theme.css`:

```css
.program-mode { position: fixed; inset: 0; display: flex; flex-direction: column; background: var(--bg); }
.program-bar { display: flex; align-items: center; gap: 8px; padding: 8px 12px; border-bottom: 1px solid var(--border); }
.program-bar .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--muted); }
.program-bar .dot.ready { background: var(--success); } .program-bar .dot.starting { background: var(--warning); }
.program-bar .dot.failed { background: var(--danger); }
.program-error { margin: 4px 12px; }
.program-columns { flex: 1; min-height: 0; display: grid; grid-template-columns: auto minmax(320px, auto) 1fr; }
.conversations { width: 220px; min-width: 160px; max-width: 360px; resize: horizontal; overflow: auto; border-right: 1px solid var(--border); padding: 8px; }
.conversations ul { list-style: none; padding: 0; margin: 8px 0 0; display: grid; gap: 2px; }
.conversations li { display: flex; align-items: center; gap: 4px; border-radius: 6px; }
.conversations li.active { background: var(--accent-soft, rgba(99,102,241,.15)); }
.conversations li.broken .conv-title { color: var(--danger); }
.conversations .conv-title { flex: 1; text-align: left; background: none; border: 0; padding: 6px; color: inherit; cursor: pointer; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.chat-column { width: 440px; min-width: 320px; max-width: 70vw; resize: horizontal; overflow: hidden; border-right: 1px solid var(--border); display: flex; flex-direction: column; }
.workspace-column { min-width: 0; display: flex; flex-direction: column; }
.viewer-pane { position: relative; flex: 1; min-height: 0; }
```

- [ ] **Step 3: Run tests, full suite, type-check** — Expected: PASS, tsc clean. If App-level tests assumed the Program step renders inside the step panel, update only those expectations.

- [ ] **Step 4: Commit**

```bash
git add -A frontend/src
git commit -m "feat: full-window program mode with a conversations sidebar

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 7: Chat column

**Files:**
- Create: `frontend/src/ui/program/Chat.tsx`
- Modify: `frontend/src/ui/program/ProgramMode.tsx` (fill the chat slot, run handling), `frontend/package.json` (+`react-markdown@9.0.1`), `frontend/src/ui/theme.css`
- Test: `frontend/src/ui/program/chat.test.tsx`

**Interfaces:**
- Consumes: `Message`, `ClaudeMessage`, `applyEvent`, `emptyClaude` (Task 5); `runConversation`, `getConversation`.
- Produces: `Chat({ messages, live, running, disabled, onSend })` where `live: ClaudeMessage | null` is the in-progress message; `onSend(task: string): void`. In `ProgramMode`: `send(task)` runs the active conversation (creating one first if none), streams into `live`, then reloads the conversation from the server; aborts on unmount; refreshes ROS status after an error; the stream writes only into the conversation it started in (`runningFor` ref).

- [ ] **Step 1: Install and write the failing tests**

Run: `cd frontend && npm install react-markdown@9.0.1 --save-exact`.

`frontend/src/ui/program/chat.test.tsx`:

```tsx
import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import type { Message } from "../../model/chat";
import Chat from "./Chat";

const msgs: Message[] = [
  { role: "user", text: "Move the tool" },
  { role: "claude", items: [{ kind: "text", text: "Planning **carefully**" }, { kind: "file", path: "programs/task_001.py" },
    { kind: "run", n: 1, ok: false, message: "failed: Unreachable: too far", output: "trace...\nRESULT: failed" },
    { kind: "run", n: 2, ok: true, message: "ok", output: "RESULT: ok" }],
    summary: "**Moved** the tool.\n- run 2 ok", result: { ok: true, runs: 2 }, error: null, program: "programs/task_001.py" },
  { role: "run", path: "bot_tasks/programs/task_001.py", ok: true, output: "RESULT: ok" },
];

describe("Chat", () => {
  it("renders bubbles, markdown, steps and one result line", () => {
    render(<Chat messages={msgs} live={null} running={false} disabled={false} onSend={() => {}} />);
    expect(screen.getByText("Move the tool").closest(".bubble")).toBeTruthy();
    expect(screen.getByText("carefully").tagName).toBe("STRONG");
    expect(screen.getByText("Moved").tagName).toBe("STRONG");
    expect(screen.getAllByText(/Moved/)).toHaveLength(1);
    expect(screen.getByText(/Wrote programs\/task_001\.py/)).toBeTruthy();
    expect(screen.getByText(/Run 1 ✗ failed: Unreachable: too far/)).toBeTruthy();
    expect(screen.getByText(/Worked on run 2\/3/)).toBeTruthy();
    expect(screen.getByText(/You ran bot_tasks\/programs\/task_001\.py ✓/)).toBeTruthy();
  });

  it("expands a run to its full output", () => {
    render(<Chat messages={msgs} live={null} running={false} disabled={false} onSend={() => {}} />);
    expect(screen.queryByText(/trace\.\.\./)).toBeNull();
    fireEvent.click(screen.getByText(/Run 1 ✗/));
    expect(screen.getByText(/trace\.\.\./)).toBeTruthy();
  });

  it("sends with Enter, keeps Shift+Enter for new lines, and is disabled while running", () => {
    const onSend = vi.fn();
    const { rerender } = render(<Chat messages={[]} live={null} running={false} disabled={false} onSend={onSend} />);
    const box = screen.getByLabelText(/describe a task/i);
    fireEvent.change(box, { target: { value: "go" } });
    fireEvent.keyDown(box, { key: "Enter", shiftKey: true });
    expect(onSend).not.toHaveBeenCalled();
    fireEvent.keyDown(box, { key: "Enter" });
    expect(onSend).toHaveBeenCalledWith("go");
    rerender(<Chat messages={[]} live={{ role: "claude", items: [], summary: "", result: null, error: null, program: null }}
      running={true} disabled={false} onSend={onSend} />);
    expect((screen.getByLabelText(/describe a task/i) as HTMLTextAreaElement).disabled).toBe(true);
    expect(screen.getByText(/Working/)).toBeTruthy();
  });
});
```

Append to `programMode.test.tsx`:

```tsx
it("a stream keeps writing into the conversation it started in", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }, { id: "bbbbbbbbbbbb", title: "B" }]);
  vi.spyOn(api, "getConversation").mockImplementation(async (id) => conv(id, id === "aaaaaaaaaaaa" ? "A" : "B"));
  let finish!: () => void;
  const run = vi.spyOn(api, "runConversation").mockImplementation(async (_id, _t, on) => {
    on({ kind: "text", text: "streaming for A" });
    await new Promise<void>((r) => { finish = r; });
  });
  await ready();
  await screen.findByText("A");
  fireEvent.change(screen.getByLabelText(/describe a task/i), { target: { value: "go" } });
  await act(async () => { fireEvent.keyDown(screen.getByLabelText(/describe a task/i), { key: "Enter" }); });
  expect(run.mock.calls[0][0]).toBe("aaaaaaaaaaaa");
  expect(screen.getByText("streaming for A")).toBeTruthy();
  expect((screen.getByText("B") as HTMLButtonElement).disabled).toBe(true);   // switching is locked while running
  await act(async () => { finish(); });
});
```

Run — Expected: FAIL.

- [ ] **Step 2: Implement** — `frontend/src/ui/program/Chat.tsx`:

```tsx
import { ChevronDown, ChevronRight, FileCode, Loader2, Send } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import Markdown from "react-markdown";
import type { ClaudeMessage, Item, Message } from "../../model/chat";

const MAX_RUNS = 3;

function Step({ item }: { item: Item }) {
  const [open, setOpen] = useState(false);
  if (item.kind === "text") return <div className="narration"><Markdown>{item.text}</Markdown></div>;
  if (item.kind === "file") return <div className="step"><FileCode size={12} />Wrote {item.path}</div>;
  const mark = item.ok === undefined ? "…" : item.ok ? "✓" : "✗";
  return (
    <div className={`step run ${item.ok === false ? "bad" : item.ok ? "ok" : ""}`}>
      <button className="step-head" onClick={() => setOpen((o) => !o)}>
        {open ? <ChevronDown size={12} /> : <ChevronRight size={12} />}Run {item.n} {mark} {item.message ?? "running"}
      </button>
      {open && item.output && <pre className="step-output">{item.output}</pre>}
    </div>
  );
}

function Claude({ m, live }: { m: ClaudeMessage; live: boolean }) {
  return (
    <div className="claude-msg">
      {m.items.map((it, i) => <Step key={i} item={it} />)}
      {m.summary && <div className="answer"><Markdown>{m.summary}</Markdown></div>}
      {m.result && <div className={`result ${m.result.ok ? "ok" : "bad"}`}>
        {m.result.ok ? `✓ Worked on run ${m.result.runs}/${MAX_RUNS}` : `✗ Failed after ${m.result.runs} run${m.result.runs === 1 ? "" : "s"}`}
      </div>}
      {m.error && <div className="error">{m.error}</div>}
      {live && !m.result && !m.error && <div className="muted"><Loader2 size={12} className="spin" /> Working…</div>}
    </div>
  );
}

interface Props { messages: Message[]; live: ClaudeMessage | null; running: boolean; disabled: boolean; onSend(task: string): void }

export default function Chat({ messages, live, running, disabled, onSend }: Props) {
  const [task, setTask] = useState("");
  const end = useRef<HTMLDivElement>(null);
  useEffect(() => { end.current?.scrollIntoView?.({ block: "end" }); }, [messages, live]);
  const submit = () => { const t = task.trim(); if (t && !running && !disabled) { onSend(t); setTask(""); } };
  return (
    <>
      <div className="chat-scroll">
        {messages.map((m, i) => m.role === "user" ? <div key={i} className="bubble">{m.text}</div>
          : m.role === "run" ? <div key={i} className={`step ${m.ok ? "ok" : "bad"}`}>You ran {m.path} {m.ok ? "✓" : "✗"}</div>
          : <Claude key={i} m={m} live={false} />)}
        {live && <Claude m={live} live={true} />}
        <div ref={end} />
      </div>
      <div className="prompt">
        <textarea aria-label="Describe a task" value={task} disabled={running || disabled} rows={3}
          placeholder="Describe a task, e.g. pick the box at (0.5, 0.2) and place it at (0.4, -0.3)"
          onChange={(e) => setTask(e.target.value)}
          onKeyDown={(e) => { if (e.key === "Enter" && !e.shiftKey) { e.preventDefault(); submit(); } }} />
        <button className="btn btn-primary" onClick={submit} disabled={running || disabled || !task.trim()}><Send size={14} />Send</button>
      </div>
    </>
  );
}
```

In `ProgramMode.tsx`: replace the chat slot with

```tsx
        <section className="chat-column" data-testid="chat-slot">
          <Chat messages={active?.messages ?? []} live={live} running={running} disabled={status.state !== "ready"} onSend={(t) => void send(t)} />
        </section>
```

and add state/logic:

```tsx
  const [live, setLive] = useState<ClaudeMessage | null>(null);
  const abort = useRef<AbortController | null>(null);
  useEffect(() => () => abort.current?.abort(), []);

  async function send(task: string) {
    let conv = active;
    if (!conv) { conv = await api.createConversation(); setActive(conv); }
    const id = conv.id;
    setActive((a) => (a && a.id === id ? { ...a, messages: [...a.messages, { role: "user", text: task }] } : a));
    setRunning(true);
    let msg = emptyClaude();
    setLive(msg);
    const ctl = new AbortController();
    abort.current = ctl;
    try {
      await api.runConversation(id, task, (e) => { msg = applyEvent(msg, e); setLive(msg); }, ctl.signal);
    } catch (e) {
      if ((e as Error).name === "AbortError") return;
      msg = { ...msg, error: (e as Error).message };
    }
    setRunning(false);
    setLive(null);
    await reload(id);
    if (msg.error) await ros.refresh();
  }
```

(imports: `useRef`, `Chat`, `applyEvent`, `emptyClaude`, `ClaudeMessage`). The sidebar's `disabled={running || !known}` already locks switching while a stream runs.

Append CSS:

```css
.chat-scroll { flex: 1; overflow: auto; padding: 12px; display: flex; flex-direction: column; gap: 10px; }
.bubble { align-self: flex-end; max-width: 85%; background: var(--accent, #6366f1); color: #fff; padding: 8px 12px; border-radius: 12px 12px 2px 12px; white-space: pre-wrap; }
.claude-msg { display: flex; flex-direction: column; gap: 4px; }
.narration { color: var(--muted); font-size: 13px; } .narration p { margin: 2px 0; }
.step { font-size: 12px; display: flex; flex-direction: column; gap: 2px; color: var(--muted); }
.step.ok, .result.ok { color: var(--success); } .step.bad, .result.bad { color: var(--danger); }
.step-head { background: none; border: 0; padding: 2px 0; color: inherit; cursor: pointer; display: flex; align-items: center; gap: 4px; text-align: left; }
.step-output { font-size: 11px; max-height: 240px; overflow: auto; background: var(--panel); padding: 6px; border-radius: 6px; }
.answer p { margin: 4px 0; } .result { font-weight: 600; font-size: 13px; }
.prompt { border-top: 1px solid var(--border); padding: 8px; display: flex; gap: 8px; align-items: flex-end; }
.prompt textarea { flex: 1; resize: none; }
```

- [ ] **Step 3: Run tests, full suite, type-check** — Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add frontend/package.json frontend/package-lock.json frontend/src
git commit -m "feat: chat column with markdown, collapsible steps and one result line

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: Workspace — Preview / Code tabs, file tree, editor

**Files:**
- Create: `frontend/src/ui/program/Workspace.tsx`, `frontend/src/ui/program/FileTree.tsx`, `frontend/src/ui/program/Editor.tsx`
- Modify: `frontend/src/ui/program/ProgramMode.tsx`, `frontend/package.json` (+`codemirror@6.0.1`, `@codemirror/lang-python@6.1.6`), `frontend/src/ui/theme.css`
- Test: `frontend/src/ui/program/workspace.test.tsx`

**Interfaces:**
- Consumes: `listFiles`, `readFile`, `saveFile`, `runFile` (Task 5); `ProgramMode`'s `active`, `running`, and a `changed: Set<string>` it computes (paths of `file` items in the last Claude message of the active conversation, prefixed with `<robot>_tasks/`, plus files you saved since).
- Produces: `Workspace({ preview, conversation, running, changed, onRan })`; `FileTree({ files, open, changed, onOpen })`; `Editor({ value, readOnly, onChange })` (CodeMirror wrapper). `onRan()` makes `ProgramMode` reload the conversation (the backend appended a "You ran" message).

- [ ] **Step 1: Install and write the failing tests**

Run: `cd frontend && npm install codemirror@6.0.1 @codemirror/lang-python@6.1.6 --save-exact`.

`frontend/src/ui/program/workspace.test.tsx`:

```tsx
import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as api from "../../api/program";
import Workspace from "./Workspace";

vi.mock("./Editor", () => ({
  default: ({ value, readOnly, onChange }: { value: string; readOnly: boolean; onChange(v: string): void }) =>
    <textarea data-testid="editor" value={value} readOnly={readOnly} onChange={(e) => onChange(e.target.value)} />,
}));
afterEach(() => vi.restoreAllMocks());

const files = [{ path: "bot_program/robot.py", editable: false }, { path: "bot_tasks/programs/task_001.py", editable: true }];
function ui(running = false, onRan = vi.fn()) {
  vi.spyOn(api, "listFiles").mockResolvedValue(files);
  vi.spyOn(api, "readFile").mockImplementation(async (p) => ({ path: p, content: `# ${p}`, editable: p.includes("programs") }));
  return render(<Workspace preview={<div data-testid="viewer" />} conversation="aaaaaaaaaaaa" running={running}
    changed={new Set(["bot_tasks/programs/task_001.py"])} onRan={onRan} />);
}

describe("Workspace", () => {
  it("switches between Preview and Code and keeps the viewer mounted", async () => {
    ui();
    expect(screen.getByTestId("viewer")).toBeTruthy();
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
    expect(screen.getByTestId("viewer")).toBeTruthy();          // hidden, not unmounted
    expect(await screen.findByText("task_001.py")).toBeTruthy();
    expect(screen.getByText("task_001.py").closest("li")!.className).toMatch(/changed/);
  });

  it("opens a program, saves it with its conversation, then runs it", async () => {
    const save = vi.spyOn(api, "saveFile").mockResolvedValue();
    const run = vi.spyOn(api, "runFile").mockResolvedValue({ ok: true, output: "RESULT: ok" });
    const onRan = vi.fn();
    ui(false, onRan);
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
    await act(async () => { fireEvent.click(await screen.findByText("task_001.py")); });
    fireEvent.change(screen.getByTestId("editor"), { target: { value: "print(2)" } });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /save/i })); });
    expect(save).toHaveBeenCalledWith("bot_tasks/programs/task_001.py", "print(2)", "aaaaaaaaaaaa");
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /^run$/i })); });
    expect(run).toHaveBeenCalledWith("bot_tasks/programs/task_001.py", "aaaaaaaaaaaa");
    expect(onRan).toHaveBeenCalled();
  });

  it("library files are read-only and nothing can be saved while a task runs", async () => {
    ui(true);
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
    await act(async () => { fireEvent.click(await screen.findByText("robot.py")); });
    expect((screen.getByTestId("editor") as HTMLTextAreaElement).readOnly).toBe(true);
    expect(screen.queryByRole("button", { name: /save/i })).toBeNull();
    await act(async () => { fireEvent.click(screen.getByText("task_001.py")); });
    expect((screen.getByRole("button", { name: /save/i }) as HTMLButtonElement).disabled).toBe(true);
  });
});
```

Run — Expected: FAIL.

- [ ] **Step 2: Implement**

`frontend/src/ui/program/Editor.tsx`:

```tsx
import { python } from "@codemirror/lang-python";
import { EditorView, basicSetup } from "codemirror";
import { EditorState } from "@codemirror/state";
import { useEffect, useRef } from "react";

/** CodeMirror 6 with Python highlighting. `value` replaces the document when it changes from outside. */
export default function Editor({ value, readOnly, onChange }: { value: string; readOnly: boolean; onChange(v: string): void }) {
  const host = useRef<HTMLDivElement>(null);
  const view = useRef<EditorView | null>(null);
  const change = useRef(onChange);
  change.current = onChange;
  useEffect(() => {
    view.current = new EditorView({
      parent: host.current!,
      state: EditorState.create({ doc: value, extensions: [basicSetup, python(), EditorState.readOnly.of(readOnly),
        EditorView.updateListener.of((u) => { if (u.docChanged) change.current(u.state.doc.toString()); })] }),
    });
    return () => view.current?.destroy();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [readOnly]);
  useEffect(() => {
    const v = view.current;
    if (v && v.state.doc.toString() !== value) v.dispatch({ changes: { from: 0, to: v.state.doc.length, insert: value } });
  }, [value]);
  return <div className="editor" ref={host} />;
}
```

(`@codemirror/state` ships as a dependency of `codemirror`; import it as shown — no extra install.)

`frontend/src/ui/program/FileTree.tsx`:

```tsx
import type { FileEntry } from "../../api/program";

const GROUPS: { label: string; match: (p: string) => boolean }[] = [
  { label: "Programs", match: (p) => /_tasks\/programs\//.test(p) },
  { label: "Library", match: (p) => /^[^/]+_program\//.test(p) },
  { label: "Robot", match: () => true },
];

export default function FileTree({ files, open, changed, onOpen }: {
  files: FileEntry[]; open: string | null; changed: Set<string>; onOpen(path: string): void;
}) {
  const seen = new Set<string>();
  return (
    <nav className="file-tree">
      {GROUPS.map((g) => {
        const items = files.filter((f) => !seen.has(f.path) && g.match(f.path));
        items.forEach((f) => seen.add(f.path));
        return items.length ? (
          <div key={g.label}>
            <div className="tree-group">{g.label}</div>
            <ul>
              {items.map((f) => (
                <li key={f.path} title={f.path}
                  className={`${f.path === open ? "open" : ""}${changed.has(f.path) ? " changed" : ""}${f.editable ? "" : " readonly"}`}>
                  <button onClick={() => onOpen(f.path)}>{f.path.split("/").pop()}</button>
                </li>
              ))}
            </ul>
          </div>
        ) : null;
      })}
    </nav>
  );
}
```

`frontend/src/ui/program/Workspace.tsx`:

```tsx
import { Play, Save } from "lucide-react";
import { useEffect, useState, type ReactNode } from "react";
import * as api from "../../api/program";
import type { FileEntry } from "../../api/program";
import Editor from "./Editor";
import FileTree from "./FileTree";

interface Props { preview: ReactNode; conversation: string | null; running: boolean; changed: Set<string>; onRan(): void }

export default function Workspace({ preview, conversation, running, changed, onRan }: Props) {
  const [tab, setTab] = useState<"preview" | "code">("preview");
  const [files, setFiles] = useState<FileEntry[]>([]);
  const [file, setFile] = useState<{ path: string; content: string; editable: boolean } | null>(null);
  const [text, setText] = useState("");
  const [note, setNote] = useState<string | null>(null);
  const dirty = !!file && text !== file.content;

  useEffect(() => { if (tab === "code" && !running) void api.listFiles().then(setFiles).catch((e) => setNote(e.message)); }, [tab, running]);
  useEffect(() => { if (!running && file && !dirty) void openFile(file.path); }, [running]);  // reload after a task  // eslint-disable-line react-hooks/exhaustive-deps

  async function openFile(path: string) {
    try { const f = await api.readFile(path); setFile(f); setText(f.content); setNote(null); }
    catch (e) { setNote((e as Error).message); }
  }
  async function save() {
    if (!file) return;
    try { await api.saveFile(file.path, text, conversation); setFile({ ...file, content: text }); setNote("Saved"); }
    catch (e) { setNote((e as Error).message); }
  }
  async function run() {
    if (!file) return;
    if (dirty) await save();
    try { const r = await api.runFile(file.path, conversation); setNote(r.ok ? "Run ok" : r.output.trim().split("\n").pop() ?? "Run failed"); onRan(); }
    catch (e) { setNote((e as Error).message); }
  }

  return (
    <>
      <div className="tabs" role="tablist">
        <button role="tab" aria-selected={tab === "preview"} className={tab === "preview" ? "active" : ""} onClick={() => setTab("preview")}>Preview</button>
        <button role="tab" aria-selected={tab === "code"} className={tab === "code" ? "active" : ""} onClick={() => setTab("code")}>Code</button>
      </div>
      <div className="tab-body" style={{ display: tab === "preview" ? "flex" : "none" }}>{preview}</div>
      {tab === "code" && (
        <div className="code-tab">
          <FileTree files={files} open={file?.path ?? null} changed={changed} onOpen={(p) => void openFile(p)} />
          <div className="code-pane">
            {file ? (
              <>
                <div className="code-bar">
                  <span className="muted small">{file.path}{dirty ? " •" : ""}{file.editable ? "" : " (generated, read-only)"}</span>
                  <span className="spacer" />
                  {note && <span className="muted small">{note}</span>}
                  {file.editable && <>
                    <button className="btn" onClick={() => void save()} disabled={running || !dirty}><Save size={14} />Save</button>
                    <button className="btn" onClick={() => void run()} disabled={running}><Play size={14} />Run</button>
                  </>}
                </div>
                <Editor value={text} readOnly={!file.editable || running} onChange={setText} />
              </>
            ) : <p className="muted pad">Pick a file. Programs under Programs can be edited and run.</p>}
          </div>
        </div>
      )}
    </>
  );
}
```

In `ProgramMode.tsx`, replace the workspace slot with:

```tsx
        <section className="workspace-column" data-testid="workspace-slot">
          <Workspace preview={preview} conversation={active?.id ?? null} running={running} changed={changed}
            onRan={() => { if (active) void reload(active.id); }} />
        </section>
```

and compute `changed`:

```tsx
  const changed = useMemo(() => {
    const out = new Set<string>();
    const lastClaude = [...(active?.messages ?? [])].reverse().find((m) => m.role === "claude");
    if (lastClaude && lastClaude.role === "claude" && status.robot) {
      for (const it of lastClaude.items) if (it.kind === "file") out.add(`${status.robot}_tasks/${it.path}`);
    }
    return out;
  }, [active, status.robot]);
```

Append CSS:

```css
.tabs { display: flex; gap: 2px; border-bottom: 1px solid var(--border); padding: 4px 8px 0; }
.tabs button { background: none; border: 0; padding: 6px 12px; color: var(--muted); cursor: pointer; border-bottom: 2px solid transparent; }
.tabs button.active { color: inherit; border-bottom-color: var(--accent, #6366f1); }
.tab-body { flex: 1; min-height: 0; }
.code-tab { flex: 1; min-height: 0; display: grid; grid-template-columns: 220px 1fr; }
.file-tree { border-right: 1px solid var(--border); overflow: auto; padding: 8px; font-size: 13px; }
.file-tree ul { list-style: none; margin: 0 0 8px; padding: 0; }
.file-tree .tree-group { font-size: 11px; text-transform: uppercase; color: var(--muted); margin: 6px 0 2px; }
.file-tree button { background: none; border: 0; color: inherit; padding: 3px 6px; cursor: pointer; width: 100%; text-align: left; }
.file-tree li.open button { background: var(--accent-soft, rgba(99,102,241,.15)); border-radius: 4px; }
.file-tree li.readonly button { color: var(--muted); }
.file-tree li.changed button::after { content: " ●"; color: var(--warning); }
.code-pane { min-width: 0; display: flex; flex-direction: column; }
.code-bar { display: flex; align-items: center; gap: 8px; padding: 6px 8px; border-bottom: 1px solid var(--border); }
.editor { flex: 1; min-height: 0; overflow: auto; } .editor .cm-editor { height: 100%; }
.pad { padding: 12px; }
```

- [ ] **Step 3: Run tests, full suite, type-check, production build** — `timeout 300 npx vitest run src/ui/program`, `timeout 600 npm test`, `timeout 300 npx tsc --noEmit`, `timeout 600 npm run build` — Expected: all PASS (the build proves CodeMirror and react-markdown bundle).

- [ ] **Step 4: Commit**

```bash
git add frontend/package.json frontend/package-lock.json frontend/src
git commit -m "feat: Preview and Code tabs with file tree and CodeMirror editor

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 9: End-to-end check (controller inline)

- [ ] **Step 1:** Full suites: backend `pytest -q`; frontend `npm test`, `tsc --noEmit`, `npm run build`, `npm run e2e`. Expected: all pass.
- [ ] **Step 2:** Start the app (the user runs the backend and frontend in their own terminals). In the app: import the IRB 1300, connect all 6 joints, run **Find limits for all joints**, open Program, Start ROS. Check: Program mode fills the window; Back returns to the steps; a new conversation streams a simple task with formatted text, collapsible steps and one result line; the Code tab shows the program with a changed dot; edit + Save + Run works and appears in the chat.
- [ ] **Step 3:** The user runs the 5 benchmark tasks, one conversation each, and reports results; pass is ≥ 4/5 within 3 runs. Fix library/brief issues found, each with a test, and re-run the failed task once.
