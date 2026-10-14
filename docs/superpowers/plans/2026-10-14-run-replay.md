# Run Replay Implementation Plan

> **Status: deferred (2026-10-14).** Not built. Run again buttons (re-running the program live) cover replay for now; see commit ababc6a.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every finished program run (Claude's or manual) gets a ▶ Replay button in the chat. The button plays the recorded motion and objects back in the 3D view.

**Architecture:**
- **Recording:** a recorder thread in the container's `runner.py` records `/tf` at 20 Hz and planning-scene changes, then writes `<robot>_tasks/runs/<stem>-<UTC>.json` and prints `TRACE: runs/<file>`.
- **Backend:** reads the `TRACE:` line into the run results, stores it with the conversation, and serves the trace files.
- **Frontend:** a player rebuilds a `FrameTree` per frame from the trace and reuses the live view's `bodyPoses` and `sceneObjects`. The live view pauses while a replay plays.

**Tech Stack:** ROS 2 Jazzy rclpy (container), FastAPI (Python 3.11), React 18 + three.js + vitest.

**Spec:** `docs/superpowers/specs/2026-10-14-run-replay-design.md`

## Global Constraints

- Backend tests:
  - Run from `backend/`: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q <files>`.
  - Run the full suite once before committing.
- Frontend tests, from `frontend/`:
  - `timeout 300 npx vitest run <files>` (never watch mode), `timeout 600 npm test` and `timeout 300 npx tsc --noEmit`.
  - `@testing-library/jest-dom` is not installed.
- Container check, from the repo root: `bash scripts/ros_check.sh corpus/out/<robot>.zip`.
  - It takes about 3 min per robot.
  - Run `irb1300` in the foreground. Run `six` and `four` in the background, or skip them if they are not needed.
- Trace file name: `<program stem>-<UTC %Y%m%dT%H%M%S%f>Z.json` in `<robot>_tasks/runs/`. The runner prints `TRACE: runs/<file>` before `RESULT:`, or `TRACE: none (<reason>)`.
- Trace JSON: `{"version": 1, "robot", "program", "base", "static": [tf...], "frames": [{"t", "tf": [tf...]}], "scenes": [{"t", "scene"}], "result"}`.
  - Each `tf` is a `geometry_msgs/TransformStamped` as a dict.
  - Each `scene` is a `moveit_msgs/PlanningScene` as a dict, the same shape rosbridge gives the live view.
- Rates and caps:
  - `/tf` sampled every 0.05 s.
  - The scene is polled every 0.2 s and kept only when it changed.
  - Recording stops after 300 s.
- Trace endpoint name pattern: `^[A-Za-z0-9_.-]+\.json$`.
- Commits end with `Co-Authored-By: <your model name> <noreply@anthropic.com>`.

## Review Focus

1. **A run killed by the timeout (`timeout` sends SIGTERM):** a trace is still written, up to the kill. The runner turns SIGTERM into `SystemExit`. Test: in Task 1, `run.sh` checks the trace of a program that sleeps past a short timeout.
2. **A program that raises mid-way:** its trace exists, has frames, and its `result` starts with `failed`. Test: Task 1's `run.sh` trace step.
3. **The recorder itself failing**, for example rclpy being unavailable: the run still prints `RESULT` normally, with `TRACE: none (…)`. Test: Task 1 backend test on the runner template text, plus the `try`/`except` around `Recorder` start.
4. **Trace files cluttering the Code tab:** `runs/` is excluded from the file tree. Test: Task 2.
5. **Replay pressed while a task is running, or switching conversations mid-replay:** the button is disabled while running, and a replay stops when the conversation changes. Test: Task 4.

---

## File Structure

| File | Responsibility |
|---|---|
| `backend/app/ros/templates/recorder.py` (create) | Container-side recorder thread (own rclpy context) |
| `backend/app/ros/templates/runner.py` (modify) | Start and stop the recorder around each run; SIGTERM becomes SystemExit; print `TRACE:` |
| `backend/app/ros/program_pkg.py` (modify) | Ship `recorder.py` in `<robot>_program/` |
| `docker/jazzy/check/run.sh` (modify) | Container checks of trace files |
| `backend/app/program/agent.py`, `session.py`, `conversations.py`, `files.py`, `routes.py` (modify) | `trace` in run results, fold, tree skip, `GET /program/traces/{name}` |
| `frontend/src/api/program.ts`, `frontend/src/model/chat.ts` (modify) | `trace` fields, `getTrace` |
| `frontend/src/model/replay.ts` (create) | `Trace` type, `frameAt`, `sceneAt`, `duration` |
| `frontend/src/ros/liveRobot.ts` (modify) | `robotView(model)` shared by the live view and replay |
| `frontend/src/ui/program/Replay.tsx` (create) | Player overlay |
| `frontend/src/ui/program/useRosSession.ts`, `Workspace.tsx`, `Chat.tsx`, `ProgramMode.tsx` (modify) | Pause live, overlay, buttons, wiring |

---

### Task 1: Container recorder and runner

**Files:**
- Create: `backend/app/ros/templates/recorder.py`
- Modify: `backend/app/ros/templates/runner.py`, `backend/app/ros/program_pkg.py` (`program_files`), `docker/jazzy/check/run.sh`
- Test: `backend/tests/test_ros_program_pkg.py` (append)

**Interfaces:**
- Produces:
  - The container prints `TRACE: runs/<file>.json` or `TRACE: none (<reason>)` on its own line before `RESULT:`.
  - `recorder.Recorder(path: str, program: str)` has `.start()` and `.stop(result: str) -> None`.

- [ ] **Step 1: Failing test.** Append to `backend/tests/test_ros_program_pkg.py`:

```python
def test_program_package_ships_the_recorder(ws):
    prog = ws / "bot_program"
    assert (prog / "recorder.py").exists()
    rec = (prog / "recorder.py").read_text()
    assert "class Recorder" in rec and "TRANSIENT_LOCAL" in rec and "message_to_ordereddict" in rec
    runner = (prog / "runner.py").read_text()
    assert "Recorder(" in runner and "TRACE: " in runner and "signal.SIGTERM" in runner
```

Run: `pytest -q tests/test_ros_program_pkg.py`. Expected: FAIL (no `recorder.py`).

- [ ] **Step 2: `backend/app/ros/templates/recorder.py`:**

```python
"""Records a run for replay: /tf at 20 Hz and planning-scene changes, in its own rclpy context so the
task program's node is untouched. Written as JSON next to the programs (runs/<program>-<UTC>.json)."""
import json
import threading
import time

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_msgs.msg import TFMessage

ROBOT = "__ROBOT__"
TF_PERIOD = 0.05
SCENE_PERIOD = 0.2
MAX_SECONDS = 300.0
SCENE = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY


class Recorder:
    def __init__(self, path: str, program: str):
        self.path, self.program = path, program
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx)
        self.node = rclpy.create_node("cad_to_urdf_recorder", context=self.ctx)
        self.latest, self.static, self.frames, self.scenes = {}, {}, [], []
        self.base, self._last_scene, self._pending = "", None, False
        self.node.create_subscription(TFMessage, "/tf", self._tf, 100)
        self.node.create_subscription(TFMessage, "/tf_static", self._tf_static,
                                      QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scene_client = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        self.exec = SingleThreadedExecutor(context=self.ctx)
        self.exec.add_node(self.node)
        self.t0 = None
        self.thread = threading.Thread(target=self._spin, daemon=True)

    def _now(self) -> float:
        return time.monotonic() - self.t0

    def _tf(self, m):
        for t in m.transforms:
            self.latest[t.child_frame_id] = t

    def _tf_static(self, m):
        for t in m.transforms:
            self.static[t.child_frame_id] = message_to_ordereddict(t)
            self.base = self.base or t.header.frame_id

    def _sample_tf(self):
        if self.latest and self._now() <= MAX_SECONDS:
            self.frames.append({"t": round(self._now(), 3),
                                "tf": [message_to_ordereddict(t) for t in self.latest.values()]})

    def _sample_scene(self):
        if self._pending or self._now() > MAX_SECONDS or not self.scene_client.service_is_ready():
            return
        self._pending = True
        req = GetPlanningScene.Request()
        req.components.components = SCENE
        self.scene_client.call_async(req).add_done_callback(self._got_scene)

    def _got_scene(self, fut):
        self._pending = False
        scene = message_to_ordereddict(fut.result().scene)
        key = json.dumps([scene["world"]["collision_objects"], scene["robot_state"]["attached_collision_objects"]],
                         sort_keys=True)
        if key != self._last_scene:
            self._last_scene = key
            self.scenes.append({"t": round(self._now(), 3), "scene": scene})

    def _spin(self):
        while self.ctx.ok() and not self._stop:
            self.exec.spin_once(timeout_sec=0.01)

    def start(self):
        self.t0, self._stop = time.monotonic(), False
        self.node.create_timer(TF_PERIOD, self._sample_tf)
        self.node.create_timer(SCENE_PERIOD, self._sample_scene)
        self.thread.start()

    def stop(self, result: str):
        self._stop = True
        self.thread.join(timeout=2)
        self._sample_tf()
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump({"version": 1, "robot": ROBOT, "program": self.program, "base": self.base,
                       "static": list(self.static.values()), "frames": self.frames, "scenes": self.scenes,
                       "result": result}, f)
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)
```

- [ ] **Step 3: `backend/app/ros/templates/runner.py`.** Replace the file with:

```python
"""Container side of run_task.sh: reset the simulation, run one program (recorded for replay), print RESULT."""
import os
import runpy
import signal
import sys
import time
import traceback
from pathlib import Path

from robot import Robot, RobotError


def _on_term(signum, frame):  # `timeout` sends SIGTERM: unwind so the trace is still written
    raise SystemExit(124)


def run(path: str) -> str:
    try:
        runpy.run_path(path, run_name="__main__")
    except RobotError as exc:
        return f"failed: {type(exc).__name__}: {exc}"
    except SystemExit as exc:
        if exc.code == 124:
            return "failed: timed out"
        if exc.code not in (None, 0):
            return f"failed: the program exited with {exc.code}"
    except Exception as exc:
        traceback.print_exc()
        return f"failed: {type(exc).__name__}: {exc}"
    return "ok"


def main(path: str) -> int:
    signal.signal(signal.SIGTERM, _on_term)
    try:
        Robot().reset()
    except Exception as exc:  # the session itself is broken; nothing the program can fix
        print(f"RESULT: failed: could not reset the simulation ({exc})", flush=True)
        return 1
    name = f"{Path(path).stem}-{time.strftime('%Y%m%dT%H%M%S', time.gmtime())}{int(time.time() * 1e6) % 1000000:06d}Z.json"
    rec, why = None, ""
    try:
        from recorder import Recorder
        os.makedirs("runs", exist_ok=True)
        rec = Recorder(os.path.join("runs", name), path)
        rec.start()
    except Exception as exc:  # recording is a nice-to-have; the run must not depend on it
        rec, why = None, f"{type(exc).__name__}: {exc}"
    result = "failed: interrupted"
    try:
        result = run(path)
    finally:
        if rec is not None:
            try:
                rec.stop(result)
                print(f"TRACE: runs/{name}", flush=True)
            except Exception as exc:
                print(f"TRACE: none ({type(exc).__name__}: {exc})", flush=True)
        else:
            print(f"TRACE: none ({why})", flush=True)
        print(f"RESULT: {result}", flush=True)
    return 0 if result == "ok" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
```

- [ ] **Step 4: Ship it.** In `program_pkg.program_files` add `"recorder.py": hash_note(source, robot) + _template("recorder.py", robot, source),`.

- [ ] **Step 5: Container trace checks.** In `docker/jazzy/check/run.sh`, after the `for p in /check/programs/*.py` loop, add:

```bash
# replay traces: a passing run, a failing run and a timed-out run each leave a readable trace
cat > /tmp/fails.py <<'PY'
from robot import Robot
r = Robot()
r.add_box("b", (0.05, 0.05, 0.05), (0.5, 0.0, 0.025))
raise RuntimeError("stop here")
PY
cat > /tmp/sleeps.py <<'PY'
import time
from robot import Robot
r = Robot()
r.add_box("b", (0.05, 0.05, 0.05), (0.5, 0.0, 0.025))
time.sleep(30)
PY
for spec in "/check/programs/pick_place.py:ok:180" "/tmp/fails.py:failed:180" "/tmp/sleeps.py:failed:8"; do
  IFS=: read -r prog want secs <<< "$spec"
  out=$(cd /ws/src/${ROBOT}_tasks && PYTHONPATH=/ws/src/${ROBOT}_program:$PYTHONPATH timeout "$secs" python3 -u -m runner "$prog" 2>&1) || true
  trace=$(echo "$out" | sed -n 's/^TRACE: \(runs\/.*\.json\)$/\1/p' | tail -n 1)
  if [ -n "$trace" ] && python3 - "/ws/src/${ROBOT}_tasks/$trace" "$want" <<'PY'
import json, sys
t = json.load(open(sys.argv[1]))
assert t["version"] == 1 and t["result"].startswith(sys.argv[2]), t["result"]
assert len(t["frames"]) >= 10 and t["static"] and t["scenes"], (len(t["frames"]), len(t["scenes"]))
assert any(s["scene"]["world"]["collision_objects"] or s["scene"]["robot_state"]["attached_collision_objects"] for s in t["scenes"])
PY
  then echo "PASS trace $(basename "$prog") ($want)"
  else echo "FAIL trace $(basename "$prog") ($want)"; echo "$out" | tail -n 15; status=1; fi
done
```

- [ ] **Step 6: Run.** Backend tests: `pytest -q tests/test_ros_program_pkg.py`, then the full suite. Expected: PASS.

  Container: `bash scripts/ros_check.sh corpus/out/irb1300.zip | grep -E "^(PASS|FAIL)"`. Expected:
  - all lines PASS, including `PASS trace pick_place.py (ok)`, `PASS trace fails.py (failed)` and `PASS trace sleeps.py (failed)`;
  - the `robot.py` program lines still PASS.

  If the `sleeps.py` trace fails because `timeout` sends SIGTERM to `python3` before the handler is set, report the output. Do not drop the check.

- [ ] **Step 7: Commit** `feat: record each run for replay`.

---

### Task 2: Backend — trace in run results, endpoint, tree skip

**Files:**
- Modify: `backend/app/program/agent.py` (`parse_stream`), `session.py` (`run_program`), `conversations.py` (`fold`), `files.py` (`SKIP_DIRS`), `routes.py`
- Test: `backend/tests/test_program_agent.py`, `test_program_conversations.py`, `test_program_files.py` (append); create `backend/tests/test_program_traces.py`

**Interfaces:**
- Consumes: the `TRACE: runs/<file>.json` line (Task 1).
- Produces:
  - `run_result` events and `run_program()` results gain `"trace": str | None` (for example `"runs/task_001-20261014T101500123456Z.json"`).
  - The fold's run item gains `"trace"`.
  - `GET /program/traces/{name}` returns the trace JSON. It is 400 if the name is bad, 404 if missing, and 409/503 through `_ws()`.
  - `files.SKIP_DIRS` includes `"runs"`.

- [ ] **Step 1: Failing tests.**

`backend/tests/test_program_traces.py`:

```python
import json

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.program import agent, routes, session

client = TestClient(app)
TRACE_LINE = "TRACE: runs/task_001-20261014T101500123456Z.json\nRESULT: ok\n"


@pytest.fixture
def src(tmp_path, monkeypatch):
    s = tmp_path / "ws" / "src"
    (s / "bot_tasks" / "runs").mkdir(parents=True)
    monkeypatch.setattr(routes.session, "current", lambda: {"robot": "bot", "src": s})
    return s


def test_trace_endpoint_serves_and_validates(src):
    (src / "bot_tasks" / "runs" / "task_001-x.json").write_text(json.dumps({"version": 1}))
    assert client.get("/program/traces/task_001-x.json").json() == {"version": 1}
    assert client.get("/program/traces/missing.json").status_code == 404
    for bad in ["..%2Fx.json", "a%2Fb.json", "x.txt", "%2E%2E.json"]:
        assert client.get(f"/program/traces/{bad}").status_code in (400, 404), bad


def test_run_program_reports_the_trace(monkeypatch, tmp_path):
    monkeypatch.setattr(session, "current", lambda: {"robot": "bot", "src": tmp_path})
    monkeypatch.setattr(session, "_docker", lambda *a, **k: type("R", (), {"stdout": TRACE_LINE, "stderr": ""})())
    assert session.run_program("programs/task_001.py")["trace"] == "runs/task_001-20261014T101500123456Z.json"
    monkeypatch.setattr(session, "_docker", lambda *a, **k: type("R", (), {"stdout": "RESULT: ok\n", "stderr": ""})())
    assert session.run_program("programs/task_001.py")["trace"] is None


def test_agent_run_result_carries_the_trace(tmp_path):
    lines = [json.dumps({"type": "assistant", "message": {"content": [
                {"type": "tool_use", "id": "t1", "name": "Bash", "input": {"command": "../bot_program/run_task.sh programs/task_001.py"}}]}}),
             json.dumps({"type": "user", "message": {"content": [
                {"type": "tool_result", "tool_use_id": "t1", "content": TRACE_LINE}]}})]
    ev = [e for e in agent.parse_stream(lines, tmp_path) if e["kind"] == "run_result"]
    assert ev[0]["trace"] == "runs/task_001-20261014T101500123456Z.json"
```

Append to `test_program_conversations.py`:

```python
def test_fold_keeps_the_trace():
    m = conv.fold([{"kind": "run", "n": 1}, {"kind": "run_result", "ok": True, "message": "ok", "output": "o",
                    "trace": "runs/a.json"}])
    assert m["items"][0]["trace"] == "runs/a.json"
```

Append to `test_program_files.py`:

```python
def test_tree_skips_run_traces(src):
    (src / "bot_tasks" / "runs").mkdir()
    (src / "bot_tasks" / "runs" / "task_001-x.json").write_text("{}")
    assert all("/runs/" not in f["path"] for f in client.get("/program/files").json())
```

Run them. Expected: FAIL.

Before running, read the existing `parse_stream` message shape in `test_program_agent.py`. If its fixture lines differ from the shape above, which comes from `agent.py`'s `_blocks`, use that file's shape.

- [ ] **Step 2: Implement.**
  - **`agent.py` and `session.py`:** add a module-level helper `TRACE_RE = re.compile(r"^TRACE: (runs/[A-Za-z0-9_.-]+\.json)$", re.M)`. Put it in `session.py`; `agent.py` imports it from there, or define it in each file if that avoids a circular import.
    - In `parse_stream`, the `run_result` event becomes `{..., "output": text, "trace": (TRACE_RE.findall(text) or [None])[-1]}`, where `text = _result_text(b)`.
    - In `run_program`, return `{"ok": ..., "output": out, "trace": (TRACE_RE.findall(out) or [None])[-1]}`.
  - **`conversations.fold`:** in the `run_result` branch, `it.update(ok=..., message=..., output=e.get("output", ""), trace=e.get("trace"))`.
  - **`files.py`:** `SKIP_DIRS = {"build", "install", "log", "__pycache__", ".git", "runs"}`.
  - **`routes.py`:**

```python
TRACE_NAME = re.compile(r"^[A-Za-z0-9_.-]+\.json$")


@router.get("/traces/{name}")
def get_trace(name: str) -> dict:
    src, robot = _ws()
    if not TRACE_NAME.fullmatch(name) or name.startswith("."):
        raise HTTPException(400, "Bad trace name.")
    p = Path(src) / f"{robot}_tasks" / "runs" / name
    if not p.is_file():
        raise HTTPException(404, "No such trace.")
    return json.loads(p.read_text(encoding="utf-8"))
```

(Import `re` if it isn't imported already.)

- [ ] **Step 3: Run the new tests, then the full backend suite.** Expected: PASS.
- [ ] **Step 4: Commit** `feat: run results carry their replay trace; trace endpoint`.

---

### Task 3: Frontend data — trace fields, client, replay model

**Files:**
- Modify: `frontend/src/api/program.ts`, `frontend/src/model/chat.ts`, `frontend/src/ros/liveRobot.ts`
- Create: `frontend/src/model/replay.ts`
- Test: `frontend/src/model/replay.test.ts`, `frontend/src/model/chat.test.ts` (append)

**Interfaces:**
- Consumes: the Task 2 endpoint, and `trace` on run items and run messages.
- Produces:
  - `ProgramEvent` `run_result` gains `trace?: string | null`.
  - `Item` `run` gains `trace?: string | null`.
  - `Message` `{ role: "run" }` gains `trace?: string | null`.
  - `getTrace(name: string): Promise<Trace>`, where `name` is the file name without `runs/`.
  - `replay.ts`:
    - `interface Trace { version: 1; robot: string; program: string; base: string; static: TfTransform[]; frames: { t: number; tf: TfTransform[] }[]; scenes: { t: number; scene: unknown }[]; result: string }`
    - `duration(tr: Trace): number`
    - `frameAt(tr: Trace, t: number): TfTransform[]` (static plus the latest frame at or before t; the first frame if t is before it)
    - `sceneAt(tr: Trace, t: number): unknown` (the latest scene at or before t, otherwise null)
    - `traceName(path: string): string` (strips `runs/`)
  - `liveRobot.ts`: `robotView(model: ModelState): { links: Link[]; home: Map<string, Matrix4>; base: string }`, built from the current live-effect code.

- [ ] **Step 1: Failing tests.** `frontend/src/model/replay.test.ts`:

```ts
import { describe, expect, it } from "vitest";
import { duration, frameAt, sceneAt, traceName, type Trace } from "./replay";

const tf = (child: string, x: number) => ({ header: { frame_id: "base" }, child_frame_id: child,
  transform: { translation: { x, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } } });
const tr: Trace = { version: 1, robot: "bot", program: "p", base: "base", static: [tf("s", 9)], result: "ok",
  frames: [{ t: 0.1, tf: [tf("a", 1)] }, { t: 0.2, tf: [tf("a", 2)] }],
  scenes: [{ t: 0, scene: "empty" }, { t: 0.15, scene: "box" }] };

describe("replay model", () => {
  it("frameAt clamps to the first and last frame and includes static", () => {
    expect(frameAt(tr, 0).map((f) => f.transform.translation.x)).toEqual([9, 1]);
    expect(frameAt(tr, 0.15).map((f) => f.transform.translation.x)).toEqual([9, 1]);
    expect(frameAt(tr, 5).map((f) => f.transform.translation.x)).toEqual([9, 2]);
  });
  it("sceneAt picks the latest change at or before t", () => {
    expect(sceneAt(tr, 0.1)).toBe("empty");
    expect(sceneAt(tr, 0.15)).toBe("box");
    expect(sceneAt({ ...tr, scenes: [] }, 1)).toBeNull();
  });
  it("duration and names", () => {
    expect(duration(tr)).toBe(0.2);
    expect(traceName("runs/task_001-x.json")).toBe("task_001-x.json");
  });
});
```

Append to `frontend/src/model/chat.test.ts`:

```ts
it("keeps the trace on run results", () => {
  const m = applyEvent(applyEvent(emptyClaude(), { kind: "run", n: 1 }),
    { kind: "run_result", ok: true, message: "ok", output: "o", trace: "runs/a.json" });
  expect(m.items[0]).toMatchObject({ kind: "run", trace: "runs/a.json" });
});
```

Run them. Expected: FAIL.

- [ ] **Step 2: Implement.** `frontend/src/model/replay.ts`:

```ts
import type { TfTransform } from "../ros/liveRobot";

export interface Trace {
  version: 1; robot: string; program: string; base: string; static: TfTransform[];
  frames: { t: number; tf: TfTransform[] }[]; scenes: { t: number; scene: unknown }[]; result: string;
}

export const duration = (tr: Trace) => (tr.frames.length ? tr.frames[tr.frames.length - 1].t : 0);
export const traceName = (path: string) => path.replace(/^runs\//, "");

function lastAtOrBefore<T extends { t: number }>(xs: T[], t: number): T | undefined {
  let lo = 0, hi = xs.length - 1, found: T | undefined;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (xs[mid].t <= t) { found = xs[mid]; lo = mid + 1; } else hi = mid - 1;
  }
  return found;
}

export function frameAt(tr: Trace, t: number): TfTransform[] {
  const f = lastAtOrBefore(tr.frames, t) ?? tr.frames[0];
  return [...tr.static, ...(f?.tf ?? [])];
}

export function sceneAt(tr: Trace, t: number): unknown {
  return lastAtOrBefore(tr.scenes, t)?.scene ?? null;
}
```

- `chat.ts`:
  - Add `trace?: string | null` to the `run` Item and to the `run` Message.
  - In `applyEvent` `run_result`, set `trace: e.trace ?? null`.
- `program.ts`:
  - Add `trace?: string | null` to the `run_result` event.
  - Add `export async function getTrace(name: string): Promise<Trace> { return (await send(\`/program/traces/${encodeURIComponent(name)}\`, { method: "GET" })).json(); }`, importing the `Trace` type from `../model/replay`.
  - `runFile`'s return type gains `trace?: string | null`.
- `liveRobot.ts`: move the first lines of the live effect into an exported `robotView(model)`, using `buildExportRequest`, `rootLinkId`, `base` and `homeFrames`. Make `useRosSession` call it, with no behaviour change.

- [ ] **Step 3: Run the tests, the full suite and tsc.** Expected: PASS.
- [ ] **Step 4: Commit** `feat: replay model and trace fields in the chat`.

---

### Task 4: Replay player, buttons and wiring

**Files:**
- Create: `frontend/src/ui/program/Replay.tsx`
- Modify: `frontend/src/ui/program/useRosSession.ts`, `Workspace.tsx`, `Chat.tsx`, `ProgramMode.tsx`, `frontend/src/ui/theme.css`
- Test: `frontend/src/ui/program/replay.test.tsx` (create), `chat.test.tsx` and `programMode.test.tsx` (append)

**Interfaces:**
- Consumes:
  - `getTrace`, `Trace`, `frameAt`, `sceneAt`, `duration` and `traceName` (Task 3).
  - `FrameTree`, `bodyPoses`, `sceneObjects` and `robotView` (liveRobot).
  - `viewer.setBodyPoses` and `viewer.setSceneObjects`.
- Produces:
  - `useRosSession(ctx, paused: boolean)`. While paused, live `/tf` and scene updates are not drawn. On unpause it redraws the live pose and scene at once.
  - `Replay({ ctx, name, onEnd })`, where `name` is the trace file name.
  - `Chat` gains props `onReplay(trace: string): void` and `canReplay: boolean`.
  - `Workspace` gains `overlay?: ReactNode` (rendered inside the Preview tab body) and `showPreview: number`. When `showPreview` changes, the tab switches to Preview.

- [ ] **Step 1: Failing tests.**

`frontend/src/ui/program/replay.test.tsx`:

```tsx
import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as api from "../../api/program";
import { boxBody } from "../../model/testBodies";
import { Harness, imported } from "../testing";
import Replay from "./Replay";

afterEach(() => { vi.restoreAllMocks(); vi.useRealTimers(); });
const trace = { version: 1, robot: "robot", program: "p", base: "base", static: [], result: "ok",
  frames: [{ t: 0, tf: [] }, { t: 1, tf: [] }], scenes: [{ t: 0, scene: { world: { collision_objects: [] }, robot_state: { attached_collision_objects: [] } } }] };

function ui(viewer: any, onEnd = vi.fn()) {
  render(<Harness start={imported([boxBody("base", [0, 0, 0], [1, 1, 1])])} viewer={viewer}
    ui={(ctx) => <Replay ctx={ctx} name="t.json" onEnd={onEnd} />} />);
  return onEnd;
}

describe("Replay", () => {
  it("loads the trace, draws it, and stops", async () => {
    vi.spyOn(api, "getTrace").mockResolvedValue(trace as never);
    const viewer = { setBodyPoses: vi.fn(), setSceneObjects: vi.fn() };
    const onEnd = ui(viewer);
    await screen.findByRole("button", { name: /pause/i });
    expect(viewer.setSceneObjects).toHaveBeenCalled();
    fireEvent.click(screen.getByRole("button", { name: /pause/i }));
    expect(screen.getByRole("button", { name: /play/i })).toBeTruthy();
    fireEvent.change(screen.getByRole("slider"), { target: { value: "0.5" } });
    fireEvent.click(screen.getByRole("button", { name: /stop/i }));
    expect(onEnd).toHaveBeenCalled();
  });
  it("a missing trace says so", async () => {
    vi.spyOn(api, "getTrace").mockRejectedValue(new Error("No such trace."));
    ui({ setBodyPoses: vi.fn(), setSceneObjects: vi.fn() });
    expect(await screen.findByText(/Replay not available/)).toBeTruthy();
  });
});
```

Append to `chat.test.tsx`:

```tsx
it("offers Replay on runs with a trace, disabled while running", () => {
  const onReplay = vi.fn();
  const ms: Message[] = [{ role: "claude", items: [{ kind: "run", n: 1, ok: true, message: "ok", output: "", trace: "runs/a.json" },
    { kind: "run", n: 2, ok: false, message: "x", output: "" }], summary: "", result: null, error: null, program: null },
    { role: "run", path: "bot_tasks/programs/a.py", ok: true, output: "", trace: "runs/b.json" }];
  const { rerender } = render(<Chat messages={ms} live={null} running={false} disabled={false} onSend={() => {}} onReplay={onReplay} canReplay />);
  const buttons = screen.getAllByRole("button", { name: /replay/i });
  expect(buttons).toHaveLength(2);
  fireEvent.click(buttons[0]);
  expect(onReplay).toHaveBeenCalledWith("runs/a.json");
  rerender(<Chat messages={ms} live={null} running={true} disabled={false} onSend={() => {}} onReplay={onReplay} canReplay={false} />);
  expect((screen.getAllByRole("button", { name: /replay/i })[0] as HTMLButtonElement).disabled).toBe(true);
});
```

Append to `programMode.test.tsx` (reuse its `ready` and `conv` helpers):

```tsx
it("replay switches to Preview and stops when the conversation changes", async () => {
  const withTrace = { ...conv("aaaaaaaaaaaa", "A"), messages: [{ role: "run", path: "bot_tasks/programs/a.py", ok: true, output: "", trace: "runs/a.json" }] };
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }, { id: "bbbbbbbbbbbb", title: "B" }]);
  vi.spyOn(api, "getConversation").mockImplementation(async (id) => (id === "aaaaaaaaaaaa" ? withTrace : conv(id, "B")) as never);
  vi.spyOn(api, "getTrace").mockResolvedValue({ version: 1, robot: "robot", program: "p", base: "base", static: [], result: "ok",
    frames: [{ t: 0, tf: [] }, { t: 1, tf: [] }], scenes: [] } as never);
  await ready();
  await screen.findByText("A");
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /replay/i })); });
  expect(await screen.findByRole("button", { name: /stop/i })).toBeTruthy();
  await act(async () => { fireEvent.click(screen.getByText("B")); });
  expect(screen.queryByRole("button", { name: /stop/i })).toBeNull();
});
```

(The harness `viewer` is `null` in programMode tests. `Replay` must render its controls even when `ctx.viewer` is null, and simply not draw.)

Run them. Expected: FAIL.

- [ ] **Step 2: Implement.** `frontend/src/ui/program/Replay.tsx`:

```tsx
import { Pause, Play, Square } from "lucide-react";
import { useEffect, useMemo, useRef, useState } from "react";
import { getTrace } from "../../api/program";
import { duration, frameAt, sceneAt, type Trace } from "../../model/replay";
import { FrameTree, bodyPoses, robotView, sceneObjects } from "../../ros/liveRobot";
import type { Ctx } from "../ctx";

/** Plays a recorded run in the viewer; the live view is paused by the parent while this is mounted. */
export default function Replay({ ctx, name, onEnd }: { ctx: Ctx; name: string; onEnd(): void }) {
  const [trace, setTrace] = useState<Trace | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [t, setT] = useState(0);
  const [playing, setPlaying] = useState(true);
  const [speed, setSpeed] = useState(1);
  const view = useMemo(() => robotView(ctx.model), [ctx.model]);
  const last = useRef<number | null>(null);

  useEffect(() => {
    let live = true;
    getTrace(name).then((tr) => {
      if (!live) return;
      if (tr.robot !== ctx.robotName) setError("Replay not available: this run is from another robot.");
      else setTrace(tr);
    }).catch(() => { if (live) setError("Replay not available."); });
    return () => { live = false; };
  }, [name]);  // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {  // draw the current instant
    if (!trace || !ctx.viewer) return;
    const tree = new FrameTree();
    tree.update(frameAt(trace, t) as never[]);
    ctx.viewer.setBodyPoses(bodyPoses(view.links, tree, view.home, view.base));
    ctx.viewer.setSceneObjects(sceneObjects(sceneAt(trace, t), tree, view.base));
  }, [trace, t, ctx.viewer, view]);

  useEffect(() => {  // advance time while playing
    if (!trace || !playing) { last.current = null; return; }
    let raf = 0;
    const tick = (now: number) => {
      const dt = last.current === null ? 0 : (now - last.current) / 1000;
      last.current = now;
      setT((x) => {
        const next = Math.min(duration(trace), x + dt * speed);
        if (next >= duration(trace)) setPlaying(false);
        return next;
      });
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [trace, playing, speed]);

  return (
    <div className="replay-bar">
      {error ? <span className="muted small">{error}</span> : <>
        <button className="btn" aria-label={playing ? "Pause" : "Play"} disabled={!trace}
          onClick={() => { if (!playing && trace && t >= duration(trace)) setT(0); setPlaying((p) => !p); }}>
          {playing ? <Pause size={14} /> : <Play size={14} />}
        </button>
        <input type="range" min={0} max={trace ? duration(trace) : 0} step={0.01} value={t} disabled={!trace}
          onChange={(e) => { setPlaying(false); setT(Number(e.target.value)); }} />
        <span className="muted small">{t.toFixed(1)} / {(trace ? duration(trace) : 0).toFixed(1)} s</span>
        <button className="btn" onClick={() => setSpeed((s) => (s === 1 ? 2 : 1))}>{speed}×</button>
      </>}
      <button className="btn" aria-label="Stop replay" onClick={onEnd}><Square size={14} />Stop</button>
    </div>
  );
}
```

- **`useRosSession(ctx, paused = false)`:**
  - Keep a `pausedRef` synced to `paused`.
  - In the live effect, `redraw` and `refreshScene` return early while `pausedRef.current` is set.
  - Add an effect on `[paused]` that, when it becomes false, calls the stored `redraw` and `refreshScene`. Keep them in refs (`kick.current = () => { redraw(); refreshScene(); }`).
  - Also replace the effect's first lines with `robotView(model)`.
- **`Workspace`:**
  - New props `overlay?: ReactNode` and `showPreview: number`.
  - `useEffect(() => { if (showPreview) setTab("preview"); }, [showPreview]);`
  - Render `{overlay}` inside the Preview tab body after `{preview}`, in a `<div className="replay-overlay">`.
- **`Chat`:**
  - Props `onReplay(trace: string)` and `canReplay: boolean`.
  - `RunOutput` gains an optional `trace` and renders `<button className="btn replay" aria-label="Replay" disabled={!canReplay} onClick={() => onReplay(trace)}><Play size={12} />Replay</button>` next to the head when `trace` is set.
  - Pass `item.trace` for Claude's run items and `m.trace` for "You ran" messages.
  - Thread `onReplay` and `canReplay` through the `Claude` → `Step` → `RunOutput` props.
- **`ProgramMode`:**
  - Add `const [replay, setReplay] = useState<string | null>(null);` and `const [previewTick, setPreviewTick] = useState(0);`.
  - Call `useRosSession(ctx, replay !== null)`.
  - Pass `<Chat … onReplay={(tr) => { setReplay(traceName(tr)); setPreviewTick((n) => n + 1); }} canReplay={!running} />`.
  - Pass `<Workspace … showPreview={previewTick} overlay={replay && <Replay key={replay} ctx={ctx} name={replay} onEnd={() => setReplay(null)} />} />`.
  - Clear the replay when the active conversation id changes (`useEffect(() => setReplay(null), [active?.id])`) and when `running` becomes true.
- **CSS:**

```css
.replay-overlay { position: absolute; left: 12px; right: 12px; bottom: 12px; pointer-events: none; }
.replay-bar { pointer-events: auto; display: flex; align-items: center; gap: 8px; padding: 6px 8px; border-radius: 8px;
  background: var(--panel); border: 1px solid var(--border); }
.replay-bar input[type="range"] { flex: 1; }
.step .btn.replay { margin-left: 8px; padding: 2px 6px; }
.tab-body { position: relative; }
```

- [ ] **Step 3: Run the new tests, the full suite, tsc and `npm run build`.** Expected: PASS.
- [ ] **Step 4: Commit** `feat: replay runs from the chat`.

---

### Task 5: Hands-on check (controller inline)

- [ ] Restart the backend so it loads the new code. The user presses **Restart session** so the workspace gets `recorder.py`.
- [ ] Run a pick-and-place in a new conversation. Check that each Claude run and a manual run show ▶ Replay, and that replay shows the arm and the carried box. Check that a failed run replays up to where it stopped, and that Stop returns to the live view.
