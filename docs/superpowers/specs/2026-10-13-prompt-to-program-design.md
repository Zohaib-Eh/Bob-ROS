# Prompt to Program — Design

**Date:** 2026-10-13
**Builds on:** `docs/superpowers/specs/2026-10-13-arm-kinematics-ros-design.md` (generated ROS 2 workspace, `docker/jazzy` image, `scripts/ros_check.sh`, MoveIt action code in `docker/jazzy/check/check_ros.py`).
**Scope:** roadmap project "prompt to program". A new "Program" step in the app: type a task, an LLM writes a MoveIt Python program for the robot, it runs in the ROS 2 Jazzy container on mock hardware, and the robot moves live in the app's 3D viewer. Single user, local machine.

## 1. Purpose and success

The product bet is that an LLM can turn a plain-language task into a correct robot program. This project tests that bet as cheaply as possible, on the setup that already works (MoveIt + mock hardware), before physics simulation or real hardware.

Success criteria:

- Opening the Program step for the IRB 1300 starts a session; within ~30 s the viewer shows the robot driven by live ROS joint data.
- Of the five benchmark tasks (§8), at least 4 succeed within 3 runs each through the real app with Claude, and their motion is visible in the viewer, including boxes being carried.
- The chat shows, per task, Claude's summary, the program file, and how many runs it needed.
- All automated tests pass (§8).

## 2. Decisions (from the design conversation)

| Topic | Decision |
|---|---|
| User | Just the developer, locally. No accounts, no hosting. |
| LLM | Claude Code headless (`claude -p`) on the developer's existing Claude subscription: no API key, no per-call cost, strong model. Personal use only; other users require switching to the API backend. |
| Adapter | Thin boundary: one function `run_agent(...)` → event stream; `claude -p` is the only backend for now. The Anthropic API (or Ollama) is a second backend written when needed. |
| UI | New "Program" step in the existing app: chat left, viewer right, read-only program view under the chat. |
| Scene | Objects come from the prompt as simple shapes (boxes, cylinders) added by the program to MoveIt's planning scene. |
| Gripper | None yet. "Pick" attaches the object to the tool link, "place" detaches it (MoveIt attached collision objects). Real grippers arrive with the multi-model/hardware work. |
| Live view | Long-running session container with `rosbridge_server`; browser subscribes over a plain WebSocket. ROS computes link poses (`/tf`); no kinematics in the browser. |
| Run cap | 3 runs per task (one constant), 120 s per run, clean state before every run. |

## 3. Container

- `docker/jazzy/Dockerfile` adds `ros-jazzy-rosbridge-suite`.
- A session container mounts the generated workspace at `/ws`, builds it (`colcon build`), and runs `demo.launch.py` plus `rosbridge_websocket` on port 9090 (published to the host as `localhost:9090`). It keeps running until the session stops.
- Named `cad-to-urdf-session` so there is at most one; starting a new session replaces it.

## 4. Robot task library

Generated with the workspace into a new package `<robot>_program` (regenerated freely, like the other generated packages): `robot.py`, `run_task.sh`, and the brief. Programs are the user's and live in `<robot>_tasks/programs/task_NNN.py`, which regeneration never touches; they `from robot import Robot` (`run_task.sh` puts `<robot>_program` on `PYTHONPATH`).

```python
r = Robot()                      # connects to MoveIt; robot at home, scene empty after reset()
r.add_box(name, size_xyz, xyz, rpy=(0, 0, 0))
r.add_cylinder(name, radius, height, xyz, rpy=(0, 0, 0))
r.move_to(xyz, rpy)              # planned, collision-free
r.move_line(xyz, rpy)            # straight-line Cartesian path
r.move_joints(q)                 # joint-space target, radians, in joint order
r.home()
r.attach(name)                   # object rides on the tool link
r.detach(name)
r.solutions(xyz, rpy)            # every exact IK solution (EAIK), nearest first
r.tool_pose()                    # current (xyz, rpy) of the tool link
```

- Coordinates: metres and radians, in the robot base frame (`chain.base`); the tool is `chain.tip`.
- Built on the MoveIt `MoveGroup` action, `/apply_planning_scene` service and Cartesian path service, reusing the request-building code proven in `docker/jazzy/check/check_ros.py`.
- Errors are exceptions with one plain sentence the LLM can act on: `Unreachable` (no IK / outside reach, names the pose), `Collision` (names the object hit when MoveIt reports it), `PlanningFailed` (other planner failures, with MoveIt's error code name), `UnknownObject`.
- `reset()` (called by `run_task.sh` before each run): clear all world and attached objects, move home.

## 5. Backend `backend/app/program/`

| Unit | Job |
|---|---|
| `session.py` | Export the current model (existing `export_package`), generate the workspace (`app.ros.moveit_config.generate`), start/stop the session container, report readiness (rosbridge port open and MoveIt action available). Workspace lives in a per-robot folder under the user's app data dir so programs survive restarts. |
| `run_task.sh` (generated into `<robot>_program/`, runs on the host) | Called by Claude from `<robot>_tasks/` as `../<robot>_program/run_task.sh programs/task_NNN.py`: increments `<robot>_tasks/.run_count` (the backend zeroes it when a task starts; the 4th call refuses with a clear message), `docker exec`s `Robot().reset()` then the program in `cad-to-urdf-session` with a 120 s timeout, prints the program's output and a final `RESULT: ok` or `RESULT: failed: <error sentence>`. |
| `agent.py` | `run_agent(task, workspace, session_id=None) -> Iterator[Event]`. Backend `ClaudeCode`: `claude -p --output-format stream-json --verbose --append-system-prompt <brief> --tools Read,Write,Edit,Bash --allowedTools "Read" "Write(<tasks dir>/**)" "Edit(<tasks dir>/**)" "Bash(../<robot>_program/run_task.sh:*)"`, working directory = `<robot>_tasks/`, `--resume <session_id>` for follow-ups. Parses the stream into events: `text`, `file_written`, `run_started`, `run_result`, `done(summary, runs, session_id)`, `error(message)`. Exact flag syntax is verified by the plan's first task against the installed CLI (2.1.296). |
| `brief.py` | Builds the brief: robot facts (joint names, limits, base/tip frames, approximate reach from FK sampling), the `robot.py` API with two short examples, and the rules (write one file `programs/task_NNN.py`, test with `run_task.sh`, fix and re-run until `RESULT: ok`, at most 3 runs, finish with a two-line summary). |
| `routes.py` | `POST /program/session` (start; body = current model as for export), `GET /program/session` (status), `DELETE /program/session`, `POST /program/run` (task text → Server-Sent Events of agent events), `POST /program/rerun` (run the current program again without the LLM). |

## 6. Frontend — Program step

- New step after Export: `{ id: "program", label: "Program" }` in the stepper.
- Left: chat (task input, streamed agent text, per-task card with summary and "worked on run k/3" or "failed after 3 runs"), and a read-only view of the current program file; buttons Run again and Restart session.
- Right: the existing viewer. A small `rosbridge.ts` client (browser `WebSocket`, rosbridge JSON protocol, no new dependency) subscribes to `/tf` and `/monitored_planning_scene`:
  - Each link's body meshes move by `T_link(now) · T_link(home)⁻¹` (mesh vertices are in the exported world frame, which equals the base frame at home).
  - World objects render as boxes/cylinders; attached objects follow the tool link.
- While the session starts: status "Starting ROS… (first start builds the workspace)".

## 7. Errors shown in the chat

- `claude` not installed or not signed in: "Run `claude` once in a terminal to sign in."
- Usage limit: Claude's own message, as is.
- Docker not running: "Start Docker Desktop."
- Session container died: last 30 launch-log lines + Restart session.
- Model has unsupported structure (from `app.ros`): its one-sentence error.

## 8. Testing

**Automated (no LLM):**
- Backend pytest: claude command line (allowed tools, cwd, resume); stream-json parsing against a recorded fixture replayed by a fake `claude` script; run cap and reset enforced by `run_task.sh` (fake `docker` on PATH); session start/stop builds the expected `docker run` command.
- Container check (`scripts/ros_check.sh` extended): hand-written programs against `robot.py` — add/attach/move/detach succeeds; unreachable pose raises `Unreachable`; path through an obstacle raises `Collision` or `PlanningFailed`; `solutions()` returns ≥1 solution that reproduces the pose.
- Frontend vitest: Program step with a fake rosbridge socket — `/tf` messages move link meshes; scene messages draw boxes; chat renders streamed events, program view and run count.

**Benchmark (real Claude, manual):**
1. "Move the tool to 0.5 m in front of the robot, 0.4 m up."
2. "Put a 5 cm box on the floor at (0.5, 0.2), pick it up and place it at (0.4, -0.3)."
3. "Trace a 20 cm square in the air with straight lines."
4. "Stack two boxes."
5. "Pick and place a box with a 30 cm tall obstacle between the two spots."

Record per task: success, runs needed. Pass: ≥4/5 within 3 runs.

## 9. Out of scope

Physics simulation, grippers and mimic joints, other users / hosting, API or Ollama backends, scene editing in the viewer, CAD work-cell import, real hardware.
