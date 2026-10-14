# Run Replay — Design

> **Status: deferred (2026-10-14).** Not built. Run again buttons (re-running the program live) cover replay for now; see commit ababc6a.

Date: 2026-10-14. Branch: prompt-to-program. Builds on the Program workspace (`2026-10-13-program-workspace-design.md`).

## 1. Goal

Every program run (Claude's runs and your manual runs) can be watched again in the 3D view after it finishes. Today the motion is visible only live, while it happens.

Success criteria:
- Each run step in the chat ("Run 2 ✓ ok", "You ran … ✓") has a ▶ Replay button once the run has finished.
- Replay shows the same thing the live view showed: the arm moving, objects appearing, and boxes being carried and set down, at real speed.
- Replays survive a page reload, a backend restart and a session restart. They are stored with the robot's workspace.
- Failed runs replay too, up to the point where they stopped. That is how you see what went wrong.

## 2. Decisions

| Topic | Decision |
|---|---|
| What is recorded | Exactly what the live view uses: `/tf` transforms of the robot's links, and the planning-scene objects (world plus attached). Replay then reuses `FrameTree`, `bodyPoses` and `sceneObjects`, with no new kinematics. |
| Who records | `runner.py` inside the container. It already wraps every run, both Claude's and manual ones. It starts a recorder before the program and stops it after `RESULT`. |
| Rate | `/tf` sampled at 20 Hz. The scene is recorded only when it changes, by polling `/get_planning_scene` at 5 Hz and keeping a sample only when its content differs. |
| Storage | `<robot>_tasks/runs/<program stem>-<UTC timestamp>.json`, next to `programs/`. It sits outside `src` regeneration, the same as programs. |
| Link to the chat | The run result carries the trace file name. The agent stream's `run_result` event and the manual-run response both gain `"trace": "runs/…json" \| null`. The conversation stores it on the run item or message. |
| Playback controls | Play/pause, a progress bar you can drag, and speed 1× or 2×. Replay takes over the Preview tab while it plays; the live view resumes when it ends or you press Stop. |
| Size cap | Recording stops after 5 minutes (the run timeout is 2 minutes, so this is never hit in practice). A 30 s run is about 300 KB of JSON. |

## 3. Recording (container)

New `<robot>_program/recorder.py`, used by `runner.py`:
- `Recorder(path)` starts a thread with its own rclpy node.
  - It subscribes to `/tf` and keeps the latest transform per child frame.
  - Every 50 ms it appends `{"t": seconds_since_start, "tf": [transforms of moving frames]}`.
  - `/tf_static` is captured once into `"static"`.
- The scene is polled every 200 ms with the same components as the live view (`SCENE_COMPONENTS` = 20). A sample `{"t", "scene": <msg as dict>}` is appended only when the objects differ from the previous sample.
- `stop()` joins the thread and writes `{"version": 1, "robot", "program", "base", "static": [...], "frames": [...], "scenes": [...], "result": "ok" | "failed: …"}`.
- `runner.py` calls `stop()` in a `finally`, so failed runs and exceptions still save their trace. It prints `TRACE: runs/<file>` before `RESULT:`.
- If the recorder fails, the run itself never fails. `runner.py` prints `TRACE: none (<reason>)` and carries on.

## 4. Backend

- `agent.parse_stream`: the `run_result` event takes `trace` from the `TRACE:` line in the tool output (`null` if missing).
- `session.run_program` returns `trace` the same way. `/program/files/run` passes it through, and the stored `{"role": "run"}` message keeps it.
- `conversations.fold` copies `trace` onto the run item. The frontend `applyEvent` mirrors it.
- New `GET /program/traces/{name}` returns the JSON file from `<robot>_tasks/runs/`.
  - The name must match `^[A-Za-z0-9_.-]+\.json$`, with no path separators.
  - It returns 404 if the file is missing and 409 if there is no session.

## 5. Frontend

- `api/program.ts`: `getTrace(name)`.
- `model/replay.ts` (pure, tested):
  - `frameAt(trace, t)` returns the latest `/tf` sample at or before `t`, plus `static`.
  - `sceneAt(trace, t)` returns the latest scene sample at or before `t`.
- `ui/program/Replay.tsx`: the overlay controls inside the Preview tab (play/pause, a draggable progress bar, 1×/2×, Stop).
  - It drives a `FrameTree` from the trace and calls the existing `bodyPoses` and `sceneObjects`, then `viewer.setBodyPoses` and `viewer.setSceneObjects`.
- `useRosSession`'s live-robot effect pauses while a replay is active, so live `/tf` doesn't fight the replay. It resumes afterwards.
- Chat: run items and "You ran" messages with a `trace` get a ▶ Replay button. Clicking it switches the workspace to Preview and starts playback.
  - Replay buttons are disabled while a task is running, so the live view stays honest.

## 6. Errors

- Missing or unreadable trace: the Replay button shows "Replay not available" and the chat stays usable.
- A trace from a different robot (`robot` field mismatch) is refused with the same message.
- Links in the trace that the current model lacks are skipped, as the live view does.

## 7. Testing

- Backend:
  - `TRACE:` parsing in `parse_stream` and `run_program`.
  - The trace endpoint: name validation (`../x.json`, `a/b.json`, `x.txt` → 400), 404 and 409.
  - `fold` carries `trace`.
- Container check: `trace.py` runs a short pick-and-place through `runner.py` and asserts the trace file exists, has at least 10 frames, has at least 2 scene samples (box added, then attached), and ends with the run's result. A program that raises mid-way still leaves a trace.
- Frontend:
  - `frameAt` and `sceneAt` at the boundaries (before the first sample, between samples, after the last).
  - Replay controls: play, pause, scrub, stop restores live.
  - The chat button appears only when there is a trace, and is disabled while running.
- Manual: replay one successful and one failed run on the IRB 1300.

## 8. Out of scope

Exporting replays as video, comparing two runs side by side, replaying runs from before this feature (they have no trace), and recording joint efforts or velocities.
