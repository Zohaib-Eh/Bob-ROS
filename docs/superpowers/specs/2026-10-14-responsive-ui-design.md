# Responsive UI: progress and no dead screens — design

Date: 2026-10-14

## Goal

The app must never look stuck. The user always sees what the app is doing, and can keep working on anything that does
not need the busy resource.

What the user reported: creating a new chat sometimes hangs; importing a CAD file takes long with no feedback; while a
task runs nothing else can be done; the app looks stuck rather than slow.

Out of scope (deferred): making steps faster — caching the workspace, skipping regeneration when the model is
unchanged, colcon build caching. Also out: a task queue (one task at a time stays), cancelling an import.

## Mechanism

- **Import uses push (streaming).** One request that lives as long as the import. The backend streams progress events,
  then the result. Same pattern as the agent run.
- **ROS start uses pull (polling).** The session lives in Docker and outlives the page, so the page must be able to ask
  "where are you?" at any time. Extends the existing status poll.

No generic observer/job framework: each case reuses a pattern already in the app.

## 1. Import progress

**UI.** While importing, the Import button area in the top bar (where "Checking joints 3/12" shows) becomes a progress
strip. Every stage shows an elapsed timer, so even an opaque stage visibly ticks.

| Stage | Source | Display |
|---|---|---|
| Uploading | browser upload progress | bar (only visible for big files) |
| Reading file | `_read_step_named` / `_read_shape` (one OCCT call) | spinner + timer |
| Building parts | `for sources, prefix, repaired in groups` loop in `import_cad` | bar, "N / M" |
| Finding faces | `segment_mesh_body` (STL/OBJ) | spinner + timer |
| Sending to viewer | result download and scene build | spinner + timer |

**Backend.**
- `import_cad` and `import_path` take an optional `on_progress(stage: str, done: int, total: int)` callback; no
  callback = current behaviour.
- `/import` becomes a streaming response (`text/event-stream`, `data: {...}` lines, like the agent run). The import
  runs in a thread; the response yields `{"kind": "progress", "stage", "done", "total"}` events, then exactly one of
  `{"kind": "done", "bodies", "applied_scale", "warnings"}` or `{"kind": "error", "message"}`.
- Validation that fails before the stream starts (unsupported extension) stays an HTTP 400. Errors during import become
  an `error` event with the same text the 422 carries today.
- The plain-JSON response is removed (the frontend is the only caller).

**Frontend.**
- `importFile` switches from `fetch` to `XMLHttpRequest` (upload progress), and reads the event stream from
  `xhr.responseText` as it grows (`onprogress`).
- `importFile(file, scale, onProgress)`; `Stepper` keeps the stage in state and renders the strip.

**Tests.**
- Backend: a small STEP yields progress events then `done`; a corrupt file yields an `error` event; unsupported
  extension is 400.
- Frontend: the strip shows "Building parts 2 / 5" from a faked stream; a `done` event dispatches the import.

## 2. ROS start checklist

**UI.** Start returns at once. The header pill reads `Starting ROS · <stage> · <elapsed>`, e.g.
`Starting ROS · Checking collisions 4,200 / 10,000 · 0:38`. Clicking it opens a `StartupPanel` popover:

```
✓ Exporting robot
● Checking collisions   4,200 / 10,000
○ Building Docker image   (first time only)
○ Building workspace
○ Starting MoveIt
○ Starting bridge
○ Ready
  ── log ──
  [last 8 lines]
```

On failure the panel opens by itself, marks the failed stage ✗ and keeps the log. Elapsed time is counted in the
browser from the moment Start was pressed (or from the first `starting` status seen after a reload).

**Stage names** (shared by backend and frontend): `export`, `collisions`, `image`, `build`, `moveit`, `bridge`, `ready`.

**Backend.**
- `POST /session` stays synchronous for the fast checks only: `export_package` (bad model → 422) and `docker info`
  ("Start Docker Desktop." → 503). It then starts a background thread and returns `{"state": "starting", ...}`.
- The thread runs `workspace_from_zip`, the image build if missing, `docker rm -f` and `docker run`, recording
  `{stage, done, total, error}` in module state guarded by a lock.
- `disabled_pairs` takes an optional `on_progress(done, total)` called every few hundred samples from the
  `for _ in range(samples)` loop; passed down through `generate` and `workspace_from_zip`.
- `start.sh` writes the current stage to `/tmp/stage` before each step (`build`, `moveit`, `bridge`) and `ready` at the
  end (`/tmp/ready` is kept for the existing checks).
- `status()`: while the host thread runs, return its stage; if it failed, `state: "failed"` with the error and stage.
  Once the container runs, `docker exec cat /tmp/stage` replaces `test -f /tmp/ready` (same cost). While `starting`,
  add `log` from `docker logs --tail 8`.
- `POST /session` while the host thread runs → 409. `DELETE /session` during the host thread sets a stop flag; the
  thread exits before `docker run`, and the status is `stopped`.
- A backend restart mid-prep loses the thread: status is `stopped`, the user presses Start again.

**Frontend.**
- `SessionStatus` gains `stage?`, `done?`, `total?`, `log?`, `error?`.
- `useRosSession` exposes them; `StartupPanel` (new, `frontend/src/ui/program/StartupPanel.tsx`) renders the list.

**Tests.**
- Backend: with docker and generation stubbed, the status sequence goes `export → collisions (with counts) → image →
  build → moveit → bridge → ready`; an exception in the thread gives `failed` with its stage and message; stop during
  prep means `docker run` is never called; a second start during prep is 409.
- Frontend: the panel renders the checklist from a faked status; it opens by itself on `failed`.

## 3. Program mode unlocked

**Rule.** Only what needs ROS or the agent waits. Every disabled control says why (tooltip and, for Send/Run, inline
text).

**3a. Conversations without ROS.** Conversations are files in `~/.cad-to-urdf/workspaces/<robot>/conversations/`.
Conversation routes take `?robot=<name>` and resolve `WORKSPACES / <name>` directly instead of via
`session.current()`. The name must match the export's `_NAME_RE` (`urdf_writer.py`), else 400 — this also blocks
path traversal. The sidebar loads at once; browse, create, rename and delete work while ROS
is stopped or starting. Send and Run stay disabled until ready, showing "Waiting for ROS… (<stage>)". Routes that need
the container (run, files that live in the workspace `src`) are unchanged.

**3b. Free to move during a run.**
- The run is tracked by its conversation id (`runningId`), not by the open conversation. The live stream state belongs
  to `runningId`; it is shown only when that conversation is open.
- The sidebar stays enabled during a run: switch chats, create chats. The running chat shows a spinner dot in the list.
- When the run ends, the list refreshes and the open conversation stays open (the finished one reloads if it is open).
- Send in another chat is disabled: "Waiting for task in <title>…".
- Still locked during a run, with a tooltip each: Back to robot, Stop ROS, Restart session (leaving or stopping would
  kill the agent), and editing files in the Code tab (the agent writes them).

**3c. "New chat hangs".** Reproduce before fixing. Measure request times for create/list conversation while ROS is
starting and while a task runs, and record RAM per piece (backend, browser tab, Docker VM, agent process). Likely
causes:
1. The sidebar is disabled while starting/running, so the click does nothing. Fixed by 3a/3b.
2. The status poll uses `setInterval`; each tick runs `docker inspect` + `docker exec`. When Docker is slow (low RAM),
   ticks overlap and pile up in the backend thread pool, and conversation requests wait behind them. Fix: chain polls
   with `setTimeout` so the next starts only after the previous returns.

Any other cause found during reproduction gets fixed at its root and noted in the plan.

**Tests.**
- Backend: list/create/rename/delete conversations with no container.
- Frontend: sidebar enabled while starting and while running; switch chats mid-run and back shows the live stream;
  Send in another chat shows the waiting reason; at most one status request is in flight at a time.

## Error handling summary

- Import: errors arrive as an `error` event and show in the existing notify banner, as today.
- ROS start: host-side errors and container failures both end as `state: "failed"` with stage, message and log; the
  panel opens.
- Agent run: unchanged.
