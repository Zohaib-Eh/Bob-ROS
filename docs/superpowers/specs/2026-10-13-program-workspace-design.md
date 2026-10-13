# Program Workspace — Design

**Date:** 2026-10-13
**Builds on:** `2026-10-13-prompt-to-program-design.md` (session container, `claude -p` agent, `robot.py` task library, Program step). Same branch: `prompt-to-program`.
**Scope:** replace the cramped Program step panel with a full-window, bolt.new-style workspace (conversations, chat, Preview / Code with an editable file tree), and fix the task-library gaps the first live test exposed.

## 1. Why

The first live test (2026-10-13) showed:

- The Program step lives in a ~420 px side panel: raw markdown, the final answer shown twice, no distinction between the task, Claude's narration and run results, controls floating at the top, no code view worth using.
- With a 4-joint model, every `move_to` failed because it always demands an exact orientation. Claude then searched for joint angles by moving the robot over and over, which is slow, ignores obstacles, and collided with the box.
- The brief didn't say which way the robot faces, so "in front" was a guess.

Success criteria:

- Opening Program switches the whole window to Program mode; "← Back to robot" returns to the CAD steps unchanged.
- Separate conversations in a sidebar, each with its own history and Claude session, surviving an app restart.
- Chat renders markdown, shows each file write and run as a collapsible step, shows the final answer once and one result line per task.
- Code tab: file tree of the robot's workspace; `programs/*.py` editable and runnable without Claude; other files view-only.
- On a properly connected 6-joint IRB 1300, at least 4 of the 5 benchmark tasks (prompt-to-program spec §8) work within 3 runs, each in its own conversation.

## 2. Decisions

| Topic | Decision |
|---|---|
| Placement | Program mode takes over the whole window when the Program step is chosen; the other steps are unchanged. |
| Conversations | Sidebar list; one JSON file per conversation in the robot's workspace; each keeps its Claude session id for follow-ups. |
| ROS session | One per robot, shared by all conversations; one task or manual run at a time. |
| Editor | CodeMirror 6 (new dependency, ~150 KB): Python highlighting, editable for `<robot>_tasks/programs/*.py` only. |
| Chat text | Markdown rendered with `react-markdown` (new dependency); no raw HTML. |
| Library | `move_to` / `move_line` take `rpy=None` for a position-only goal; `tool_down()` helper; brief states joint count, facing direction, and forbids searching joint angles by moving the robot. |

## 3. Layout

```
┌ ← Back to robot ── <robot> ── ● ROS ready  ↻ Restart ──────────────────────────────┐
│ Conversations │ Chat                         │ [ Preview ] [ Code ]                │
│ + New         │  You: bubble                 │  Preview: live 3D robot + objects   │
│ ● <title>     │  Claude: markdown, steps     │  Code: file tree │ CodeMirror       │
│   <title>     │  ✓ Worked on run k/3         │        ● changed │ Save  Run        │
│               │  [ Describe a task…      ⏎ ] │                                     │
└───────────────┴──────────────────────────────┴─────────────────────────────────────┘
```

Columns are resizable (drag handles); widths remembered per browser (localStorage, optional).

## 4. Frontend parts

| Part | Job |
|---|---|
| `ProgramMode` | Full-window shell: top bar (back, robot name, ROS status, Start/Restart/Stop), three columns, owns the session and live-robot wiring (existing rosbridge / liveRobot code). |
| `ConversationList` | List, create, rename (inline), delete (confirm); active one highlighted. |
| `Chat` | Messages: user bubbles; Claude messages = markdown text + steps (`Wrote <path>`, `Run n ✓/✗ <message>`, `You ran <path> ✓/✗`) each expandable to full output; result line; prompt box pinned at the bottom; disabled while anything runs. |
| `Workspace` | Preview / Code tabs. Preview reuses the existing `Viewer` (moved into the right column). |
| `FileTree` | Workspace files grouped: `programs` (editable), `library` (`<robot>_program`), `robot` (description, MoveIt config, kinematics; view-only). Changed-in-last-turn dot. |
| `Editor` | CodeMirror; Save (programs only, disabled while running); Run (runs the open program without Claude). Reloads the file after a task ends. |

## 5. Backend

- Conversations under `<workspace>/conversations/<id>.json`: `{id, title, created, claude_session_id, messages: [...]}`; user message `{role: "user", text}`; Claude message `{role: "claude", text, steps: [{kind, path?, n?, ok?, message?, output?}], result: {ok, runs} | null, error?}`.
- `GET /program/conversations`, `POST /program/conversations`, `PATCH /program/conversations/{id}` (title), `DELETE /program/conversations/{id}`.
- `POST /program/conversations/{id}/run` (`{task}`) — streams the same events as `/program/run`, resumes the conversation's Claude session, and saves the user message and the finished Claude message to the file (also on error).
- `GET /program/files` (tree), `GET /program/files/{path}` (content), `PUT /program/files/{path}` (only `<robot>_tasks/programs/<name>.py`; 400 otherwise, incl. `..` or absolute paths), `POST /program/files/{path}/run` (manual run, appended to a conversation as a "You ran" step when a conversation id is given).
- "User edited" note: the backend records which program files were saved by the user since the conversation's last run and prepends one line per file to the next task ("The user edited programs/x.py since your last turn.").
- Existing `/program/run` and `/program/rerun` are replaced by the conversation endpoints.

## 6. Task library fixes (`templates/robot.py`, `program_pkg.py`)

- `move_to(xyz, rpy=None)`, `move_line(xyz, rpy=None)`: without `rpy`, only a position constraint (move_to) / the current orientation kept along the line (move_line).
- `tool_down() -> rpy`: an orientation whose tool approach axis points along −z for this robot, found from the tool frame at home (choose the tool-frame axis most aligned with the arm's last joint axis), returned as rpy.
- Brief additions: joint count and, when fewer than 6, "this arm cannot hold arbitrary orientations; use move_to(xyz) without rpy"; facing direction from the home tool position projected on the floor ("the arm reaches toward +x+y (bearing 45°)"); rule: "Never search for joint angles by moving the robot; use move_to, solutions() or known joint values."

## 7. Errors

- Save outside `programs/` or bad path → 400; editor shows non-program files read-only.
- Save while a task runs → disabled; file reloads when the task ends.
- ROS down during a run → error step + Restart in the top bar.
- Corrupt conversation file → skipped with a warning in the list, never a crash.
- Concurrent runs → Send and Run disabled while anything runs.

## 8. Testing

- Backend pytest: conversation CRUD and saving a finished run; file endpoints (tree, read, save allowed only under programs, `..`/absolute refused); "user edited" note; `move_to` without rpy builds a position-only goal (template text check plus container check); brief contains joint count, facing direction, no-probing rule.
- Container check: a 4-joint variant of the synthetic arm reaches a position with `move_to(xyz)`; `tool_down()` + `move_to` works on the IRB 1300.
- Frontend vitest: Program mode layout and Back; conversation switch/new/rename; markdown rendering, collapsible steps, final text once; file tree + editor save then run; changed dots.
- Manual: 5 benchmark tasks on a 6-joint IRB 1300, one conversation each.

## 9. Out of scope

Physics, grippers, multiple robots per window, editing generated files, git-style history of program versions, streaming the editor live while Claude types.
