# Responsive UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The app never looks stuck: the CAD import and ROS startup show live progress, and Program mode stays usable while ROS starts and while a task runs.

**Architecture:** Import pushes progress over a streaming response (`text/event-stream`, the same format as the agent run). ROS start returns at once and runs its slow host-side work in a background thread. The existing status poll reports that thread's stage, then the container's stage from `/tmp/stage`. Conversation routes resolve their folder from a `?robot=` name, so they no longer need Docker. ProgramMode tracks the running task by conversation id instead of a global lock.

**Tech Stack:** FastAPI + starlette `StreamingResponse`, Python `threading`/`queue`; React 18 + TypeScript, vitest + Testing Library; Docker (ROS 2 Jazzy session container).

**Spec:** `docs/superpowers/specs/2026-10-14-responsive-ui-design.md`

## Global Constraints

- Backend tests: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest <path> -q` (OpenBLAS crashes above 1 thread on this machine).
- Frontend tests: `cd frontend && npx vitest run <path>`; type check: `cd frontend && npx tsc --noEmit`.
- Stage names, shared by backend and frontend, exactly: `export`, `collisions`, `image`, `build`, `moveit`, `bridge`, `ready`.
- Import event kinds, exactly: `progress` (`stage` ∈ `read`, `parts`, `faces`; `done`, `total`), `done` (`bodies`, `applied_scale`, `warnings`), `error` (`message`).
- No speed work (caching, skipping regeneration), no task queue, no import cancel.
- Copy style: plain sentences ending in a period for errors; "Waiting for …" text for disabled controls.
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- Long real-app checks (Docker start ≈ 1–3 min): run in the background and say how long they'll take.

## Review Focus

1. **A second Start while host prep still runs** (double click, or Stop then Start fast): expect a 409 "Startup is still finishing; try again in a moment.", not two threads writing one workspace. Test in Task 4.
2. **Stop pressed during host prep:** expect no `docker run` afterwards and the status `stopped`, not a container appearing a minute later. Test in Task 4.
3. **The run ends while another chat is open:** expect the open chat to stay open and the finished chat's answer to be saved, not a jump to the finished chat. Test in Task 7.
4. **An import that fails mid-stream** (corrupt file after the stream started): expect the same "Import of x failed: …" banner as today, not a silent hang or an unparsed stream. Test in Task 2 (backend) and Task 3 (frontend).
5. **Robot name in the conversation routes is a path:** `?robot=../x` must be a 400, never a read or write outside `~/.cad-to-urdf/workspaces`. Test in Task 6.

---

## File map

| File | Change |
|---|---|
| `backend/app/importers/step_importer.py` | `import_cad(..., on_progress=None)`; groups planned up front so the total is known |
| `backend/app/importers/pipeline.py` | `import_path(..., on_progress=None)` |
| `backend/app/routes/upload.py` | split `run_import` into `save_upload` + `import_saved` (shared error text) |
| `backend/app/routes/import_routes.py` | `/import` streams events |
| `backend/app/ros/collisions.py`, `moveit_config.py`, `cli.py` | `on_progress` passed down to the sampling loop |
| `backend/app/program/session.py` | background start thread, stages, `SessionBusy` |
| `backend/app/program/routes.py` | 409 for `SessionBusy`; `?robot=` on conversation CRUD |
| `docker/jazzy/session/start.sh` | writes `/tmp/stage` |
| `frontend/src/api/client.ts` | `parseSse` + `readEvents` moved here; `importFile(file, scale, onProgress?)` |
| `frontend/src/api/program.ts` | uses shared SSE helpers; `SessionStatus` gains stage fields; robot arg on conversation calls |
| `frontend/src/ui/useElapsed.ts` (new) | elapsed-seconds hook + `fmtElapsed` |
| `frontend/src/ui/Stepper.tsx` | import progress strip |
| `frontend/src/ui/program/StartupPanel.tsx` (new) | stage checklist + log |
| `frontend/src/ui/program/useRosSession.ts` | chained poll; keeps last stage on failure |
| `frontend/src/ui/program/ProgramMode.tsx` | pill + panel; `runningId`; robot for conversations; waiting reasons |
| `frontend/src/ui/program/ConversationList.tsx` | `runningId` prop: spinner, no delete on the running chat |
| `frontend/src/ui/program/Chat.tsx` | `waiting` prop |
| `frontend/src/ui/theme.css` | small styles for strip, pill, panel, spinner dot |

---

### Task 1: Reproduce the "new chat hangs" and measure RAM

Throwaway measurement; nothing here is committed except the findings appended to the spec.

**Files:**
- Create (scratch, not committed): `$CLAUDE_JOB_DIR/tmp/measure.py`
- Modify: `docs/superpowers/specs/2026-10-14-responsive-ui-design.md` (append a "Findings" section)

- [ ] **Step 1: Start Docker Desktop and the backend** (backend: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m uvicorn app.main:app --port 8000`, in the background).

- [ ] **Step 2: Write the timing script.** It must poll status at 1 Hz *without waiting* for the previous poll, like the current frontend's `setInterval`. In parallel, it times `GET /program/conversations` and `POST /program/conversations` every 3 s, and prints the latency of each.

```python
import json, threading, time, urllib.request
B = "http://127.0.0.1:8000"
def call(method, path, body=None):
    t = time.perf_counter()
    req = urllib.request.Request(B + path, method=method, data=json.dumps(body).encode() if body is not None else None,
                                 headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=60).read(); ok = "ok"
    except Exception as e:
        ok = type(e).__name__
    return time.perf_counter() - t, ok
def poller(stop):
    while not stop.is_set():
        threading.Thread(target=lambda: print("status %.2fs %s" % call("GET", "/program/session")), daemon=True).start()
        time.sleep(1)
stop = threading.Event(); threading.Thread(target=poller, args=(stop,), daemon=True).start()
for _ in range(40):
    print("list   %.2fs %s" % call("GET", "/program/conversations"))
    print("create %.2fs %s" % call("POST", "/program/conversations", {}))
    time.sleep(3)
stop.set()
```

- [ ] **Step 3: Run it while ROS starts.** Start the session from the app UI (Program mode, irb1300), then run `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe $CLAUDE_JOB_DIR/tmp/measure.py` in the background. This takes about 2 min. Run it once more while a task runs in the app.

- [ ] **Step 4: Record RAM per piece** during the run:

```powershell
Get-Process python,msedge,chrome,claude,vmmem*,"Docker Desktop" -ErrorAction SilentlyContinue | Group-Object ProcessName | % { "{0} {1:N0} MB" -f $_.Name, (($_.Group | Measure-Object WorkingSet64 -Sum).Sum/1MB) }
docker stats --no-stream --format "{{.Name}} {{.MemUsage}}"
```

- [ ] **Step 5: Append the findings to the spec.** Add a `## Findings (Task 1)` section with: slowest list/create latency while starting and while running, the status latency range, RAM per piece, and which cause (sidebar disabled / poll pile-up / other) explains the hang. If a cause other than those two is found, stop and tell the user before continuing. It needs its own fix task.

- [ ] **Step 6: Commit**

```bash
git add docs/superpowers/specs/2026-10-14-responsive-ui-design.md
git commit -m "docs: new-chat hang and RAM findings"
```

---

### Task 2: Backend — import streams progress

**Files:**
- Modify: `backend/app/importers/step_importer.py:293-340` (`import_cad`)
- Modify: `backend/app/importers/pipeline.py:27-38` (`import_path`)
- Modify: `backend/app/routes/upload.py`
- Modify: `backend/app/routes/import_routes.py`
- Test: `backend/tests/test_end_to_end.py` (existing `/import` tests), `backend/tests/test_step_importer.py`

**Interfaces:**
- Produces: `import_cad(file_path, brep_scale=0.001, on_progress=None)`, `import_path(path, suffix, scale, display_name=None, on_progress=None)` where `on_progress(stage: str, done: int, total: int)`.
- Produces: `upload.import_saved(tmp_path, suffix, filename, scale, on_progress=None) -> (bodies, applied_scale, warnings)`; raises `HTTPException(422, ...)`; always deletes `tmp_path`.
- Produces: `POST /import` → `text/event-stream`; frames `data: {json}\n\n`; last frame is `done` or `error`. Unsupported extension → HTTP 400; non-finite or non-positive `scale` → HTTP 422 (both before the stream).

- [ ] **Step 1: Write the failing tests.** In `backend/tests/test_end_to_end.py`, add a helper and rewrite the `/import` tests to read events:

```python
def _import_events(c, name, data, query=""):
    r = c.post(f"/import{query}", files={"file": (name, data)})
    return r, [json.loads(x[6:]) for x in r.text.split("\n\n") if x.startswith("data: ")]


def test_import_box_stl_segments_into_six_planes(box_stl_path):
    with open(box_stl_path, "rb") as f:
        r, ev = _import_events(client, "box.stl", f.read())
    assert r.status_code == 200 and r.headers["content-type"].startswith("text/event-stream")
    stages = [e["stage"] for e in ev if e["kind"] == "progress"]
    assert stages[:1] == ["read"] and "faces" in stages
    done = ev[-1]
    assert done["kind"] == "done" and done["applied_scale"] == 0.001 and len(done["bodies"]) == 1
    faces = done["bodies"][0]["faces"]
    assert len(faces) == 6 and all(f["kind"] == "plane" and f["origin"] == "segmented" for f in faces)


def test_import_nan_scale_is_422(box_stl_path):
    with open(box_stl_path, "rb") as f:
        r = client.post("/import?scale=nan", files={"file": ("box.stl", f.read())})
    assert r.status_code == 422


def test_import_scale_override(box_stl_path):
    with open(box_stl_path, "rb") as f:
        _, ev = _import_events(client, "box.stl", f.read(), "?scale=1.0")
    assert ev[-1]["kind"] == "done" and ev[-1]["applied_scale"] == 1.0
```

Rewrite `test_import_corrupt_file_is_422_naming_upload` (keep its parametrize) as:

```python
def test_import_corrupt_file_is_an_error_event_naming_upload(name, data):
    r, ev = _import_events(_raising_client, name, data)
    assert r.status_code == 200 and ev[-1]["kind"] == "error"
    msg = ev[-1]["message"]
    assert name in msg and "tmp" not in msg.lower() and "temp" not in msg.lower()
```

Keep `test_import_unsupported_extension_is_400` unchanged. Add `import json` at the top if missing. In `backend/tests/test_step_importer.py`, add (use the STEP fixture that file already uses; name it `<fixture>` below):

```python
def test_import_cad_reports_parts_progress(<fixture>):
    seen = []
    bodies, _ = import_cad(<fixture path>, on_progress=lambda s, d, t: seen.append((s, d, t)))
    assert seen[0] == ("read", 0, 0)
    parts = [x for x in seen if x[0] == "parts"]
    assert parts and parts[-1][1] == parts[-1][2] and [p[1] for p in parts] == list(range(1, len(parts) + 1))
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_end_to_end.py tests/test_step_importer.py -q`
Expected: FAIL. The event tests find no `data:` frames, and `import_cad` gets an unexpected keyword `on_progress`.

- [ ] **Step 3: Implement `import_cad` progress.** Plan all groups first so the total is known, then build them:

```python
def import_cad(file_path: str, brep_scale: float = 0.001, on_progress=None) -> tuple[list[Body], list[PartWarning]]:
    """...(keep docstring)... `on_progress(stage, done, total)` reports "read", then "parts" per group."""
    if not (math.isfinite(brep_scale) and brep_scale > 0):
        raise ValueError(f"brep_scale must be finite and positive, got {brep_scale}")
    report = on_progress or (lambda *a: None)
    report("read", 0, 0)
    ext = os.path.splitext(file_path)[1].lower()
    named = (_read_step_named(file_path) if ext in (".step", ".stp")
             else [(None, _read_shape(file_path, brep_scale))])

    plan = []
    total_faces = accounted = 0
    for product, shape in named:
        total_faces += len(_subshapes(shape, TopAbs_FACE))
        groups = [([s], "solid", False) for s in _subshapes(shape, TopAbs_SOLID)]
        groups += [([sh], "shell", True) for sh in _subshapes(shape, TopAbs_SHELL, TopAbs_SOLID)]
        loose = _loose_faces(shape)
        if loose:
            groups.append((loose, "surface", True))
        plan.append((product, groups))
    total, built = sum(len(g) for _, g in plan), 0

    bodies: list[Body] = []
    for product, groups in plan:
        mine: list[Body] = []
        if not groups:
            mine.append(_failed_body(product or f"part_{len(bodies)}", "no geometry was transferred"))
        for sources, prefix, repaired in groups:
            # ... existing try/except body unchanged ...
            mine += new
            built += 1
            report("parts", built, total)
        # ... existing product renaming unchanged ...
        bodies += mine
    # ... rest unchanged ...
```

- [ ] **Step 4: Implement `import_path` progress**

```python
def import_path(path: str, suffix: str, scale: float, display_name: str | None = None,
                on_progress=None) -> tuple[list[Body], float, list[PartWarning]]:
    report = on_progress or (lambda *a: None)
    if suffix in STEP_EXTENSIONS:
        if suffix == ".brep":
            bodies, warnings = import_cad(path, brep_scale=scale, on_progress=on_progress)
            return bodies, scale, warnings
        bodies, warnings = import_cad(path, on_progress=on_progress)
        return bodies, 1.0, warnings
    if suffix in MESH_EXTENSIONS:
        report("read", 0, 0)
        body, applied = import_mesh(path, scale=scale, display_name=display_name)
        report("faces", 0, 0)
        return [segment_mesh_body(body)], applied, []
    raise ValueError(f"Unsupported file type: {suffix}")
```

- [ ] **Step 5: Split `upload.py`** (`replace_routes.py` keeps calling `run_import` unchanged):

```python
def import_saved(tmp_path: str, suffix: str, filename: str | None, scale: float,
                 on_progress=None) -> tuple[list[Body], float, list[PartWarning]]:
    name = os.path.splitext(os.path.basename(filename or ""))[0] or None
    try:
        return import_path(tmp_path, suffix, scale, display_name=name, on_progress=on_progress)
    except ValueError as exc:
        raise HTTPException(422, f"{filename}: {exc}")
    except Exception as exc:  # corrupt uploads can fail anywhere in OCCT/trimesh;
        # library messages may carry the temp path, so name only the error type
        raise HTTPException(422, f"{filename}: could not import file ({type(exc).__name__})")
    finally:
        os.unlink(tmp_path)


def run_import(file: UploadFile, scale: float) -> tuple[list[Body], float, list[PartWarning]]:
    tmp_path, suffix = save_upload(file)
    return import_saved(tmp_path, suffix, file.filename, scale)
```

- [ ] **Step 6: Stream from `/import`**

```python
import json
import math
import queue
import threading

from fastapi import APIRouter, HTTPException, UploadFile
from fastapi.responses import StreamingResponse

from app.routes.upload import import_saved, save_upload

router = APIRouter()


@router.post("/import")
def import_file(file: UploadFile, scale: float = 0.001) -> StreamingResponse:
    """Streams progress events, then one done or error event (same frames as the agent run)."""
    if not (math.isfinite(scale) and scale > 0):
        raise HTTPException(422, f"scale must be finite and positive, got {scale}")
    tmp_path, suffix = save_upload(file)  # 400 for unsupported types, before the stream
    events: queue.Queue = queue.Queue()

    def work() -> None:
        try:
            bodies, applied, warnings = import_saved(
                tmp_path, suffix, file.filename, scale,
                on_progress=lambda s, d, t: events.put({"kind": "progress", "stage": s, "done": d, "total": t}))
            events.put({"kind": "done", "bodies": [b.model_dump() for b in bodies], "applied_scale": applied,
                        "warnings": [w.model_dump() for w in warnings]})
        except HTTPException as exc:
            events.put({"kind": "error", "message": exc.detail})
        except Exception as exc:  # never leave the stream hanging
            events.put({"kind": "error", "message": f"{file.filename}: could not import file ({type(exc).__name__})"})
        finally:
            events.put(None)

    threading.Thread(target=work, daemon=True).start()

    def stream():
        while (e := events.get()) is not None:
            yield f"data: {json.dumps(e)}\n\n"

    return StreamingResponse(stream(), media_type="text/event-stream")
```

Check the existing nan test's 422 still comes from this guard. Before this change it came from `import_mesh`.

- [ ] **Step 7: Run the tests**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_end_to_end.py tests/test_step_importer.py tests/test_pipeline.py -q`
Expected: PASS

- [ ] **Step 8: Commit**

```bash
git add backend/app/importers backend/app/routes backend/tests/test_end_to_end.py backend/tests/test_step_importer.py
git commit -m "feat: /import streams progress (reading, parts N/M, faces) then the result"
```

---

### Task 3: Frontend — import progress strip

**Files:**
- Modify: `frontend/src/api/client.ts` (move in `parseSse`, add `readEvents`, new `importFile`)
- Modify: `frontend/src/api/program.ts` (use the moved helpers; re-export `parseSse`)
- Create: `frontend/src/ui/useElapsed.ts`
- Modify: `frontend/src/ui/Stepper.tsx:37-50,83-87`
- Modify: `frontend/src/ui/theme.css`
- Test: `frontend/src/api/client.test.ts`, `frontend/src/ui/shell.test.tsx`

**Interfaces:**
- Consumes: the Task 2 event stream.
- Produces: `parseSse<T>(buffer: string): { events: T[]; rest: string }`, `readEvents<T>(res: Response, onEvent: (e: T) => void): Promise<void>` in `client.ts`.
- Produces: `type ImportProgress = { stage: "upload" | "read" | "parts" | "faces"; done: number; total: number }`; `importFile(file, scale, onProgress?: (p: ImportProgress) => void): Promise<ImportResponse>`. An `error` event throws `ApiError(422, message)`.
- Produces: `useElapsed(active: boolean): number` (whole seconds since `active` became true; 0 when inactive), `fmtElapsed(s: number): string` → `"0:07"`, `"1:23"`.

- [ ] **Step 1: Write the failing tests.** In `client.test.ts`, replace the two `importFile` tests:

```ts
const sse = (...events: unknown[]) => new Response(events.map((e) => `data: ${JSON.stringify(e)}\n\n`).join(""), { status: 200 });

it("posts uploads to /api/import with the scale and reports progress", async () => {
  const fetchMock = vi.spyOn(globalThis, "fetch").mockResolvedValue(sse(
    { kind: "progress", stage: "read", done: 0, total: 0 },
    { kind: "progress", stage: "parts", done: 2, total: 5 },
    { kind: "done", bodies: [], applied_scale: 0.001, warnings: [] }));
  const seen: unknown[] = [];
  const res = await importFile(new File(["x"], "part.stl"), 0.001, (p) => seen.push(p));
  const [url, init] = fetchMock.mock.calls[0];
  expect(url).toBe("/api/import?scale=0.001");
  expect((init as RequestInit).body).toBeInstanceOf(FormData);
  expect(seen).toEqual([{ stage: "upload", done: 0, total: 0 }, { stage: "read", done: 0, total: 0 }, { stage: "parts", done: 2, total: 5 }]);
  expect(res).toEqual({ bodies: [], applied_scale: 0.001, warnings: [] });
});

it("raises ApiError for an error event and for an HTTP error", async () => {
  vi.spyOn(globalThis, "fetch").mockResolvedValueOnce(sse({ kind: "error", message: "part.stl: could not read mesh" }))
    .mockResolvedValueOnce(new Response(JSON.stringify({ detail: "Unsupported file type: .txt" }), { status: 400 }));
  await expect(importFile(new File(["x"], "part.stl"), 0.001)).rejects.toMatchObject({ status: 422, message: "part.stl: could not read mesh" });
  await expect(importFile(new File(["x"], "a.txt"), 0.001)).rejects.toBeInstanceOf(ApiError);
});

it("a stream that ends without a result is an error", async () => {
  vi.spyOn(globalThis, "fetch").mockResolvedValue(sse({ kind: "progress", stage: "read", done: 0, total: 0 }));
  await expect(importFile(new File(["x"], "part.stl"), 0.001)).rejects.toThrow(/ended early/);
});
```

In `shell.test.tsx`, change `expect(spy).toHaveBeenCalledWith(file, 0.001);` to `expect(spy).toHaveBeenCalledWith(file, 0.001, expect.any(Function));` and add:

```ts
it("shows import progress while importing", async () => {
  let report!: (p: client.ImportProgress) => void;
  let finish!: (r: Awaited<ReturnType<typeof client.importFile>>) => void;
  vi.spyOn(client, "importFile").mockImplementation((_f, _s, onProgress) => {
    report = onProgress!;
    return new Promise((r) => { finish = r; });
  });
  render(<Harness start={imported([])} ui={(ctx) => <Stepper ctx={ctx} progress={idle} onCancel={vi.fn()} onResume={vi.fn()} />} />);
  await act(async () => { fireEvent.change(screen.getByTestId("import-input"), { target: { files: [new File(["x"], "a.step")] } }); });
  await act(async () => { report({ stage: "parts", done: 2, total: 5 }); });
  expect(screen.getByTestId("import-progress").textContent).toMatch(/Building parts 2 \/ 5/);
  await act(async () => { finish({ bodies: [], applied_scale: 1, warnings: [] }); });
  expect(screen.queryByTestId("import-progress")).toBeNull();
});
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd frontend && npx vitest run src/api/client.test.ts src/ui/shell.test.tsx`
Expected: FAIL (`importFile` returns `res.json()`; no `import-progress` element).

- [ ] **Step 3: Move the SSE helpers into `client.ts`** and generalise them. Delete `parseSse` from `program.ts`, and have `streamEvents` there call `readEvents`.

```ts
export function parseSse<T>(buffer: string): { events: T[]; rest: string } {
  const parts = buffer.replace(/\r\n/g, "\n").split("\n\n");
  const rest = parts.pop() ?? "";
  const events: T[] = [];
  for (const p of parts) {
    if (!p.startsWith("data: ")) continue;
    try { events.push(JSON.parse(p.slice(6)) as T); } catch { /* skip bad frame */ }
  }
  return { events, rest };
}

export async function readEvents<T>(res: Response, onEvent: (e: T) => void): Promise<void> {
  const reader = res.body!.getReader();
  const decoder = new TextDecoder();
  let buffer = "";
  for (;;) {
    const { done, value } = await reader.read();
    if (value) buffer += decoder.decode(value, { stream: true });
    const { events, rest } = parseSse<T>(buffer);
    buffer = rest;
    events.forEach(onEvent);
    if (done) break;
  }
  parseSse<T>(buffer + "\n\n").events.forEach(onEvent);
}
```

In `program.ts`:

```ts
import { json, parseSse, readEvents, send } from "./client";
export { parseSse };  // program.test.ts imports it from here
// ...
async function streamEvents(path: string, body: unknown, onEvent: (e: ProgramEvent) => void,
  signal?: AbortSignal): Promise<void> {
  await readEvents<ProgramEvent>(await send(path, { ...json(body), signal }), onEvent);
}
```

Existing `program.test.ts` calls `parseSse(...)` without a type argument. If `tsc` complains about `unknown` there, re-export a typed alias instead: `export const parseSse = (b: string) => sse<ProgramEvent>(b);` with `import { parseSse as sse }`.

- [ ] **Step 4: New `importFile`**

```ts
export type ImportProgress = { stage: "upload" | "read" | "parts" | "faces"; done: number; total: number };
type ImportEvent = ({ kind: "progress" } & ImportProgress) | ({ kind: "done" } & ImportResponse) | { kind: "error"; message: string };

export async function importFile(file: File, scale: number,
  onProgress?: (p: ImportProgress) => void): Promise<ImportResponse> {
  const form = new FormData();
  form.append("file", file);
  onProgress?.({ stage: "upload", done: 0, total: 0 });
  const res = await send(`/import?scale=${scale}`, { method: "POST", body: form });
  let result: ImportResponse | null = null;
  let error: string | null = null;
  await readEvents<ImportEvent>(res, (e) => {
    if (e.kind === "progress") onProgress?.({ stage: e.stage, done: e.done, total: e.total });
    else if (e.kind === "done") result = { bodies: e.bodies, applied_scale: e.applied_scale, warnings: e.warnings };
    else error = e.message;
  });
  if (error) throw new ApiError(422, error);
  if (!result) throw new ApiError(0, "The import ended early. Is the backend still running?");
  return result;
}
```

- [ ] **Step 5: `useElapsed.ts`**

```ts
import { useEffect, useState } from "react";

/** Whole seconds since `active` turned true; 0 while inactive. Ticks once a second. */
export function useElapsed(active: boolean): number {
  const [s, setS] = useState(0);
  useEffect(() => {
    setS(0);
    if (!active) return;
    const t0 = Date.now();
    const t = setInterval(() => setS(Math.floor((Date.now() - t0) / 1000)), 1000);
    return () => clearInterval(t);
  }, [active]);
  return s;
}

export const fmtElapsed = (s: number) => `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
```

- [ ] **Step 6: Stepper strip.** Add state `const [imp, setImp] = useState<ImportProgress | null>(null);` and `const elapsed = useElapsed(busy);`. Pass `setImp` as the third arg of `importFile`, and clear it with `setImp(null)` in `finally`. Render it before the Import label:

```tsx
const IMPORT_LABEL = { upload: "Uploading", read: "Reading file", parts: "Building parts", faces: "Finding faces" } as const;
// ...
{busy && imp && (
  <span className="progress import-progress" data-testid="import-progress">
    {IMPORT_LABEL[imp.stage]}{imp.total > 0 ? ` ${imp.done} / ${imp.total}` : "…"}
    {imp.total > 0 && <progress value={imp.done} max={imp.total} />}
    <span className="muted">{fmtElapsed(elapsed)}</span>
  </span>
)}
```

CSS (`theme.css`, next to the existing `.progress` rules):

```css
.import-progress progress { width: 80px; height: 6px; vertical-align: middle; margin: 0 6px; }
```

- [ ] **Step 7: Run tests + type check**

Run: `cd frontend && npx vitest run src/api src/ui/shell.test.tsx && npx tsc --noEmit`
Expected: PASS, no type errors.

- [ ] **Step 8: Commit**

```bash
git add frontend/src/api frontend/src/ui/useElapsed.ts frontend/src/ui/Stepper.tsx frontend/src/ui/theme.css frontend/src/ui/shell.test.tsx
git commit -m "feat: import shows a progress strip (stage, parts N/M, elapsed time)"
```

---

### Task 4: Backend — ROS start in the background with stages

**Files:**
- Modify: `backend/app/ros/collisions.py:52-78`, `backend/app/ros/moveit_config.py:254-266`, `backend/app/ros/cli.py:23-32`
- Modify: `backend/app/program/session.py`
- Modify: `backend/app/program/routes.py:16-40`
- Modify: `docker/jazzy/session/start.sh`
- Test: `backend/tests/test_program_session.py`, `backend/tests/test_program_routes.py`, `backend/tests/test_ros_collisions.py`

**Interfaces:**
- Produces: `disabled_pairs(chain, meshes, samples=10000, seed=0, on_progress=None)`, `generate(..., samples=10000, on_progress=None)`, `workspace_from_zip(zip_source, out_dir, samples=10000, on_progress=None)`. `on_progress(done: int, total: int)` is called every 250 samples and once at the end with `(samples, samples)`.
- Produces: `session.SessionBusy(SessionError)`; `session.start(...)` returns `{"state": "starting", "robot": name, "stage": "collisions", "done": 0, "total": SAMPLES}` at once; `session._thread` (the prep thread, joinable in tests).
- Produces: `session.status()` → `{"state", "robot", "stage"?, "done"?, "total"?, "log"?, "error"?}`. `stage` is one of the global stage names.
- Produces: routes map `SessionBusy` → 409.

- [ ] **Step 1: Write the failing tests.** In `test_ros_collisions.py` (reuse that file's existing chain/mesh fixture, called `<chain>, <meshes>` below):

```python
def test_disabled_pairs_reports_progress(<fixtures>):
    seen = []
    disabled_pairs(<chain>, <meshes>, samples=600, on_progress=lambda d, t: seen.append((d, t)))
    assert seen[-1] == (600, 600) and all(t == 600 for _, t in seen)
```

In `test_program_session.py`, update `FakeDocker`. The `exec` branch answers `cat /tmp/stage` with the current `self.stage` (default `"ready"`):

```python
        if cmd == "exec" and args[-2:] == ["cat", "/tmp/stage"]:
            out = self.stage
```

(add `self.stage = "ready"` in `__init__`, and delete the `/tmp/ready` branch). Add a helper, and make every existing test that calls `session.start(...)` and then checks status or runs a program wait for the thread:

```python
def _started(*model):
    out = session.start("bot", *(model or _model()))
    session._thread.join(timeout=60)
    return out
```

Also reset module state in the `fake` fixture: `monkeypatch.setattr(session, "_prep", None); monkeypatch.setattr(session, "_thread", None); monkeypatch.setattr(session, "_current", None)`.

New tests:

```python
def test_start_returns_at_once_and_status_walks_the_stages(fake, monkeypatch):
    gate = threading.Event()
    real = session.workspace_from_zip

    def slow(data, ws, samples, on_progress):
        on_progress(250, samples)
        gate.wait(10)
        return real(data, ws, samples=samples, on_progress=on_progress)
    monkeypatch.setattr(session, "workspace_from_zip", slow)
    out = session.start("bot", *_model())
    assert out["state"] == "starting" and out["stage"] == "collisions"
    st = session.status()
    assert st["state"] == "starting" and st["stage"] == "collisions" and st["done"] == 250
    gate.set(); session._thread.join(60)
    fake.stage = "moveit"
    assert session.status()["stage"] == "moveit" and session.status()["state"] == "starting"
    fake.stage = "ready"
    assert session.status()["state"] == "ready"


def test_missing_image_shows_the_image_stage(fake, monkeypatch):
    fake.image = False
    seen = []
    real = session._docker

    def spy(*args, **kw):
        if args[0] == "build":
            seen.append(session.status()["stage"])
        return real(*args, **kw)
    monkeypatch.setattr(session, "_docker", spy)
    _started()
    assert seen == ["image"]


def test_a_failure_in_the_thread_is_failed_with_stage_and_message(fake, monkeypatch):
    def boom(*a, **k):
        raise ValueError("Robot name 'Bot' is not a valid ROS package name.")
    monkeypatch.setattr(session, "workspace_from_zip", boom)
    _started()
    st = session.status()
    assert st["state"] == "failed" and st["stage"] == "collisions" and "not a valid ROS package name" in st["error"]


def test_stop_during_prep_never_runs_the_container(fake, monkeypatch):
    gate = threading.Event()
    real = session.workspace_from_zip
    monkeypatch.setattr(session, "workspace_from_zip",
                        lambda data, ws, samples, on_progress: (gate.wait(10), real(data, ws, samples=samples))[1])
    session.start("bot", *_model())
    session.stop()
    gate.set(); session._thread.join(60)
    assert not any(c[1] == "run" for c in fake.calls)
    assert session.status()["state"] == "stopped"


def test_second_start_during_prep_is_busy(fake, monkeypatch):
    gate = threading.Event()
    monkeypatch.setattr(session, "workspace_from_zip", lambda *a, **k: gate.wait(10) and "bot")
    session.start("bot", *_model())
    with pytest.raises(session.SessionBusy):
        session.start("bot", *_model())
    gate.set(); session._thread.join(60)


def test_status_while_starting_carries_a_log_tail(fake):
    _started()
    fake.stage = "build"
    st = session.status()
    assert st["state"] == "starting" and st["stage"] == "build" and "log" in st
```

Update `test_start_generates_the_workspace_and_runs_the_container` to use `_started()` and assert `out["state"] == "starting" and out["robot"] == "bot"`. Add `import threading` at the top.

In `test_program_routes.py`:

```python
def test_start_while_busy_is_409(monkeypatch):
    def busy(*a):
        raise routes.session.SessionBusy("Startup is still finishing; try again in a moment.")
    monkeypatch.setattr(routes.session, "start", busy)
    body = {"robot_name": "bot", "links": [], "joints": [], "bodies_by_link": {}}
    r = client.post("/program/session", json=body)
    assert r.status_code == 409 and "still finishing" in r.json()["detail"]
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_program_session.py tests/test_program_routes.py tests/test_ros_collisions.py -q`
Expected: FAIL (`on_progress` unknown, `SessionBusy`/`_thread` missing).

- [ ] **Step 3: Thread `on_progress` down.** In `collisions.py`:

```python
def disabled_pairs(chain: Chain, meshes, samples: int = 10000, seed: int = 0,
                   on_progress=None) -> dict[tuple[str, str], str]:
    # ... unchanged until the sampling loop ...
    rng = np.random.default_rng(seed)
    for i in range(samples):
        if not open_pairs:
            break
        place(_random_q(chain, rng))
        open_pairs = [p for p in open_pairs if not collides(objs[p[0]], objs[p[1]])]
        if on_progress and (i + 1) % 250 == 0:
            on_progress(i + 1, samples)
    if on_progress:
        on_progress(samples, samples)
    # ... unchanged ...
```

In `moveit_config.generate`, add the `on_progress=None` param and pass `on_progress=on_progress` to `disabled_pairs`. In `cli.workspace_from_zip`, add `on_progress=None` and pass it to `generate`.

- [ ] **Step 4: Rewrite `session.start` / `status` / `stop`.** Replace `start`, `status`, `stop` in `session.py` (keep `_docker`, `current`, `logs`, `run_program`):

```python
import threading

class SessionBusy(SessionError):
    pass


_lock = threading.Lock()
_prep: dict | None = None  # host-side startup: robot, stage, done, total, error, stop, launched
_thread: threading.Thread | None = None


def start(robot_name: str, links, joints, bodies_by_link) -> dict:
    """Check fast, then export the workspace and (re)start the container in a background thread."""
    global _prep, _thread
    data = export_package(robot_name, links, joints, bodies_by_link)  # ValueError -> caller
    _docker("info")
    with _lock:
        if _thread is not None and _thread.is_alive():
            raise SessionBusy("Startup is still finishing; try again in a moment.")
        _prep = prep = {"robot": robot_name, "stage": "collisions", "done": 0, "total": SAMPLES,
                        "error": None, "stop": False, "launched": False}
        _thread = threading.Thread(target=_prepare, args=(prep, data), daemon=True)
        _thread.start()
    return {"state": "starting", "robot": robot_name, "stage": "collisions", "done": 0, "total": SAMPLES}


def _prepare(prep: dict, data: bytes) -> None:
    global _current
    try:
        ws = WORKSPACES / prep["robot"]
        robot = workspace_from_zip(data, ws, samples=SAMPLES,
                                   on_progress=lambda d, t: prep.update(done=d, total=t))
        if _docker("image", "inspect", IMAGE, check=False).returncode != 0:
            prep.update(stage="image", done=0, total=0)
            _docker("build", "-t", IMAGE, str(DOCKER_DIR))
        with _lock:
            if prep["stop"]:
                return
            _docker("rm", "-f", SESSION_CONTAINER, check=False)
            src = ws / "src"
            _docker("run", "-d", "--name", SESSION_CONTAINER, "-p", "127.0.0.1:9090:9090",
                    "--label", f"cad-to-urdf.robot={robot}", "--label", f"cad-to-urdf.src={src}",
                    "-v", f"{ws}:/ws", "-v", f"{DOCKER_DIR / 'session'}:/session:ro",
                    IMAGE, "bash", "/session/start.sh")
            _current = {"robot": robot, "src": src}
            prep.update(stage="build", done=0, total=0, launched=True)
    except Exception as exc:  # any failure must reach the UI, not die with the thread
        prep["error"] = str(exc) or type(exc).__name__


def status() -> dict:
    prep = _prep
    if prep is not None and not prep["stop"]:
        if prep["error"]:
            return {"state": "failed", "robot": prep["robot"], "stage": prep["stage"], "error": prep["error"]}
        if not prep["launched"]:
            return {"state": "starting", "robot": prep["robot"], "stage": prep["stage"],
                    "done": prep["done"], "total": prep["total"]}
    cur = current()
    robot = cur["robot"] if cur else None
    r = _docker("inspect", "-f", "{{.State.Status}}", SESSION_CONTAINER, check=False)
    if r.returncode != 0 or not cur:
        return {"state": "stopped", "robot": robot}
    if r.stdout.strip() != "running":
        return {"state": "failed", "robot": robot, "log": logs()}
    stage = _docker("exec", SESSION_CONTAINER, "cat", "/tmp/stage", check=False).stdout.strip() or "build"
    if stage == "ready":
        return {"state": "ready", "robot": robot}
    return {"state": "starting", "robot": robot, "stage": stage, "log": logs(8)}


def stop() -> None:
    global _current, _prep
    with _lock:
        if _prep is not None:
            _prep["stop"] = True
        _prep = None
        _docker("rm", "-f", SESSION_CONTAINER, check=False)
        _current = None
```

Note: `start()` no longer touches the old container. The thread removes it right before `docker run`, so a running session stays usable during prep.

- [ ] **Step 5: Routes.** In `start_session`, catch `SessionBusy` before `SessionError`:

```python
    except session.SessionBusy as exc:
        raise HTTPException(409, str(exc))
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
```

- [ ] **Step 6: `start.sh` writes stages.** Insert stage writes (keep `touch /tmp/ready`):

```bash
echo build > /tmp/stage
cd /ws && colcon build 2>&1 | tail -n 30
source /ws/install/setup.bash
ROBOT=$(basename "$(ls -d src/*_moveit_config)" _moveit_config)
echo moveit > /tmp/stage
ros2 launch "${ROBOT}_moveit_config" demo.launch.py &
LAUNCH=$!
echo bridge > /tmp/stage
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
# ... wait loop unchanged ...
touch /tmp/ready
echo ready > /tmp/stage
```

The MoveIt and bridge launches start back to back, so `bridge` overwrites `moveit` almost at once. Make it truthful: inside the `until` loop, write `moveit` until `/move_action` exists, then `bridge`:

```bash
until ros2 action list 2>/dev/null | grep -q '^/move_action$' && bash -c '</dev/tcp/127.0.0.1/9090' 2>/dev/null; do
  ros2 action list 2>/dev/null | grep -q '^/move_action$' && echo bridge > /tmp/stage
  # ... existing kill/deadline checks ...
  sleep 1
done
```

(Remove the `echo bridge` line before the rosbridge launch; the loop writes it.)

- [ ] **Step 7: Run tests**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_program_session.py tests/test_program_routes.py tests/test_ros_collisions.py tests/test_ros_moveit_config.py tests/test_ros_cli.py tests/test_program_conversation_run.py -q`
Expected: PASS

- [ ] **Step 8: Commit**

```bash
git add backend/app/ros backend/app/program/session.py backend/app/program/routes.py docker/jazzy/session/start.sh backend/tests
git commit -m "feat: ROS start returns at once and reports its stage (collisions N/M, image, build, MoveIt, bridge)"
```

---

### Task 5: Frontend — startup checklist and non-overlapping poll

**Files:**
- Modify: `frontend/src/api/program.ts` (`SessionStatus`)
- Modify: `frontend/src/ui/program/useRosSession.ts`
- Create: `frontend/src/ui/program/StartupPanel.tsx`
- Modify: `frontend/src/ui/program/ProgramMode.tsx:166-184` (header pill; remove the separate failed `<pre>`)
- Modify: `frontend/src/ui/theme.css`
- Test: `frontend/src/ui/program/programMode.test.tsx`

**Interfaces:**
- Consumes: Task 4 status shape.
- Produces: `type Stage = "export" | "collisions" | "image" | "build" | "moveit" | "bridge" | "ready"`; `SessionStatus { state; robot?; stage?: Stage; done?: number; total?: number; log?: string; error?: string }`.
- Produces: `STAGES: { id: Stage; label: string }[]` and `stageText(s: SessionStatus): string` exported from `StartupPanel.tsx` (used by Task 7 for "Waiting for ROS… (<stage>)").
- Produces: `<StartupPanel status={SessionStatus} />`.

- [ ] **Step 1: Write the failing tests** (add to `programMode.test.tsx`):

```ts
it("shows the startup stage in the header and the checklist on click", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([]);
  vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "starting", robot: "bot", stage: "collisions", done: 4200, total: 10000 });
  ui();
  const pill = await screen.findByRole("button", { name: /Checking collisions 4,200 \/ 10,000/ });
  fireEvent.click(pill);
  const panel = screen.getByTestId("startup-panel");
  expect(panel.querySelector('[data-stage="export"]')!.className).toMatch(/done/);
  expect(panel.querySelector('[data-stage="collisions"]')!.className).toMatch(/current/);
  expect(panel.querySelector('[data-stage="build"]')!.className).toMatch(/todo/);
});

it("opens the checklist by itself on failure, marking the stage that failed", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([]);
  vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "failed", robot: "bot", stage: "image", error: "docker build failed" });
  ui();
  const panel = await screen.findByTestId("startup-panel");
  expect(panel.querySelector('[data-stage="image"]')!.className).toMatch(/failed/);
  expect(panel.textContent).toMatch(/docker build failed/);
});

it("never has two status requests in flight", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([]);
  let inFlight = 0, most = 0;
  vi.spyOn(api, "sessionStatus").mockImplementation(async () => {
    inFlight++; most = Math.max(most, inFlight);
    await new Promise((r) => setTimeout(r, 40));  // slower than POLL.starting (10 ms)
    inFlight--;
    return { state: "starting", robot: "bot", stage: "build" };
  });
  ui();
  await act(async () => { await new Promise((r) => setTimeout(r, 200)); });
  expect(most).toBe(1);
});
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd frontend && npx vitest run src/ui/program/programMode.test.tsx`
Expected: FAIL (no pill button, no panel; the overlap test sees `most > 1`).

- [ ] **Step 3: Types** in `program.ts`:

```ts
export type Stage = "export" | "collisions" | "image" | "build" | "moveit" | "bridge" | "ready";
export interface SessionStatus { state: "stopped" | "starting" | "ready" | "failed"; robot?: string | null;
  stage?: Stage; done?: number; total?: number; log?: string; error?: string }
```

- [ ] **Step 4: Chained poll, and the last stage kept on failure**, in `useRosSession.ts`. Replace `refresh` and the poll effect:

```ts
  async function refresh() {
    try {
      const s = await api.sessionStatus();
      // a dead container reports no stage: keep the last one seen so the panel can mark it
      if (mounted.current) setStatus((p) => (s.state === "failed" && !s.stage ? { ...s, stage: p.stage } : s));
    } catch (e) { if (mounted.current) setError((e as Error).message); }
  }

  useEffect(() => {  // poll while starting (fast) and ready (detects a dead session); never two at once
    if (status.state !== "starting" && status.state !== "ready") return;
    let stopped = false;
    let t = 0;
    const tick = async () => { await refresh(); if (!stopped) t = window.setTimeout(tick, status.state === "starting" ? POLL.starting : POLL.ready); };
    t = window.setTimeout(tick, status.state === "starting" ? POLL.starting : POLL.ready);
    return () => { stopped = true; clearTimeout(t); };
  }, [status.state]);  // eslint-disable-line react-hooks/exhaustive-deps
```

- [ ] **Step 5: `StartupPanel.tsx`**

```tsx
import { Check, Circle, Loader2, X } from "lucide-react";
import type { SessionStatus, Stage } from "../../api/program";

export const STAGES: { id: Stage; label: string }[] = [
  { id: "export", label: "Exporting robot" },
  { id: "collisions", label: "Checking collisions" },
  { id: "image", label: "Building Docker image (first time only)" },
  { id: "build", label: "Building workspace" },
  { id: "moveit", label: "Starting MoveIt" },
  { id: "bridge", label: "Starting bridge" },
  { id: "ready", label: "Ready" },
];

const count = (s: SessionStatus) => (s.total ? ` ${s.done!.toLocaleString("en-US")} / ${s.total.toLocaleString("en-US")}` : "");

/** "Checking collisions 4,200 / 10,000" — the current stage in words. */
export function stageText(s: SessionStatus): string {
  const st = STAGES.find((x) => x.id === s.stage);
  return st ? st.label + count(s) : "Starting";
}

export default function StartupPanel({ status }: { status: SessionStatus }) {
  const cur = STAGES.findIndex((x) => x.id === (status.stage ?? "export"));
  return (
    <div className="startup-panel" data-testid="startup-panel">
      <ol>
        {STAGES.map((x, i) => {
          const cls = status.state === "ready" || i < cur ? "done"
            : i > cur ? "todo" : status.state === "failed" ? "failed" : "current";
          return (
            <li key={x.id} data-stage={x.id} className={cls}>
              {cls === "done" ? <Check size={13} /> : cls === "failed" ? <X size={13} />
                : cls === "current" ? <Loader2 size={13} className="spin" /> : <Circle size={13} />}
              {x.label}{cls === "current" ? count(status) : ""}
            </li>
          );
        })}
      </ol>
      {status.error && <p className="error">{status.error}</p>}
      {status.log && <pre className="log">{status.log}</pre>}
    </div>
  );
}
```

- [ ] **Step 6: Header pill in `ProgramMode.tsx`.** Replace the `<span className="muted">…</span>` status text with:

```tsx
const [panel, setPanel] = useState(false);
const elapsed = useElapsed(status.state === "starting");
const showPanel = panel || status.state === "failed";
// ...
{status.state === "starting" || status.state === "failed" ? (
  <span className="startup-anchor">
    <button className="btn btn-ghost startup-pill" aria-expanded={showPanel} onClick={() => setPanel((p) => !p)}>
      {status.state === "failed" ? "ROS session stopped" : `Starting ROS · ${stageText(status)} · ${fmtElapsed(elapsed)}`}
    </button>
    {showPanel && <StartupPanel status={status} />}
  </span>
) : (
  <span className="muted">{status.state === "ready" ? "ROS ready" : "ROS not running"}</span>
)}
```

Delete the old `{status.state === "failed" && status.log && <pre className="log program-error">…</pre>}` line, since the panel shows the log now. Imports: `useElapsed, fmtElapsed` from `../useElapsed`, `StartupPanel, { stageText }` from `./StartupPanel`.

CSS:

```css
.startup-anchor { position: relative; }
.startup-panel { position: absolute; top: 100%; left: 0; z-index: 20; min-width: 320px; margin-top: 4px; padding: 8px 12px;
  background: var(--panel, var(--bg)); border: 1px solid var(--border); border-radius: 6px; box-shadow: 0 4px 16px rgb(0 0 0 / .15); }
.startup-panel ol { list-style: none; margin: 0; padding: 0; display: grid; gap: 4px; }
.startup-panel li { display: flex; align-items: center; gap: 6px; }
.startup-panel li.todo { color: var(--muted); }
.startup-panel li.failed { color: var(--danger); }
.startup-panel .log { max-height: 160px; overflow: auto; font-size: 11px; margin: 8px 0 0; }
.spin { animation: spin 1s linear infinite; }
@keyframes spin { to { transform: rotate(360deg); } }
```

(Check `theme.css` for an existing `.spin`/`@keyframes spin` or panel background variable first, and reuse it if present.)

- [ ] **Step 7: Run tests + type check.** Existing tests that matched `/Starting ROS…/` or the failed `<pre>` need their expectations moved to the pill/panel text.

Run: `cd frontend && npx vitest run src/ui/program && npx tsc --noEmit`
Expected: PASS

- [ ] **Step 8: Commit**

```bash
git add frontend/src/api/program.ts frontend/src/ui/program frontend/src/ui/theme.css
git commit -m "feat: ROS start shows its stage with elapsed time and a checklist; polls never overlap"
```

---

### Task 6: Backend — conversations without the ROS session

**Files:**
- Modify: `backend/app/program/routes.py` (`_src`, `_conv`, the five conversation CRUD routes)
- Test: `backend/tests/test_program_conversations.py`

**Interfaces:**
- Produces: `GET/POST /program/conversations?robot=<name>`, `GET/PATCH/DELETE /program/conversations/{cid}?robot=<name>`. With `robot`, the folder is `session.WORKSPACES / robot / "conversations"`, with no Docker call. Without it, the routes behave as today (the session's robot). A `robot` that fails `ROS_NAME` gives 400 "Invalid robot name.".
- Unchanged: `/conversations/{cid}/run` and `/files/*` still use the session.

- [ ] **Step 1: Write the failing tests** (append to `test_program_conversations.py`):

```python
def test_conversations_work_with_no_session(tmp_path, monkeypatch):
    monkeypatch.setattr(routes.session, "WORKSPACES", tmp_path)

    def no_docker():
        raise AssertionError("must not ask docker")
    monkeypatch.setattr(routes.session, "current", no_docker)
    a = client.post("/program/conversations?robot=bot", json={}).json()
    assert [c["id"] for c in client.get("/program/conversations?robot=bot").json()] == [a["id"]]
    assert client.patch(f"/program/conversations/{a['id']}?robot=bot", json={"title": "Pick"}).json()["title"] == "Pick"
    assert client.get(f"/program/conversations/{a['id']}?robot=bot").json()["title"] == "Pick"
    assert (tmp_path / "bot" / "conversations" / f"{a['id']}.json").exists()
    assert client.delete(f"/program/conversations/{a['id']}?robot=bot").status_code == 200


def test_robot_name_cannot_escape_the_workspaces(tmp_path, monkeypatch):
    monkeypatch.setattr(routes.session, "WORKSPACES", tmp_path / "ws")
    for bad in ("..", "../x", "Bot", "a/b", ""):
        r = client.get("/program/conversations", params={"robot": bad})
        assert r.status_code == 400, bad
    assert not (tmp_path / "x").exists()
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_program_conversations.py -q`
Expected: FAIL (the `robot` query is ignored and `current` is called).

- [ ] **Step 3: Implement.** In `routes.py`:

```python
from app.ros.moveit_config import ROS_NAME


def _src(robot: str | None = None) -> Path:
    """The robot's workspace src: from the name when given (no Docker), else from the session."""
    if robot is not None:
        if not ROS_NAME.match(robot):
            raise HTTPException(400, "Invalid robot name.")
        return session.WORKSPACES / robot / "src"
    try:
        cur = session.current()
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    if not cur:
        raise HTTPException(409, "Start the ROS session first.")
    return Path(cur["src"])


def _conv(cid: str, robot: str | None = None) -> tuple[Path, dict]:
    src = _src(robot)
    # ... unchanged ...
```

Add `robot: str | None = None` to `list_conversations`, `new_conversation`, `get_conversation`, `rename_conversation` and `delete_conversation`, and pass it to `_src` / `_conv`. FastAPI reads it from the query string. `conversations.folder(src)` is `src.parent / "conversations"`, so `WORKSPACES/<robot>/conversations` matches where session conversations already live.

- [ ] **Step 4: Run tests**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_program_conversations.py tests/test_program_conversation_run.py tests/test_program_files.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add backend/app/program/routes.py backend/tests/test_program_conversations.py
git commit -m "feat: conversation routes take ?robot= and work without the ROS session"
```

---

### Task 7: Frontend — Program mode unlocked

**Files:**
- Modify: `frontend/src/api/program.ts` (robot arg on conversation calls)
- Modify: `frontend/src/ui/program/ProgramMode.tsx`
- Modify: `frontend/src/ui/program/ConversationList.tsx`
- Modify: `frontend/src/ui/program/Chat.tsx`
- Modify: `frontend/src/ui/theme.css`
- Test: `frontend/src/ui/program/programMode.test.tsx`, `frontend/src/api/program.test.ts`, `frontend/src/ui/program/chat.test.tsx`

**Interfaces:**
- Consumes: Task 6 routes; `stageText` from Task 5.
- Produces: `listConversations(robot)`, `createConversation(robot, title?)`, `getConversation(robot, id)`, `renameConversation(robot, id, title)`, `deleteConversation(robot, id)`; each appends `?robot=<encodeURIComponent(robot)>`.
- Produces: `ConversationList` prop `runningId: string | null`; `Chat` prop `waiting?: string`.

- [ ] **Step 1: Write the failing tests** (add to `programMode.test.tsx`):

```ts
it("lists and creates conversations while ROS is stopped", async () => {
  vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "stopped" });
  const list = vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "Square" }]);
  vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, "Square"));
  const create = vi.spyOn(api, "createConversation").mockResolvedValue(conv("bbbbbbbbbbbb", "New conversation"));
  ui();
  await screen.findByText("Square");
  expect(list).toHaveBeenCalledWith("robot");
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /new/i })); });
  expect(create).toHaveBeenCalledWith("robot");
  expect(screen.getByRole("textbox", { name: /describe a task/i })).toHaveProperty("disabled", true);
  expect(screen.getByText(/Start ROS to send a task/)).toBeTruthy();
});

it("while a task runs: other chats can be opened, the running one keeps its live stream", async () => {
  const two = [{ id: "aaaaaaaaaaaa", title: "Alpha" }, { id: "bbbbbbbbbbbb", title: "Beta" }];
  vi.spyOn(api, "listConversations").mockResolvedValue(two);
  vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, id === "aaaaaaaaaaaa" ? "Alpha" : "Beta"));
  let push!: (e: api.ProgramEvent) => void;
  let end!: () => void;
  vi.spyOn(api, "runConversation").mockImplementation((_id, _t, onEvent) => { push = onEvent; return new Promise((r) => { end = () => r(); }); });
  await ready();
  await screen.findByText("Alpha");
  fireEvent.change(screen.getByRole("textbox", { name: /describe a task/i }), { target: { value: "go" } });
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /send/i })); });
  await act(async () => { push({ kind: "text", text: "Working on Alpha" }); });
  await act(async () => { fireEvent.click(screen.getByText("Beta")); });          // sidebar not locked
  expect(screen.getByTestId("conv-bbbbbbbbbbbb").className).toMatch(/active/);
  expect(screen.queryByText("Working on Alpha")).toBeNull();
  expect(screen.getByText(/Waiting for task in .Alpha./)).toBeTruthy();
  expect(screen.getByTestId("conv-aaaaaaaaaaaa").querySelector(".running-dot")).toBeTruthy();
  expect(screen.queryByLabelText("Delete Alpha")).toBeNull();                      // can't delete the running chat
  await act(async () => { fireEvent.click(screen.getByText("Alpha")); });
  expect(screen.getByText("Working on Alpha")).toBeTruthy();
  await act(async () => { fireEvent.click(screen.getByText("Beta")); });
  await act(async () => { end(); });
  expect(screen.getByTestId("conv-bbbbbbbbbbbb").className).toMatch(/active/);   // not yanked back
});

it("Back to robot and Stop ROS say why they are disabled during a run", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([]);
  vi.spyOn(api, "createConversation").mockResolvedValue(conv("aaaaaaaaaaaa", "New conversation"));
  vi.spyOn(api, "runConversation").mockImplementation(() => new Promise(() => {}));
  await ready();
  fireEvent.change(screen.getByRole("textbox", { name: /describe a task/i }), { target: { value: "go" } });
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /send/i })); });
  const back = screen.getByRole("button", { name: /back to robot/i });
  expect(back).toHaveProperty("disabled", true);
  expect(back.getAttribute("title")).toMatch(/task is running/i);
});
```

Update the existing tests:
- Mocks that read the conversation id from the first argument change from `(id) =>` to `(_r, id) =>`.
- `expect(rename).toHaveBeenCalledWith("aaaaaaaaaaaa", "Square path")` becomes `("robot", "aaaaaaaaaaaa", "Square path")`.
- In `program.test.ts`, add a URL check: `listConversations("bot")` fetches `/api/program/conversations?robot=bot`.

- [ ] **Step 2: Run to verify they fail**

Run: `cd frontend && npx vitest run src/ui/program src/api/program.test.ts`
Expected: FAIL

- [ ] **Step 3: API.** In `program.ts`:

```ts
const q = (robot: string) => `?robot=${encodeURIComponent(robot)}`;
export async function listConversations(robot: string): Promise<ConversationInfo[]> {
  return (await send(`/program/conversations${q(robot)}`, { method: "GET" })).json();
}
export async function createConversation(robot: string, title?: string): Promise<Conversation> {
  return (await send(`/program/conversations${q(robot)}`, json(title ? { title } : {}))).json();
}
export async function getConversation(robot: string, id: string): Promise<Conversation> {
  return (await send(`/program/conversations/${id}${q(robot)}`, { method: "GET" })).json();
}
export async function renameConversation(robot: string, id: string, title: string): Promise<Conversation> {
  return (await send(`/program/conversations/${id}${q(robot)}`, patch({ title }))).json();
}
export async function deleteConversation(robot: string, id: string): Promise<void> {
  await send(`/program/conversations/${id}${q(robot)}`, { method: "DELETE" });
}
```

- [ ] **Step 4: `ProgramMode.tsx` changes**
  1. `const robot = status.robot ?? ctx.robotName;` and pass `robot` as the first argument to every conversation API call.
  2. Replace `const [running, setRunning] = useState(false)` with `const [runningId, setRunningId] = useState<string | null>(null); const running = runningId !== null;`. In `send`, set `setRunningId(id)` and clear it with `setRunningId(null)`. In `runAgain`, use `setRunningId(id)` / `setRunningId(null)`. Change Workspace's `onRunning={(r) => { if (mounted.current) setRunningId(r ? active?.id ?? null : null); }}`.
  3. Change the reload effect from `useEffect(() => { if (known) void reload(); }, [known])` to `useEffect(() => { void reload(); }, [robot])`, and delete `known`.
  4. At the end of `send`, replace `await reload(id)` with `await reload()`. That keeps whatever is open; if it's the finished chat, it reloads with the saved answer. `setLive(null)` stays after it.
  5. Live stream only in its own chat: `live={active?.id === runningId ? live : null}` and `running={active?.id === runningId}` on `<Chat>`.
  6. Waiting reason:

```tsx
const runningTitle = convs.find((c) => c.id === runningId)?.title ?? "another chat";
const waiting = runningId && runningId !== active?.id ? `Waiting for task in “${runningTitle}”…`
  : status.state === "starting" ? `Waiting for ROS… (${stageText(status)})`
  : status.state !== "ready" ? "Start ROS to send a task." : undefined;
```

   Pass `disabled={!!waiting}` and `waiting={waiting}` to `<Chat>`.
  7. `<ConversationList ... disabled={false} runningId={runningId} />`. Since `disabled` is now always false, remove the prop and its uses in `ConversationList`.
  8. Tooltips: `title={running ? "A task is running. Wait for it to finish first." : undefined}` on Back to robot, Stop ROS and Restart session. Same on the Preview Run button. Its `disabled` becomes `running || status.state !== "ready"`, and its title says `"Waiting for ROS…"` when ROS isn't ready.
  9. `const [live, setLive]` stays. It belongs to the running chat because `send` only ever writes it for `id`.

- [ ] **Step 5: `ConversationList.tsx`.** Add a `runningId: string | null` prop and remove `disabled`. On the running item, render `<span className="running-dot" aria-label="Task running" />` after the title button and do not render its Delete button (`{c.id !== runningId && <button ... aria-label={`Delete ${c.title}`} ...>}`). Title buttons stay disabled only for `c.broken`.

- [ ] **Step 6: `Chat.tsx`.** Add a `waiting?: string` prop and render `{waiting && <p className="muted chat-waiting">{waiting}</p>}` right above the textarea. The existing `disabled` keeps the textarea and Send disabled.

CSS:

```css
.running-dot { width: 8px; height: 8px; border-radius: 50%; flex: none; background: var(--warning); animation: pulse 1s ease-in-out infinite alternate; }
@keyframes pulse { to { opacity: .3; } }
.chat-waiting { margin: 0 0 4px; font-size: 12px; }
```

- [ ] **Step 7: Run all frontend tests + type check**

Run: `cd frontend && npx vitest run && npx tsc --noEmit`
Expected: PASS. Fix existing tests that relied on the old locks. Example: "disables Back to robot while a task runs" still holds. "locks the chat and Run while a manual run is pending" still holds for the same conversation.

- [ ] **Step 8: Commit**

```bash
git add frontend/src
git commit -m "feat: Program mode stays usable while ROS starts and while a task runs"
```

---

### Task 8: Real-app check

**Files:** none (verification only; fix anything found in a new commit).

- [ ] **Step 1: Full test suites**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q` and `cd frontend && npx vitest run && npx tsc --noEmit`
Expected: all PASS.

- [ ] **Step 2: Import check.** Start the backend and `npm run dev`, then import the irb1300 STEP. The strip should show "Reading file… 0:0x", then "Building parts N / M" counting up, then disappear when the parts appear.

- [ ] **Step 3: ROS start check (≈ 1–3 min, run in the background, tell the user).** In Program mode, press Start ROS:
   - Start returns at once.
   - The pill walks Checking collisions N / 10,000 → Building workspace → Starting MoveIt → Starting bridge → ROS ready.
   - The sidebar works throughout.
   - Creating a chat during startup is instant.

- [ ] **Step 4: Run check.** Send a task. While it runs:
   - switch to another chat, create one, then switch back: the live stream is there;
   - when the run ends, the open chat stays open.

- [ ] **Step 5: Re-run the Task 1 timing script** against the new frontend behaviour. The status poll now runs one at a time, so only the conversation timings matter. Note before/after in the spec's Findings section and commit:

```bash
git add docs/superpowers/specs/2026-10-14-responsive-ui-design.md
git commit -m "docs: responsive UI before/after timings"
```
