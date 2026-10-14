import json
import math
import multiprocessing
import queue

from fastapi import APIRouter, HTTPException, UploadFile
from fastapi.responses import StreamingResponse

from app.routes.upload import import_saved, save_upload

router = APIRouter()
_spawn = multiprocessing.get_context("spawn")


def _import_worker(tmp_path: str, suffix: str, filename: str | None, scale: float, events) -> None:
    """Runs in a child process: OCCT holds the GIL for its whole file read, which would freeze the server.
    Puts progress events, then one done or error event, then None."""
    try:
        bodies, applied, warnings = import_saved(
            tmp_path, suffix, filename, scale,
            on_progress=lambda s, d, t: events.put({"kind": "progress", "stage": s, "done": d, "total": t}))
        events.put({"kind": "done", "bodies": [b.model_dump() for b in bodies], "applied_scale": applied,
                    "warnings": [w.model_dump() for w in warnings]})
    except HTTPException as exc:
        events.put({"kind": "error", "message": exc.detail})
    except Exception as exc:  # never leave the stream hanging
        events.put({"kind": "error", "message": f"{filename}: could not import file ({type(exc).__name__})"})
    finally:
        events.put(None)


def _stream(proc, events, filename: str | None):
    """Relay the child's events as SSE frames. A dropped client (generator closed) kills the child."""
    try:
        while True:
            try:
                e = events.get(timeout=1)
            except queue.Empty:
                if proc.is_alive():
                    continue
                e = {"kind": "error", "message": f"{filename}: could not import file (the importer crashed)"}
                yield f"data: {json.dumps(e)}\n\n"
                return
            if e is None:
                return
            yield f"data: {json.dumps(e)}\n\n"
    finally:
        if proc.is_alive():
            proc.kill()
        proc.join(timeout=5)


@router.post("/import")
def import_file(file: UploadFile, scale: float = 0.001) -> StreamingResponse:
    """Streams progress events, then one done or error event (same frames as the agent run)."""
    if not (math.isfinite(scale) and scale > 0):
        raise HTTPException(422, f"scale must be finite and positive, got {scale}")
    tmp_path, suffix = save_upload(file)  # 400 for unsupported types, before the stream
    events = _spawn.Queue()
    proc = _spawn.Process(target=_import_worker, args=(tmp_path, suffix, file.filename, scale, events), daemon=True)
    proc.start()

    return StreamingResponse(_stream(proc, events, file.filename), media_type="text/event-stream")
