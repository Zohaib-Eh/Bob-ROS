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
