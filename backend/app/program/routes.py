import json
import threading
from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from starlette.concurrency import iterate_in_threadpool

from app.program import agent, conversations, files, session
from app.routes.export_routes import ExportRequest

router = APIRouter(prefix="/program")


class RunRequest(BaseModel):
    task: str
    resume: str | None = None


class RerunRequest(BaseModel):
    program: str


@router.post("/session")
def start_session(req: ExportRequest) -> dict:
    try:
        return session.start(req.robot_name, req.links, req.joints, req.bodies_by_link)
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    except ValueError as exc:
        raise HTTPException(422, str(exc))


@router.get("/session")
def get_session() -> dict:
    try:
        return session.status()
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))


@router.delete("/session")
def stop_session() -> dict:
    try:
        session.stop()
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    return {"state": "stopped"}


@router.post("/run")
def run(req: RunRequest) -> StreamingResponse:
    try:
        st = session.status()
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    cur = session.current()
    if not cur or st["state"] != "ready":
        detail = "Start the ROS session first and wait until it is ready."
        if st.get("log"):
            detail += "\n" + st["log"]
        raise HTTPException(409, detail)
    cancel = threading.Event()
    events = agent.run_agent(cur["robot"], cur["src"], req.task, req.resume, cancel=cancel)

    async def stream():
        try:
            async for e in iterate_in_threadpool(events):
                yield f"data: {json.dumps(e)}\n\n"
        finally:
            cancel.set()  # client gone or stream done: stop the agent's child process

    return StreamingResponse(stream(), media_type="text/event-stream")


@router.post("/rerun")
def rerun(req: RerunRequest) -> dict:
    try:
        return session.run_program(req.program)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))


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
    src = _src()
    try:
        conversations.delete(src, cid)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except KeyError:
        raise HTTPException(404, "No such conversation.")
    return {"deleted": cid}


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
