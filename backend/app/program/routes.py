import json
import threading
from pathlib import Path

from fastapi import APIRouter, Form, HTTPException, UploadFile
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from starlette.concurrency import iterate_in_threadpool

from app.program import agent, conversations, files, session, traces, workcell
from app.ros.moveit_config import ROS_NAME
from app.routes.export_routes import ExportRequest

router = APIRouter(prefix="/program")


@router.post("/session")
def start_session(req: ExportRequest) -> dict:
    try:
        return session.start(req.robot_name, req.links, req.joints, req.bodies_by_link, req.gripper)
    except session.SessionBusy as exc:
        raise HTTPException(409, str(exc))
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


_busy = threading.Lock()  # one agent/program run at a time
# ponytail: a client that drops before the stream is first iterated leaves _busy held until restart; add a
# timestamp + stale takeover if that ever shows up


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
    if not _busy.acquire(blocking=False):
        raise HTTPException(409, "A task is already running.")
    try:
        robot = session.current()["robot"]
        notes = [f"The user edited {p.removeprefix(robot + '_tasks/')} since your last turn." for p in conv["edited"]]
        prompt = ("\n".join(notes) + "\n\n" + req.task) if notes else req.task
        conv["messages"].append({"role": "user", "text": req.task})
        if conv["title"] == "New conversation":
            conv["title"] = conversations.title_from(req.task)
        conv["edited"] = []
        conversations.save(src, conv)
        cancel = threading.Event()
        events = agent.run_agent(robot, src, prompt, conv["claude_session_id"], cancel=cancel)
    except BaseException:
        _busy.release()
        raise
    collected: list[dict] = []

    async def stream():
        try:
            async for e in iterate_in_threadpool(events):
                collected.append(e)
                yield f"data: {json.dumps(e)}\n\n"
        finally:
            try:
                cancel.set()  # client gone or stream done: stop the agent's child process
                try:
                    latest = conversations.load(src, cid)
                except KeyError:
                    latest = None  # conversation deleted mid-run: nothing to save into
                if latest is not None and collected:
                    latest["messages"].append(conversations.fold(collected))
                    done = next((e for e in reversed(collected) if e.get("kind") == "done"), None)
                    if done and done.get("session_id"):
                        latest["claude_session_id"] = done["session_id"]
                    conversations.save(src, latest)
            finally:
                _busy.release()

    return StreamingResponse(stream(), media_type="text/event-stream")


class NewConversation(BaseModel):
    title: str | None = None


class Rename(BaseModel):
    title: str


def _src(robot: str | None = None) -> Path:
    """The robot's workspace src: from the name when given (no Docker), else from the session."""
    if robot is not None:
        if not ROS_NAME.fullmatch(robot):
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
    try:
        return src, conversations.load(src, cid)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except KeyError:
        raise HTTPException(404, "No such conversation.")


@router.get("/conversations")
def list_conversations(robot: str | None = None) -> list:
    return conversations.list_all(_src(robot))


@router.post("/conversations")
def new_conversation(req: NewConversation, robot: str | None = None) -> dict:
    return conversations.create(_src(robot), req.title or "New conversation")


@router.get("/conversations/{cid}")
def get_conversation(cid: str, robot: str | None = None) -> dict:
    return _conv(cid, robot)[1]


@router.patch("/conversations/{cid}")
def rename_conversation(cid: str, req: Rename, robot: str | None = None) -> dict:
    src, _ = _conv(cid, robot)
    return conversations.rename(src, cid, req.title)


@router.delete("/conversations/{cid}")
def delete_conversation(cid: str, robot: str | None = None) -> dict:
    src = _src(robot)
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


def _ws() -> tuple[Path, str]:
    try:
        cur = session.current()
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    if not cur:
        raise HTTPException(409, "Start the ROS session first.")
    return Path(cur["src"]), cur["robot"]


@router.get("/files")
def list_files() -> list:
    src, robot = _ws()
    return files.tree(src, robot)


@router.get("/files/content")
def read_file(path: str) -> dict:
    src, robot = _ws()
    try:
        return {"path": path, "content": files.read(src, path), "editable": files.editable(robot, path)}
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except FileNotFoundError:
        raise HTTPException(404, "No such file.")


@router.delete("/files/content")
def delete_file(path: str) -> dict:
    src, robot = _ws()
    if not _busy.acquire(blocking=False):
        raise HTTPException(409, "A task is already running.")
    try:
        files.delete(src, robot, path)
    except (ValueError, PermissionError) as exc:
        raise HTTPException(400, str(exc))
    except FileNotFoundError:
        raise HTTPException(404, "No such file.")
    finally:
        _busy.release()
    return {"deleted": path}


@router.put("/files/content")
def save_file(req: SaveFile) -> dict:
    src, robot = _ws()
    if req.conversation:
        _conv(req.conversation)  # resolve first: a bad or unknown conversation writes nothing
    try:
        files.write(src, robot, req.path, req.content)
    except (ValueError, PermissionError) as exc:
        raise HTTPException(400, str(exc))
    if req.conversation:
        _, conv = _conv(req.conversation)  # re-load so a concurrent edit isn't overwritten
        if req.path not in conv["edited"]:
            conv["edited"].append(req.path)
        conversations.save(src, conv)
    return {"saved": req.path}


@router.post("/files/run")
def run_file(req: RunFile) -> dict:
    src, robot = _ws()
    if req.conversation:
        _conv(req.conversation)  # resolve first: a bad or unknown conversation runs nothing
    try:
        files.safe(src, req.path)
        if not files.editable(robot, req.path):
            raise ValueError(f"Only {robot}_tasks/programs/*.py can be run.")
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    if not files.safe(src, req.path).is_file():
        raise HTTPException(404, "No such file.")
    if not _busy.acquire(blocking=False):
        raise HTTPException(409, "A task is already running.")
    try:
        result = session.run_program(req.path.split("/", 1)[1])
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
    finally:
        _busy.release()
    if req.conversation:
        _, conv = _conv(req.conversation)  # re-load so a concurrent edit isn't overwritten
        conv["messages"].append({"role": "run", "path": req.path, **result})
        conversations.save(src, conv)
    return result


class WorkcellBody(BaseModel):
    robot: str
    objects: list


@router.get("/traces/{name}")
def get_trace(name: str, robot: str) -> dict:
    """A run's recording for replay (see templates/recorder.py)."""
    try:
        return traces.read(robot, name)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except FileNotFoundError:
        raise HTTPException(404, "Replay not available: this run has no recording.")


@router.get("/workcell")
def get_workcell(robot: str) -> dict:
    try:
        return workcell.read(robot)
    except workcell.BadRobot as exc:
        raise HTTPException(422, str(exc))


@router.post("/workcell/fixture")
def import_fixture(file: UploadFile, robot: str = Form(...), scale: float = Form(0.001)) -> dict:
    try:
        return workcell.import_fixture(robot, file.filename or "fixture.stl", file.file.read(), scale)
    except ValueError as exc:
        raise HTTPException(422, str(exc))


@router.get("/workcell/mesh")
def get_fixture_mesh(robot: str, file: str) -> dict:
    try:
        return workcell.fixture_mesh(robot, file)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    except FileNotFoundError:
        raise HTTPException(404, f"No fixture mesh '{file}'.")


@router.put("/workcell")
def put_workcell(req: WorkcellBody) -> dict:
    try:
        return {"objects": workcell.write(req.robot, req.objects)}
    except ValueError as exc:   # BadRobot, or SceneError with the reason in plain words
        raise HTTPException(422, str(exc))
