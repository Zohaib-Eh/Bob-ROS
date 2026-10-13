import json

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from app.program import agent, session
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
    session.stop()
    return {"state": "stopped"}


@router.post("/run")
def run(req: RunRequest) -> StreamingResponse:
    cur = session.current()
    if not cur or session.status()["state"] != "ready":
        raise HTTPException(409, "Start the ROS session first and wait until it is ready.")
    events = agent.run_agent(cur["robot"], cur["src"], req.task, req.resume)
    return StreamingResponse((f"data: {json.dumps(e)}\n\n" for e in events), media_type="text/event-stream")


@router.post("/rerun")
def rerun(req: RerunRequest) -> dict:
    try:
        return session.run_program(req.program)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except session.SessionError as exc:
        raise HTTPException(503, str(exc))
