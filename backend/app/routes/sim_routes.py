from fastapi import APIRouter, HTTPException

from app.export.urdf_writer import suggest_efforts as _suggest
from app.routes.export_routes import ExportRequest
from app.sim import check
from app.sim.child import ChildFailed, run_in_child

router = APIRouter()


@router.post("/suggest-efforts")
def suggest_efforts(req: ExportRequest) -> dict:
    """Joint id -> suggested motor strength. Runs in a child process: MuJoCo's compile holds the GIL."""
    try:
        return run_in_child(_suggest, req.robot_name, req.links, req.joints, req.bodies_by_link)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    except ChildFailed:
        raise HTTPException(503, "Could not compute motor strengths.")


@router.post("/simulate/check")
def simulate_check(req: ExportRequest) -> dict:
    """Hold home pose for 2 s in MuJoCo; problems come back as results, never as errors."""
    try:
        return run_in_child(check.physics_check, req.robot_name, req.links, req.joints, req.bodies_by_link, req.gripper)
    except ChildFailed as exc:
        why = "took longer than 60 s" if str(exc) == "timed out" else "crashed"
        return {"ok": False, "kind": "crash", "message": f"The physics check {why}."}
