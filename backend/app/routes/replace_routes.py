import math

from fastapi import APIRouter, Form, HTTPException, UploadFile

from app.importers.placement import combine_bodies, place_replacement
from app.routes.upload import run_import

router = APIRouter()


def _vec(s: str | None, field: str):
    if s is None:
        return None
    try:
        v = tuple(float(x) for x in s.split(","))
    except ValueError:
        raise HTTPException(422, f"{field} must be 'x,y,z' numbers")
    if len(v) != 3 or not all(math.isfinite(x) for x in v):
        raise HTTPException(422, f"{field} must be three finite numbers")
    return v


@router.post("/replace-part")
def replace_part(
    file: UploadFile,
    body_id: str = Form(...),
    name: str = Form("replacement"),
    bbox_min: str | None = Form(None),
    bbox_max: str | None = Form(None),
    scale: float = Form(0.001),
):
    lo, hi = _vec(bbox_min, "bbox_min"), _vec(bbox_max, "bbox_max")
    if (lo is None) != (hi is None):
        raise HTTPException(422, "bbox_min and bbox_max must be given together")
    if lo is not None and any(a > b for a, b in zip(lo, hi)):
        raise HTTPException(422, "bbox_min must not exceed bbox_max")
    bodies, _, file_warnings = run_import(file, scale)
    combined = combine_bodies(bodies, body_id, name)
    combined.warnings += file_warnings
    body = place_replacement(combined, lo, hi)
    return body.model_dump()
