from fastapi import APIRouter, UploadFile

from app.routes.upload import run_import

router = APIRouter()


@router.post("/import")
def import_file(file: UploadFile, scale: float = 0.001):  # sync: runs in threadpool
    bodies, applied_scale, warnings = run_import(file, scale)
    return {
        "bodies": [b.model_dump() for b in bodies],
        "applied_scale": applied_scale,
        "warnings": [w.model_dump() for w in warnings],
    }
