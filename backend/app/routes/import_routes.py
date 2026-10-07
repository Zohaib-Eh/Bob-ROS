import os
import tempfile

from fastapi import APIRouter, HTTPException, UploadFile

from app.importers.mesh_importer import import_mesh
from app.importers.step_importer import import_step
from app.segment.primitive_fit import fit_face
from app.segment.region_growing import grow_regions

router = APIRouter()

_STEP_EXTENSIONS = {".step", ".stp", ".iges", ".igs", ".brep"}
_MESH_EXTENSIONS = {".stl", ".obj"}


@router.post("/import")
async def import_file(file: UploadFile, scale: float = 0.001):
    suffix = os.path.splitext(file.filename or "")[1].lower()
    if suffix not in _STEP_EXTENSIONS | _MESH_EXTENSIONS:
        raise HTTPException(400, f"Unsupported file type: {suffix}")
    content = await file.read()
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        tmp_path = tmp.name
    try:
        with open(tmp_path, "wb") as f:
            f.write(content)
        if suffix in _STEP_EXTENSIONS:
            bodies = import_step(tmp_path)
            applied_scale = 1.0  # STEP carries its own units; no mm assumption
        elif suffix in _MESH_EXTENSIONS:
            body, applied_scale = import_mesh(tmp_path, scale=scale)
            regions = grow_regions(body.vertices, body.triangles)
            body.faces = [fit_face(body.vertices, body.triangles, r) for r in regions]
            bodies = [body]
    except HTTPException:
        raise
    except ValueError as exc:
        raise HTTPException(422, f"{file.filename}: {exc}")
    except Exception as exc:  # corrupt uploads can fail anywhere in OCCT/trimesh;
        # library messages may carry the temp path, so name only the error type
        raise HTTPException(422, f"{file.filename}: could not import file ({type(exc).__name__})")
    finally:
        os.unlink(tmp_path)

    return {"bodies": [b.model_dump() for b in bodies], "applied_scale": applied_scale}
