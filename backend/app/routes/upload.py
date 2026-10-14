import os
import tempfile

from fastapi import HTTPException, UploadFile

from app.importers.pipeline import MESH_EXTENSIONS, STEP_EXTENSIONS, import_path
from app.models import Body, PartWarning


def save_upload(file: UploadFile) -> tuple[str, str]:
    suffix = os.path.splitext(file.filename or "")[1].lower()
    if suffix not in STEP_EXTENSIONS | MESH_EXTENSIONS:
        raise HTTPException(400, f"Unsupported file type: {suffix}")
    content = file.file.read()
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        tmp.write(content)
        return tmp.name, suffix


def import_saved(tmp_path: str, suffix: str, filename: str | None, scale: float,
                 on_progress=None) -> tuple[list[Body], float, list[PartWarning]]:
    """Import a saved upload, then delete it. Errors become a 422 naming the upload."""
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
