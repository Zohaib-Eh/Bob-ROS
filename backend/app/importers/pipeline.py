import trimesh
import numpy as np

from app.importers.mesh_importer import import_mesh
from app.importers.step_importer import import_cad
from app.models import Body, PartWarning
from app.segment.primitive_fit import fit_face
from app.segment.region_growing import grow_regions

STEP_EXTENSIONS = {".step", ".stp", ".iges", ".igs", ".brep"}
MESH_EXTENSIONS = {".stl", ".obj"}


def segment_mesh_body(body: Body) -> Body:
    """Reconstruct faces for a mesh body. Never raises for bad geometry: a mesh
    with no shared edges gets no faces and a `degenerate` warning instead."""
    mesh = trimesh.Trimesh(np.asarray(body.vertices), np.asarray(body.triangles), process=False)
    if len(body.triangles) > 1 and len(mesh.face_adjacency) == 0:
        return body.model_copy(update={"faces": [], "warnings": body.warnings + [PartWarning(
            code="degenerate", fixes=["replace", "ignore"],
            message=f"{body.name}: triangles share no edges, so no faces could be found")]})
    regions = grow_regions(body.vertices, body.triangles, allow_open=body.status != "ok")
    faces = [fit_face(body.vertices, body.triangles, r) for r in regions]
    return body.model_copy(update={"faces": faces})


def import_path(path: str, suffix: str, scale: float,
                display_name: str | None = None) -> tuple[list[Body], float, list[PartWarning]]:
    if suffix in STEP_EXTENSIONS:
        if suffix == ".brep":  # unitless: assumed mm unless overridden, like STL
            bodies, warnings = import_cad(path, brep_scale=scale)
            return bodies, scale, warnings
        bodies, warnings = import_cad(path)
        return bodies, 1.0, warnings  # STEP/IGES carry their own unit
    if suffix in MESH_EXTENSIONS:
        body, applied = import_mesh(path, scale=scale, display_name=display_name)
        return [segment_mesh_body(body)], applied, []
    raise ValueError(f"Unsupported file type: {suffix}")
