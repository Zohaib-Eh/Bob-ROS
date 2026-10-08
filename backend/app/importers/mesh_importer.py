import math
import os
import uuid
import trimesh

from app.models import Body, PartWarning


def import_mesh(file_path: str, scale: float = 0.001, display_name: str | None = None) -> tuple[Body, float]:
    """Load an STL/OBJ/GLTF file. STL/OBJ carry no unit metadata, so the
    hobbyist convention (millimeters) is assumed by default; callers must
    surface `scale` to the user so a wrong assumption is visible, per the
    spec's documented mm/m pitfall."""
    if not (math.isfinite(scale) and scale > 0):
        raise ValueError(f"scale must be positive, got {scale}")

    try:
        mesh = trimesh.load(file_path, force="mesh")
    except Exception as exc:
        raise ValueError(f"could not read mesh ({type(exc).__name__})") from exc
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise ValueError("could not read mesh (no triangles found)")

    name = display_name or os.path.splitext(os.path.basename(file_path))[0] or "mesh"
    status, warnings = "ok", []
    if not mesh.is_watertight:
        n_open = len(trimesh.grouping.group_rows(mesh.edges_sorted, require_count=1))
        status = "approximate"
        warnings.append(PartWarning(
            code="open_geometry", fixes=["set_mass", "replace", "ignore"],
            message=f"{name}: mesh is not closed "
                    f"({f'{n_open} open edges' if n_open else 'non-manifold edges'}); mass is approximate"))
    elif mesh.volume < 0:  # inward-wound file: flip so normals/volume are outward
        mesh.invert()
        warnings.append(PartWarning(
            code="inverted_mesh", fixes=[],
            message=f"{name}: mesh was inside-out and has been flipped"))

    mesh.apply_scale(scale)

    body = Body(
        id=str(uuid.uuid4()),
        name=name,
        vertices=[tuple(v) for v in mesh.vertices.tolist()],
        triangles=[tuple(t) for t in mesh.faces.tolist()],
        faces=[],
        status=status,
        warnings=warnings,
    )
    return body, scale
