import math
import uuid
import trimesh

from app.models import Body


def import_mesh(file_path: str, scale: float = 0.001) -> tuple[Body, float]:
    """Load an STL/OBJ/GLTF file. STL/OBJ carry no unit metadata, so the
    hobbyist convention (millimeters) is assumed by default; callers must
    surface `scale` to the user so a wrong assumption is visible, per the
    spec's documented mm/m pitfall."""
    if not (math.isfinite(scale) and scale > 0):
        raise ValueError(f"scale must be positive, got {scale}")

    mesh = trimesh.load(file_path, force="mesh")

    if not mesh.is_watertight:
        raise ValueError(
            f"Mesh in {file_path} is not watertight (has holes or "
            "non-manifold edges) -- cannot reliably compute volume/inertia "
            "or segment faces from it."
        )

    mesh.apply_scale(scale)

    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]

    body = Body(
        id=str(uuid.uuid4()),
        name="imported_mesh",
        vertices=vertices,
        triangles=triangles,
        faces=[],
    )
    return body, scale
