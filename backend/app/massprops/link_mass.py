import numpy as np
import trimesh
from app.models import Body

_MIN_VOLUME_M3 = 1e-9  # below this, treat geometry as degenerate


def compute_link_mass_properties(
    bodies: list[Body], density_kg_m3: float
) -> tuple[float, tuple[float, float, float], list[float]]:
    """Combine every Body belonging to a Link into one mesh and compute its
    mass properties via signed-tetrahedra volume integration (trimesh's
    built-in implementation), rather than hand-rolling the integral."""
    combined: trimesh.Trimesh | None = None
    for body in bodies:
        mesh = trimesh.Trimesh(
            vertices=np.array(body.vertices),
            faces=np.array(body.triangles),
            process=False,
        )
        combined = mesh if combined is None else combined + mesh

    if combined is None or combined.volume <= _MIN_VOLUME_M3:
        raise ValueError(
            "Link has zero, negative, or negligible volume -- check that its "
            "member bodies form a closed solid with outward-wound faces and "
            "were not left empty."
        )

    combined.density = density_kg_m3
    mass = float(combined.mass)
    com = tuple(combined.center_mass.tolist())
    inertia_tensor = combined.moment_inertia  # 3x3, about center of mass
    inertia_vector = [
        float(inertia_tensor[0, 0]), float(inertia_tensor[1, 1]),
        float(inertia_tensor[2, 2]), float(inertia_tensor[0, 1]),
        float(inertia_tensor[0, 2]), float(inertia_tensor[1, 2]),
    ]
    return mass, com, inertia_vector
