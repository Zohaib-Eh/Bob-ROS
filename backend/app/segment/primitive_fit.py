import numpy as np
from app.models import Face

_PLANE_RESIDUAL_TOLERANCE = 1e-4   # meters, max deviation from fitted plane
_CYLINDER_RESIDUAL_TOLERANCE = 0.02  # relative radius deviation


def _region_points_and_normals(vertices, triangles, region):
    points = []
    normals = []
    verts_arr = np.array(vertices)
    for tri_idx in region:
        a, b, c = triangles[tri_idx]
        pa, pb, pc = verts_arr[a], verts_arr[b], verts_arr[c]
        points.extend([pa, pb, pc])
        normal = np.cross(pb - pa, pc - pa)
        norm = np.linalg.norm(normal)
        if norm > 1e-12:
            normals.append(normal / norm)
    return np.array(points), np.array(normals)


def _try_fit_plane(points: np.ndarray, normals: np.ndarray):
    centroid = points.mean(axis=0)
    centered = points - centroid
    cov = centered.T @ centered
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = eigvecs[:, 0]  # smallest eigenvalue direction
    residual = float(np.sqrt(max(eigvals[0], 0.0) / max(len(points), 1)))
    if residual > _PLANE_RESIDUAL_TOLERANCE:
        return None
    return normal, tuple(centroid), residual


def _try_fit_cylinder(points: np.ndarray, normals: np.ndarray):
    if len(normals) < 3:
        return None
    normal_centroid = normals.mean(axis=0)
    centered_normals = normals - normal_centroid
    cov = centered_normals.T @ centered_normals
    eigvals, eigvecs = np.linalg.eigh(cov)
    axis_dir = eigvecs[:, 0]  # smallest-variance direction = cylinder axis
    axis_dir = axis_dir / np.linalg.norm(axis_dir)

    centroid = points.mean(axis=0)
    # project points onto plane perpendicular to axis, fit a circle there
    arbitrary = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(arbitrary, axis_dir)) > 0.9:
        arbitrary = np.array([0.0, 1.0, 0.0])
    u = np.cross(axis_dir, arbitrary)
    u /= np.linalg.norm(u)
    v = np.cross(axis_dir, u)

    rel = points - centroid
    xs = rel @ u
    ys = rel @ v
    # algebraic circle fit: x^2+y^2 + D*x + E*y + F = 0
    A = np.column_stack([xs, ys, np.ones_like(xs)])
    b = -(xs**2 + ys**2)
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    d, e, f = sol
    cx, cy = -d / 2, -e / 2
    radius_sq = cx**2 + cy**2 - f
    if radius_sq <= 0:
        return None
    radius = float(np.sqrt(radius_sq))

    distances = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2)
    residual = float(np.std(distances) / radius) if radius > 0 else 1.0
    if residual > _CYLINDER_RESIDUAL_TOLERANCE:
        return None

    center_3d = centroid + cx * u + cy * v
    return axis_dir, radius, tuple(center_3d), residual


def fit_face(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    region: list[int],
) -> Face:
    points, normals = _region_points_and_normals(vertices, triangles, region)
    if len(points) == 0:
        return Face(
            kind="unclassified", origin="segmented", center=(0.0, 0.0, 0.0),
            residual=1.0, triangle_indices=region,
        )

    plane_fit = _try_fit_plane(points, normals)
    if plane_fit is not None:
        normal, center, residual = plane_fit
        return Face(
            kind="plane", origin="segmented",
            normal=(float(normal[0]), float(normal[1]), float(normal[2])),
            axis=None, center=center, radius=None, residual=residual,
            triangle_indices=region,
        )

    cylinder_fit = _try_fit_cylinder(points, normals)
    if cylinder_fit is not None:
        axis_dir, radius, center, residual = cylinder_fit
        return Face(
            kind="cylinder", origin="segmented", normal=None,
            axis=(float(axis_dir[0]), float(axis_dir[1]), float(axis_dir[2])),
            center=center, radius=radius, residual=residual,
            triangle_indices=region,
        )

    return Face(
        kind="unclassified", origin="segmented",
        center=tuple(points.mean(axis=0).tolist()), residual=1.0,
        triangle_indices=region,
    )
