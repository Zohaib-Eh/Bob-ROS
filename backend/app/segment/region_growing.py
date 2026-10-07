import math
import numpy as np
import trimesh


def grow_regions(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    angle_threshold_deg: float = 15.0,
) -> list[list[int]]:
    """Flood-fill triangles into regions, merging across an edge only when
    the dihedral angle between the two adjacent faces is below threshold.
    A fine-tessellated curved surface (e.g. a cylinder's side) has small
    adjacent-triangle angle changes and stays one region; a sharp edge
    (e.g. box corner, cylinder cap boundary) does not get merged across."""
    mesh = trimesh.Trimesh(
        vertices=np.array(vertices), faces=np.array(triangles), process=False
    )
    if not mesh.is_watertight:
        raise ValueError("mesh must be watertight to grow regions")
    threshold_rad = math.radians(angle_threshold_deg)

    adjacency = mesh.face_adjacency
    angles = mesh.face_adjacency_angles

    n_faces = len(triangles)
    adjacency_list: list[list[int]] = [[] for _ in range(n_faces)]
    for (a, b), angle in zip(adjacency, angles):
        if angle < threshold_rad:
            adjacency_list[a].append(b)
            adjacency_list[b].append(a)

    visited = [False] * n_faces
    regions: list[list[int]] = []
    for start in range(n_faces):
        if visited[start]:
            continue
        stack = [start]
        region: list[int] = []
        visited[start] = True
        while stack:
            current = stack.pop()
            region.append(current)
            for neighbor in adjacency_list[current]:
                if not visited[neighbor]:
                    visited[neighbor] = True
                    stack.append(neighbor)
        regions.append(region)

    return regions
