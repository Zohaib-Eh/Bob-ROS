"""Gripper geometry for export: finger axes that close inward, the tool point between the fingertips,
and convex collision pieces for the gripper's links (MuJoCo collides a mesh as its convex hull, which
would fill an L-shaped finger's hook)."""
import functools
import hashlib
import json
import os
from pathlib import Path

import numpy as np

from app.models import Body, Gripper, Joint

DEFAULT_GRIP_FORCE = 40.0   # N per finger when the finger joint has no max_effort
NOT_SLIDERS = "Gripper fingers must be two sliding joints of this robot."


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def inward_axes(leader_axis, follower_axis, leader_centroid, follower_centroid):
    """Each finger's axis turned to point at the other finger. Returns (leader_axis, follower_axis,
    leader_flipped, follower_flipped); a flipped joint's limits become (-upper, -lower)."""
    towards = _unit(np.subtract(follower_centroid, leader_centroid))
    la, fa = _unit(leader_axis), _unit(follower_axis)
    lflip, fflip = bool(la @ towards < 0), bool(fa @ -towards < 0)
    return tuple(-la if lflip else la), tuple(-fa if fflip else fa), lflip, fflip


def tcp_point(base_vertices, finger_a_vertices, finger_b_vertices, offset_m: float, closing=None, wrist=None):
    """Tool point: midway between the fingers, pushed out to their far end along the gripper axis
    (base centroid -> fingers' midpoint, square to the `closing` direction), plus offset_m. A `wrist`
    roll axis within 30 deg of that is used instead: when the gripper plate is merged into the arm's
    last link, that link's centroid is off the axis. Returns (xyz, approach_axis)."""
    a, b = np.asarray(finger_a_vertices, float), np.asarray(finger_b_vertices, float)
    mid = (a.mean(axis=0) + b.mean(axis=0)) / 2
    axis = mid - np.asarray(base_vertices, float).mean(axis=0)
    if closing is not None:
        c = _unit(np.asarray(closing, float))
        axis = axis - (axis @ c) * c
    axis = _unit(axis)
    if wrist is not None:
        w = _unit(np.asarray(wrist, float))
        w = w if w @ axis > 0 else -w
        if w @ axis > np.cos(np.radians(30)):
            axis = w
    reach = float(((np.vstack([a, b]) - mid) @ axis).max()) + offset_m
    return tuple(mid + axis * reach), tuple(axis)


TIP_BAND = 0.015   # m back from the fingertips: where objects are gripped


def jaw_gap(leader_mesh, follower_mesh, closing, approach, tip) -> float:
    """Gap between the jaw tips along the leader's (inward) closing direction, as modelled: the
    narrowest point within TIP_BAND of the fingertips. Not anywhere on the fingers: real jaws (e.g.
    an air gripper's) can touch on slides hidden in the gripper body while the tips are 30 mm apart.
    Meshes are (vertices, triangles); they are densified so big flat faces count, not just corners."""
    from trimesh.remesh import subdivide_to_size
    c, a, tip = _unit(np.asarray(closing, float)), _unit(np.asarray(approach, float)), np.asarray(tip, float)

    def band(mesh):
        pts = _dense(mesh)
        return pts[(pts - tip) @ a > -TIP_BAND] @ c
    return float(band(follower_mesh).min() - band(leader_mesh).max())


def _dense(mesh) -> np.ndarray:
    from trimesh.remesh import subdivide_to_size
    return subdivide_to_size(np.asarray(mesh[0], float), np.asarray(mesh[1], int), 0.002)[0]


def jaw_depth(base_mesh, leader_mesh, follower_mesh, closing, approach, tip) -> float:
    """How far an object can reach up between the jaws before it hits the gripper (m): from the tips
    to the nearest part that juts in between them, i.e. any part of a jaw inward of that jaw's own tip
    face (an air gripper's slides, 37 mm up its jaws) or of the base between the tip faces; else to the
    base. Meshes are (vertices, triangles), as modelled (q = 0)."""
    c, a, tip = _unit(np.asarray(closing, float)), _unit(np.asarray(approach, float)), np.asarray(tip, float)
    base, lead, foll = _dense(base_mesh), _dense(leader_mesh), _dense(follower_mesh)
    back = lambda pts: -(pts - tip) @ a
    face_l = lead[back(lead) < TIP_BAND] @ c
    face_f = foll[back(foll) < TIP_BAND] @ c
    lo, hi = face_l.max() + 1e-4, face_f.min() - 1e-4
    inward = [lead[lead @ c > lo], foll[foll @ c < hi], base[(base @ c > lo) & (base @ c < hi)]]
    hits = [back(p) for p in inward if len(p)]
    hits = [h[h > 1e-4] for h in hits]
    hits = [h.min() for h in hits if len(h)]
    return float(min(hits) if hits else back(base).min())


def convex_pieces(vertices, triangles, max_pieces: int = 8) -> list[tuple[list, list]]:
    """The mesh split into convex parts (CoACD), each as (vertices, triangles). Cached per mesh:
    Simulate and Program export the same fingers again and again."""
    v, f = np.ascontiguousarray(vertices, float), np.ascontiguousarray(triangles, np.int64)
    return _pieces(v.tobytes(), f.tobytes(), max_pieces)


# Pieces per finger mesh on disk: each physics check runs in a fresh child process, which an in-memory
# cache doesn't survive. The version part of the key changes whenever the decomposition does.
CACHE_DIR = Path.home() / ".cache" / "cad-to-urdf" / "convex"
_CACHE_VERSION = "2"


@functools.lru_cache(maxsize=32)
def _pieces(vertex_bytes: bytes, triangle_bytes: bytes, max_pieces: int):
    key = hashlib.sha1(vertex_bytes + triangle_bytes + f"|{max_pieces}|{_CACHE_VERSION}".encode()).hexdigest()
    path = CACHE_DIR / f"{key}.json"
    try:
        return [tuple(p) for p in json.loads(path.read_text())]
    except (OSError, ValueError):
        pass
    parts = _decompose(np.frombuffer(vertex_bytes, float).reshape(-1, 3),
                       np.frombuffer(triangle_bytes, np.int64).reshape(-1, 3), max_pieces)
    try:   # a cache that can't be written only costs time
        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(f".{os.getpid()}.tmp")
        tmp.write_text(json.dumps(parts))
        os.replace(tmp, path)
    except OSError:
        pass
    return parts


def _decompose(vertices, triangles, max_pieces):
    import coacd
    coacd.set_log_level("error")
    import trimesh
    # CoACD cuts along axis-aligned planes, so a finger turned on the robot takes it ~7x longer (AirTAC
    # HFK40 jaw: 36 s mounted, 5 s in its own frame): split in the finger's principal axes, then put back.
    centre = vertices.mean(axis=0)
    axes = np.linalg.svd(vertices - centre, full_matrices=False)[2]   # rows: principal directions
    if np.linalg.det(axes) < 0:   # a proper rotation, not a mirror
        axes[2] = -axes[2]
    vertices = (vertices - centre) @ axes.T
    # Light settings: ~5 s for a real CAD jaw (AirTAC HFK40, tapped holes and chamfers; ~8-18 s at
    # resolution 400 / 20 iterations, ~33 s for a simple finger with the defaults), pieces within ~6%
    # of the solid's volume. CoACD's remesh step (needed only for leaky meshes) inflates a closed
    # finger ~12%, so it runs only when the mesh isn't watertight.
    closed = trimesh.Trimesh(vertices, triangles, process=False).is_watertight
    parts = coacd.run_coacd(coacd.Mesh(vertices, triangles),
                            threshold=0.1, max_convex_hull=max_pieces, preprocess_mode="off" if closed else "on",
                            resolution=200, mcts_iterations=10, mcts_nodes=10, mcts_max_depth=2)
    return [((np.asarray(v) @ axes + centre).tolist(), f.tolist()) for v, f in parts]


def checked(gripper: Gripper, joints: list[Joint]) -> tuple[Joint, Joint]:
    by_id = {j.id: j for j in joints}
    pair = [by_id.get(gripper.leader_joint_id), by_id.get(gripper.follower_joint_id)]
    if any(j is None or j.type != "prismatic" for j in pair) or pair[0].id == pair[1].id:
        raise ValueError(NOT_SLIDERS)
    if pair[0].parent_link_id != pair[1].parent_link_id:
        raise ValueError(NOT_SLIDERS)
    return pair[0], pair[1]


def _centroid(bodies: list[Body]) -> np.ndarray:
    return np.vstack([np.asarray(b.vertices, float) for b in bodies if b.vertices]).mean(axis=0)


def inward_joints(gripper: Gripper, joints: list[Joint], bodies_by_link: dict[str, list[Body]]) -> list[Joint]:
    """The robot's joints with both fingers' axes pointing inward (positive = closing)."""
    leader, follower = checked(gripper, joints)
    la, fa, lflip, fflip = inward_axes(leader.axis, follower.axis, _centroid(bodies_by_link[leader.child_link_id]),
                                       _centroid(bodies_by_link[follower.child_link_id]))

    def turned(j: Joint, axis, flip: bool) -> Joint:
        if not flip:
            return j.model_copy(update={"axis": axis})
        return j.model_copy(update={"axis": axis, "limit_lower": -j.limit_upper, "limit_upper": -j.limit_lower})
    return [turned(j, la, lflip) if j.id == leader.id else turned(j, fa, fflip) if j.id == follower.id else j
            for j in joints]
