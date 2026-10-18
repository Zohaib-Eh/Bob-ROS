"""Gripper geometry for export: finger axes that close inward, the tool point between the fingertips,
and convex collision pieces for the gripper's links (MuJoCo collides a mesh as its convex hull, which
would fill an L-shaped finger's hook)."""
import functools

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


def jaw_gap(leader_vertices, follower_vertices, closing) -> float:
    """Narrowest gap between the fingers along the leader's (inward) closing direction, as modelled."""
    c = _unit(np.asarray(closing, float))
    return float((np.asarray(follower_vertices, float) @ c).min() - (np.asarray(leader_vertices, float) @ c).max())


def convex_pieces(vertices, triangles, max_pieces: int = 8) -> list[tuple[list, list]]:
    """The mesh split into convex parts (CoACD), each as (vertices, triangles). Cached per mesh:
    Simulate and Program export the same fingers again and again."""
    v, f = np.ascontiguousarray(vertices, float), np.ascontiguousarray(triangles, np.int64)
    return _pieces(v.tobytes(), f.tobytes(), max_pieces)


@functools.lru_cache(maxsize=32)   # ponytail: per process; a disk cache if restarts make exports slow
def _pieces(vertex_bytes: bytes, triangle_bytes: bytes, max_pieces: int):
    import coacd
    coacd.set_log_level("error")
    import trimesh
    vertices = np.frombuffer(vertex_bytes, float).reshape(-1, 3)
    triangles = np.frombuffer(triangle_bytes, np.int64).reshape(-1, 3)
    # Light settings: ~5 s for a real CAD jaw (AirTAC HFK40, tapped holes and chamfers; ~8-18 s at
    # resolution 400 / 20 iterations, ~33 s for a simple finger with the defaults), pieces within ~6%
    # of the solid's volume. CoACD's remesh step (needed only for leaky meshes) inflates a closed
    # finger ~12%, so it runs only when the mesh isn't watertight.
    closed = trimesh.Trimesh(vertices, triangles, process=False).is_watertight
    parts = coacd.run_coacd(coacd.Mesh(vertices, triangles),
                            threshold=0.1, max_convex_hull=max_pieces, preprocess_mode="off" if closed else "on",
                            resolution=200, mcts_iterations=10, mcts_nodes=10, mcts_max_depth=2)
    return [(v.tolist(), f.tolist()) for v, f in parts]


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
