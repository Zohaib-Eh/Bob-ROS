"""Gripper decisions for robot.py, pure (tested on the host): did the grip catch something, did the
held object slip, which way the jaws face, does an object fit between them."""
import math

import numpy as np

EMPTY_TOL = 0.002   # m: fingers this close to fully closed caught nothing
SLIP_TOL = 0.01     # m: a held object drifting this far relative to the tool point has slipped


def outcome(width: float, closed_width: float, tol: float = EMPTY_TOL) -> str:
    """'held' when the fingers stopped with a gap left, 'empty' when they (nearly) fully closed."""
    return "empty" if width - closed_width <= tol else "held"


def slipped(rel_before: np.ndarray, rel_now: np.ndarray, tol: float = SLIP_TOL) -> bool:
    """rel_*: 4x4 object-in-tool-point poses; only the position drift counts."""
    return float(np.linalg.norm(np.asarray(rel_now)[:3, 3] - np.asarray(rel_before)[:3, 3])) > tol


def _rot(axis, angle) -> np.ndarray:
    x, y, z = np.asarray(axis, float) / np.linalg.norm(axis)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def yaw_rotation(R_tool: np.ndarray, closing_axis_tcp: np.ndarray, yaw: float) -> np.ndarray:
    """R_tool turned about its own approach (z) axis so the jaws close along (cos yaw, sin yaw) in
    the base x-y plane. Exact for a vertical approach (tool_down); approximate when tilted."""
    R = np.asarray(R_tool, float)
    jaws = R @ np.asarray(closing_axis_tcp, float)
    if math.hypot(jaws[0], jaws[1]) < 1e-9:   # jaws vertical: yaw means nothing
        return R
    delta = yaw - math.atan2(jaws[1], jaws[0])
    # turning about a downward axis turns x-y directions the other way round
    return _rot(R[:, 2], delta if R[2, 2] >= 0 else -delta) @ R


def _object_R(rpy) -> np.ndarray:
    r, p, y = rpy
    return _rot((0, 0, 1), y) @ _rot((0, 1, 0), p) @ _rot((1, 0, 0), r)


def fits(obj: dict, closing_dir_base, open_width: float) -> float | None:
    """The object's extent along the jaws' closing direction when that exceeds open_width, else None.
    obj: a scene object (shape, size, rpy) as in meta.json."""
    d = np.asarray(closing_dir_base, float)
    d /= np.linalg.norm(d)
    local = _object_R(obj.get("rpy", (0, 0, 0))).T @ d
    if obj["shape"] == "box":
        extent = float(np.abs(local) @ np.asarray(obj["size"], float))
    else:   # cylinder: axis along local z, size = (radius, height)
        r, h = obj["size"]
        along = abs(local[2])
        extent = 2 * r * math.sqrt(max(0.0, 1 - along ** 2)) + h * along
    return extent if extent > open_width + 1e-9 else None
