"""drive_to's arithmetic, without ROS (robot.py drives with it; tests run it on a simulated base).

A differential drive turns in place about the middle of its axles (`center`, in the base frame), not
about the base frame's origin: drive_to steers that point to where it must be for the base to end at the
goal, then turns about it. Poses are (x, y, yaw) of the base frame in the world."""
import math

import numpy as np

POS_TOL = 0.02              # m: drive_to ends this close
YAW_TOL = math.radians(2)   # rad
MARGIN = 0.05               # m kept clear of objects around the base's outline
TURN_GAIN, STEER_GAIN, SPEED_GAIN = 2.5, 3.0, 1.5
MAX_TURN = 1.0              # rad/s


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def pivot(pose, center) -> np.ndarray:
    """Where the turning point is in the world for the base at pose."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x + c * center[0] - s * center[1], y + s * center[0] + c * center[1]])


def plan(start, goal_xy, goal_yaw, center):
    """(pivot target, final yaw): with no goal yaw the base ends facing the way it drove."""
    p0 = pivot(start, center)
    if goal_yaw is None:
        travel = np.asarray(goal_xy, float) - p0
        goal_yaw = math.atan2(travel[1], travel[0]) if np.linalg.norm(travel) > POS_TOL else start[2]
    return pivot((*goal_xy, goal_yaw), center), goal_yaw


def command(phase: str, pose, target, final_yaw: float, center, speed: float):
    """(v, w, next phase) for one control tick. Phases: 'face' (turn toward the target), 'go' (drive to it,
    steering), 'turn' (to the final yaw), 'done'."""
    p = pivot(pose, center)
    d = np.asarray(target, float) - p
    dist = float(np.linalg.norm(d))
    yaw = pose[2]
    if phase == "face":
        if dist < POS_TOL:
            return 0.0, 0.0, "turn"
        err = wrap(math.atan2(d[1], d[0]) - yaw)
        if abs(err) < YAW_TOL * 2:
            return 0.0, 0.0, "go"
        return 0.0, float(np.clip(TURN_GAIN * err, -MAX_TURN, MAX_TURN)), "face"
    if phase == "go":
        if dist < POS_TOL:
            return 0.0, 0.0, "turn"
        along = float(d @ [math.cos(yaw), math.sin(yaw)])
        err = wrap(math.atan2(d[1], d[0]) - yaw) if along > 0 else 0.0
        w = float(np.clip(STEER_GAIN * err, -MAX_TURN, MAX_TURN)) if dist > 0.05 else 0.0
        return float(np.clip(SPEED_GAIN * along, -speed, speed)), w, "go"
    if phase == "turn":
        err = wrap(final_yaw - yaw)
        if abs(err) < YAW_TOL:
            return 0.0, 0.0, "done"
        return 0.0, float(np.clip(TURN_GAIN * err, -MAX_TURN, MAX_TURN)), "turn"
    return 0.0, 0.0, "done"


def reach(footprint, center) -> float:
    """Radius of the circle the base sweeps turning in place about `center`, plus the margin."""
    x0, y0, x1, y1 = footprint
    return max(math.hypot(x - center[0], y - center[1]) for x in (x0, x1) for y in (y0, y1)) + MARGIN


def _rect_distance(p, obj) -> float:
    """Plan distance from point p to an object's footprint (centre, half sizes, yaw)."""
    (cx, cy), (hx, hy), yaw = obj["xy"], obj["half"], obj["yaw"]
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = p[0] - cx, p[1] - cy
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return math.hypot(max(abs(lx) - hx, 0.0), max(abs(ly) - hy, 0.0))


def blocker(start, target, radius: float, objects, height: float) -> str | None:
    """The first object in the way of the turning point's straight path (swept by the base's turning
    circle), or None. objects: [{name, xy, half, yaw, z0}] with z0 its bottom above the floor; objects the
    base already overlaps (resting on it, or touching it) and ones above the base are not in the way."""
    start, target = np.asarray(start, float), np.asarray(target, float)
    n = max(2, int(np.linalg.norm(target - start) / 0.02) + 1)
    pts = [start + (target - start) * k / (n - 1) for k in range(n)]
    for o in objects:
        if o["z0"] >= height or _rect_distance(start, o) < radius:
            continue
        if any(_rect_distance(p, o) < radius for p in pts):
            return o["name"]
    return None


def remaining(phase: str, pose, target, final_yaw: float, center) -> float:
    """What is left of the current phase: metres while driving, radians while turning."""
    d = np.asarray(target, float) - pivot(pose, center)
    if phase == "go":
        return float(np.linalg.norm(d))
    goal = final_yaw if phase == "turn" else math.atan2(d[1], d[0])
    return abs(wrap(goal - pose[2]))


def stuck(history, now: float, window: float = 2.0) -> bool:
    """history: [(t, remaining error)]: no 5 mm (or 0.02 rad) of progress in the last `window` s."""
    old = [e for t, e in history if now - t >= window]
    return bool(old) and old[-1] - history[-1][1] < 0.005
