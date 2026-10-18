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


def plan(start, goal_xy, goal_yaw, center, reverse: bool = False):
    """(pivot target, final yaw): with no goal yaw the base ends facing the way it drove (reverse: the way
    it faced while backing up)."""
    p0 = pivot(start, center)
    if goal_yaw is None:
        travel = np.asarray(goal_xy, float) - p0
        goal_yaw = (math.atan2(travel[1], travel[0]) + (math.pi if reverse else 0.0)
                    if np.linalg.norm(travel) > POS_TOL else start[2])
    return pivot((*goal_xy, goal_yaw), center), goal_yaw


def command(phase: str, pose, target, final_yaw: float, center, speed: float, reverse: bool = False):
    """(v, w, next phase) for one control tick. Phases: 'face' (turn toward the target, or away from it to
    back up), 'go' (drive to it, steering), 'turn' (to the final yaw), 'done'."""
    p = pivot(pose, center)
    d = np.asarray(target, float) - p
    dist = float(np.linalg.norm(d))
    yaw = pose[2]
    back = math.pi if reverse else 0.0
    if phase == "face":
        if dist < POS_TOL:
            return 0.0, 0.0, "turn"
        err = wrap(math.atan2(d[1], d[0]) + back - yaw)
        if abs(err) < YAW_TOL * 2:
            return 0.0, 0.0, "go"
        return 0.0, float(np.clip(TURN_GAIN * err, -MAX_TURN, MAX_TURN)), "face"
    if phase == "go":
        if dist < POS_TOL:
            return 0.0, 0.0, "turn"
        along = float(d @ [math.cos(yaw + back), math.sin(yaw + back)])   # ahead along the way it travels
        err = wrap(math.atan2(d[1], d[0]) + back - yaw) if along > 0 else 0.0
        w = float(np.clip(STEER_GAIN * err, -MAX_TURN, MAX_TURN)) if dist > 0.05 else 0.0
        v = float(np.clip(SPEED_GAIN * along, -speed, speed))
        return (-v if reverse else v), w, "go"
    if phase == "turn":
        err = wrap(final_yaw - yaw)
        if abs(err) < YAW_TOL:
            return 0.0, 0.0, "done"
        return 0.0, float(np.clip(TURN_GAIN * err, -MAX_TURN, MAX_TURN)), "turn"
    return 0.0, 0.0, "done"


def _outline(footprint, step: float = 0.05) -> np.ndarray:
    """Points around the base's outline (base frame), `step` apart."""
    x0, y0, x1, y1 = footprint
    nx, ny = max(2, int((x1 - x0) / step) + 1), max(2, int((y1 - y0) / step) + 1)
    xs, ys = np.linspace(x0, x1, nx), np.linspace(y0, y1, ny)
    return np.array([(x, y) for x in xs for y in (y0, y1)] + [(x, y) for y in ys for x in (x0, x1)])


def sweep(start, target, final_yaw: float, center, reverse: bool = False) -> list:
    """The base poses drive_to passes through: turn about the pivot to face the target (away from it when
    backing up), drive straight to it, turn to the final yaw (5° and 2 cm apart)."""
    def at(p, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return (p[0] - c * center[0] + s * center[1], p[1] - s * center[0] - c * center[1], yaw)

    def turn(p, a, b):
        n = max(1, int(abs(wrap(b - a)) / math.radians(5)) + 1)
        return [at(p, a + wrap(b - a) * k / n) for k in range(n + 1)]

    p0, p1 = pivot(start, center), np.asarray(target, float)
    d = p1 - p0
    if np.linalg.norm(d) < POS_TOL:
        return turn(p0, start[2], final_yaw)
    heading = math.atan2(d[1], d[0]) + (math.pi if reverse else 0.0)
    n = max(1, int(np.linalg.norm(d) / 0.02) + 1)
    return (turn(p0, start[2], heading) + [at(p0 + d * k / n, heading) for k in range(n + 1)]
            + turn(p1, heading, final_yaw))


def _rect_distance(p, obj) -> float:
    """Plan distance from point p to an object's footprint (centre, half sizes, yaw)."""
    (cx, cy), (hx, hy), yaw = obj["xy"], obj["half"], obj["yaw"]
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = p[0] - cx, p[1] - cy
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return math.hypot(max(abs(lx) - hx, 0.0), max(abs(ly) - hy, 0.0))


def _touches(pose, outline, obj) -> bool:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return any(_rect_distance((pose[0] + c * x - s * y, pose[1] + s * x + c * y), obj) < MARGIN for x, y in outline)


def blocker(poses, footprint, objects, height: float) -> str | None:
    """The first object the base's outline (plus MARGIN) would run into along `poses` (from sweep), or None.
    objects: [{name, xy, half, yaw, z0}], z0 its bottom above the floor. Objects above the base, and ones it
    already touches at the start (resting on it, or what it is parked against), are not in the way."""
    outline = _outline(footprint)
    x0, y0, x1, y1 = footprint

    def on_base(o):   # its centre inside the outline at the start: it rides along
        c, s = math.cos(poses[0][2]), math.sin(poses[0][2])
        dx, dy = o["xy"][0] - poses[0][0], o["xy"][1] - poses[0][1]
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        return x0 <= lx <= x1 and y0 <= ly <= y1

    for o in objects:
        if o["z0"] >= height or on_base(o) or _touches(poses[0], outline, o):
            continue
        if any(_touches(p, outline, o) for p in poses):
            return o["name"]
    return None


def remaining(phase: str, pose, target, final_yaw: float, center, reverse: bool = False) -> float:
    """What is left of the current phase: metres while driving, radians while turning."""
    d = np.asarray(target, float) - pivot(pose, center)
    if phase == "go":
        return float(np.linalg.norm(d))
    goal = final_yaw if phase == "turn" else math.atan2(d[1], d[0]) + (math.pi if reverse else 0.0)
    return abs(wrap(goal - pose[2]))


def stuck(history, now: float, window: float = 2.0) -> bool:
    """history: [(t, remaining error)]: no 5 mm (or 0.02 rad) of progress in the last `window` s."""
    old = [e for t, e in history if now - t >= window]
    return bool(old) and old[-1] - history[-1][1] < 0.005
