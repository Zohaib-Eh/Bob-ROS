"""tool_down() points the tool straight down with the wrist above the flange; on 6-joint arms the tool can get there."""
import math

import numpy as np
from robot import Robot, _matrix


def joint_frames(chain, q):
    """Base-frame pose of each joint frame, computed here so the check doesn't trust the library's own helper."""
    T, out = np.eye(4), []
    for j, qi in zip(chain.joints, q):
        T = T @ j.origin
        out.append(T.copy())
        ax, M = np.asarray(j.axis, float), np.eye(4)
        if j.type == "prismatic":
            M[:3, 3] = ax * qi
        else:
            k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            M[:3, :3] = np.eye(3) + math.sin(qi) * k + (1 - math.cos(qi)) * k @ k
        T = T @ M
    return out


def wrist_centre(r, q):
    """Closest point on the second-to-last joint's axis line to the last joint's axis line (None if parallel)."""
    F = joint_frames(r.chain, q)
    p1, d1 = F[-2][:3, 3], F[-2][:3, :3] @ np.asarray(r.chain.joints[-2].axis, float)
    p2, d2 = F[-1][:3, 3], F[-1][:3, :3] @ np.asarray(r.chain.joints[-1].axis, float)
    n = np.cross(d1, d2)
    if np.linalg.norm(n) < 1e-6:
        return None
    return p1 + np.dot(np.cross(p2 - p1, d2), n) / np.dot(n, n) * d1


r = Robot()
rpy = r.tool_down()
a = _matrix((0, 0, 0), rpy)[:3, :3] @ np.asarray(r.chain.joints[-1].axis, float)
assert abs(abs(a[2]) - 1) < 1e-9, a                  # the approach axis is vertical
if len(r.joint_names) >= 6:
    (x, y, z), _ = r.tool_pose()
    # a reachable tool-down spot: pull in from the (often fully stretched) home position and go lower
    spot = next(((x * f, y * f, z * h) for f in (0.7, 0.5, 0.85, 0.35) for h in (0.6, 0.8, 0.4, 1.0)
                 if r.solutions((x * f, y * f, z * h), rpy)), None)
    assert spot is not None, "no reachable tool-down pose in front of the robot"
    r.move_to(spot, rpy)
    q = r._current()
    w, tip = wrist_centre(r, q), r._kin.fk(q)[:3, 3]
    if w is not None and np.linalg.norm(w - tip) > 1e-3:
        assert w[2] > tip[2], ("wrist below the flange: the tool faces up", w, tip)
print("tool_down ok")
