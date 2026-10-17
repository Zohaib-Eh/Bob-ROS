"""tool_facing(d) points the flange (away from the wrist) along d, for several directions including the
one opposite to home; the outward side is worked out here from the wrist centre, not from the library."""
import math

import numpy as np
from robot import Robot, _home, _matrix


def joint_frames(chain, q):
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


r = Robot()
q = _home(r.chain)
F = joint_frames(r.chain, q)
tip = r._kin.fk(q)
# the last joint's axis in the tip frame (the tip may be a tool point past the flange, e.g. a gripper's tcp)
ax = tip[:3, :3].T @ (F[-1][:3, :3] @ np.asarray(r.chain.joints[-1].axis, float))
a = tip[:3, :3] @ ax
p1, d1 = F[-2][:3, 3], F[-2][:3, :3] @ np.asarray(r.chain.joints[-2].axis, float)
n = np.cross(d1, a)
if np.linalg.norm(n) > 1e-6:
    w = p1 + np.dot(np.cross(F[-1][:3, 3] - p1, a), n) / np.dot(n, n) * d1   # wrist centre
    out_local = ax if a @ (tip[:3, 3] - w) > 0 else -ax                        # flange side, tool-frame axis
    home_out = tip[:3, :3] @ out_local
    for d in [(0, 0, -1), (0, 0, 1), (1, 0, 0), (0, -1, 0), tuple(-home_out), tuple(home_out)]:
        d = np.asarray(d, float) / np.linalg.norm(d)
        got = _matrix((0, 0, 0), r.tool_facing(d))[:3, :3] @ out_local
        assert np.allclose(got, d, atol=1e-6), (d, got)
    assert np.allclose(_matrix((0, 0, 0), r.tool_down())[:3, :3] @ out_local, (0, 0, -1), atol=1e-6)
print("tool_facing ok")
