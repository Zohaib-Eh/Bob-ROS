"""_pose's quaternion round-trips every rotation, including the w = 0 tool-down ones."""
import math

import numpy as np

import robot


def rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


for rpy in [(0, 0, 0), (math.pi, 0, -0.5), (math.pi, 0, 0.5), (0, math.pi / 2, 0), (math.pi / 2, 0, math.pi),
            (-math.pi, 0.3, 2.0)]:
    p = robot._pose((0, 0, 0), rpy).orientation
    err = np.abs(rot((p.x, p.y, p.z, p.w)) - robot._matrix((0, 0, 0), rpy)[:3, :3]).max()
    assert err < 1e-9, (rpy, err)
print("orientation ok")
