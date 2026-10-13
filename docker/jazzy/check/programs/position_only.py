"""move_to without rpy reaches a reachable position on any arm, using model IK (no probing moves)."""
import numpy as np
from robot import Robot

r = Robot()
q = np.array([0.4 * (-1) ** i for i in range(len(r.joint_names))])
lo = np.array([j.lower if j.lower is not None else -3.0 for j in r.chain.joints])
hi = np.array([j.upper if j.upper is not None else 3.0 for j in r.chain.joints])
target = r._kin.fk(np.clip(q, lo, hi))[:3, 3]
r.move_to(tuple(target))
(x, y, z), _ = r.tool_pose()
assert np.linalg.norm(np.array([x, y, z]) - target) < 2e-3, ((x, y, z), target)
print("position_only ok")
