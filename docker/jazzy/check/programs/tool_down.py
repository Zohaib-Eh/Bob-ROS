"""tool_down() points the approach axis straight down; on 6-joint arms the tool can get there."""
import numpy as np
from robot import Robot, _matrix

r = Robot()
rpy = r.tool_down()
a = _matrix((0, 0, 0), rpy)[:3, :3] @ np.asarray(r.chain.joints[-1].axis, float)
assert abs(abs(a[2]) - 1) < 1e-9, a                  # the approach axis is vertical
if len(r.joint_names) >= 6:
    (x, y, z), _ = r.tool_pose()
    r.move_to((x, y, z - 0.1), rpy)
print("tool_down ok")
