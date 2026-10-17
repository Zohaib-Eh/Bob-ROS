"""robot.py round trip: add, attach, carry, detach, (solutions on 6-joint arms). The placed box still
touches the flange, so the check ends there (every run resets the robot)."""
import numpy as np
from robot import Robot


def beyond_flange(r, dist):
    """Point `dist` metres out from the tool frame along the flange direction (away from the arm)."""
    q = r._current()
    T = r._kin.fk(q)
    a = r._joint_frames(q)[-1][:3, :3] @ np.asarray(r.chain.joints[-1].axis, float)   # last joint's axis, base frame
    if sum(float(a @ (F[:3, 3] - T[:3, 3])) for F in r._joint_frames(q)[:-1]) > 0:
        a = -a
    return tuple(T[:3, 3] + a * dist)


r = Robot()
six = len(r.joint_names) >= 6
(x, y, z), rpy = r.tool_pose()
keep = rpy if six else None                       # <6 joints: let the library choose the orientation
r.move_to((x, y, z - 0.10), keep)
(x, y, z), pick_rpy = r.tool_pose()
r.add_box("box", (0.04, 0.04, 0.04), beyond_flange(r, 0.025))   # touching the flange face
r.attach("box")
r.move_line((x, y + 0.05, z), keep)
r.detach("box")
(x2, y2, z2), _ = r.tool_pose()
assert abs(y2 - (y + 0.05)) < 2e-3, (y, y2)
if six:
    assert r.solutions((x, y, z), pick_rpy), "expected at least one exact solution"
print("pick_place ok")
