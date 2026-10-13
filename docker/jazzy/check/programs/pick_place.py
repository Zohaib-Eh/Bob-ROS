"""robot.py round trip: add, attach, carry, detach, (solutions on 6-joint arms), home."""
from robot import Robot

r = Robot()
six = len(r.joint_names) >= 6
(x, y, z), rpy = r.tool_pose()
keep = rpy if six else None                       # <6 joints: let the library choose the orientation
r.add_box("box", (0.04, 0.04, 0.04), (x, y, z - 0.20))
r.move_to((x, y, z - 0.10), keep)
r.attach("box")
r.move_line((x, y + 0.05, z - 0.10), keep)
r.detach("box")
(x2, y2, z2), _ = r.tool_pose()
assert abs(y2 - (y + 0.05)) < 2e-3, (y, y2)
if six:
    assert r.solutions((x, y, z - 0.10), rpy), "expected at least one exact solution"
r.home()
print("pick_place ok")
