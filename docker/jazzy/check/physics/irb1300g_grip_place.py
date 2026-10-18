"""Gripper acceptance on the irb1300 (the test gripper mounted on link6's flange, tests/irb1300_gripper.py):
pick a 0.2 kg box off a post by squeezing it, carry it to a second post, release it, and check where it
really landed (spec 2026-10-17-gripper §1). Nothing is glued. The test gripper's flat pads (50 mm
open) are lowered around the 30 mm box and squeeze it; only friction holds it."""
from robot import Robot

SCENE = [
    dict(name="post_a", shape="cylinder", size=(0.008, 0.4), xyz=(0.55, 0.45, 0.2), fixed=True),
    dict(name="post_b", shape="cylinder", size=(0.008, 0.4), xyz=(0.45, 0.55, 0.2), fixed=True),
    dict(name="box", shape="box", size=(0.03, 0.03, 0.03), xyz=(0.55, 0.45, 0.415), mass=0.2),
]

r = Robot()
down = r.tool_down(yaw=0.0)                  # jaws close along x, across two faces of the box
r.release()
r.move_to((0.55, 0.45, 0.52), down)
r.move_line((0.55, 0.45, 0.395), down)       # fingertips 5 mm below the box's bottom face (post top 0.4)
r.grip("box")
assert r.gripper_width() > 0.02, r.gripper_width()
r.move_line((0.55, 0.45, 0.52), down)
r.move_to((0.45, 0.55, 0.52), down)
r.move_line((0.45, 0.55, 0.397), down)       # box 2 mm above the post
r.release()
r.move_line((0.45, 0.55, 0.52), down)
(x, y, z), _ = r.pose("box")
assert abs(x - 0.45) < 0.01 and abs(y - 0.55) < 0.01 and abs(z - 0.415) < 0.01, f"box ended at {(x, y, z)}"
print(f"irb1300g_grip_place ok: box at ({x:.3f}, {y:.3f}, {z:.3f})")
