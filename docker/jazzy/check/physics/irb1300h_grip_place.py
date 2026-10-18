"""Gripper acceptance with a real gripper: the AirTAC HFK40 air gripper (GrabCAD STEP, body + two jaws;
tests/irb1300_gripper.py mounts it on the irb1300's flange, jaw travel 15 mm each from the datasheet).
Its jaw tips are 30 mm apart closed and 60 open, and the jaws are 37 mm deep (the slides jut in above
that). Pick a 0.2 kg box, 40 mm across (10 mm clearance each side) and 30 tall, off a post by squeezing
it, carry it to a second post, release it, and check where it really landed. Nothing is glued."""
from robot import Robot

SCENE = [
    dict(name="post_a", shape="cylinder", size=(0.008, 0.4), xyz=(0.55, 0.45, 0.2), fixed=True),
    dict(name="post_b", shape="cylinder", size=(0.008, 0.4), xyz=(0.45, 0.55, 0.2), fixed=True),
    dict(name="box", shape="box", size=(0.04, 0.04, 0.03), xyz=(0.55, 0.45, 0.415), mass=0.2),
]

r = Robot()
down = r.tool_down(yaw=0.0)                  # jaws close along x, across two faces of the box
r.release()
r.move_to((0.55, 0.45, 0.52), down)
r.move_line((0.55, 0.45, 0.402), down)       # jaw tips 2 mm above the box's bottom face (post top 0.4)
r.grip("box")
assert r.gripper_width() > 0.035, r.gripper_width()   # stopped on the 40 mm box, not fully closed
r.move_line((0.55, 0.45, 0.52), down)
r.move_to((0.45, 0.55, 0.52), down)
r.move_line((0.45, 0.55, 0.404), down)       # box 2 mm above the post
r.release()
r.move_line((0.45, 0.55, 0.52), down)
(x, y, z), _ = r.pose("box")
assert abs(x - 0.45) < 0.01 and abs(y - 0.55) < 0.01 and abs(z - 0.415) < 0.01, f"box ended at {(x, y, z)}"
print(f"irb1300h_grip_place ok: box at ({x:.3f}, {y:.3f}, {z:.3f})")
