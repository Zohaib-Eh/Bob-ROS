"""Gripper acceptance (six-axis test arm + the two-finger test gripper): pick a 0.2 kg box off a post
by squeezing it, carry it to a second post, release it, and check where it really landed. Nothing is
glued. The test gripper's fingers end in inward hooks (tips 42 mm apart), so it is lowered around a
30 mm box: the uprights squeeze it and the hooks end up under it, as with real hook fingers."""
from robot import Robot

SCENE = [
    dict(name="post_a", shape="cylinder", size=(0.008, 0.2), xyz=(0.5, 0.0, 0.1), fixed=True),
    dict(name="post_b", shape="cylinder", size=(0.008, 0.2), xyz=(0.5, 0.2, 0.1), fixed=True),
    dict(name="box", shape="box", size=(0.03, 0.03, 0.03), xyz=(0.5, 0.0, 0.215), mass=0.2),
]

r = Robot()
down = r.tool_down(yaw=0.0)                  # jaws close along x, across two faces of the box
r.release()
r.move_to((0.5, 0.0, 0.32), down)
r.move_line((0.5, 0.0, 0.19), down)          # fingertips (hooks) just under the box's bottom face
r.grip("box")
assert r.gripper_width() > 0.02, r.gripper_width()   # stopped on the 30 mm box, not fully closed
r.move_line((0.5, 0.0, 0.32), down)
r.move_to((0.5, 0.2, 0.32), down)
r.move_line((0.5, 0.2, 0.192), down)
r.release()
r.move_line((0.5, 0.2, 0.32), down)
(x, y, z), _ = r.pose("box")
assert abs(x - 0.5) < 0.01 and abs(y - 0.2) < 0.01 and abs(z - 0.215) < 0.01, f"box ended at {(x, y, z)}"
print(f"sixg_grip_place ok: box at ({x:.3f}, {y:.3f}, {z:.3f})")
