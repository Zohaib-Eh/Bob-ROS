"""Gripper acceptance: a 10 kg box (heavier than 2 fingers x 40 N x friction 1 can hold) slips out
when lifted, and the program is told so in plain words. The hook tips squeeze the box at mid-height,
so only friction holds it (no hook underneath)."""
from robot import GripError, Robot

SCENE = [
    dict(name="post_a", shape="cylinder", size=(0.008, 0.2), xyz=(0.5, 0.0, 0.1), fixed=True),
    dict(name="box", shape="box", size=(0.03, 0.03, 0.03), xyz=(0.5, 0.0, 0.215), mass=10.0),
]

r = Robot()
down = r.tool_down(yaw=0.0)
r.release()
r.move_to((0.5, 0.0, 0.32), down)
r.move_line((0.5, 0.0, 0.211), down)         # hooks (8 mm tall) level with the box's middle
r.grip("box")
try:
    r.move_line((0.5, 0.0, 0.32), down)
except GripError as exc:
    assert "slipped out of the gripper" in str(exc), exc
    print(f"sixg_grip_slip ok: {exc}")
else:
    raise AssertionError("a 10 kg box should slip out of a 40 N gripper")
