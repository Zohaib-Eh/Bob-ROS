"""Physics on the irb1300 without a gripper: attach() refuses (nothing is glued to the tool any more),
and the box stays where it was."""
from robot import GripError, Robot

SCENE = [dict(name="box", shape="box", size=(0.05, 0.05, 0.05), xyz=(0.5, 0.3, 0.025), mass=0.2)]

r = Robot()
down = r.tool_down()
r.move_to((0.5, 0.3, 0.15), down)
r.move_line((0.5, 0.3, 0.055), down)        # 5 mm above the box's top face
try:
    r.attach("box")
    raise AssertionError("attach() glued the box without a gripper")
except GripError as e:
    assert "no gripper" in str(e), e
r.move_line((0.5, 0.3, 0.15), down)
(x, y, z), _ = r.pose("box")
assert abs(x - 0.5) < 0.01 and abs(y - 0.3) < 0.01 and z < 0.03, f"box moved to {(x, y, z)}"
r.home()
print(f"irb1300_pick_place ok: no gripper, attach refused, box at ({x:.3f}, {y:.3f}, {z:.3f})")
