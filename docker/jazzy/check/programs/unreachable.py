"""A pose far outside the reach raises Unreachable."""
from robot import Robot, Unreachable

r = Robot()
_, rpy = r.tool_pose()
try:
    r.move_to((10.0, 0.0, 0.5), rpy)
except Unreachable as exc:
    print(f"unreachable ok: {exc}")
else:
    raise AssertionError("expected Unreachable")
