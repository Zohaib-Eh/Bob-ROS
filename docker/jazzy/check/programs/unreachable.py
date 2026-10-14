"""A pose far outside the reach raises Unreachable."""
from robot import Robot, Unreachable

r = Robot()
try:
    r.move_to((10.0, 0.0, 0.5))
except Unreachable as exc:
    print(f"unreachable ok: {exc}")
else:
    raise AssertionError("expected Unreachable")
