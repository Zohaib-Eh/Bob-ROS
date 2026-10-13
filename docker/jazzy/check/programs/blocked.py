"""A target inside an obstacle raises Collision (or PlanningFailed)."""
from robot import Collision, PlanningFailed, Robot

r = Robot()
(x, y, z), rpy = r.tool_pose()
r.add_box("wall", (0.3, 0.3, 0.3), (x, y, z - 0.05))
try:
    r.move_to((x, y, z - 0.05), rpy)
except (Collision, PlanningFailed) as exc:
    print(f"blocked ok: {type(exc).__name__}: {exc}")
else:
    raise AssertionError("expected Collision or PlanningFailed")
