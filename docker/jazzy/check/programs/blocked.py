"""A target inside an obstacle that does not touch the start pose raises Collision or PlanningFailed naming it."""
from robot import Collision, PlanningFailed, Robot

r = Robot()
(x, y, z), rpy = r.tool_pose()
target = (x, y, z - 0.20)  # 0.2 m from the home tool position
r.add_box("wall", (0.1, 0.1, 0.1), target)
try:
    r.move_to(target, rpy if len(r.joint_names) >= 6 else None)
except (Collision, PlanningFailed) as exc:
    assert "wall" in str(exc), exc
    print(f"blocked ok: {type(exc).__name__}: {exc}")
else:
    raise AssertionError("expected Collision or PlanningFailed")
