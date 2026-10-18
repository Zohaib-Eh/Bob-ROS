"""Mobile base acceptance: drive_to ends where it was asked (MuJoCo's truth, via localization), backs up,
and refuses a path through a table instead of crashing into it."""
import math

from robot import PathBlocked, Robot

SCENE = [dict(name="table", shape="box", size=(0.4, 0.6, 0.3), xyz=(1.5, 0.0, 0.15), fixed=True)]


def near(goal, tol=0.03, tol_yaw=math.radians(3)):
    x, y, yaw = r.base_pose()
    assert math.hypot(x - goal[0], y - goal[1]) < tol and abs((yaw - goal[2] + math.pi) % (2 * math.pi) - math.pi) < tol_yaw, \
        f"base at ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg), wanted {goal}"


r = Robot()
near((0.0, 0.0, 0.0))
r.drive_to(0.8, 0.6, math.pi / 2)
near((0.8, 0.6, math.pi / 2))
r.drive_to(0.8, 0.2, reverse=True)           # back up, still facing +y
near((0.8, 0.2, math.pi / 2))
try:
    r.drive_to(0.8, -1.0, -math.pi / 2)      # turning round there swings the base into the table
    raise AssertionError("drive_to went through the table")
except PathBlocked as e:
    assert "table" in str(e), e
r.drive_to(0.0, 0.0, 0.0)
near((0.0, 0.0, 0.0))
print("mobi_drive ok:", ", ".join(f"{v:.3f}" for v in r.base_pose()))
