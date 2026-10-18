"""Mobile manipulation acceptance: drive up to a table, pick a 0.2 kg box off it by squeezing, back away and
come back carrying it (only friction holds it), set it down 15 cm to the side, check where it really is."""
import math

from robot import Robot

SCENE = [
    dict(name="table", shape="box", size=(0.4, 0.6, 0.3), xyz=(1.5, 0.0, 0.15), fixed=True),
    dict(name="box", shape="box", size=(0.03, 0.03, 0.03), xyz=(1.4, 0.0, 0.315), mass=0.2),
]

r = Robot()
r.drive_to(0.85, 0.0, 0.0)                   # the box 0.55 m ahead of the base
(x, y, z), _ = r.pose("box")
down = r.tool_down(yaw=0.0)
r.release()
r.move_to((x, y, z + 0.12), down)
r.move_line((x, y, z - 0.01), down)
r.grip("box")
r.move_line((x, y, z + 0.12), down)
r.drive_to(0.5, 0.0, reverse=True)           # carry it away and back
r.drive_to(0.85, 0.0, 0.0)
r.move_to((1.4, 0.15, z + 0.12), down)
r.move_line((1.4, 0.15, z + 0.002), down)
r.release()
r.move_line((1.4, 0.15, z + 0.12), down)
(bx, by, bz), _ = r.pose("box")
assert abs(bx - 1.4) < 0.015 and abs(by - 0.15) < 0.015 and abs(bz - 0.315) < 0.01, f"box ended at {(bx, by, bz)}"
print(f"mobi_drive_pick ok: box at ({bx:.3f}, {by:.3f}, {bz:.3f}), base at {', '.join(f'{v:.3f}' for v in r.base_pose())}")
