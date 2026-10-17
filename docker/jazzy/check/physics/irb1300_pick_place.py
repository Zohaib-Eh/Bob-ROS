"""Physics acceptance on the irb1300: pick a 0.2 kg box off the floor, carry it, set it down;
it must land within 1 cm of the target (spec 2026-10-17-mujoco-programs §1)."""
from robot import Robot

SCENE = [dict(name="box", shape="box", size=(0.05, 0.05, 0.05), xyz=(0.5, 0.3, 0.025), mass=0.2)]

r = Robot()
down = r.tool_down()
r.move_to((0.5, 0.3, 0.15), down)
r.move_line((0.5, 0.3, 0.055), down)        # 5 mm above the box's top face
r.attach("box")
r.move_line((0.5, 0.3, 0.15), down)
r.move_to((0.3, 0.5, 0.15), down)
r.move_line((0.3, 0.5, 0.056), down)        # box bottom ~1 mm above the floor
r.detach("box")
r.move_line((0.3, 0.5, 0.15), down)
(x, y, z), _ = r.pose("box")
assert abs(x - 0.3) < 0.01 and abs(y - 0.5) < 0.01 and abs(z - 0.025) < 0.005, f"box ended at {(x, y, z)}"
print(f"irb1300_pick_place ok: box at ({x:.3f}, {y:.3f}, {z:.3f})")
