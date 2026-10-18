"""CAD fixture acceptance (sixf: the six-axis test arm with the test gripper; sixf_workcell/ holds its
workcell: a table imported as a CAD fixture by the app's own import, 7 convex pieces). Pick a 0.2 kg box off
a post, set it on the imported table, and check it really rests on the table's top. Nothing is glued."""
from robot import Robot

SCENE = [
    dict(name="post_a", shape="cylinder", size=(0.008, 0.2), xyz=(0.5, 0.0, 0.1), fixed=True),
    dict(name="box", shape="box", size=(0.03, 0.03, 0.03), xyz=(0.5, 0.0, 0.215), mass=0.2),
]

r = Robot()
down = r.tool_down(yaw=0.0)                  # jaws close along x, across two faces of the box
r.release()
r.move_to((0.5, 0.0, 0.32), down)
r.move_line((0.5, 0.0, 0.202), down)         # fingertips 2 mm above the box's bottom: none of the pad hangs below it
r.grip("box")
r.move_line((0.5, 0.0, 0.42), down)
r.move_to((0.5, 0.25, 0.42), down)           # over the table (top at 0.30)
r.move_line((0.5, 0.25, 0.304), down)        # box 2 mm above the table top
r.release()
r.move_line((0.5, 0.25, 0.42), down)
(x, y, z), _ = r.pose("box")
assert abs(x - 0.5) < 0.01 and abs(y - 0.25) < 0.01 and abs(z - 0.315) < 0.005, f"box ended at {(x, y, z)}"
print(f"sixf_table_place ok: box on the table at ({x:.3f}, {y:.3f}, {z:.3f})")
