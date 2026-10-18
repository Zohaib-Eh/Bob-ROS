"""Files of the generated <robot>_program folder: the task library LLM-written programs call
(robot.py), the container-side runner, run_task.sh for the agent, and the agent's brief."""
import importlib.util
import math
from pathlib import Path

import numpy as np

from .chain import Chain, home
from .fk import fk
from .moveit_config import hash_note

HERE = Path(__file__).parent
MAX_RUNS = 3
RUN_TIMEOUT_S = 120
SESSION_CONTAINER = "cad-to-urdf-session"

API_DOC = """## Library

```python
from robot import Robot   # errors: Unreachable, Collision, PlanningFailed, UnknownObject (all RobotError)
SCENE = [dict(name="part", shape="box", size=(x, y, z), xyz=(x, y, z), rpy=(0, 0, 0), mass=0.2)]
                          # at the top of the program: objects that exist when the run starts
                          # (box size = full x y z; cylinder size = (radius, height), axis along local z;
                          # mass in kg, optional; fixed=True = bolted down). Objects can't be added mid-run.
r = Robot()                                    # connected; robot at home, workcell + SCENE objects placed
r.move_to(xyz, rpy=None)  # tool link to a position; with rpy also that orientation; planned around obstacles
r.move_line(xyz, rpy=None)  # straight line; without rpy keeps the current orientation (under 6 joints: any orientation, short steps)
r.tool_down()             # an orientation (rpy) with the tool pointing straight down, for this robot
r.tool_facing(direction)  # an orientation (rpy) pointing the tool along a base-frame direction, e.g. (0, 0, 1) = up
r.move_joints(q)          # joint values in joint order
r.home()
r.attach(name)            # the object rides on the tool link (pick); the tool must be within 2 cm of it
r.detach(name)            # the object is released and falls under gravity (place it low)
r.pose(name)              # where an object really is now: (xyz, rpy)
r.solutions(xyz, rpy)     # every exact joint solution, nearest first; [] = unreachable
r.tool_pose()             # current (xyz, rpy) of the tool link
```

Example (pick and place):

```python
from robot import Robot

SCENE = [dict(name="box", shape="box", size=(0.05, 0.05, 0.05), xyz=(0.5, 0.2, 0.025), mass=0.2)]

r = Robot()
down = r.tool_down()                       # tool pointing straight down
r.move_to((0.5, 0.2, 0.15), down)
r.move_line((0.5, 0.2, 0.055), down)       # tool 5 mm above the box's top face (0.05)
r.attach("box")
r.move_line((0.5, 0.2, 0.15), down)
r.move_to((0.4, -0.3, 0.15), down)
r.move_line((0.4, -0.3, 0.055), down)
r.detach("box")
(x, y, z), _ = r.pose("box")               # check where it really landed
assert abs(x - 0.4) < 0.01 and abs(y + 0.3) < 0.01, f"box landed at {(x, y, z)}"
r.home()
```"""

GRIPPER_DOC = """## Gripper

```python
from robot import GripError   # the gripper caught nothing, an object doesn't fit, or a held object slipped
r.release()               # open the fingers fully (lets go of what is held)
r.grip(name)              # close on the object between the fingers; it moves with the tool from then on
r.gripper_width()         # current gap between the fingers, metres
r.tool_down(yaw)          # tool pointing down, jaws closing along (cos yaw, sin yaw) in the base x-y plane
```

Example (pick and place):

```python
from robot import Robot

SCENE = [dict(name="box", shape="box", size=(0.04, 0.04, 0.04), xyz=(0.5, 0.2, 0.02), mass=0.2)]

r = Robot()
down = r.tool_down(yaw=0.0)                # pointing down, jaws closing along x: across two faces of the box
r.release()                                # open before going down around the box
r.move_to((0.5, 0.2, 0.15), down)
r.move_line((0.5, 0.2, 0.02), down)        # tool point (between the fingertips) at the box's middle
r.grip("box")
r.move_line((0.5, 0.2, 0.15), down)
r.move_to((0.4, -0.3, 0.15), down)
r.move_line((0.4, -0.3, 0.025), down)      # box just above the floor
r.release()
r.move_line((0.4, -0.3, 0.15), down)       # fingers clear before checking
(x, y, z), _ = r.pose("box")               # check where it really landed
assert abs(x - 0.4) < 0.01 and abs(y + 0.3) < 0.01, f"box landed at {(x, y, z)}"
r.home()
```"""

EXAMPLE_HEAD = "\n\nExample (pick and place):"

RULES = """## Rules
- Write exactly one program per task: `programs/task_NNN_<name>.py`, with the next free number and a short snake_case name saying what it does (2-4 words), e.g. `programs/task_007_stack_two_boxes.py`. For a follow-up, write a new file starting from the previous one.
- Test it with `../__ROBOT___program/run_task.sh programs/task_NNN_<name>.py`. Every run starts clean: robot at home, the workcell's objects plus your SCENE at their declared places. It ends with `RESULT: ok` or `RESULT: failed: <reason>`.
- Physics is real: motors have limits, objects have weight, and released objects fall. Check results with r.pose() instead of assuming them.
- Fix and re-run until `RESULT: ok`. You have at most __MAX_RUNS__ runs per task; after that, stop and explain what is still wrong.
- Positions are in metres in the base frame, z up; keep objects above the floor (z > 0) and inside the reach.
- Only edit files under `programs/`. Do not run any other command.
- Never search for joint angles by moving the robot; use move_to (with or without rpy), solutions() or known joint values.
- Finish with a two-line summary: what the program does, and the last run result."""


def _gripper_lines(g, tip: str) -> list[str]:
    mm = lambda v: f"{v * 1000:.0f} mm"
    closed = max(0.0, g.width(g.upper))
    size = (f"- Opens to {mm(g.opening)} and closes fully" if closed < 0.001 else
            f"- Grips objects {closed * 1000:.0f} to {mm(g.opening)} across the jaws: fully closed the jaw tips are still "
            f"{mm(closed)} apart, so anything narrower can't be gripped")
    return ["Gripper: two fingers that squeeze an object to hold it (friction only, nothing is glued).",
            size + ".",
            f"- Grip force {g.effort:g} N per finger: holds about {2 * g.effort / 9.81:.1f} kg here (friction 1; real pads"
            " often have 0.3-0.6, so less). Heavier, or fast moves, and it slips.",
            "- Leave at least 5 mm each side between the open jaws and the object on the way down: with less, a",
            "  slight lag swings a jaw into it.",
            *([f"- The jaws are {mm(g.jaw_depth)} deep: an object reaching further up between them hits the gripper,"
               " so grip tall objects nearer their top."] if g.jaw_depth > 0 else []),
            f"- The tool point ({tip}) is between the fingertips: move_to / move_line place it, not the flange.",
            "- To pick: r.release() to open, approach from above with r.tool_down(yaw=...) so the jaws close across",
            "  two opposite faces (yaw = the object's yaw, or that + pi/2), lower so the object sits between the",
            "  fingertips, r.grip(name), then lift. r.release() lets go."]


def reach(chain: Chain, samples: int = 2000, seed: int = 0) -> float:
    rng = np.random.default_rng(seed)
    best = 0.0
    for _ in range(samples):
        q = [rng.uniform(j.lower, j.upper) if j.lower is not None else rng.uniform(-math.pi, math.pi)
             for j in chain.joints]
        best = max(best, float(np.linalg.norm(fk(chain, q)[:3, 3])))
    return best


def bearing(chain: Chain) -> float:
    """Direction (degrees, x axis = 0°, y axis = 90°) the arm reaches toward at home, in the base x-y plane."""
    T = fk(chain, home(chain))
    return math.degrees(math.atan2(T[1, 3], T[0, 3])) if math.hypot(T[0, 3], T[1, 3]) > 1e-9 else 0.0


def brief(chain: Chain, robot: str, workcell_text: str | None = None) -> str:
    T = fk(chain, home(chain))
    lines = [f"# Robot: {robot}", "",
             f"You write Python programs that move the {robot} robot arm in a ROS 2 simulation.",
             f"Base frame: {chain.base}. Tool link: {chain.tip}. Units: metres and radians.",
             f"Approximate reach from the base origin: {reach(chain):.2f} m.",
             f"At home the tool link is at xyz ({T[0, 3]:.3f}, {T[1, 3]:.3f}, {T[2, 3]:.3f}).",
             f'"In front of the robot" means toward bearing {bearing(chain):.0f}° in the base x-y plane '
             "(x axis = 0°, y axis = 90°), where the arm reaches at home.",
             f"The arm has {len(chain.joints)} moving joints."
             + ("" if len(chain.joints) >= 6 else " It cannot hold arbitrary tool orientations: call "
                "r.move_to(xyz) without rpy and let the library choose the orientation."),
             "", "Joints, in order:"]
    for j, h in zip(chain.joints, home(chain)):
        unit = "m" if j.type == "prismatic" else "rad"
        span = "continuous" if j.lower is None else f"{j.lower:.3f} .. {j.upper:.3f} {unit}"
        lines.append(f"- {j.name}: {j.type}, {span}, home {h:.3f}")
    lines += ["", "Workcell (always present; see workcell.yaml):"] + _workcell_lines(workcell_text)
    if chain.gripper is not None:
        lines += [""] + _gripper_lines(chain.gripper, chain.tip)
    rules = RULES.replace("__MAX_RUNS__", str(MAX_RUNS)).replace("__ROBOT__", robot)
    # with a gripper, its own pick example replaces the glue one (attach 5 mm above the box)
    api = API_DOC if chain.gripper is None else API_DOC.split(EXAMPLE_HEAD)[0] + "\n\n" + GRIPPER_DOC
    return "\n".join(lines + ["", api, "", rules]) + "\n"


def _scene_module():
    spec = importlib.util.spec_from_file_location("cad_to_urdf_scene", HERE / "templates" / "scene.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _workcell_lines(text: str | None) -> list[str]:
    scene = _scene_module()
    try:
        objs = scene.validate(scene.load_workcell(text), "workcell.yaml")
    except scene.SceneError as exc:  # never break generation; every run will report it too
        return [f"- (workcell.yaml has an error: {exc})"]
    out = []
    for o in objs:
        s = o["size"]
        shape = (f"box {s[0]:g} × {s[1]:g} × {s[2]:g} m" if o["shape"] == "box"
                 else f"cylinder radius {s[0]:g} m, height {s[1]:g} m")
        where = ", ".join(f"{v:g}" for v in o["xyz"])
        out.append(f"- {o['name']}: {shape} at ({where}), " + ("fixed" if o["fixed"] else f"{o['mass']:.3g} kg"))
    return out or ["- (empty)"]


def _template(name: str, robot: str, source: str) -> str:
    text = (HERE / "templates" / name).read_text(encoding="utf-8")
    return (text.replace("__ROBOT__", robot).replace("__SOURCE__", source)
            .replace("__MAX_RUNS__", str(MAX_RUNS)).replace("__TIMEOUT__", str(RUN_TIMEOUT_S))
            .replace("__CONTAINER__", SESSION_CONTAINER))


def glue_files() -> dict[str, str]:
    """The cad_to_urdf_glue MuJoCo plugin package (robot-independent)."""
    d = HERE / "templates" / "glue"
    return {p.relative_to(d).as_posix(): p.read_text(encoding="utf-8") for p in d.rglob("*") if p.is_file()}


def program_files(chain: Chain, robot: str, source: str, workcell_text: str | None = None) -> dict[str, str]:
    return {
        "brief.md": brief(chain, robot, workcell_text),
        "run_task.sh": _template("run_task.sh", robot, source),
        "robot.py": hash_note(source, robot) + _template("robot.py", robot, source),
        "runner.py": hash_note(source, robot) + _template("runner.py", robot, source),
        **{name: hash_note(source, robot) + _template(name, robot, source)
           for name in ("scene.py", "sim_ctl.py", "sim_errors.py", "scene_sync.py", "grip_logic.py")},
    }
