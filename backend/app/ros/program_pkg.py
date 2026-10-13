"""Files of the generated <robot>_program folder: the task library LLM-written programs call
(robot.py), the container-side runner, run_task.sh for the agent, and the agent's brief."""
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
r = Robot()                                    # connected; robot at home, scene empty
r.add_box(name, size_xyz, xyz, rpy=(0, 0, 0))  # box centred at xyz
r.add_cylinder(name, radius, height, xyz, rpy=(0, 0, 0))  # axis along its local z
r.move_to(xyz, rpy)       # tool link to a pose, planned around obstacles
r.move_line(xyz, rpy)     # tool link along a straight line
r.move_joints(q)          # joint values in joint order
r.home()
r.attach(name)            # the object rides on the tool link (pick)
r.detach(name)            # the object stays where it is (place)
r.solutions(xyz, rpy)     # every exact joint solution, nearest first; [] = unreachable
r.tool_pose()             # current (xyz, rpy) of the tool link
```

Example (pick and place):

```python
from robot import Robot
r = Robot()
r.add_box("box", (0.05, 0.05, 0.05), (0.5, 0.2, 0.025))
xyz, down = r.tool_pose()                 # keep the home orientation of the tool
r.move_to((0.5, 0.2, 0.15), down)
r.move_line((0.5, 0.2, 0.06), down)
r.attach("box")
r.move_line((0.5, 0.2, 0.15), down)
r.move_to((0.4, -0.3, 0.15), down)
r.move_line((0.4, -0.3, 0.06), down)
r.detach("box")
r.home()
```"""

RULES = """## Rules
- Write exactly one program per task: `programs/task_NNN.py` (next free number). For a follow-up, write a new file starting from the previous one.
- Test it with `../__ROBOT___program/run_task.sh programs/task_NNN.py`. Every run starts clean (robot at home, no objects) and ends with `RESULT: ok` or `RESULT: failed: <reason>`.
- Fix and re-run until `RESULT: ok`. You have at most __MAX_RUNS__ runs per task; after that, stop and explain what is still wrong.
- Positions are in metres in the base frame, z up; keep objects above the floor (z > 0) and inside the reach.
- Only edit files under `programs/`. Do not run any other command.
- Finish with a two-line summary: what the program does, and the last run result."""


def reach(chain: Chain, samples: int = 2000, seed: int = 0) -> float:
    rng = np.random.default_rng(seed)
    best = 0.0
    for _ in range(samples):
        q = [rng.uniform(j.lower, j.upper) if j.lower is not None else rng.uniform(-math.pi, math.pi)
             for j in chain.joints]
        best = max(best, float(np.linalg.norm(fk(chain, q)[:3, 3])))
    return best


def brief(chain: Chain, robot: str) -> str:
    T = fk(chain, home(chain))
    lines = [f"# Robot: {robot}", "",
             f"You write Python programs that move the {robot} robot arm in a ROS 2 simulation.",
             f"Base frame: {chain.base}. Tool link: {chain.tip}. Units: metres and radians.",
             f"Approximate reach from the base origin: {reach(chain):.2f} m.",
             f"At home the tool link is at xyz ({T[0, 3]:.3f}, {T[1, 3]:.3f}, {T[2, 3]:.3f}).",
             "", "Joints, in order:"]
    for j, h in zip(chain.joints, home(chain)):
        unit = "m" if j.type == "prismatic" else "rad"
        span = "continuous" if j.lower is None else f"{j.lower:.3f} .. {j.upper:.3f} {unit}"
        lines.append(f"- {j.name}: {j.type}, {span}, home {h:.3f}")
    rules = RULES.replace("__MAX_RUNS__", str(MAX_RUNS)).replace("__ROBOT__", robot)
    return "\n".join(lines + ["", API_DOC, "", rules]) + "\n"


def _template(name: str, robot: str, source: str) -> str:
    text = (HERE / "templates" / name).read_text(encoding="utf-8")
    return (text.replace("__ROBOT__", robot).replace("__SOURCE__", source)
            .replace("__MAX_RUNS__", str(MAX_RUNS)).replace("__TIMEOUT__", str(RUN_TIMEOUT_S))
            .replace("__CONTAINER__", SESSION_CONTAINER))


def program_files(chain: Chain, robot: str, source: str) -> dict[str, str]:
    return {
        "brief.md": brief(chain, robot),
        "run_task.sh": _template("run_task.sh", robot, source),
        "robot.py": hash_note(source, robot) + _template("robot.py", robot, source),
        "runner.py": hash_note(source, robot) + _template("runner.py", robot, source),
    }
