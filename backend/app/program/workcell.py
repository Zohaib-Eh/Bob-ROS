"""The workcell editor's side of <robot>_tasks/workcell.yaml: read and write it as objects, by robot name, with
or without ROS running (the workspace stays on disk). Validation is the runs' own (templates/scene.py)."""
import functools
import math
from pathlib import Path

import numpy as np
import yaml

from app.program.session import WORKSPACES
from app.ros.chain import parse_chain
from app.ros.fk import fk
from app.ros.moveit_config import ROS_NAME
from app.ros.program_pkg import _scene_module, brief

HEADER = ("# The workcell: objects in every run (Simulate > Workcell edits this file).\n"
          "# box size = full x y z, cylinder size = (radius, height); metres, base frame; fixed = bolted down.\n")


class BadRobot(ValueError):
    pass


def _src(robot: str) -> Path:
    if not ROS_NAME.match(robot or ""):
        raise BadRobot(f"'{robot}' is not a robot name")
    return WORKSPACES / robot / "src"


def path(robot: str) -> Path:
    return _src(robot) / f"{robot}_tasks" / "workcell.yaml"


def read(robot: str) -> dict:
    """{objects, error, reach}: a file with an error reads as no objects plus the error to show. reach =
    [[z, r], ...]: how far (horizontally) the tool point gets at each height, in BAND-high bands; None until
    ROS has set the robot up once."""
    scene = _scene_module()
    p = path(robot)
    try:
        objs = scene.validate(scene.load_workcell(p.read_text(encoding="utf-8") if p.exists() else None), "workcell.yaml")
        error = None
    except scene.SceneError as exc:
        objs, error = [], str(exc)
    return {"objects": objs, "error": error, "reach": _reach(robot)}


def write(robot: str, objects: list) -> list[dict]:
    """Validate (ValueError with the reason, nothing written), write, and refresh the program brief."""
    scene = _scene_module()
    # null means "not set" (read() returns fixed objects with mass: None), not "mass: nothing"
    objects = [{k: v for k, v in o.items() if v is not None} if isinstance(o, dict) else o for o in objects]
    try:
        objs = scene.validate(objects, "workcell")
    except scene.SceneError as exc:
        raise ValueError(str(exc)) from None
    clean = []
    for o in objs:
        c = {"name": o["name"], "shape": o["shape"], "size": o["size"], "xyz": o["xyz"]}
        if any(o["rpy"]):
            c["rpy"] = o["rpy"]
        if o["fixed"]:
            c["fixed"] = True
        elif o["mass"] is not None:
            c["mass"] = o["mass"]
        if o["friction"] is not None:
            c["friction"] = o["friction"]
        clean.append(c)
    text = HEADER + (yaml.safe_dump(clean, sort_keys=False, default_flow_style=None) if clean else "[]\n")
    p = path(robot)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".yaml.tmp")
    tmp.write_text(text, encoding="utf-8", newline="\n")
    tmp.replace(p)
    _refresh_brief(robot, text)
    return objs


def _urdf(robot: str) -> Path:
    return _src(robot) / robot / "urdf" / f"{robot}.urdf"


def _refresh_brief(robot: str, workcell_text: str) -> None:
    """The chat reads the workcell from brief.md, written at session start; keep it in step with edits."""
    b = _src(robot) / f"{robot}_program" / "brief.md"
    if b.exists() and _urdf(robot).exists():
        b.write_text(brief(parse_chain(_urdf(robot).read_text(encoding="utf-8")), robot, workcell_text),
                     encoding="utf-8", newline="\n")


BAND = 0.1   # m


def _reach(robot: str) -> list[list[float]] | None:
    u = _urdf(robot)
    return _reach_of(str(u), u.stat().st_mtime) if u.exists() else None


@functools.lru_cache(maxsize=8)
def _reach_of(urdf_path: str, _mtime: float, samples: int = 4000) -> list[list[float]]:
    """Not one radius: an arm on a pedestal reaches far up high but much less at table height."""
    # ponytail: random joint samples; a reachable spot between samples can read as just out of reach
    chain = parse_chain(Path(urdf_path).read_text(encoding="utf-8"))
    rng = np.random.default_rng(0)
    best: dict[int, float] = {}
    for _ in range(samples):
        q = [rng.uniform(j.lower, j.upper) if j.lower is not None else rng.uniform(-math.pi, math.pi) for j in chain.joints]
        x, y, z = fk(chain, q)[:3, 3]
        k = int(math.floor(z / BAND))
        best[k] = max(best.get(k, 0.0), math.hypot(x, y))
    return [[round((k + 0.5) * BAND, 3), round(r, 3)] for k, r in sorted(best.items())]
