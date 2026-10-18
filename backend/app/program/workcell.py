"""The workcell editor's side of <robot>_tasks/workcell.yaml: read and write it as objects, by robot name, with
or without ROS running (the workspace stays on disk). Validation is the runs' own (templates/scene.py)."""
import functools
import math
import re
import tempfile
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
        if o["shape"] == "mesh":
            c["file"], c["pieces"] = o["file"], o["pieces"]
        clean.append(c)
    text = HEADER + (yaml.safe_dump(clean, sort_keys=False, default_flow_style=None) if clean else "[]\n")
    p = path(robot)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".yaml.tmp")
    tmp.write_text(text, encoding="utf-8", newline="\n")
    tmp.replace(p)
    _refresh_brief(robot, text)
    return objs


def meshes(robot: str) -> Path:
    return path(robot).parent / "meshes"


def import_fixture(robot: str, filename: str, data: bytes, scale: float) -> dict:
    """A CAD file (a table, a conveyor) as one bolted-down object: its shape and convex pieces saved in the
    workcell's meshes folder, centred on its bounding box, set on the floor in front of the robot."""
    import trimesh
    from app.export.gripper import convex_pieces
    from app.importers.pipeline import import_path
    suffix = Path(filename).suffix.lower()
    with tempfile.TemporaryDirectory() as tmp:
        f = Path(tmp) / f"fixture{suffix}"
        f.write_bytes(data)
        bodies, _, _ = import_path(str(f), suffix, scale)
    parts = [trimesh.Trimesh(b.vertices, b.triangles, process=False) for b in bodies if b.status != "failed" and b.vertices]
    if not parts:
        raise ValueError(f"{filename} has no usable shape")
    whole = trimesh.util.concatenate(parts)
    lo, hi = whole.bounds
    whole.apply_translation(-(lo + hi) / 2)              # the object's frame: its bounding box's centre
    size = [round(float(v), 6) for v in hi - lo]   # STL coordinates are float32
    current = read(robot)
    taken = {o["name"] for o in current["objects"]}
    stem = re.sub(r"[^A-Za-z0-9_]", "_", Path(filename).stem) or "fixture"
    stem = stem if stem[0].isalpha() else f"fixture_{stem}"
    name, n = stem, 1
    while name in taken:
        n += 1
        name = f"{stem}_{n}"
    folder = meshes(robot)
    folder.mkdir(parents=True, exist_ok=True)
    whole.export(folder / f"{name}.stl", file_type="stl")
    pieces = convex_pieces(whole.vertices, whole.faces, max_pieces=16)
    for i, (v, t) in enumerate(pieces):
        trimesh.Trimesh(v, t, process=False).export(folder / f"{name}_c{i}.stl", file_type="stl")
    reach = current["reach"]
    out = next((r for z, r in reach if abs(z - size[2]) <= BAND / 2), None) if reach else None
    x = 0.6 * out if out else 0.8
    obj = {"name": name, "shape": "mesh", "file": f"{name}.stl", "pieces": len(pieces), "size": size,
           "xyz": [x, 0.0, size[2] / 2], "fixed": True}
    write(robot, current["objects"] + [obj])
    return read(robot)


def fixture_mesh(robot: str, file: str) -> dict:
    """{vertices, triangles} of a fixture's <name>.stl, for the editor's 3D view. FileNotFoundError if absent."""
    import trimesh
    if not _scene_module().STL_NAME.match(file or ""):
        raise ValueError(f"'{file}' is not a fixture mesh name")
    f = meshes(robot) / file
    if not f.exists():
        raise FileNotFoundError(f)
    m = trimesh.load(str(f))
    return {"vertices": m.vertices.tolist(), "triangles": m.faces.tolist()}


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
