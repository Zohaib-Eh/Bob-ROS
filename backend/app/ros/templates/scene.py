"""Objects for a run: workcell.yaml plus the program's SCENE, validated, merged and turned into
the MuJoCo scene. Pure Python + mujoco, so the backend tests run it without ROS."""
import ast
import math
import re

import yaml

FIELDS = {"name", "shape", "size", "xyz", "rpy", "fixed", "mass", "friction"}
DENSITY = 700.0  # kg/m³ (wood) when an object has no mass
NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


class SceneError(Exception):
    """A problem with the declared objects; the run fails before the sim starts."""


def _number(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _numbers(v, n, positive=False) -> bool:
    return (isinstance(v, (list, tuple)) and len(v) == n and all(_number(x) for x in v)
            and (not positive or all(x > 0 for x in v)))


def _one(o, where: str) -> dict:
    if not isinstance(o, dict):
        raise SceneError(f"{where}: every object must be a mapping of fields")
    if "name" not in o:
        raise SceneError(f"{where}: an object is missing 'name'")
    name = o["name"]
    if not isinstance(name, str) or not NAME.match(name):
        raise SceneError(f"{where}: '{name}': name must start with a letter and use only letters, digits and _")
    for k in o:
        if k not in FIELDS:
            raise SceneError(f"{where}: '{name}': unknown field '{k}'")
    shape = o.get("shape")
    if shape not in ("box", "cylinder"):
        raise SceneError(f"{where}: '{name}': shape must be box or cylinder")
    n = 3 if shape == "box" else 2
    if not _numbers(o.get("size"), n, positive=True):
        raise SceneError(f"{where}: '{name}': size must be {n} numbers > 0"
                         + ("" if shape == "box" else " (radius, height)"))
    if not _numbers(o.get("xyz"), 3):
        raise SceneError(f"{where}: '{name}': xyz must be 3 numbers")
    rpy = o.get("rpy", [0, 0, 0])
    if not _numbers(rpy, 3):
        raise SceneError(f"{where}: '{name}': rpy must be 3 numbers")
    fixed = o.get("fixed", False)
    if not isinstance(fixed, bool):
        raise SceneError(f"{where}: '{name}': fixed must be true or false")
    size = [float(x) for x in o["size"]]
    if fixed:
        if "mass" in o:
            raise SceneError(f"{where}: '{name}': fixed objects have no mass")
        mass = None
    elif "mass" in o:
        if not (_number(o["mass"]) and o["mass"] > 0):
            raise SceneError(f"{where}: '{name}': mass must be > 0 (kg)")
        mass = float(o["mass"])
    else:
        vol = size[0] * size[1] * size[2] if shape == "box" else math.pi * size[0] ** 2 * size[1]
        mass = DENSITY * vol
    fr = o.get("friction")
    if fr is not None and not (_number(fr) and fr > 0):
        raise SceneError(f"{where}: '{name}': friction must be > 0")
    return {"name": name, "shape": shape, "size": size, "xyz": [float(x) for x in o["xyz"]],
            "rpy": [float(x) for x in rpy], "fixed": fixed, "mass": mass,
            "friction": None if fr is None else float(fr)}


def validate(objs, where: str) -> list[dict]:
    if not isinstance(objs, list):
        raise SceneError(f"{where} must be a list of objects")
    out = [_one(o, where) for o in objs]
    seen = set()
    for o in out:
        if o["name"] in seen:
            raise SceneError(f"{where}: '{o['name']}' is declared twice")
        seen.add(o["name"])
    return out


def merge(workcell: list[dict], program: list[dict]) -> list[dict]:
    names = {o["name"] for o in workcell}
    for o in program:
        if o["name"] in names:
            raise SceneError(f"'{o['name']}' is in both workcell.yaml and SCENE; rename one")
    return workcell + program


def _plain(node):
    """Literal values, plus dict(key=literal, ...) calls; anything computed raises ValueError."""
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "dict" and not node.args:
        if any(k.arg is None for k in node.keywords):
            raise ValueError("dict(**...)")
        return {k.arg: _plain(k.value) for k in node.keywords}
    if isinstance(node, (ast.List, ast.Tuple)):
        return [_plain(e) for e in node.elts]
    if isinstance(node, ast.Dict):
        return {_plain(k): _plain(v) for k, v in zip(node.keys, node.values)}
    return ast.literal_eval(node)


def read_scene(source: str) -> list[dict]:
    """The program's top-level SCENE = [...] as raw objects (validate separately); [] if absent."""
    for stmt in ast.parse(source).body:
        if (isinstance(stmt, ast.Assign) and len(stmt.targets) == 1
                and isinstance(stmt.targets[0], ast.Name) and stmt.targets[0].id == "SCENE"):
            try:
                return _plain(stmt.value)
            except (ValueError, TypeError, SyntaxError):
                raise SceneError(f"SCENE must be plain values (line {stmt.lineno})")
    return []


def load_workcell(text: str | None) -> list[dict]:
    data = yaml.safe_load(text) if text else None
    if data is None:
        return []
    if not isinstance(data, list):
        raise SceneError("workcell.yaml must be a list of objects (each starting with '- name: ...')")
    return data
