"""Objects for a run: workcell.yaml plus the program's SCENE, validated, merged and turned into
the MuJoCo scene. Pure Python + mujoco, so the backend tests run it without ROS."""
import ast
import math
import re
import xml.etree.ElementTree as ET

import numpy as np
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


PENETRATION = 0.001  # m: resting contact is fine, deeper overlap is a setup mistake

def _quat(rpy) -> list[float]:
    r, p, y = (v / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def _fmt(v) -> str:
    return " ".join(repr(float(x)) for x in v)


def build_scene_xml(robot_mjcf: str, objs: list[dict], tool: str, meshdir: str) -> str:
    """The robot MJCF plus a floor and the objects. Movable objects get a free joint named after
    them and an inactive weld 'glue_<name>' to the tool; fixed objects get neither."""
    root = ET.fromstring(robot_mjcf)
    robot_bodies = {b.get("name") for b in root.iter("body")}
    for o in objs:
        if o["name"] in robot_bodies:
            raise SceneError(f"'{o['name']}' is the name of a robot link; rename the object")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", meshdir)
    world = root.find("worldbody")
    if world.find("geom[@name='floor']") is None:
        ET.SubElement(world, "geom", {"name": "floor", "type": "plane", "size": "0 0 1"})
    eq = None
    for o in objs:
        body = ET.SubElement(world, "body", {"name": o["name"], "pos": _fmt(o["xyz"]), "quat": _fmt(_quat(o["rpy"]))})
        if not o["fixed"]:
            ET.SubElement(body, "freejoint", {"name": o["name"]})
        half = [s / 2 for s in o["size"]] if o["shape"] == "box" else [o["size"][0], o["size"][1] / 2]
        g = {"type": o["shape"], "size": _fmt(half)}
        if o["mass"] is not None:
            g["mass"] = repr(o["mass"])
        if o["friction"] is not None:
            g["friction"] = f"{o['friction']!r} 0.005 0.0001"
        ET.SubElement(body, "geom", g)
        if not o["fixed"]:
            if eq is None:  # not `find() or ...`: an Element with no children is falsy
                eq = root.find("equality")
                if eq is None:
                    eq = ET.SubElement(root, "equality")
            ET.SubElement(eq, "weld", {"name": f"glue_{o['name']}", "body1": tool, "body2": o["name"],
                                       "active": "false"})
    return ET.tostring(root, encoding="unicode")


def glue_on(m, d, e: int) -> None:
    """Switch weld e on, holding body2 where it is now relative to body1 (no jump).
    Mirrors glue_plugin.cpp: eq_data = [anchor(3), relpos(3), relquat(4), torquescale]."""
    import mujoco
    b1, b2 = m.eq_obj1id[e], m.eq_obj2id[e]
    q1inv = np.zeros(4)
    mujoco.mju_negQuat(q1inv, d.xquat[b1])
    relpos = np.zeros(3)
    mujoco.mju_rotVecQuat(relpos, d.xpos[b2] - d.xpos[b1], q1inv)
    relquat = np.zeros(4)
    mujoco.mju_mulQuat(relquat, q1inv, d.xquat[b2])
    m.eq_data[e, 0:3] = 0.0
    m.eq_data[e, 3:6] = relpos
    m.eq_data[e, 6:10] = relquat
    d.eq_active[e] = 1


def check_start(xml: str, meshes, robot_bodies: set[str]) -> None:
    """Fail on objects sunk more than PENETRATION into the robot, the floor or each other at home.
    robot_bodies holds every robot body plus "world"; contacts between two of those are skipped."""
    import mujoco
    m = mujoco.MjModel.from_xml_string(xml, meshes or {})
    d = mujoco.MjData(m)
    for j in range(m.njnt):  # home: 0 clamped into the limits, like the hold check
        if m.jnt_limited[j] and m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            d.qpos[m.jnt_qposadr[j]] = min(max(0.0, m.jnt_range[j][0]), m.jnt_range[j][1])
    mujoco.mj_forward(m, d)
    for i in range(d.ncon):
        c = d.contact[i]
        if c.dist >= -PENETRATION:
            continue
        a, b = (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "world" for g in (c.geom1, c.geom2))
        if a in robot_bodies and b in robot_bodies:
            continue  # built into the robot (or the robot on the floor)
        a, b = sorted((a, b))
        raise SceneError(f"'{a}' and '{b}' overlap by {-c.dist * 1000:.0f} mm at the start; move one of them")


def meta(robot_mjcf: str, objs) -> dict:
    """What error messages need: motor limits per joint, masses of movable objects, fixed names."""
    root = ET.fromstring(robot_mjcf)
    efforts = {a.get("joint"): float(a.get("forcerange").split()[1])
               for a in root.iter("position") if a.get("forcerange")}
    return {"efforts": efforts, "masses": {o["name"]: o["mass"] for o in objs if not o["fixed"]},
            "fixed": [o["name"] for o in objs if o["fixed"]]}
