"""URDF -> MJCF with MuJoCo's own URDF importer, plus position motors."""
import os
import re
import tempfile
import threading
import xml.etree.ElementTree as ET

import mujoco

TIMESTEP = 0.002
HOLD_ERROR = 0.01  # rad (or m) of error at which a motor gives full torque: kp = E / HOLD_ERROR
# (a proportional servo trails a moving target in proportion to speed; 0.05 let a weak wrist trail
# 0.075 rad on a slow descent, a real drive tracks far tighter)
STIFF = 0.1  # kp <= STIFF * inertia / timestep²: a stiffer motor on a light link oscillates instead of holding
GRIP_ERROR = 0.002          # m short of target at which a finger motor pushes with its full grip force
GRIP_FRICTION = "1 0.005 0.0001"
GRIP_SPEED = 0.1            # m/s: a real gripper's closing speed (~50-150 mm/s), set by finger damping
GRIP_SOLREF = "0.004 1"     # time constant 2 × timestep: the stiffest MuJoCo keeps stable
GRIP_SOLIMP = "0.99 0.999 0.0001"   # near-rigid pads and coupling (defaults 0.9 0.95 0.001)
WHEEL_FRICTION = "1 0.005 0.0001"   # rubber on a floor
WHEEL_SLIP = 0.5            # rad/s short of its target at which a wheel motor pushes with full torque
LIFT = 0.0005               # m: the wheels start this far above the floor, settling onto it
BASE_JOINTS = [("base_x", "slide", "1 0 0"), ("base_y", "slide", "0 1 0"), ("base_z", "slide", "0 0 1"),
               ("base_yaw", "hinge", "0 0 1"), ("base_pitch", "hinge", "0 1 0"), ("base_roll", "hinge", "1 0 0")]
_last_xml = threading.Lock()  # mj_saveLastXML saves MuJoCo's process-wide "last loaded model"


def _urdf_for_mujoco(urdf: str) -> str:
    """Bare mesh names (assets are passed by name), keep every link as a body, keep inertias as given."""
    urdf = re.sub(r'filename="package://[^"]*/meshes/([^"]+)"', r'filename="\1"', urdf)
    urdf = re.sub(r"<gripper\b[^>]*/>", "", urdf)   # our extension tag, not MuJoCo's
    ext = ('<mujoco><compiler strippath="true" fusestatic="false" discardvisual="true" '
           'balanceinertia="false"/></mujoco>')
    return re.sub(r"(<robot[^>]*>)", lambda m: m.group(1) + ext, urdf, count=1)


def compile_mjcf(xml: str, meshes: dict[str, bytes]) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(xml.replace('meshdir="../meshes"', 'meshdir=""'), meshes)


def _exclude_touching_at_home(root, model: mujoco.MjModel) -> None:
    """Bodies whose collision shapes already touch at home pose never collide (MuJoCo already skips
    parent/child pairs), so no hold check fails on overlap that is built into the robot."""
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    pairs = set()
    for i in range(d.ncon):
        c = d.contact[i]
        a, b = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]) for g in (c.geom1, c.geom2))
        if a and b and a != b:
            pairs.add(tuple(sorted((a, b))))
    if pairs:
        contact = root.find("contact")
        if contact is None:
            contact = ET.SubElement(root, "contact")
        for a, b in sorted(pairs):
            ET.SubElement(contact, "exclude", {"body1": a, "body2": b})


def _mobile(root, mobile: dict, efforts: dict[str, float] | None) -> None:
    """The base free on the floor: six scalar joints, not a free joint, so the arm's gravity compensation can be
    kept inside the robot (a free joint takes its share as an outside force: the floor carried 98 of 147 N);
    wheels roll on fitted cylinders with velocity motors; casters are frictionless spheres; the base's own
    shapes (contype 2) hit objects but not the floor."""
    world = root.find("worldbody")
    base = world.find(f"body[@name='{mobile['base']}']")
    base.set("pos", f"0 0 {-mobile['ground'] + LIFT!r}")
    base.set("gravcomp", "0")
    at = 1 if base.find("inertial") is not None else 0
    for k, (name, kind, axis) in enumerate(BASE_JOINTS):
        base.insert(at + k, ET.Element("joint", {
            "name": name, "type": kind, "axis": axis, "limited": "false", "damping": "0",
            "actuatorgravcomp": "true", "actuatorfrclimited": "true", "actuatorfrcrange": "-1e-9 1e-9"}))
    for g in base.findall("geom"):
        g.set("contype", "2")
        g.set("conaffinity", "2")
    for i, (x, y, z, r) in enumerate(mobile["casters"]):
        ET.SubElement(base, "geom", {"name": f"caster_{i}", "type": "sphere", "size": repr(r), "pos": f"{x!r} {y!r} {z!r}",
                                     # priority: its condim wins over the floor's (MuJoCo takes the larger otherwise)
                                     "condim": "1", "priority": "1", "rgba": "0.3 0.3 0.3 1"})
    act = root.find("actuator")
    if act is None:
        act = ET.SubElement(root, "actuator")
    for joint, w in mobile["wheels"].items():
        body = next(b for b in base.iter("body") if b.get("name") == w["link"])
        body.set("gravcomp", "0")
        for g in body.findall("geom"):
            g.set("contype", "2")
            g.set("conaffinity", "2")
        c = [a - o for a, o in zip(w["center"], w["origin"])]   # the body's frame is the joint's, world-aligned
        ET.SubElement(body, "geom", {"name": f"{w['link']}_tire", "type": "cylinder", "size": f"{w['radius']!r} {w['width'] / 2!r}",
                                     "pos": " ".join(repr(v) for v in c), "zaxis": " ".join(repr(float(v)) for v in w["axis"]),
                                     "friction": WHEEL_FRICTION, "rgba": "0.1 0.1 0.1 1"})
        for a in list(act):
            if a.get("joint") == joint:
                act.remove(a)
        if efforts and joint in efforts:
            e = float(efforts[joint])
            ET.SubElement(act, "velocity", {"name": f"{joint}_motor", "joint": joint, "kv": repr(e / WHEEL_SLIP),
                                            "forcelimited": "true", "forcerange": f"{-e:g} {e:g}"})
        for j in body.findall("joint"):
            j.attrib.pop("actuatorgravcomp", None)


def build_mjcf(urdf: str, meshes: dict[str, bytes], efforts: dict[str, float] | None, grip: dict | None = None,
               mobile: dict | None = None) -> str:
    """MJCF text for our URDF. `meshes` maps "<link>.stl" to bytes; `efforts` maps URDF joint names to
    motor strength (None = no motors). Meshes are referenced from ../meshes, next to urdf/.
    `grip` = {"links": [base, finger, finger], "fingers": [leader, follower], "follower": follower}
    (link and joint names): the finger motor reaches its full
    grip force GRIP_ERROR short of its target (a gripper squeezes with its set force, it isn't a soft
    spring), the gripper's geoms get GRIP_FRICTION, and the follower's mimic equality is made stiff."""
    fd, path = tempfile.mkstemp(suffix=".xml")
    os.close(fd)
    try:
        with _last_xml:
            model = mujoco.MjModel.from_xml_string(_urdf_for_mujoco(urdf), meshes)
            mujoco.mj_saveLastXML(path, model)
        root = ET.parse(path).getroot()
    finally:
        os.unlink(path)
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", "../meshes")
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(1, option)
    option.set("timestep", str(TIMESTEP))
    if efforts:
        act = root.find("actuator")
        if act is None:
            act = ET.SubElement(root, "actuator")
        for j in root.iter("joint"):
            name = j.get("name")
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) if name else -1
            if name in efforts and jid >= 0:
                e = float(efforts[name])
                inertia = float(model.dof_M0[model.jnt_dofadr[jid]])
                finger = grip is not None and name in grip["fingers"]   # not by link: the base may be the arm's last link
                kp = e / GRIP_ERROR if finger else min(e / HOLD_ERROR, STIFF * inertia / TIMESTEP ** 2)
                if finger:   # a geared finger drive's reflected inertia: what keeps this stiff motor stable
                    finger_armature = max(0.0, kp * TIMESTEP ** 2 / STIFF - inertia)
                    for f in root.iter("joint"):
                        if f.get("name") in (name, grip["follower"]):
                            f.set("armature", repr(finger_armature))
                            # leader and follower share it: full force against both dampers is GRIP_SPEED;
                            # damping only resists motion, so the squeeze at rest keeps its full force
                            f.set("damping", repr(e / (2 * GRIP_SPEED)))
                ET.SubElement(act, "position", {
                    "name": f"{name}_motor", "joint": name, "kp": repr(kp), "dampratio": "1",
                    "forcelimited": "true", "forcerange": f"{-e:g} {e:g}"})
                # Gravity fed forward through the motor, as a real arm controller does, and
                # capped with it: no droop at rest, but a too-weak motor or a payload still sags.
                j.set("actuatorgravcomp", "true")
                j.set("actuatorfrcrange", f"{-e:g} {e:g}")
        for b in root.find("worldbody").iter("body"):
            b.set("gravcomp", "1")
    if grip is not None:
        for b in root.find("worldbody").iter("body"):
            if b.get("name") in grip["links"]:
                for g in b.findall("geom"):   # hard pads: MuJoCo's default contact lets 40 N sink a pad mm deep
                    g.set("friction", GRIP_FRICTION)
                    g.set("solref", GRIP_SOLREF)
                    g.set("solimp", GRIP_SOLIMP)
        for eq in root.iter("joint"):
            if eq.get("joint1") == grip["follower"] or eq.get("joint2") == grip["follower"]:
                eq.set("solref", GRIP_SOLREF)   # an <equality><joint>; MuJoCo's default gives way under grip force
                eq.set("solimp", GRIP_SOLIMP)
    if mobile is not None:
        _mobile(root, mobile, efforts)
    _exclude_touching_at_home(root, model)
    return ET.tostring(root, encoding="unicode")
