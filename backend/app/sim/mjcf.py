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
GRIP_SOLREF = "0.004 1"     # time constant 2 × timestep: the stiffest MuJoCo keeps stable
GRIP_SOLIMP = "0.99 0.999 0.0001"   # near-rigid pads and coupling (defaults 0.9 0.95 0.001)
_last_xml = threading.Lock()  # mj_saveLastXML saves MuJoCo's process-wide "last loaded model"


def _urdf_for_mujoco(urdf: str) -> str:
    """Bare mesh names (assets are passed by name), keep every link as a body, keep inertias as given."""
    urdf = re.sub(r'filename="package://[^"]*/meshes/([^"]+)"', r'filename="\1"', urdf)
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


def build_mjcf(urdf: str, meshes: dict[str, bytes], efforts: dict[str, float] | None, grip: dict | None = None) -> str:
    """MJCF text for our URDF. `meshes` maps "<link>.stl" to bytes; `efforts` maps URDF joint names to
    motor strength (None = no motors). Meshes are referenced from ../meshes, next to urdf/.
    `grip` = {"links": [base, finger, finger], "follower": joint}: the finger motor reaches its full
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
                finger = grip is not None and model.body(model.jnt_bodyid[jid]).name in grip["links"]
                kp = e / GRIP_ERROR if finger else min(e / HOLD_ERROR, STIFF * inertia / TIMESTEP ** 2)
                if finger:   # a geared finger drive's reflected inertia: what keeps this stiff motor stable
                    finger_armature = max(0.0, kp * TIMESTEP ** 2 / STIFF - inertia)
                    for f in root.iter("joint"):
                        if f.get("name") in (name, grip["follower"]):
                            f.set("armature", repr(finger_armature))
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
    _exclude_touching_at_home(root, model)
    return ET.tostring(root, encoding="unicode")
