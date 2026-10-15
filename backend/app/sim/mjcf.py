"""URDF -> MJCF with MuJoCo's own URDF importer, plus position motors."""
import os
import re
import tempfile
import threading
import xml.etree.ElementTree as ET

import mujoco

TIMESTEP = 0.002
HOLD_ERROR = 0.05  # rad (or m) of error at which a motor gives full torque: kp = E / HOLD_ERROR
STIFF = 0.1  # kp <= STIFF * inertia / timestep²: a stiffer motor on a light link oscillates instead of holding
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


def build_mjcf(urdf: str, meshes: dict[str, bytes], efforts: dict[str, float] | None) -> str:
    """MJCF text for our URDF. `meshes` maps "<link>.stl" to bytes; `efforts` maps URDF joint names to
    motor strength (None = no motors). Meshes are referenced from ../meshes, next to urdf/."""
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
                kp = min(e / HOLD_ERROR, STIFF * inertia / TIMESTEP ** 2)
                ET.SubElement(act, "position", {
                    "name": f"{name}_motor", "joint": name, "kp": repr(kp), "dampratio": "1",
                    "forcelimited": "true", "forcerange": f"{-e:g} {e:g}"})
    _exclude_touching_at_home(root, model)
    return ET.tostring(root, encoding="unicode")
