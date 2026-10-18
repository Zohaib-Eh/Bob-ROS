"""The physics check: hold home pose with the motors for a couple of seconds; a mobile base also stands still
on the floor and drives straight."""
import io
import math
import xml.etree.ElementTree as ET
import zipfile

import mujoco
import numpy as np

from app.sim.mjcf import BASE_JOINTS, build_mjcf, compile_mjcf
from app.sim.torque import required_efforts

SAG_HINGE = math.radians(5)
SAG_SLIDE = 0.005
KICK_HINGE = 0.01   # rad: start offset from home, well under SAG_HINGE
KICK_SLIDE = 0.0005  # m
BASE_DRIFT = 0.005   # m a standing base may roll
BASE_TILT = math.radians(1)
DRIVE_SPEED, DRIVE_TIME = 0.2, 2.0   # m/s for s: the drive check
DRIVE_HEADING = math.radians(3)
_BASE = {n for n, _, _ in BASE_JOINTS}


def _name(m, kind, i) -> str:
    return mujoco.mj_id2name(m, kind, i) or f"#{i}"


_BAD = (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQPOS)


def _blow_up_link(m, d) -> str | None:
    """MuJoCo resets the state itself when it goes bad and records which dof (or qpos) did it."""
    for w in _BAD:
        if d.warning[w].number > 0:
            i = int(d.warning[w].lastinfo)
            if w == mujoco.mjtWarning.mjWARN_BADQPOS:
                joint = max((j for j in range(m.njnt) if m.jnt_qposadr[j] <= i), default=0,
                            key=lambda j: m.jnt_qposadr[j])
            else:
                joint = int(m.dof_jntid[min(i, m.nv - 1)]) if m.nv else 0
            return _name(m, mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[joint]) if m.njnt else "the robot"
    return None


def _with_floor(xml: str) -> str:
    root = ET.fromstring(xml)
    ET.SubElement(root.find("worldbody"), "geom", {"name": "floor", "type": "plane", "size": "0 0 1"})
    return ET.tostring(root, encoding="unicode")


def _q(m, d, name) -> float:
    return float(d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]])


def _drive_check(m, d, mobile) -> dict | None:
    """Wheels at DRIVE_SPEED for DRIVE_TIME: the base should go DRIVE_SPEED * DRIVE_TIME straight ahead."""
    r = mobile["radius"]
    for a in range(m.nu):
        name = _name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0])
        if name in mobile["left"] + mobile["right"]:
            d.ctrl[a] = DRIVE_SPEED / r
    x0, y0, yaw0 = _q(m, d, "base_x"), _q(m, d, "base_y"), _q(m, d, "base_yaw")
    for _ in range(int(round(DRIVE_TIME / m.opt.timestep))):
        mujoco.mj_step(m, d)
    moved = math.hypot(_q(m, d, "base_x") - x0, _q(m, d, "base_y") - y0)
    turned = abs(_q(m, d, "base_yaw") - yaw0)
    want = DRIVE_SPEED * DRIVE_TIME
    if abs(moved - want) > 0.1 * want:
        return {"ok": False, "kind": "drive", "moved": round(moved, 3),
                "message": f"The base drove {moved:.2f} m of {want:.2f} m: the wheels slip or their motors are too weak "
                           "(set a higher wheel strength), or something drags on the floor."}
    if turned > DRIVE_HEADING:
        return {"ok": False, "kind": "drive", "message": f"The base veers {math.degrees(turned):.0f}° in {want:.1f} m: "
                                                         "the wheels differ in size or grip, or the weight sits to one side."}
    return None


def hold_check(xml: str, meshes: dict[str, bytes], seconds: float = 2.0, mobile: dict | None = None) -> dict:
    """Hold home pose (0, clamped into the limits) with the motors for `seconds` of simulated time.
    mobile ({left, right, radius}): the robot stands on a floor, must not roll or tip, then drives straight."""
    try:
        m = compile_mjcf(_with_floor(xml) if mobile else xml, meshes)
    except (ValueError, mujoco.FatalError) as exc:  # MuJoCo reports compile errors as ValueError
        return {"ok": False, "kind": "load", "message": f"Model won't load: {exc}"}
    d = mujoco.MjData(m)
    home = np.zeros(m.nq)
    for j in range(m.njnt):
        if m.jnt_limited[j]:
            a = m.jnt_qposadr[j]
            home[a] = min(max(0.0, m.jnt_range[j][0]), m.jnt_range[j][1])
    for a in range(m.nu):
        d.ctrl[a] = home[m.jnt_qposadr[m.actuator_trnid[a][0]]]
    # Start slightly off home: with gravity fed forward an exact home pose is a perfect balance,
    # which would hide a model that blows up on the first real motion.
    d.qpos[:] = home
    for j in range(m.njnt):
        kind = m.jnt_type[j]
        if kind in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE) and _name(m, mujoco.mjtObj.mjOBJ_JOINT, j) not in _BASE:
            a = m.jnt_qposadr[j]
            kick = KICK_HINGE if kind == mujoco.mjtJoint.mjJNT_HINGE else KICK_SLIDE
            hi = m.jnt_range[j][1] if m.jnt_limited[j] else math.inf
            d.qpos[a] = home[a] + kick if home[a] + kick <= hi else home[a] - kick
    for step in range(int(round(seconds / m.opt.timestep))):
        mujoco.mj_step(m, d)
        link = _blow_up_link(m, d)
        if link:
            t = round((step + 1) * m.opt.timestep, 3)
            return {"ok": False, "kind": "unstable", "link": link,
                    "message": f"Simulation blew up after {t:g} s: check {link}'s mass and inertia."}
    worst = None
    for j in range(m.njnt):
        kind = m.jnt_type[j]
        if kind not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        if mobile and _name(m, mujoco.mjtObj.mjOBJ_JOINT, j) in _BASE | set(mobile["left"] + mobile["right"]):
            continue   # the base and its wheels are judged below, not as a sagging arm
        a = m.jnt_qposadr[j]
        dev = abs(float(d.qpos[a] - home[a]))
        limit = SAG_HINGE if kind == mujoco.mjtJoint.mjJNT_HINGE else SAG_SLIDE
        if dev > limit and (worst is None or dev / limit > worst[1] / worst[2]):
            worst = (j, dev, limit)
    if worst is None and mobile:
        drift = math.hypot(_q(m, d, "base_x"), _q(m, d, "base_y"))
        tilt = max(abs(_q(m, d, "base_pitch")), abs(_q(m, d, "base_roll")))
        if tilt > BASE_TILT:
            return {"ok": False, "kind": "tip", "message": f"The base tips {math.degrees(tilt):.0f}° standing still: "
                                                           "add a caster or move the weight between the wheels."}
        if drift > BASE_DRIFT:
            return {"ok": False, "kind": "roll", "message": f"The base rolls {drift * 1000:.0f} mm standing still."}
        problem = _drive_check(m, d, mobile)
        if problem:
            return problem
        return {"ok": True, "kind": "ok", "message": f"Holds pose. Drives straight ({DRIVE_SPEED * DRIVE_TIME:.2f} m in {DRIVE_TIME:g} s)."}
    if worst is None:
        return {"ok": True, "kind": "ok", "message": "Holds pose."}
    j, dev, _ = worst
    joint = _name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    link = _name(m, mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[j])
    hinge = m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
    has = next((float(m.actuator_forcerange[a][1]) for a in range(m.nu) if m.actuator_trnid[a][0] == j), 0.0)
    needed = required_efforts(compile_mjcf(xml, meshes)).get(joint, 0.0)
    unit = "N·m" if hinge else "N"
    amount = f"{math.degrees(dev):.0f}°" if hinge else f"{dev * 1000:.0f} mm"
    return {"ok": False, "kind": "sag", "joint": joint, "link": link, "needed": round(needed, 3), "has": has,
            "message": f"{link} sags {amount}: needs ~{needed:.3g} {unit}, has {has:g}."}


def physics_check(robot_name, links, joints, bodies_by_link, gripper=None, mobile=None) -> dict:
    """Runs in a child process: export (which builds the MJCF), then hold (and drive)."""
    from app.export.urdf_writer import export_package
    try:
        data = export_package(robot_name, links, joints, bodies_by_link, gripper=gripper, mobile=mobile)
    except ValueError as exc:
        return {"ok": False, "kind": "load", "message": str(exc)}
    z = zipfile.ZipFile(io.BytesIO(data))
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if "/meshes/" in n}
    path = f"{robot_name}/mjcf/{robot_name}.xml"
    if path not in z.namelist():  # MuJoCo couldn't load it at export: rebuild to get its reason
        try:
            build_mjcf(z.read(f"{robot_name}/urdf/{robot_name}.urdf").decode(), meshes, None)
        except (ValueError, mujoco.FatalError) as exc:
            return {"ok": False, "kind": "load", "message": f"Model won't load: {exc}"}
        return {"ok": False, "kind": "load", "message": "Model won't load."}
    tag = ET.fromstring(z.read(f"{robot_name}/urdf/{robot_name}.urdf")).find("mobile_base")
    drive = None if tag is None else {"left": tag.get("left").split(), "right": tag.get("right").split(),
                                      "radius": float(tag.get("wheel_radius"))}
    return hold_check(z.read(path).decode(), meshes, mobile=drive)
