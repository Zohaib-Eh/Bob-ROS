"""The physics check: hold home pose with the motors for a couple of seconds."""
import io
import math
import zipfile

import mujoco
import numpy as np

from app.sim.mjcf import build_mjcf, compile_mjcf
from app.sim.torque import required_efforts

SAG_HINGE = math.radians(5)
SAG_SLIDE = 0.005


def _name(m, kind, i) -> str:
    return mujoco.mj_id2name(m, kind, i) or f"#{i}"


def _blew_up(d) -> bool:
    return (not (np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel)))
            or d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > 0)


def hold_check(xml: str, meshes: dict[str, bytes], seconds: float = 2.0) -> dict:
    """Hold home pose (0, clamped into the limits) with the motors for `seconds` of simulated time."""
    try:
        m = compile_mjcf(xml, meshes)
    except ValueError as exc:  # MuJoCo reports compile errors as ValueError
        return {"ok": False, "kind": "load", "message": f"Model won't load: {exc}"}
    d = mujoco.MjData(m)
    home = np.zeros(m.nq)
    for j in range(m.njnt):
        if m.jnt_limited[j]:
            a = m.jnt_qposadr[j]
            home[a] = min(max(0.0, m.jnt_range[j][0]), m.jnt_range[j][1])
    d.qpos[:] = home
    for a in range(m.nu):
        d.ctrl[a] = home[m.jnt_qposadr[m.actuator_trnid[a][0]]]
    for step in range(int(round(seconds / m.opt.timestep))):
        mujoco.mj_step(m, d)
        if _blew_up(d):
            speeds = [float(np.linalg.norm(d.cvel[b])) if np.all(np.isfinite(d.cvel[b])) else math.inf
                      for b in range(1, m.nbody)]
            link = _name(m, mujoco.mjtObj.mjOBJ_BODY, 1 + int(np.argmax(speeds))) if speeds else "the robot"
            t = round((step + 1) * m.opt.timestep, 2)
            return {"ok": False, "kind": "unstable", "link": link,
                    "message": f"Simulation blew up after {t} s: check {link}'s mass and inertia."}
    worst = None
    for j in range(m.njnt):
        kind = m.jnt_type[j]
        if kind not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        a = m.jnt_qposadr[j]
        dev = abs(float(d.qpos[a] - home[a]))
        limit = SAG_HINGE if kind == mujoco.mjtJoint.mjJNT_HINGE else SAG_SLIDE
        if dev > limit and (worst is None or dev / limit > worst[1] / worst[2]):
            worst = (j, dev, limit)
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


def physics_check(robot_name, links, joints, bodies_by_link) -> dict:
    """Runs in a child process: export (which builds the MJCF), then hold."""
    from app.export.urdf_writer import export_package
    try:
        data = export_package(robot_name, links, joints, bodies_by_link)
    except ValueError as exc:
        return {"ok": False, "kind": "load", "message": str(exc)}
    z = zipfile.ZipFile(io.BytesIO(data))
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if "/meshes/" in n}
    path = f"{robot_name}/mjcf/{robot_name}.xml"
    if path not in z.namelist():  # MuJoCo couldn't load it at export: rebuild to get its reason
        try:
            build_mjcf(z.read(f"{robot_name}/urdf/{robot_name}.urdf").decode(), meshes, None)
        except ValueError as exc:
            return {"ok": False, "kind": "load", "message": f"Model won't load: {exc}"}
        return {"ok": False, "kind": "load", "message": "Model won't load."}
    return hold_check(z.read(path).decode(), meshes)
