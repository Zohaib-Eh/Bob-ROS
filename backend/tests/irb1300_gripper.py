"""Builds irb1300g.zip: the corpus irb1300 with the test gripper (tests/ros_helpers: base plate, two
L-fingers) mounted on link6's flange, for the container check (docker/jazzy/check/physics/irb1300g_*).

    python -m tests.irb1300_gripper <irb1300_asm.stp> <irb1300g.zip>             test gripper
    python -m tests.irb1300_gripper <irb1300_asm.stp> <irb1300h.zip> <hfk40.stp>  AirTAC HFK40 (GrabCAD)
    (from backend/, PYTHONPATH=.; the package is named after the zip)
"""
import sys
from pathlib import Path

import numpy as np
import yaml

from app.export import urdf_writer
from app.models import Gripper, Joint, Link
from corpus import run
from tests.ros_helpers import _boxes_body, l_finger_body


def irb1300_model(step: Path):
    """(links, joints, bodies_by_link) exactly as the corpus runner exports the irb1300."""
    captured = {}
    real = run.export_package

    def capture(name, links, joints, bodies):
        captured.update(links=links, joints=joints, bodies=bodies)
        return real(name, links, joints, bodies)
    run.export_package = capture
    try:
        exp = yaml.safe_load((run.HERE / "expected" / "irb1300.yaml").read_text())
        run.score_model(exp, step, robot_name="irb1300")
    finally:
        run.export_package = real
    return captured["links"], captured["joints"], captured["bodies"]


def mount_gripper(links, joints, bodies):
    """Adds the gripper on link6's flange: plate top face flush on the flange, fingers pointing out."""
    j6 = next(j for j in joints if j.child_link_id == "link6")
    a, p = np.asarray(j6.axis, float), np.asarray(j6.origin_xyz, float)
    v = np.array([q for b in bodies["link6"] for q in b.vertices])
    v5 = np.array([q for b in bodies["link5"] for q in b.vertices])
    out = a if (v.mean(axis=0) - v5.mean(axis=0)) @ a > 0 else -a   # away from the arm (link5)
    flange = p + out * float(((v - p) @ out).max())
    u = np.cross(out, [0.0, 0.0, 1.0])
    u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-6 else np.array([1.0, 0.0, 0.0])
    R = np.column_stack([u, np.cross(-out, u), -out])              # local z -> back at the flange
    t = flange - R @ [0.0, 0.0, 0.01]
    bodies = {**bodies,
              "gbase": [_boxes_body("gbase", [((-0.04, -0.02, 0.0), (0.04, 0.02, 0.01))], R, t)],
              "gfl": [l_finger_body("gfl", -1.0, R, t)], "gfr": [l_finger_body("gfr", 1.0, R, t)]}
    links = links + [Link(id=n, name=n, body_ids=[f"{n}_body"], density_kg_m3=1000.0) for n in ("gbase", "gfl", "gfr")]
    close = tuple(float(x) for x in R @ [1.0, 0.0, 0.0])
    mid = tuple(float(x) for x in R @ [0.0, 0.0, -0.03] + t)
    slide = lambda id_, child, lo, hi: Joint(
        id=id_, parent_link_id="gbase", child_link_id=child, type="prismatic", origin_xyz=mid,
        origin_rpy=(0.0, 0.0, 0.0), axis=close, limit_lower=lo, limit_upper=hi, source="confirmed", max_effort=40.0)
    joints = joints + [Joint(id="gfix", parent_link_id="link6", child_link_id="gbase", type="fixed",
                             origin_xyz=tuple(float(x) for x in flange), origin_rpy=(0.0, 0.0, 0.0), source="confirmed"),
                       slide("finger_l", "gfl", 0.0, 0.025), slide("finger_r", "gfr", -0.025, 0.0)]
    return links, joints, bodies, Gripper(leader_joint_id="finger_l", follower_joint_id="finger_r")


def _flange(joints, bodies):
    """(flange point, outward axis) of link6: away from the arm (link5)."""
    j6 = next(j for j in joints if j.child_link_id == "link6")
    a, p = np.asarray(j6.axis, float), np.asarray(j6.origin_xyz, float)
    v = np.array([q for b in bodies["link6"] for q in b.vertices])
    v5 = np.array([q for b in bodies["link5"] for q in b.vertices])
    out = a if (v.mean(axis=0) - v5.mean(axis=0)) @ a > 0 else -a
    return p + out * float(((v - p) @ out).max()), out


def mount_hfk(links, joints, bodies, step: Path):
    """The AirTAC HFK40 air gripper (GrabCAD STEP: body + two jaws, modelled closed) on link6's flange.
    In its file the back (mounting) face is x = 0, the jaws stick out along -x and close along y;
    datasheet: 15 mm stroke per jaw (30 mm closed, 60 mm open, outer width)."""
    from app.importers.step_importer import import_cad
    parts = {b.name: b for b in import_cad(str(step))[0] if b.status == "ok"}
    flange, out = _flange(joints, bodies)
    u = np.cross(out, [0.0, 0.0, 1.0])
    u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-6 else np.array([1.0, 0.0, 0.0])
    R = np.column_stack([-out, u, np.cross(-out, u)])     # local x -> back onto the flange, y -> u
    moved = lambda b, id_: b.model_copy(update={
        "id": f"{id_}_body", "name": id_, "faces": [],
        "vertices": [tuple(float(c) for c in R @ v + flange) for v in np.asarray(b.vertices, float)]})
    names = {"hfk_base": "HFZ40(CL)_body", "hfk_fl": "HFZ40(CL)_fl", "hfk_fr": "HFZ40(CL)_fr"}
    bodies = {**bodies, **{k: [moved(parts[v], k)] for k, v in names.items()}}
    links = links + [Link(id=n, name=n, body_ids=[f"{n}_body"], density_kg_m3=2700.0) for n in names]
    close = tuple(float(x) for x in R @ [0.0, 1.0, 0.0])   # +y: the -y jaw (fl) closes along it
    mid = tuple(float(x) for x in R @ [-0.114, 0.0, 0.0] + flange)
    slide = lambda id_, child, axis: Joint(
        id=id_, parent_link_id="hfk_base", child_link_id=child, type="prismatic", origin_xyz=mid,
        origin_rpy=(0.0, 0.0, 0.0), axis=axis, limit_lower=-0.015, limit_upper=0.0, source="confirmed", max_effort=40.0)
    joints = joints + [Joint(id="hfix", parent_link_id="link6", child_link_id="hfk_base", type="fixed",
                             origin_xyz=tuple(float(x) for x in flange), origin_rpy=(0.0, 0.0, 0.0), source="confirmed"),
                       slide("finger_l", "hfk_fl", close), slide("finger_r", "hfk_fr", tuple(-c for c in close))]
    return links, joints, bodies, Gripper(leader_joint_id="finger_l", follower_joint_id="finger_r")


if __name__ == "__main__":
    model = irb1300_model(Path(sys.argv[1]))
    links, joints, bodies, g = mount_hfk(*model, Path(sys.argv[3])) if len(sys.argv) > 3 else mount_gripper(*model)
    Path(sys.argv[2]).write_bytes(urdf_writer.export_package(Path(sys.argv[2]).stem, links, joints, bodies, gripper=g))
    print("wrote", sys.argv[2])
