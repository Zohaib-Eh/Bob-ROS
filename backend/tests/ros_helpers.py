"""Small robots exported through the real exporter, for app.ros tests."""
import zipfile
from pathlib import Path

import trimesh

from app.export.urdf_writer import export_package
from app.models import Body, Joint, Link


def _link(id_):
    return Link(id=id_, name=id_, body_ids=[f"{id_}_body"], density_kg_m3=1000.0)


def _body(id_, center, size=0.1):
    box = trimesh.creation.box(extents=(size, size, size))
    box.apply_translation(center)
    return Body(id=f"{id_}_body", name=id_, vertices=[tuple(v) for v in box.vertices],
                triangles=[tuple(t) for t in box.faces], faces=[])


def _joint(id_, parent, child, xyz, axis, limit=1.5):
    return Joint(id=id_, parent_link_id=parent, child_link_id=child, type="revolute",
                 origin_xyz=xyz, origin_rpy=(0.0, 0.0, 0.0), axis=axis,
                 limit_lower=-limit, limit_upper=limit, source="confirmed")


def two_joint_zip(tmp_path: Path, name: str = "bot") -> Path:
    """base -> arm (revolute about z at (0,0,0.2)) -> hand (revolute about y at (0.5,0,0.2)).
    Joints export as arm_joint and hand_joint, limits ±1.5 rad."""
    links = [_link("base"), _link("arm"), _link("hand")]
    joints = [_joint("j1", "base", "arm", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0)),
              _joint("j2", "arm", "hand", (0.5, 0.0, 0.2), (0.0, 1.0, 0.0))]
    bodies = {"base": [_body("base", (0.0, 0.0, 0.0))],
              "arm": [_body("arm", (0.25, 0.0, 0.2))],
              "hand": [_body("hand", (0.7, 0.0, 0.2))]}
    path = tmp_path / f"{name}.zip"
    path.write_bytes(export_package(name, links, joints, bodies))
    return path


def _boxes_body(id_, boxes, R=None, t=(0.0, 0.0, 0.0)):
    """One body from touching axis-aligned boxes [(min, max), ...], then rotated by R and moved by t."""
    import numpy as np
    parts = []
    for lo, hi in boxes:
        b = trimesh.creation.box(extents=np.subtract(hi, lo))
        b.apply_translation(np.add(lo, hi) / 2)
        parts.append(b)
    m = trimesh.util.concatenate(parts)
    v = m.vertices @ (np.eye(3) if R is None else R).T + t
    return Body(id=f"{id_}_body", name=id_, vertices=[tuple(p) for p in v], triangles=[tuple(f) for f in m.faces], faces=[])


def l_finger_body(id_, side=-1.0, R=None, t=(0.0, 0.0, 0.0)):
    """The fixture's L-shaped finger as ONE watertight solid (an L outline in x-z extruded along y,
    y ±0.01): upright x 0.025..0.035 from the centre, hook at the bottom reaching in to 0.021.
    side -1 = the finger on -x, +1 = its mirror on +x."""
    import numpy as np
    xz = [(0.035, -0.06), (0.021, -0.06), (0.021, -0.052), (0.025, -0.052), (0.025, 0.0), (0.035, 0.0)]
    ring = [(-side * -x if side > 0 else -x, z) for x, z in xz]   # x mirrored onto the chosen side
    v = np.array([(x, y, z) for y in (-0.01, 0.01) for x, z in ring])
    n = len(ring)
    faces = [f for k in (1, 2, 3, 4) for f in ((0, k + 1, k), (n, n + k, n + k + 1))]   # caps: fan from the corner
    faces += [f for i in range(n) for f in ((i, (i + 1) % n, n + (i + 1) % n), (i, n + (i + 1) % n, n + i))]
    m = trimesh.Trimesh(v, faces, process=True)
    if m.volume < 0:   # windings are consistent; flip all if the ring ran the other way (mirrored side)
        m.invert()
    pts = m.vertices @ (np.eye(3) if R is None else R).T + t
    return Body(id=f"{id_}_body", name=id_, vertices=[tuple(p) for p in pts], triangles=[tuple(f) for f in m.faces], faces=[])


def six_axis_with_gripper(name: str = "sixg"):
    """six_axis_zip's arm with the test gripper (tests/conftest.gripper_shapes: base plate, two L-fingers)
    mounted on l6's far end (x = 0.78): plate top face on the flange, fingers pointing along +x.
    Returns (links, joints, bodies_by_link, Gripper). The follower's axis is given outward with negative
    limits, as a user might, so export has to turn it inward."""
    import numpy as np
    from app.models import Gripper
    names = ["base", "l1", "l2", "l3", "l4", "l5", "l6"]
    joints = [_joint("j1", "base", "l1", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0), 3.0),
              _joint("j2", "l1", "l2", (0.1, 0.0, 0.4), (0.0, 1.0, 0.0), 3.0),
              _joint("j3", "l2", "l3", (0.1, 0.0, 0.9), (0.0, 1.0, 0.0), 3.0),
              _joint("j4", "l3", "l4", (0.3, 0.0, 0.95), (1.0, 0.0, 0.0), 3.0),
              _joint("j5", "l4", "l5", (0.6, 0.0, 0.95), (0.0, 1.0, 0.0), 3.0),
              _joint("j6", "l5", "l6", (0.7, 0.0, 0.95), (1.0, 0.0, 0.0), 3.0)]
    centers = [(0.0, 0.0, 0.1), (0.05, 0.0, 0.3), (0.1, 0.0, 0.65), (0.2, 0.0, 0.93),
               (0.45, 0.0, 0.95), (0.6, 0.0, 0.95), (0.75, 0.0, 0.95)]
    bodies = {n: [_body(n, c, size=0.06)] for n, c in zip(names, centers)}
    R = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])   # local z -> world -x, local x -> world +z
    t = np.array([0.78, 0.0, 0.95]) - R @ [0.0, 0.0, 0.01]                 # plate top centre onto the flange
    bodies["gbase"] = [_boxes_body("gbase", [((-0.04, -0.02, 0.0), (0.04, 0.02, 0.01))], R, t)]
    bodies["gfl"] = [l_finger_body("gfl", -1.0, R, t)]
    bodies["gfr"] = [l_finger_body("gfr", 1.0, R, t)]
    links = [_link(n) for n in names + ["gbase", "gfl", "gfr"]]
    up = tuple(float(x) for x in R @ [1.0, 0.0, 0.0])                       # world +z: gfl closes along it
    mid = tuple(float(x) for x in R @ [0.0, 0.0, -0.03] + t)
    slide = lambda id_, child, axis, lo, hi: Joint(
        id=id_, parent_link_id="gbase", child_link_id=child, type="prismatic", origin_xyz=mid,
        origin_rpy=(0.0, 0.0, 0.0), axis=axis, limit_lower=lo, limit_upper=hi, source="confirmed", max_effort=40.0)
    joints += [Joint(id="gfix", parent_link_id="l6", child_link_id="gbase", type="fixed",
                     origin_xyz=(0.78, 0.0, 0.95), origin_rpy=(0.0, 0.0, 0.0), source="confirmed"),
               slide("finger_l", "gfl", up, 0.0, 0.025),
               slide("finger_r", "gfr", up, -0.025, 0.0)]          # outward axis, negative travel
    return links, joints, bodies, Gripper(leader_joint_id="finger_l", follower_joint_id="finger_r")


def six_axis_with_gripper_zip(tmp_path: Path, name: str = "sixg") -> Path:
    links, joints, bodies, g = six_axis_with_gripper(name)
    path = tmp_path / f"{name}.zip"
    path.write_bytes(export_package(name, links, joints, bodies, gripper=g))
    return path


def six_axis_zip(tmp_path: Path, name: str = "six", inward_last: bool = False) -> Path:
    """Industrial-style 6-axis arm with an exact spherical wrist (joints 4-6 meet at (0.6, 0, 0.95)),
    limits ±3 rad. EAIK solves it in closed form; scripts/ros_check.sh uses it to prove that path.
    inward_last: the last joint's axis points back into the arm, as CAD exports sometimes give it."""
    names = ["base", "l1", "l2", "l3", "l4", "l5", "l6"]
    joints = [_joint("j1", "base", "l1", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0), 3.0),
              _joint("j2", "l1", "l2", (0.1, 0.0, 0.4), (0.0, 1.0, 0.0), 3.0),
              _joint("j3", "l2", "l3", (0.1, 0.0, 0.9), (0.0, 1.0, 0.0), 3.0),
              _joint("j4", "l3", "l4", (0.3, 0.0, 0.95), (1.0, 0.0, 0.0), 3.0),
              _joint("j5", "l4", "l5", (0.6, 0.0, 0.95), (0.0, 1.0, 0.0), 3.0),
              _joint("j6", "l5", "l6", (0.7, 0.0, 0.95), (-1.0 if inward_last else 1.0, 0.0, 0.0), 3.0)]
    centers = [(0.0, 0.0, 0.1), (0.05, 0.0, 0.3), (0.1, 0.0, 0.65), (0.2, 0.0, 0.93),
               (0.45, 0.0, 0.95), (0.6, 0.0, 0.95), (0.75, 0.0, 0.95)]
    bodies = {n: [_body(n, c, size=0.06)] for n, c in zip(names, centers)}
    path = tmp_path / f"{name}.zip"
    path.write_bytes(export_package(name, [_link(n) for n in names], joints, bodies))
    return path


def four_axis_zip(tmp_path: Path, name: str = "four") -> Path:
    """A 4-joint arm (the six-axis arm without its last two wrist joints): it can reach positions but
    cannot hold arbitrary tool orientations. Limits ±3 rad; reaches along +x at home. The last body ends at the tool frame (the flange face)."""
    names = ["base", "l1", "l2", "l3", "l4"]
    joints = [_joint("j1", "base", "l1", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0), 3.0),
              _joint("j2", "l1", "l2", (0.1, 0.0, 0.4), (0.0, 1.0, 0.0), 3.0),
              _joint("j3", "l2", "l3", (0.1, 0.0, 0.9), (0.0, 1.0, 0.0), 3.0),
              _joint("j4", "l3", "l4", (0.3, 0.0, 0.95), (1.0, 0.0, 0.0), 3.0)]
    centers = [(0.0, 0.0, 0.1), (0.05, 0.0, 0.3), (0.1, 0.0, 0.65), (0.2, 0.0, 0.93), (0.27, 0.0, 0.95)]
    bodies = {n: [_body(n, c, size=0.06)] for n, c in zip(names, centers)}
    path = tmp_path / f"{name}.zip"
    path.write_bytes(export_package(name, [_link(n) for n in names], joints, bodies))
    return path


def unzip(zip_path: Path, dest: Path) -> Path:
    with zipfile.ZipFile(zip_path) as z:
        z.extractall(dest)
    return Path(dest)
