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
