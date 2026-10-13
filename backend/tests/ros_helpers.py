"""Small robots exported through the real exporter, for app.ros tests."""
import zipfile
from pathlib import Path

import trimesh

from app.export.urdf_writer import export_package
from app.models import Body, Joint, Link


def _link(id_):
    return Link(id=id_, name=id_, body_ids=[f"{id_}_body"], density_kg_m3=1000.0)


def _body(id_, center):
    box = trimesh.creation.box(extents=(0.1, 0.1, 0.1))
    box.apply_translation(center)
    return Body(id=f"{id_}_body", name=id_, vertices=[tuple(v) for v in box.vertices],
                triangles=[tuple(t) for t in box.faces], faces=[])


def _joint(id_, parent, child, xyz, axis):
    return Joint(id=id_, parent_link_id=parent, child_link_id=child, type="revolute",
                 origin_xyz=xyz, origin_rpy=(0.0, 0.0, 0.0), axis=axis,
                 limit_lower=-1.5, limit_upper=1.5, source="confirmed")


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


def unzip(zip_path: Path, dest: Path) -> Path:
    with zipfile.ZipFile(zip_path) as z:
        z.extractall(dest)
    return Path(dest)
