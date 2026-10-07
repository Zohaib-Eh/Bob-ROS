import io
import re
import zipfile
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape
from xml.dom import minidom

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from app.models import Body, Joint, Link
from app.validate.structural import validate_structure


def _fmt(v) -> str:
    return " ".join(repr(float(x)) for x in v)


def _pose(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


def _xyz_rpy(T: np.ndarray) -> tuple[str, str]:
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz")
    return _fmt(T[:3, 3]), _fmt(rpy)


def _link_frames(links: list[Link], joints: list[Joint]) -> dict[str, np.ndarray]:
    """World pose of each link frame: its parent joint's pose (root = identity)."""
    return {
        **{l.id: np.eye(4) for l in links},
        **{j.child_link_id: _pose(j.origin_xyz, j.origin_rpy) for j in joints},
    }


def _mesh_ref(robot_name: str, link: Link) -> str:
    return f"package://{robot_name}/meshes/{link.name}.stl"


def _add_link_element(robot, robot_name: str, link: Link, T: np.ndarray) -> None:
    link_el = ET.SubElement(robot, "link", {"name": link.name})
    inv = np.linalg.inv(T)
    R = T[:3, :3]

    # Inertial origin is the COM (never the joint origin, spec §8), in link axes.
    com = np.array(link.center_of_mass or (0.0, 0.0, 0.0))
    inertial_el = ET.SubElement(link_el, "inertial")
    ET.SubElement(inertial_el, "origin", {"xyz": _fmt((inv @ [*com, 1.0])[:3]), "rpy": "0 0 0"})
    ET.SubElement(inertial_el, "mass", {"value": str(link.mass_kg or 0.0)})
    ixx, iyy, izz, ixy, ixz, iyz = link.inertia or [0.0] * 6
    I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    I = R.T @ I @ R  # world axes -> link axes
    ET.SubElement(inertial_el, "inertia", {
        "ixx": repr(float(I[0, 0])), "iyy": repr(float(I[1, 1])), "izz": repr(float(I[2, 2])),
        "ixy": repr(float(I[0, 1])), "ixz": repr(float(I[0, 2])), "iyz": repr(float(I[1, 2])),
    })

    # Meshes are in world coordinates, so geometry origin = inverse(link frame).
    xyz, rpy = _xyz_rpy(inv)
    for tag in ("visual", "collision"):
        geom_el = ET.SubElement(link_el, tag)
        ET.SubElement(geom_el, "origin", {"xyz": xyz, "rpy": rpy})
        geometry = ET.SubElement(geom_el, "geometry")
        ET.SubElement(geometry, "mesh", {"filename": _mesh_ref(robot_name, link)})


def _add_joint_element(robot, joint: Joint, links_by_id, frames) -> None:
    joint_el = ET.SubElement(robot, "joint", {"name": f"joint_{joint.id}", "type": joint.type})
    ET.SubElement(joint_el, "parent", {"link": links_by_id[joint.parent_link_id].name})
    ET.SubElement(joint_el, "child", {"link": links_by_id[joint.child_link_id].name})
    rel = np.linalg.inv(frames[joint.parent_link_id]) @ frames[joint.child_link_id]
    xyz, rpy = _xyz_rpy(rel)
    ET.SubElement(joint_el, "origin", {"xyz": xyz, "rpy": rpy})
    if joint.axis is not None:
        # URDF reads the axis in the joint (child link) frame; ours is world.
        ax = frames[joint.child_link_id][:3, :3].T @ np.array(joint.axis)
        ET.SubElement(joint_el, "axis", {"xyz": _fmt(ax / np.linalg.norm(ax))})
    if joint.type in ("revolute", "prismatic") and joint.limit_lower is not None:
        ET.SubElement(joint_el, "limit", {
            "lower": str(joint.limit_lower), "upper": str(joint.limit_upper),
            "effort": "100", "velocity": "1.0",
        })


_NAME_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_-]*$")


def _urdf_errors(robot_name, links, joints, bodies_by_link) -> list[str]:
    errors = []
    if not _NAME_RE.match(robot_name):
        errors.append(f"Invalid robot name '{robot_name}'.")
    seen = set()
    for l in links:
        if not _NAME_RE.match(l.name):
            errors.append(f"Invalid link name '{l.name}'.")
        if l.name in seen:
            errors.append(f"Duplicate link name '{l.name}'.")
        seen.add(l.name)
    for j in joints:
        if j.type in ("cylindrical", "spherical"):
            errors.append(
                f"Joint {j.id} is {j.type}, which URDF cannot represent; confirm it as "
                "revolute/continuous/prismatic/fixed/planar."
            )
        if j.type in ("revolute", "continuous", "prismatic", "planar") and j.axis is None:
            errors.append(f"Joint {j.id} ({j.type}) requires an axis.")
        if j.type in ("revolute", "prismatic") and (j.limit_lower is None or j.limit_upper is None):
            errors.append(f"Joint {j.id} ({j.type}) requires limit_lower and limit_upper.")
    if bodies_by_link is not None:
        for l in links:
            if not any(b.vertices for b in bodies_by_link.get(l.id, [])):
                errors.append(f"Link '{l.id}' has no mesh geometry.")
    return errors


def export_urdf(
    robot_name: str,
    links: list[Link],
    joints: list[Joint],
    bodies_by_link: dict[str, list[Body]],
) -> str:
    errors = validate_structure(links, joints) + _urdf_errors(
        robot_name, links, joints, bodies_by_link
    )
    if errors:
        raise ValueError(
            "Cannot export URDF -- structural validation failed: " + "; ".join(errors)
        )

    robot = ET.Element("robot", {"name": robot_name})
    links_by_id = {link.id: link for link in links}
    frames = _link_frames(links, joints)

    for link in links:
        _add_link_element(robot, robot_name, link, frames[link.id])
    for joint in joints:
        _add_joint_element(robot, joint, links_by_id, frames)

    return minidom.parseString(ET.tostring(robot, encoding="unicode")).toprettyxml(indent="  ")


def _package_xml(robot_name: str) -> str:
    return f"""<?xml version="1.0"?>
<package format="2">
  <name>{escape(robot_name)}</name>
  <version>0.1.0</version>
  <description>URDF package for {escape(robot_name)}</description>
  <maintainer email="none@example.com">cad-to-urdf</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
"""


def export_package(
    robot_name: str,
    links: list[Link],
    joints: list[Joint],
    bodies_by_link: dict[str, list[Body]],
) -> bytes:
    urdf = export_urdf(robot_name, links, joints, bodies_by_link)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr(f"{robot_name}/urdf/{robot_name}.urdf", urdf)
        z.writestr(f"{robot_name}/package.xml", _package_xml(robot_name))
        for link in links:
            meshes = [
                trimesh.Trimesh(vertices=b.vertices, faces=b.triangles, process=False)
                for b in bodies_by_link.get(link.id, []) if b.vertices
            ]
            if meshes:
                stl = trimesh.util.concatenate(meshes).export(file_type="stl")
                z.writestr(f"{robot_name}/meshes/{link.name}.stl", stl)
    return buf.getvalue()
