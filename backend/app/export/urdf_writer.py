import io
import math
import re
import zipfile
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape
from xml.dom import minidom

import mujoco
import numpy as np
import trimesh

from scipy.spatial.transform import Rotation

from app.export.gripper import DEFAULT_GRIP_FORCE, checked, convex_pieces, inward_joints, jaw_gap, tcp_point
from app.massprops.link_mass import compute_link_mass_properties
from app.models import Body, Gripper, Joint, Link, PartWarning
from app.sim.mjcf import build_mjcf, compile_mjcf
from app.sim.torque import required_efforts, suggest
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


def _add_link_element(robot, robot_name: str, link: Link, T: np.ndarray, pieces: int = 0) -> None:
    """pieces > 0: the collision is that many convex meshes `<link>_c<i>.stl` (gripper links)."""
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
    refs = [("visual", _mesh_ref(robot_name, link))]
    refs += ([("collision", f"package://{robot_name}/meshes/{link.name}_c{i}.stl") for i in range(pieces)]
             if pieces else [("collision", _mesh_ref(robot_name, link))])
    for tag, ref in refs:
        geom_el = ET.SubElement(link_el, tag)
        ET.SubElement(geom_el, "origin", {"xyz": xyz, "rpy": rpy})
        geometry = ET.SubElement(geom_el, "geometry")
        ET.SubElement(geometry, "mesh", {"filename": ref})


def _joint_names(joints: list[Joint], links_by_id) -> dict[str, str]:
    """`<child link>_joint`, ROS-safe characters, numbered on collision."""
    names, taken = {}, set()
    for j in joints:
        base = re.sub(r"[^A-Za-z0-9_]", "_", links_by_id[j.child_link_id].name) + "_joint"
        name, n = base, 2
        while name in taken:
            name, n = f"{base}_{n}", n + 1
        taken.add(name)
        names[j.id] = name
    return names


def _add_joint_element(robot, joint: Joint, name: str, links_by_id, frames, effort: float = 100.0) -> None:
    joint_el = ET.SubElement(robot, "joint", {"name": name, "type": joint.type})
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
            "effort": f"{effort:g}", "velocity": "1.0",
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
    efforts: dict[str, float] | None = None,
    gripper: Gripper | None = None,
    collision_pieces: dict[str, int] | None = None,
) -> str:
    """URDF text. `efforts` maps joint ids to motor strength; a missing one is written as 100.
    With `gripper`: both finger axes point inward, the follower <mimic>s the leader, and a fixed
    `tcp` link sits between the fingertips. `collision_pieces` maps link ids to their convex piece count."""
    errors = validate_structure(links, joints) + _urdf_errors(
        robot_name, links, joints, bodies_by_link
    )
    if gripper is not None and any(l.name == "tcp" for l in links):
        errors.append("A link is named 'tcp'; that name is the gripper's tool point. Rename the link.")
    if errors:
        raise ValueError(
            "Cannot export URDF -- structural validation failed: " + "; ".join(errors)
        )
    if gripper is not None:
        joints = inward_joints(gripper, joints, bodies_by_link)

    robot = ET.Element("robot", {"name": robot_name})
    links_by_id = {link.id: link for link in links}
    frames = _link_frames(links, joints)

    for link in links:
        _add_link_element(robot, robot_name, link, frames[link.id], (collision_pieces or {}).get(link.id, 0))
    names = _joint_names(joints, links_by_id)
    for joint in joints:
        _add_joint_element(robot, joint, names[joint.id], links_by_id, frames, (efforts or {}).get(joint.id, 100.0))
    if gripper is not None:
        _add_gripper(robot, gripper, joints, names, links_by_id, frames, bodies_by_link)

    return minidom.parseString(ET.tostring(robot, encoding="unicode")).toprettyxml(indent="  ")


def _wrist_axis(link_id, joints):
    """Axis of the first moving joint above link_id (through fixed joints), if it turns; else None."""
    by_child = {j.child_link_id: j for j in joints}
    j = by_child.get(link_id)
    while j is not None and j.type == "fixed":
        j = by_child.get(j.parent_link_id)
    return j.axis if j is not None and j.type in ("revolute", "continuous") else None


def _add_gripper(robot, gripper: Gripper, joints, names, links_by_id, frames, bodies_by_link) -> None:
    """Follower <mimic> of the leader, and the fixed `tcp` link on the gripper base: z along the
    approach axis, x along the leader's closing direction."""
    leader, follower = checked(gripper, joints)
    for el in robot.iter("joint"):
        if el.get("name") == names[follower.id]:
            ET.SubElement(el, "mimic", {"joint": names[leader.id], "multiplier": "1", "offset": "0"})
    verts = lambda link_id: [v for b in bodies_by_link.get(link_id, []) for v in b.vertices]
    xyz, z = tcp_point(verts(leader.parent_link_id), verts(leader.child_link_id), verts(follower.child_link_id),
                       gripper.tcp_offset_m, leader.axis, _wrist_axis(leader.parent_link_id, joints))
    z = np.asarray(z)
    x = np.asarray(leader.axis, float) - (np.asarray(leader.axis, float) @ z) * z
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = xyz
    ET.SubElement(robot, "link", {"name": "tcp"})
    j = ET.SubElement(robot, "joint", {"name": "tcp_joint", "type": "fixed"})
    ET.SubElement(j, "parent", {"link": links_by_id[leader.parent_link_id].name})
    ET.SubElement(j, "child", {"link": "tcp"})
    o_xyz, o_rpy = _xyz_rpy(np.linalg.inv(frames[leader.parent_link_id]) @ T)
    ET.SubElement(j, "origin", {"xyz": o_xyz, "rpy": o_rpy})
    def mesh(link_id):
        parts = [trimesh.Trimesh(b.vertices, b.triangles, process=False) for b in bodies_by_link.get(link_id, []) if b.vertices]
        whole = trimesh.util.concatenate(parts)
        return whole.vertices, whole.faces
    tip = np.asarray(xyz) - z * gripper.tcp_offset_m          # the fingertips (tcp_offset pushes past them)
    gap = jaw_gap(mesh(leader.child_link_id), mesh(follower.child_link_id), leader.axis, z, tip)
    # an extension tag (like <gazebo>; URDF parsers skip it): the jaw gap at q = 0, which travel alone can't give
    ET.SubElement(robot, "gripper", {"jaw_gap": f"{gap:.6f}"})


def package_xml(robot_name: str) -> str:
    return f"""<?xml version="1.0"?>
<package format="3">
  <name>{escape(robot_name)}</name>
  <version>0.1.0</version>
  <description>URDF package for {escape(robot_name)}</description>
  <maintainer email="none@example.com">cad-to-urdf</maintainer>
  <license>BSD</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""


def cmakelists(robot_name: str) -> str:
    return f"""cmake_minimum_required(VERSION 3.8)
project({robot_name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes DESTINATION share/${{PROJECT_NAME}})
ament_package()
"""


def _bbox_mass_properties(bodies: list[Body], mass: float):
    """Fallback when the user set a mass but the geometry has no measurable
    volume: uniform solid box over the vertex bounding box."""
    v = np.array([p for b in bodies for p in b.vertices], dtype=float)
    lo, hi = v.min(axis=0), v.max(axis=0)
    dx, dy, dz = hi - lo
    inertia = [mass / 12 * (dy**2 + dz**2), mass / 12 * (dx**2 + dz**2),
               mass / 12 * (dx**2 + dy**2), 0.0, 0.0, 0.0]
    return tuple(((lo + hi) / 2).tolist()), inertia


def _with_mass_properties(link: Link, bodies: list[Body]) -> Link:
    """Mass/COM/inertia always come from geometry + density, never the client.
    A user mass override rescales the computed inertia (uniform density)."""
    override = link.mass_override_kg
    if not (math.isfinite(link.density_kg_m3) and link.density_kg_m3 > 0):
        raise ValueError(f"Link '{link.id}': density must be finite and > 0")
    try:
        mass, com, inertia = compute_link_mass_properties(bodies, link.density_kg_m3)
    except ValueError as exc:
        if override is None and any(b.status == "approximate" for b in bodies):
            raise ValueError(f"Link '{link.id}': volume of approximate part(s) can't be "
                             "measured; set mass_override_kg for this link.") from exc
        if override is None or not any(b.vertices for b in bodies):
            raise ValueError(f"Link '{link.id}': {exc}") from exc
        com, inertia = _bbox_mass_properties(bodies, override)
        mass = override
    else:
        if override is not None:
            ratio = override / mass
            mass, inertia = override, [x * ratio for x in inertia]
    return link.model_copy(update={"mass_kg": mass, "center_of_mass": com, "inertia": inertia})


def _part_status_errors(links: list[Link], bodies_by_link: dict[str, list[Body]]) -> list[str]:
    errors = []
    for l in links:
        for b in bodies_by_link.get(l.id, []):
            if b.status == "failed":
                errors.append(f"Link '{l.id}' contains part '{b.name}' that failed to import; "
                              "replace it or remove it from the link.")
            elif (b.status == "approximate" and l.mass_override_kg is None
                  and not l.accept_approximate_mass):
                errors.append(f"Link '{l.id}' contains approximate part '{b.name}'; set "
                              "mass_override_kg or accept_approximate_mass.")
    return errors


def _used_bodies(links: list[Link], bodies_by_link: dict[str, list[Body]]) -> dict[str, list[Body]]:
    """A link's geometry is the bodies sent under its id AND listed in its body_ids."""
    return {l.id: [b for b in bodies_by_link.get(l.id, []) if b.id in l.body_ids] for l in links}


def unused_body_warnings(links: list[Link], bodies_by_link: dict[str, list[Body]]) -> list[PartWarning]:
    used = {id(b) for bs in _used_bodies(links, bodies_by_link).values() for b in bs}
    return [
        PartWarning(code="unused_body", fixes=[],
                    message=f"Part '{b.name}' is in no link and was not exported.")
        for bodies in bodies_by_link.values() for b in bodies if id(b) not in used
    ]


def link_mass_properties(link: Link, bodies: list[Body]) -> Link:
    """Mass/COM/inertia for one link exactly as export computes them."""
    used = _used_bodies([link], {link.id: bodies})
    errors = _part_status_errors([link], used)
    if errors:
        raise ValueError("; ".join(errors))
    return _with_mass_properties(link, used[link.id])


def _prepared(robot_name, links, joints, bodies_by_link, gripper: Gripper | None = None):
    """Validated links (with mass properties), used bodies, per-link STL bytes, and the convex
    collision piece count per gripper link (`<link>_c<i>.stl` meshes)."""
    bodies_by_link = _used_bodies(links, bodies_by_link)
    errors = _part_status_errors(links, bodies_by_link)
    if errors:
        raise ValueError("Cannot export URDF -- " + "; ".join(errors))
    links = [_with_mass_properties(l, bodies_by_link.get(l.id, [])) for l in links]
    meshes, pieces = {}, {}
    gripper_links = set()
    if gripper is not None:
        leader, follower = checked(gripper, joints)
        # only the fingers touch what is gripped; a real gripper body (ports, holes) takes CoACD
        # ~30-40 s, so it keeps one convex hull (MuJoCo's default for a mesh)
        gripper_links = {leader.child_link_id, follower.child_link_id}
    for link in links:
        parts = [trimesh.Trimesh(vertices=b.vertices, faces=b.triangles, process=False)
                 for b in bodies_by_link.get(link.id, []) if b.vertices]
        if not parts:
            continue
        whole = trimesh.util.concatenate(parts)
        meshes[f"{link.name}.stl"] = whole.export(file_type="stl")
        if link.id in gripper_links:
            convex = convex_pieces(whole.vertices, whole.faces)
            for i, (v, f) in enumerate(convex):
                meshes[f"{link.name}_c{i}.stl"] = trimesh.Trimesh(v, f, process=False).export(file_type="stl")
            pieces[link.id] = len(convex)
    return links, bodies_by_link, meshes, pieces


def _suggested_from(urdf: str, links, joints, meshes) -> dict[str, float]:
    """Joint id -> suggested motor strength (2x the worst static gravity load), moving joints only.
    Raises ValueError when MuJoCo can't load the robot (e.g. impossible inertia, flat collision mesh)."""
    by_name = suggest(required_efforts(compile_mjcf(build_mjcf(urdf, meshes, None), meshes)))
    names = _joint_names(joints, {l.id: l for l in links})
    return {j.id: by_name[names[j.id]] for j in joints if names[j.id] in by_name}


def suggest_efforts(robot_name, links, joints, bodies_by_link) -> dict[str, float]:
    links, bodies_by_link, meshes, _ = _prepared(robot_name, links, joints, bodies_by_link)
    return _suggested_from(export_urdf(robot_name, links, joints, bodies_by_link), links, joints, meshes)


def export_package(
    robot_name: str,
    links: list[Link],
    joints: list[Joint],
    bodies_by_link: dict[str, list[Body]],
    gripper: Gripper | None = None,
) -> bytes:
    """The ROS description package zip. A robot MuJoCo can't load still exports: its efforts fall back
    to 100 (or max_effort) and the zip has no mjcf/; the physics check reports why. With `gripper`
    the fingers' effort is their grip force (max_effort or 40 N) and the follower has no motor."""
    links, bodies_by_link, meshes, pieces = _prepared(robot_name, links, joints, bodies_by_link, gripper)
    kw = {"gripper": gripper, "collision_pieces": pieces}
    urdf = export_urdf(robot_name, links, joints, bodies_by_link, **kw)  # structural errors raise here
    try:
        sug = _suggested_from(urdf, links, joints, meshes)
    except (ValueError, mujoco.FatalError):
        sug = {}
    fingers = {gripper.leader_joint_id, gripper.follower_joint_id} if gripper else set()
    efforts = {j.id: j.max_effort or (DEFAULT_GRIP_FORCE if j.id in fingers else sug.get(j.id, 100.0)) for j in joints}
    urdf = export_urdf(robot_name, links, joints, bodies_by_link, efforts, **kw)
    names = _joint_names(joints, {l.id: l for l in links})
    motors = {names[j.id]: efforts[j.id] for j in joints if not (gripper and j.id == gripper.follower_joint_id)}
    grip = None
    if gripper is not None:
        leader, follower = checked(gripper, joints)
        by_id = {l.id: l for l in links}
        grip = {"links": [by_id[i].name for i in (leader.parent_link_id, leader.child_link_id, follower.child_link_id)],
                "fingers": [names[leader.id], names[follower.id]], "follower": names[follower.id]}
    try:
        mjcf = build_mjcf(urdf, meshes, motors, grip)  # fixed joints get no motor
    except (ValueError, mujoco.FatalError):
        mjcf = None
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr(f"{robot_name}/urdf/{robot_name}.urdf", urdf)
        if mjcf is not None:
            z.writestr(f"{robot_name}/mjcf/{robot_name}.xml", mjcf)
        z.writestr(f"{robot_name}/package.xml", package_xml(robot_name))
        z.writestr(f"{robot_name}/CMakeLists.txt", cmakelists(robot_name))
        for name, stl in meshes.items():
            z.writestr(f"{robot_name}/meshes/{name}", stl)
    return buf.getvalue()
