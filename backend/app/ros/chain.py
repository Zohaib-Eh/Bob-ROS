"""Serial chain from a URDF: base to tip through the moving joints.

stdlib + numpy only, relative imports only: the generated <robot>_kinematics package ships a
copy of this file."""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

MOVING = ("revolute", "prismatic", "continuous")


class UnsupportedRobot(ValueError):
    pass


def origin_matrix(el) -> np.ndarray:
    """4x4 transform from a URDF <origin xyz rpy> (rpy = fixed-axis roll, pitch, yaw)."""
    T = np.eye(4)
    if el is None:
        return T
    x, y, z = (float(v) for v in el.get("xyz", "0 0 0").split())
    r, p, w = (float(v) for v in el.get("rpy", "0 0 0").split())
    cr, sr, cp, sp, cw, sw = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(w), math.sin(w)
    T[:3, :3] = [[cw * cp, cw * sp * sr - sw * cr, cw * sp * cr + sw * sr],
                 [sw * cp, sw * sp * sr + cw * cr, sw * sp * cr - cw * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = (x, y, z)
    return T


@dataclass
class ChainJoint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray      # previous segment frame -> joint frame at q=0 (fixed joints folded in)
    axis: np.ndarray        # unit vector in the joint frame
    lower: float | None
    upper: float | None


@dataclass
class Chain:
    robot: str
    base: str
    tip: str
    joints: list[ChainJoint]
    links: dict[str, tuple[int, np.ndarray]]  # link -> (segment, pose in segment frame)


def parse_chain(urdf_text: str) -> Chain:
    root = ET.fromstring(urdf_text)
    joints = root.findall("joint")
    children: dict[str, list] = {}
    has_parent = set()
    for j in joints:
        if j.find("parent") is None or j.find("child") is None:
            raise UnsupportedRobot(f"Joint '{j.get('name')}' has no parent or child link.")
        children.setdefault(j.find("parent").get("link"), []).append(j)
        has_parent.add(j.find("child").get("link"))
    roots = [l.get("name") for l in root.findall("link") if l.get("name") not in has_parent]
    if len(roots) != 1:
        raise UnsupportedRobot(f"URDF has {len(roots)} root links; expected one.")

    def moving_below(link: str) -> bool:
        return any(j.get("type") != "fixed" or moving_below(j.find("child").get("link"))
                   for j in children.get(link, []))

    wheels = [j for j in joints
              if j.get("type") == "continuous" and not moving_below(j.find("child").get("link"))]
    if len(wheels) >= 2:
        raise UnsupportedRobot(
            f"Joint '{wheels[0].get('name')}' looks like a wheel; mobile robots are not supported yet.")
    for j in joints:
        if j.get("type") not in MOVING + ("fixed",):
            raise UnsupportedRobot(
                f"Joint '{j.get('name')}' is {j.get('type')}; only revolute, prismatic, "
                "continuous and fixed joints are supported.")
    leaves = [j for j in joints
              if j.get("type") != "fixed" and not moving_below(j.find("child").get("link"))]
    if len(leaves) > 1:
        raise UnsupportedRobot(
            f"Robot has {len(leaves)} moving branches; only single-arm robots are supported yet.")
    if not leaves:
        raise UnsupportedRobot("Robot has no moving joints.")

    chain_joints: list[ChainJoint] = []
    link_poses: dict[str, tuple[int, np.ndarray]] = {}

    def walk(link: str, seg: int, T: np.ndarray) -> None:
        link_poses[link] = (seg, T)
        for j in children.get(link, []):
            child = j.find("child").get("link")
            O = T @ origin_matrix(j.find("origin"))
            if j.get("type") == "fixed":
                walk(child, seg, O)
                continue
            lower = upper = None
            if j.get("type") != "continuous":
                lim = j.find("limit")
                if lim is None or lim.get("lower") is None or lim.get("upper") is None:
                    raise UnsupportedRobot(
                        f"Joint '{j.get('name')}' has no limits; set them in the app and export again.")
                lower, upper = float(lim.get("lower")), float(lim.get("upper"))
            ax_el = j.find("axis")
            axis = np.array([float(v) for v in (ax_el.get("xyz") if ax_el is not None else "1 0 0").split()])
            if not np.linalg.norm(axis):
                raise UnsupportedRobot(f"Joint '{j.get('name')}' has a zero-length axis.")
            chain_joints.append(ChainJoint(j.get("name"), j.get("type"), link, child, O,
                                           axis / np.linalg.norm(axis), lower, upper))
            walk(child, len(chain_joints), np.eye(4))

    walk(roots[0], 0, np.eye(4))
    return Chain(root.get("name"), roots[0], chain_joints[-1].child, chain_joints, link_poses)


def home(chain: Chain) -> list[float]:
    """Every joint at 0, or at its nearest limit when 0 is out of range."""
    return [0.0 if j.lower is None else min(max(0.0, j.lower), j.upper) for j in chain.joints]
