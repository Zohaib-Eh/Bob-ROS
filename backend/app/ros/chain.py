"""Serial chain from a URDF: base to tip through the moving joints.

stdlib + numpy only, relative imports only: the generated <robot>_kinematics package ships a
copy of this file."""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

MOVING = ("revolute", "prismatic", "continuous")
MAX_VELOCITY = 1.0       # rad/s (m/s for prismatic): placeholder until set from the datasheet
MAX_ACCELERATION = 2.0   # rad/s^2 (m/s^2); joint_limits.yaml and the motor sizing both use these


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
class GripperInfo:
    leader: str
    follower: str
    base: str
    lower: float
    upper: float
    effort: float
    closing_axis_tcp: np.ndarray   # the leader's closing direction in the tip (tcp) frame
    jaw_gap: float                 # m between the fingers at q = 0 (narrowest point, as modelled)

    @property
    def opening(self) -> float:
        """Widest the jaws open (m): fingers at their open limit."""
        return self.width(self.lower)

    def width(self, q: float) -> float:
        """Gap between the fingers (m) with the leader at q (the follower mirrors it)."""
        return self.jaw_gap - 2 * q


@dataclass
class Chain:
    robot: str
    base: str
    tip: str
    joints: list[ChainJoint]
    links: dict[str, tuple[int, np.ndarray]]  # link -> (segment, pose in segment frame)
    gripper: GripperInfo | None = None


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

    fingers = _finger_pair(joints, children)   # gripper fingers count as fixed for the arm

    finger_names = {*fingers, *fingers.values()}

    def moving(j) -> bool:
        return j.get("type") != "fixed" and j.get("name") not in finger_names

    def moving_below(link: str) -> bool:
        return any(moving(j) or moving_below(j.find("child").get("link")) for j in children.get(link, []))

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
    leaves = [j for j in joints if moving(j) and not moving_below(j.find("child").get("link"))]
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
            if not moving(j):   # fixed, or a gripper finger (at 0, so its link still has a pose)
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
    tip = "tcp" if "tcp" in link_poses else chain_joints[-1].child
    gripper = _gripper_info(fingers, joints, link_poses, tip, root.find("gripper")) if fingers else None
    return Chain(root.get("name"), roots[0], tip, chain_joints, link_poses, gripper)


def _finger_pair(joints, children) -> dict[str, str]:
    """{leader: follower} names of a gripper: two prismatic leaf joints on one link, the follower
    with <mimic joint=leader>. Empty when there is none."""
    by_name = {j.get("name"): j for j in joints}
    leaf = lambda j: not any(c.get("type") != "fixed" for c in children.get(j.find("child").get("link"), []))
    for f in joints:
        mimic = f.find("mimic")
        lead = by_name.get(mimic.get("joint")) if mimic is not None else None
        if (lead is not None and f.get("type") == lead.get("type") == "prismatic" and leaf(f) and leaf(lead)
                and f.find("parent").get("link") == lead.find("parent").get("link")):
            return {lead.get("name"): f.get("name")}
    return {}


def _gripper_info(fingers, joints, link_poses, tip, tag=None) -> GripperInfo:
    leader_name, follower_name = next(iter(fingers.items()))
    j = next(j for j in joints if j.get("name") == leader_name)
    base = j.find("parent").get("link")
    lim = j.find("limit")
    ax_el = j.find("axis")
    axis = np.array([float(v) for v in (ax_el.get("xyz") if ax_el is not None else "1 0 0").split()])
    _, T_base = link_poses[base]
    _, T_tip = link_poses[tip]
    R_joint = (T_base @ origin_matrix(j.find("origin")))[:3, :3]   # base and tip share the last segment
    closing = T_tip[:3, :3].T @ (R_joint @ (axis / np.linalg.norm(axis)))
    lower, upper = float(lim.get("lower")), float(lim.get("upper"))
    # an older package has no <gripper jaw_gap>: assume the jaws just touch when fully closed
    gap = float(tag.get("jaw_gap")) if tag is not None and tag.get("jaw_gap") else 2 * upper
    return GripperInfo(leader_name, follower_name, base, lower, upper, float(lim.get("effort", 0)), closing, gap)


def home(chain: Chain) -> list[float]:
    """Every joint at 0, or at its nearest limit when 0 is out of range."""
    return [0.0 if j.lower is None else min(max(0.0, j.lower), j.upper) for j in chain.joints]
