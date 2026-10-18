"""Every exact IK solution for a pose (EAIK closed form), each checked against this package's FK."""
import math
import os

import numpy as np

from ._chain import parse_chain
from ._fk import fk as _fk

ROBOT = "__ROBOT__"
TOL = 1e-6  # m and rad: every returned solution reproduces the pose at least this closely


def _pose_error(a, b) -> float:
    dp = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    c = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2
    return max(dp, math.acos(max(-1.0, min(1.0, c))))


def _within_limits(q, chain):
    """Wrap revolute angles into the joint's range; None when any joint can't be."""
    # ponytail: one representative per joint; ranges wider than 2π hold more, add them if a robot needs it
    out = []
    for qi, j in zip(q, chain.joints):
        if j.type == "prismatic":
            cands = [float(qi)]
        else:
            r = math.remainder(float(qi), 2 * math.pi)
            cands = [r, r - 2 * math.pi, r + 2 * math.pi]
        if j.lower is not None:
            cands = [c for c in cands if j.lower - 1e-9 <= c <= j.upper + 1e-9]
        if not cands:
            return None
        out.append(cands[0])
    return np.array(out)


def _arm_only(urdf_path: str, chain) -> str:
    """The URDF EAIK should see: a gripper's finger joints and a mobile base's wheels turned fixed (EAIK solves
    at most 6 moving joints, and neither is part of the arm's IK). Unchanged path without either."""
    skip = set()
    if chain.gripper is not None:
        skip |= {chain.gripper.leader, chain.gripper.follower}
    if getattr(chain, "mobile", None) is not None:
        skip |= set(chain.mobile.left + chain.mobile.right)
    if not skip:
        return urdf_path
    import tempfile
    import xml.etree.ElementTree as ET
    root = ET.parse(urdf_path).getroot()
    for j in root.iter("joint"):
        if j.get("name") in skip:
            j.set("type", "fixed")
            for tag in ("axis", "limit", "mimic"):
                for el in j.findall(tag):
                    j.remove(el)
    fd, path = tempfile.mkstemp(suffix=".urdf")
    with open(fd, "w", encoding="utf-8") as f:
        f.write(ET.tostring(root, encoding="unicode"))
    return path


class Kinematics:
    def __init__(self, urdf_path: str):
        with open(urdf_path, encoding="utf-8") as f:
            self.chain = parse_chain(f.read())
        self._bot, self._why = None, ""
        try:
            from eaik.IK_URDF import UrdfRobot
            bot = UrdfRobot(_arm_only(urdf_path, self.chain))
            if hasattr(bot, "hasKnownDecomposition") and not bot.hasKnownDecomposition():
                self._why = "EAIK knows no decomposition for this geometry"
            else:
                z = np.zeros(len(self.chain.joints))
                # EAIK's end frame may differ from the URDF tip by a fixed offset
                self._tip = np.linalg.inv(np.asarray(bot.fwdKin(z), float)) @ _fk(self.chain, z)
                self._bot = bot
        except Exception as exc:  # EAIK refuses geometries it cannot decompose
            self._why = f"{type(exc).__name__}: {exc}"

    def fk(self, q) -> np.ndarray:
        return _fk(self.chain, q)

    def solve_all(self, pose, current=None) -> list:
        """All joint solutions within limits for a 4x4 base-frame pose, nearest to `current` first."""
        if self._bot is None:
            raise NotImplementedError(f"no closed-form solver for this geometry ({self._why})")
        pose = np.asarray(pose, float)
        out = []
        for q in self._bot.IK(pose @ np.linalg.inv(self._tip)).Q:
            q = _within_limits(q, self.chain)
            if (q is not None and _pose_error(self.fk(q), pose) <= TOL
                    and not any(np.allclose(q, o, atol=1e-9) for o in out)):
                out.append(q)
        ref = np.zeros(len(self.chain.joints)) if current is None else np.asarray(current, float)
        return sorted(out, key=lambda q: float(np.linalg.norm(q - ref)))


_default = None


def load() -> Kinematics:
    """The installed robot's kinematics (needs a sourced ROS 2 workspace)."""
    global _default
    if _default is None:
        from ament_index_python.packages import get_package_share_directory
        _default = Kinematics(os.path.join(get_package_share_directory(ROBOT), "urdf", f"{ROBOT}.urdf"))
    return _default


def fk(q):
    return load().fk(q)


def solve_all(pose, current=None):
    return load().solve_all(pose, current)
