"""Forward kinematics for a Chain. numpy only: the generated <robot>_kinematics package ships a copy."""
import math

import numpy as np

from .chain import Chain, ChainJoint


def _motion(j: ChainJoint, q: float) -> np.ndarray:
    T = np.eye(4)
    if j.type == "prismatic":
        T[:3, 3] = j.axis * q
        return T
    x, y, z = j.axis
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    T[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return T


def segment_poses(chain: Chain, q) -> list[np.ndarray]:
    """Base-frame pose of each segment: [base, after joint 1, ..., after joint n]."""
    T = np.eye(4)
    out = [T]
    for j, qi in zip(chain.joints, q, strict=True):
        T = T @ j.origin @ _motion(j, float(qi))
        out.append(T)
    return out


def fk(chain: Chain, q) -> np.ndarray:
    return segment_poses(chain, q)[-1]


def link_poses(chain: Chain, q) -> dict[str, np.ndarray]:
    seg = segment_poses(chain, q)
    return {name: seg[i] @ T for name, (i, T) in chain.links.items()}
