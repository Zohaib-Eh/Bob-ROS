"""Which link pairs MoveIt may skip when checking self-collision (the SRDF's disable_collisions).

Same idea as MoveIt Setup Assistant: pairs joined by a joint, pairs touching at home, and pairs
that never touch in many random poses are skipped."""
import xml.etree.ElementTree as ET
from itertools import combinations
from pathlib import Path

import fcl
import numpy as np
import trimesh

from app.validate.sweep import collides, fcl_object

from .chain import Chain, home, origin_matrix
from .fk import link_poses


def link_meshes(urdf_text: str, package_root: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Collision mesh of each link in its own frame. package://<pkg>/x resolves to package_root/x."""
    out = {}
    for link in ET.fromstring(urdf_text).findall("link"):
        parts = []
        for col in link.findall("collision"):
            mesh = col.find("geometry/mesh")
            if mesh is None:
                continue
            rel = mesh.get("filename").split("://", 1)[-1].split("/", 1)[1]
            m = trimesh.load(Path(package_root) / rel, file_type="stl", force="mesh")
            m.apply_transform(origin_matrix(col.find("origin")))
            parts.append(m)
        if parts:
            m = trimesh.util.concatenate(parts)
            out[link.get("name")] = (np.asarray(m.vertices), np.asarray(m.faces))
    return out


def _random_q(chain: Chain, rng) -> list[float]:
    return [rng.uniform(j.lower, j.upper) if j.lower is not None else rng.uniform(-np.pi, np.pi)
            for j in chain.joints]


def disabled_pairs(chain: Chain, meshes, samples: int = 10000, seed: int = 0) -> dict[tuple[str, str], str]:
    objs = {n: fcl_object(v, f) for n, (v, f) in meshes.items()}
    names = sorted(objs)

    def place(q) -> None:
        for n, T in link_poses(chain, q).items():
            if n in objs:
                objs[n].setTransform(fcl.Transform(T[:3, :3], T[:3, 3]))

    out, open_pairs = {}, []
    for a, b in combinations(names, 2):
        if abs(chain.links[a][0] - chain.links[b][0]) <= 1:
            out[(a, b)] = "Adjacent"
        else:
            open_pairs.append((a, b))
    place(home(chain))
    for p in open_pairs:
        if collides(objs[p[0]], objs[p[1]]):
            out[p] = "Default"
    open_pairs = [p for p in open_pairs if p not in out]
    rng = np.random.default_rng(seed)
    for _ in range(samples):
        if not open_pairs:
            break
        place(_random_q(chain, rng))
        open_pairs = [p for p in open_pairs if not collides(objs[p[0]], objs[p[1]])]
    for p in open_pairs:
        out[p] = "Never"
    return out
