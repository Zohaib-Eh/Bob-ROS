"""File > Import URDF: a URDF (alone, or zipped with its meshes, e.g. this app's own export) as a finished robot:
one part per link (its visual shapes, in world coordinates at q = 0), the file's joints, limits, efforts and
masses. The parts have no CAD faces (joints come from the file, not from detection)."""
import io
import posixpath
import re
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from app.models import Body, Joint, Link, PartWarning

TYPES = {"revolute", "continuous", "prismatic", "fixed", "planar"}
MAX_UNZIPPED = 1 << 30   # bytes: an upload is untrusted, so no zip bombs
PLACEHOLDER = 0.01   # m: the cube that stands in for a moving link with no shape
NAME = re.compile(r"[^A-Za-z0-9_-]")
UNIT = re.compile(r"<asset>.*?<unit[^>]*\bmeter=\"([0-9.eE+-]+)\"", re.S)


def _vec(s: str | None, default) -> np.ndarray:
    return np.array([float(x) for x in s.split()]) if s else np.array(default, float)


def _origin(el) -> np.ndarray:
    T = np.eye(4)
    o = el.find("origin") if el is not None else None
    if o is not None:
        T[:3, :3] = Rotation.from_euler("xyz", _vec(o.get("rpy"), [0, 0, 0])).as_matrix()
        T[:3, 3] = _vec(o.get("xyz"), [0, 0, 0])
    return T


class _Files:
    """The upload's files: one URDF, or a zip. Mesh paths resolve like ROS's would, inside the zip."""

    def __init__(self, filename: str, data: bytes):
        self.files: dict[str, bytes] = {}
        if filename.lower().endswith(".zip"):
            with zipfile.ZipFile(io.BytesIO(data)) as z:
                if sum(i.file_size for i in z.infolist()) > MAX_UNZIPPED:
                    raise ValueError(f"the zip unpacks to over {MAX_UNZIPPED >> 20} MB")
                self.files = {n: z.read(n) for n in z.namelist() if not n.endswith("/")}
            urdfs = sorted((n for n in self.files if n.lower().endswith(".urdf")), key=lambda n: ("/urdf/" not in n, n))
            if not urdfs:
                xacro = any(n.lower().endswith(".xacro") for n in self.files)
                raise ValueError("the zip has no .urdf file" + (" (run xacro on the .xacro first)" if xacro else ""))
            self.urdf = urdfs[0]
            self.others = urdfs[1:]
        else:
            self.urdf, self.others = filename, []
            self.files[filename] = data

    def find(self, ref: str) -> str | None:
        path = re.sub(r"^(package|file|model)://", "", ref).lstrip("/")
        if not ref.startswith(("package://", "model://")):   # relative to the URDF (or an absolute path)
            near = posixpath.normpath(posixpath.join(posixpath.dirname(self.urdf), path))
            if near in self.files:
                return near
        # package://pkg/meshes/x.stl: the zip holds pkg/meshes/x.stl, maybe under a folder, maybe without pkg/;
        # the longest tail of the path that names exactly one file (ur3/ and ur5/ both have a shoulder.dae)
        parts = path.split("/")
        for k in range(len(parts)):
            tail = "/".join(parts[k:])
            hits = [n for n in self.files if n == tail or n.endswith("/" + tail)]
            if len(hits) == 1:
                return hits[0]
            if hits:
                return None   # ambiguous: guessing would put the wrong shape on the link
        return None


def _mesh(files: _Files, geom, warn) -> trimesh.Trimesh | None:
    if (b := geom.find("box")) is not None:
        return trimesh.creation.box(_vec(b.get("size"), [0, 0, 0]))
    if (c := geom.find("cylinder")) is not None:
        return trimesh.creation.cylinder(float(c.get("radius", 0)), float(c.get("length", 0)), sections=32)
    if (s := geom.find("sphere")) is not None:
        return trimesh.creation.icosphere(subdivisions=2, radius=float(s.get("radius", 0)))
    m = geom.find("mesh")
    if m is None:
        return None
    ref = m.get("filename", "")
    name = files.find(ref)
    if name is None:
        warn("missing_mesh", f"{ref} is not in the upload; that shape is left out. Zip the URDF with its meshes.")
        return None
    data, ext = files.files[name], posixpath.splitext(name)[1].lower().lstrip(".")
    try:
        mesh = trimesh.load(io.BytesIO(data), file_type=ext, force="mesh")
    except Exception as exc:   # an unreadable mesh drops that shape, not the robot
        warn("bad_mesh", f"{posixpath.basename(name)} could not be read ({type(exc).__name__}); that shape is left out.")
        return None
    scale = _vec(m.get("scale"), [1, 1, 1])
    if ext == "dae" and (u := UNIT.search(data[:4096].decode("utf-8", "ignore"))):
        scale = scale * float(u.group(1))   # ROS applies the COLLADA unit; trimesh doesn't
    mesh.apply_scale(scale) if np.allclose(scale, scale[0]) else mesh.apply_transform(np.diag([*scale, 1]))
    return mesh


def _below(link: str, root: str, child_of) -> bool:
    while link in child_of:
        link = child_of[link].find("parent").get("link")
        if link == root:
            return True
    return False


def import_urdf(filename: str, data: bytes) -> dict:
    """{bodies, links, joints, root_link_id, gripper, warnings}, in this app's model: link frames and joint axes
    in world coordinates (the robot at q = 0), parts' vertices in world coordinates, metres."""
    files = _Files(filename, data)
    warnings: list[PartWarning] = []
    warn = lambda code, msg: warnings.append(PartWarning(code=code, message=msg, fixes=[]))
    if files.others:
        warn("several_urdf", f"The zip holds several URDFs; imported {posixpath.basename(files.urdf)}.")
    try:
        robot = ET.fromstring(files.files[files.urdf])
    except ET.ParseError as exc:
        raise ValueError(f"{posixpath.basename(files.urdf)} is not valid XML ({exc})") from None
    if robot.tag != "robot":
        raise ValueError("not a URDF (the top element isn't <robot>)")
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = robot.findall("joint")
    child_of = {j.find("child").get("link"): j for j in joints}
    roots = [n for n in links if n not in child_of]
    if len(roots) != 1:
        raise ValueError(f"a URDF robot needs exactly one root link, this one has {len(roots)}")

    world = {roots[0]: np.eye(4)}
    order = [roots[0]]
    for name in order:   # breadth first from the root: each link's frame at q = 0
        for j in joints:
            if j.find("parent").get("link") == name:
                c = j.find("child").get("link")
                world[c] = world[name] @ _origin(j)
                order.append(c)
    if len(order) != len(links):
        raise ValueError("the URDF's joints don't connect every link (or a link has two parents)")

    shapes: dict[str, trimesh.Trimesh] = {}
    for name in order:
        el = links[name]
        parts = []
        for tag in ("visual", "collision"):   # the visual shape; the collision one when there is none
            for g in el.findall(tag):
                mesh = _mesh(files, g.find("geometry"), warn) if g.find("geometry") is not None else None
                if mesh is not None and len(mesh.faces):
                    parts.append(mesh.apply_transform(world[name] @ _origin(g)))
            if parts:
                break
        if parts:
            shapes[name] = trimesh.util.concatenate(parts)

    # a shapeless link on a fixed joint (tool0, base_footprint, this app's tcp) carries nothing: leave it out; its
    # children hang off its parent (frames are world poses, so nothing moves)
    kids = lambda n: [j for j in joints if j.find("parent").get("link") == n]
    root = roots[0]
    while root not in shapes and len(kids(root)) == 1 and kids(root)[0].get("type") == "fixed":
        root = kids(root)[0].find("child").get("link")
    keep = [n for n in order if n == root or
            (_below(n, root, child_of) and (n in shapes or child_of[n].get("type") != "fixed"))]
    dropped = [n for n in order if n not in keep]
    if dropped:
        warn("dropped_links", f"Left out {len(dropped)} link(s) with no shape on fixed joints: {', '.join(dropped)}.")

    used_names: set[str] = set()
    ids, out_links, bodies = {}, [], []
    for n in keep:
        clean = NAME.sub("_", n) or "link"
        clean = clean if clean not in used_names else f"{clean}_{len(used_names)}"
        used_names.add(clean)
        ids[n] = f"urdf_{clean}"
        mesh = shapes.get(n)
        if mesh is None:   # a moving link with no shape: a small cube at its frame, so it can still be a link
            mesh = trimesh.creation.box([PLACEHOLDER] * 3).apply_transform(world[n])
            warn("placeholder", f"Link '{n}' has no shape; a {PLACEHOLDER * 100:g} cm cube stands in for it.")
        mass_el = links[n].find("inertial/mass")
        mass = float(mass_el.get("value", 0)) if mass_el is not None else 0.0
        bodies.append(Body(id=f"{ids[n]}_part", name=clean, vertices=mesh.vertices.tolist(),
                           triangles=mesh.faces.tolist(), faces=[],
                           status="ok" if mesh.is_volume else "approximate"))
        out_links.append(Link(id=ids[n], name=clean, body_ids=[f"{ids[n]}_part"], density_kg_m3=1240.0,
                              mass_override_kg=mass if mass > 0 else None, accept_approximate_mass=True))

    def kept_parent(link: str) -> str | None:
        while link not in ids:
            if link not in child_of:
                return None
            link = child_of[link].find("parent").get("link")
        return link

    out_joints, by_name, mimics = [], {}, []
    for j in joints:
        c = j.find("child").get("link")
        parent = kept_parent(j.find("parent").get("link"))
        if c not in ids or parent is None:   # left out, or below a left-out root (the new root)
            continue
        kind = j.get("type")
        if kind not in TYPES:
            warn("joint_type", f"Joint '{j.get('name')}' is {kind}; imported as fixed.")
            kind = "fixed"
        T = world[c]
        axis = None
        if kind != "fixed":
            a = T[:3, :3] @ _vec(j.find("axis").get("xyz") if j.find("axis") is not None else None, [1, 0, 0])
            axis = (a / np.linalg.norm(a)).tolist()
        lim = j.find("limit")
        lo = hi = effort = None
        if lim is not None:
            effort = float(lim.get("effort", 0)) or None
            if kind in ("revolute", "prismatic"):
                lo, hi = float(lim.get("lower", 0)), float(lim.get("upper", 0))
        out = Joint(id=f"urdf_{NAME.sub('_', j.get('name') or c)}", parent_link_id=ids[parent],
                    child_link_id=ids[c], type=kind, origin_xyz=T[:3, 3].tolist(),
                    origin_rpy=Rotation.from_matrix(T[:3, :3]).as_euler("xyz").tolist(), axis=axis,
                    limit_lower=lo, limit_upper=hi, source="confirmed", max_effort=effort)
        out_joints.append(out)
        by_name[j.get("name")] = out
        if (m := j.find("mimic")) is not None:
            mimics.append((out, m.get("joint")))

    gripper = None   # two sliding fingers, one mimicking the other: this app's gripper
    for follower, leader_name in mimics:
        leader = by_name.get(leader_name)
        if (leader and leader.type == follower.type == "prismatic"
                and leader.parent_link_id == follower.parent_link_id):
            gripper = {"leader": leader.id, "follower": follower.id, "tcp_offset": 0.0}
            break
    warn("no_faces", "Imported from a URDF: the parts have no CAD faces, so joint detection and Mount snapping "
                     "don't work on them. The joints come from the file. Inertia is recomputed from each shape at "
                     "the file's mass.")
    return {"robot_name": NAME.sub("_", robot.get("name") or "") or None, "bodies": bodies, "links": out_links, "joints": out_joints, "root_link_id": ids[root],
            "gripper": gripper, "warnings": warnings}
