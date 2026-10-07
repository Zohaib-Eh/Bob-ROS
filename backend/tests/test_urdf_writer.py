import io
import math
import zipfile
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import trimesh

from app.export.urdf_writer import export_package, export_urdf
from app.models import Body, Joint, Link


def _link(id_, com=(0.0, 0.0, 0.1), inertia=None):
    return Link(
        id=id_, name=id_, body_ids=[f"{id_}_body"], density_kg_m3=1000.0,
        mass_kg=2.0, center_of_mass=com,
        inertia=inertia or [0.01, 0.01, 0.01, 0.0, 0.0, 0.0],
    )


def _joint(id_, parent, child, xyz=(0.0, 0.0, 0.2), rpy=(0.0, 0.0, 0.0)):
    return Joint(
        id=id_, parent_link_id=parent, child_link_id=child, type="revolute",
        origin_xyz=xyz, origin_rpy=rpy, axis=(0.0, 0.0, 1.0),
        limit_lower=-1.57, limit_upper=1.57, source="confirmed",
    )


def _body(link_id):
    box = trimesh.creation.box(extents=(1, 1, 1))
    return Body(id=f"{link_id}_body", name=link_id,
                vertices=[tuple(v) for v in box.vertices],
                triangles=[tuple(t) for t in box.faces], faces=[])


def _vec(el, attr="xyz"):
    return np.array([float(x) for x in el.attrib[attr].split()])


def _link_el(root, name):
    return next(el for el in root.findall("link") if el.attrib["name"] == name)


def _export(links, joints):
    bodies = {l.id: [_body(l.id)] for l in links}
    return ET.fromstring(export_urdf("test_robot", links, joints, bodies))


def test_valid_model_produces_parseable_urdf_with_expected_joint():
    root = _export([_link("base"), _link("arm")], [_joint("j1", "base", "arm")])
    assert root.tag == "robot"
    assert root.attrib["name"] == "test_robot"
    assert {el.attrib["name"] for el in root.findall("link")} == {"base", "arm"}
    joint_el = root.find("joint")
    assert joint_el.attrib["type"] == "revolute"
    assert joint_el.find("parent").attrib["link"] == "base"
    assert joint_el.find("child").attrib["link"] == "arm"
    assert float(joint_el.find("limit").attrib["lower"]) == pytest.approx(-1.57)


def test_invalid_structure_blocks_export():
    links = [_link("a"), _link("b")]
    joints = [_joint("j1", "a", "b"), _joint("j2", "b", "a")]
    with pytest.raises(ValueError, match="[Cc]ycle"):
        export_urdf("test_robot", links, joints, {})


def test_inertial_origin_is_com_in_link_frame_not_joint_origin():
    root = _export([_link("base"), _link("arm")], [_joint("j1", "base", "arm")])
    origin = _link_el(root, "arm").find("inertial").find("origin")
    assert _vec(origin) == pytest.approx([0, 0, -0.1])
    base_origin = _link_el(root, "base").find("inertial").find("origin")
    assert _vec(base_origin) == pytest.approx([0, 0, 0.1])


def test_joint_origin_is_parent_relative_and_visual_origin_inverts_link_frame():
    links = [_link("base"), _link("arm"), _link("hand")]
    joints = [
        _joint("j1", "base", "arm", xyz=(0, 0, 0.2)),
        _joint("j2", "arm", "hand", xyz=(0, 0, 0.5)),
    ]
    root = _export(links, joints)
    j2 = next(j for j in root.findall("joint") if j.find("child").attrib["link"] == "hand")
    assert _vec(j2.find("origin")) == pytest.approx([0, 0, 0.3])
    for tag in ("visual", "collision"):
        o = _link_el(root, "arm").find(tag).find("origin")
        assert _vec(o) == pytest.approx([0, 0, -0.2])


def test_rotated_joint_expresses_inertial_in_rotated_frame():
    links = [_link("base"), _link("arm", com=(1.0, 0.0, 0.0),
                                  inertia=[1.0, 2.0, 3.0, 0.0, 0.0, 0.0])]
    joints = [_joint("j1", "base", "arm", xyz=(0, 0, 0), rpy=(0, 0, math.pi / 2))]
    joints[0].axis = (1.0, 0.0, 0.0)
    root = _export(links, joints)
    assert _vec(root.find("joint").find("axis")) == pytest.approx([0, -1, 0], abs=1e-9)
    inertial = _link_el(root, "arm").find("inertial")
    # world x axis is the link's -y axis after a +90deg yaw
    assert _vec(inertial.find("origin")) == pytest.approx([0, -1, 0], abs=1e-9)
    inertia = inertial.find("inertia").attrib
    assert float(inertia["ixx"]) == pytest.approx(2.0)
    assert float(inertia["iyy"]) == pytest.approx(1.0)
    assert float(inertia["izz"]) == pytest.approx(3.0)


def test_export_package_zip_layout_and_meshes():
    box = trimesh.creation.box(extents=(1, 2, 3))
    body = Body(id="base_body", name="base",
                vertices=[tuple(v) for v in box.vertices],
                triangles=[tuple(t) for t in box.faces], faces=[])
    links = [_link("base"), _link("arm")]
    joints = [_joint("j1", "base", "arm")]
    data = export_package("bot", links, joints,
                          {"base": [body], "arm": [body]})
    z = zipfile.ZipFile(io.BytesIO(data))
    names = set(z.namelist())
    assert {"bot/urdf/bot.urdf", "bot/package.xml",
            "bot/meshes/base.stl", "bot/meshes/arm.stl"} <= names
    root = ET.fromstring(z.read("bot/urdf/bot.urdf"))
    mesh = _link_el(root, "base").find("visual").find("geometry").find("mesh")
    assert mesh.attrib["filename"] == "package://bot/meshes/base.stl"
    assert ET.fromstring(z.read("bot/package.xml")).find("name").text == "bot"
    m = trimesh.load(io.BytesIO(z.read("bot/meshes/base.stl")), file_type="stl")
    assert m.extents == pytest.approx([1, 2, 3])


def _export_err(links, joints, name="bot", bodies=None):
    bodies = {l.id: [_body(l.id)] for l in links} if bodies is None else bodies
    with pytest.raises(ValueError) as e:
        export_urdf(name, links, joints, bodies)
    return str(e.value)


def test_bad_robot_name_rejected():
    assert "../evil" in _export_err([_link("a")], [], name="../evil")


def test_bad_or_duplicate_link_names_rejected():
    a, b = _link("a"), _link("b")
    b.name = "a"
    assert "Duplicate" in _export_err([a, b], [_joint("j1", "a", "b")])
    c = _link("c")
    c.name = "x/y"
    assert "x/y" in _export_err([c], [])


@pytest.mark.parametrize("t", ["cylindrical", "spherical"])
def test_unrepresentable_joint_type_rejected(t):
    j = _joint("j1", "a", "b")
    j.type = t
    assert "j1" in _export_err([_link("a"), _link("b")], [j])


def test_missing_limits_rejected():
    j = _joint("j1", "a", "b")
    j.limit_upper = None
    msg = _export_err([_link("a"), _link("b")], [j])
    assert "j1" in msg and "limit" in msg


def test_link_without_mesh_vertices_rejected():
    links = [_link("a")]
    empty = Body(id="a_body", name="a", vertices=[], triangles=[], faces=[])
    assert "'a'" in _export_err(links, [], bodies={"a": [empty]})


@pytest.mark.parametrize("t", ["revolute", "continuous", "prismatic", "planar"])
def test_joint_type_needing_axis_without_axis_rejected(t):
    j = _joint("j1", "a", "b")
    j.type, j.axis = t, None
    msg = _export_err([_link("a"), _link("b")], [j])
    assert "j1" in msg and "axis" in msg
