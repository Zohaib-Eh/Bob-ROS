import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    "scene", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "scene.py")
scene = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scene)


def box(**kw):
    return {"name": "part_a", "shape": "box", "size": [0.05, 0.05, 0.05], "xyz": [0.5, 0, 0.025], **kw}


def test_defaults_fill_in_and_mass_comes_from_wood_density():
    (o,) = scene.validate([box()], "SCENE")
    assert o["rpy"] == [0.0, 0.0, 0.0] and o["fixed"] is False and o["friction"] is None
    assert o["mass"] == pytest.approx(700 * 0.05 ** 3)


def test_cylinder_volume_uses_radius_and_height():
    (o,) = scene.validate([{"name": "can", "shape": "cylinder", "size": [0.03, 0.1], "xyz": [0, 0, 0.05]}], "SCENE")
    assert o["mass"] == pytest.approx(700 * 3.141592653589793 * 0.03 ** 2 * 0.1)


@pytest.mark.parametrize("bad, words", [
    (box(colour="red"), "unknown field 'colour'"),
    (box(shape="sphere"), "shape must be box or cylinder"),
    (box(size=[0.05, 0, 0.05]), "size must be 3 numbers > 0"),
    (box(mass=0), "mass must be > 0"),
    (box(fixed=True, mass=1.0), "fixed objects have no mass"),
    (box(name="2fast"), "name must start with a letter"),
    (box(name="a b"), "name must start with a letter"),
    ({"shape": "box"}, "missing 'name'"),
])
def test_bad_objects_say_what_is_wrong(bad, words):
    with pytest.raises(scene.SceneError, match=words):
        scene.validate([bad], "SCENE")


def test_merge_rejects_duplicate_names_and_keeps_workcell_first():
    table = {"name": "table", "shape": "box", "size": [1, 1, 0.05], "xyz": [0.6, 0, 0.3], "fixed": True}
    out = scene.merge(scene.validate([table], "workcell.yaml"), scene.validate([box()], "SCENE"))
    assert [o["name"] for o in out] == ["table", "part_a"]
    with pytest.raises(scene.SceneError, match="'part_a' is in both workcell.yaml and SCENE"):
        scene.merge(scene.validate([box()], "workcell.yaml"), scene.validate([box()], "SCENE"))


def test_scene_reads_dict_calls_and_literals_without_running_code():
    src = ("import os\nos.system('boom')\n"
           "SCENE = [dict(name='a', shape='box', size=(0.05, 0.05, 0.05), xyz=(0.5, 0, 0.025), mass=0.2),\n"
           "         {'name': 'b', 'shape': 'cylinder', 'size': [0.02, 0.1], 'xyz': [0.4, 0.1, 0.05]}]\n")
    objs = scene.read_scene(src)
    assert [o["name"] for o in objs] == ["a", "b"] and objs[0]["size"] == [0.05, 0.05, 0.05]


def test_scene_with_computed_values_names_the_line():
    with pytest.raises(scene.SceneError, match=r"SCENE must be plain values \(line 2\)"):
        scene.read_scene("x = 0.5\nSCENE = [dict(name='a', shape='box', size=(x, x, x), xyz=(0, 0, 0))]\n")


def test_no_scene_and_empty_workcell_mean_no_objects():
    assert scene.read_scene("from robot import Robot\nRobot().home()\n") == []
    assert scene.load_workcell(None) == [] and scene.load_workcell("") == [] and scene.load_workcell("# none\n") == []


def test_workcell_yaml_typo_is_a_scene_error():  # review I-2
    with pytest.raises(scene.SceneError, match="workcell.yaml is not valid YAML"):
        scene.load_workcell("- {name: table, size: [1,2}\n")


def test_program_with_a_syntax_error_has_no_scene_so_python_reports_it():  # review I-6
    assert scene.read_scene("SCENE = [\nprint(\n") == []


def test_workcell_must_be_a_list():
    with pytest.raises(scene.SceneError, match="workcell.yaml must be a list of objects"):
        scene.load_workcell("table: {}\n")
