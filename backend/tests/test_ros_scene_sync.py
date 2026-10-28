import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "scene_sync_core", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "scene_sync.py")
core = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(core)   # scene_sync imports rclpy lazily, inside main()


def test_only_moved_and_not_carried_objects_update():
    last = {"a": (0, 0, 0, 0, 0, 0, 1), "b": (1, 0, 0, 0, 0, 0, 1)}
    now = {"a": (0, 0, 0.001, 0, 0, 0, 1), "b": (1, 0.05, 0, 0, 0, 0, 1), "c": (2, 0, 0, 0, 0, 0, 1)}
    assert core.moved(now, last) == {"b": now["b"], "c": now["c"]}


def test_rotation_counts_as_moved():
    last = {"a": (0, 0, 0, 0, 0, 0, 1)}
    assert "a" in core.moved({"a": (0, 0, 0, 0, 0, 0.0998, 0.995)}, last)


def test_same_rotation_with_flipped_quaternion_sign_is_not_moved():
    last = {"a": (0, 0, 0, 0, 0, 0, 1)}
    assert core.moved({"a": (0, 0, 0, 0, 0, 0, -1)}, last) == {}
