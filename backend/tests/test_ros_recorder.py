"""Run replay's recorder (templates/recorder.py): the pure Trace part, tested without ROS."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

_spec = importlib.util.spec_from_file_location(
    "recorder", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "recorder.py")
recorder = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(recorder)


def tf(child, parent, x):
    return NS(child_frame_id=child, header=NS(frame_id=parent),
              transform=NS(translation=NS(x=x, y=0.0, z=0.0), rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))


def test_frames_hold_the_latest_transform_per_link_and_the_object_poses():
    t = recorder.Trace("bot", "programs/task_001.py", "base", start=100.0)
    t.tf([tf("arm", "base", 0.1), tf("hand", "arm", 0.2)])
    t.tf([tf("arm", "base", 0.3)])                       # newer pose of the same link replaces the old
    t.poses["box"] = [0.5, 0, 0.2, 0, 0, 0, 1]
    t.sample(100.05)
    [f] = t.frames
    assert f["t"] == 0.05 and sorted(f["tf"]) == [["arm", "base", 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], ["hand", "arm", 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
    assert f["poses"] == {"box": [0.5, 0, 0.2, 0, 0, 0, 1]}


def test_scenes_are_kept_only_when_they_change():
    t = recorder.Trace("bot", "p", "base", start=0.0)
    a, b = {"world": {"collision_objects": [{"id": "box"}]}}, {"world": {"collision_objects": []}}
    for now, s in ((0.2, a), (0.4, a), (0.6, b), (0.8, b)):
        t.scene(now, s)
    assert [x["t"] for x in t.scenes] == [0.2, 0.6]


def test_recording_stops_after_the_cap_and_the_file_says_how_the_run_ended(tmp_path):
    t = recorder.Trace("bot", "programs/task_001.py", "base", start=0.0)
    t.sample(1.0)
    t.sample(recorder.MAX_S + 1)                         # past the cap: not kept
    t.tf([tf("w", "base", 0.0)], static=True)
    p = tmp_path / "runs" / "x.json"
    t.write(p, "failed: Unreachable: too far")
    d = json.loads(p.read_text())
    assert (d["version"], d["robot"], d["program"], d["base"], d["result"]) == (1, "bot", "programs/task_001.py", "base", "failed: Unreachable: too far")
    assert len(d["frames"]) == 1 and d["static"] == [["w", "base", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]


def test_the_trace_file_is_named_after_the_program_and_the_time():
    name = recorder.trace_name("programs/task_007_stack.py", 1792351200.0)
    assert name.startswith("runs/task_007_stack-") and name.endswith("Z.json")


def test_a_run_prints_its_trace_just_before_its_result():
    lib = (Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "runner.py").read_text(encoding="utf-8")
    assert "recorder.begin(" in lib and lib.index("rec.end(") < lib.index('print(f"RESULT: {result}"')
