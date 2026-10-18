import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location(
    "grip_logic", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "grip_logic.py")
gl = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gl)


def test_outcome_held_when_the_fingers_stopped_short_of_closed():
    assert gl.outcome(width=0.010, closed_width=0.0) == "held"
    assert gl.outcome(width=0.0015, closed_width=0.0) == "empty"     # within 2 mm of fully closed
    assert gl.outcome(width=0.0025, closed_width=0.0) == "held"


def test_slipped_means_drifted_more_than_a_centimetre():
    T = np.eye(4)
    moved = lambda d: np.block([[np.eye(3), np.array([[d], [0], [0]])], [np.zeros((1, 3)), np.ones((1, 1))]])
    assert not gl.slipped(T, moved(0.009))
    assert gl.slipped(T, moved(0.011))


def test_yaw_rotation_lines_the_jaws_up_with_the_requested_direction():
    # tool pointing down (z = -world z), jaws closing along tool x
    R_down = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    R = gl.yaw_rotation(R_down, np.array([1.0, 0.0, 0.0]), math.pi / 2)
    jaws = R @ [1.0, 0.0, 0.0]
    assert jaws[:2] == pytest.approx([0.0, 1.0], abs=1e-9)
    assert (R @ [0.0, 0.0, 1.0]) == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)   # still pointing down


def test_extent_is_the_objects_size_across_the_jaws():
    box12 = {"shape": "box", "size": [0.12, 0.04, 0.04], "rpy": [0, 0, 0]}
    assert gl.extent(box12, (1.0, 0.0, 0.0)) == pytest.approx(0.12)
    assert gl.extent(box12, (0.0, 1.0, 0.0)) == pytest.approx(0.04)     # jaws across its narrow side
    can = {"shape": "cylinder", "size": [0.05, 0.1], "rpy": [0, 0, 0]}
    assert gl.extent(can, (1.0, 0.0, 0.0)) == pytest.approx(0.1)        # 0.1 diameter across the axis


def test_size_problem_says_why_an_object_cannot_be_gripped():
    # an AirTAC HFK40's bare jaws: 30 mm apart closed, 60 open
    assert gl.size_problem("box", 0.04, 0.06, 0.03) is None
    wide = gl.size_problem("box", 0.08, 0.06, 0.03)
    assert "'box' is 80 mm across the jaws" in wide and "open only to 60 mm" in wide
    narrow = gl.size_problem("box", 0.02, 0.06, 0.03)
    assert "'box' is 20 mm across the jaws" in narrow and "still 30 mm apart" in narrow and "fingers" in narrow
