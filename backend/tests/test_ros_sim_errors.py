import importlib.util
import math
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "sim_errors", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "sim_errors.py")
se = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(se)


def test_lag_names_worst_joint_load_and_limit():
    msg = se.lag_message(["j1", "j2"], [0.0, 1.0], [0.01, 1.0 - math.radians(12)],
                         {"j1": 40, "j2": 150}, {"part_a": 0.2}, {"j1": True, "j2": True})
    assert msg == ("j2 fell 12° behind while carrying part_a (0.2 kg). Motor limit 150 N·m: "
                   "raise Max torque in Simulate, or use a lighter load.")


def test_lag_without_load_and_prismatic_units():
    msg = se.lag_message(["s"], [0.5], [0.47], {"s": 80}, {}, {"s": False})
    assert msg == ("s fell 30 mm behind. Motor limit 80 N: raise Max torque in Simulate, "
                   "or check what it hit.")


def test_blow_up_names_the_link_and_ignores_normal_motion():
    assert se.blow_up(["j1", "j2"], [0.1, 0.2], [1.0, -2.0], {"j1": "l1", "j2": "l2"}) is None
    assert se.blow_up(["j1", "j2"], [0.1, float("nan")], [1.0, 0.0], {"j1": "l1", "j2": "l2"}) == \
        "Simulation blew up: check l2's mass and inertia."
    assert se.blow_up(["j1", "j2"], [0.1, 0.2], [900.0, 1.0], {"j1": "l1", "j2": "l2"}) == \
        "Simulation blew up: check l1's mass and inertia."
