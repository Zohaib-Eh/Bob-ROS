import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location(
    "drive_logic", Path(__file__).resolve().parents[1] / "app" / "ros" / "templates" / "drive_logic.py")
dl = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dl)

CENTER = (0.1, 0.0)   # the fixture's axles: 0.1 m ahead of the base frame


def _drive(start, goal_xy, goal_yaw, slip=1.0, dt=0.05, limit=60.0, reverse=False, log=None):
    """drive_to's loop on a differential drive that turns about CENTER (slip < 1: the wheels lose grip)."""
    target, final = dl.plan(start, goal_xy, goal_yaw, CENTER, reverse)
    pose, phase, t = np.array(start, float), "face", 0.0
    while phase != "done" and t < limit:
        v, w, phase = dl.command(phase, pose, target, final, CENTER, 0.3, reverse)
        if log is not None:
            log.append((v, pose[2]))
        p = dl.pivot(pose, CENTER) + slip * v * dt * np.array([math.cos(pose[2]), math.sin(pose[2])])
        yaw = pose[2] + slip * w * dt
        c, s = math.cos(yaw), math.sin(yaw)
        pose = np.array([p[0] - c * CENTER[0] + s * CENTER[1], p[1] - s * CENTER[0] - c * CENTER[1], yaw])
        t += dt
    return pose, phase


@pytest.mark.parametrize("goal", [((1.0, 0.5), math.pi / 2), ((-0.5, 0.0), None), ((0.0, 0.0), math.pi), ((2.0, -1.0), 0.0)])
def test_drive_to_ends_at_the_goal_frame_not_just_near_it(goal):
    (gx, gy), gyaw = goal
    pose, phase = _drive((0.0, 0.0, 0.0), (gx, gy), gyaw)
    assert phase == "done"
    assert math.hypot(pose[0] - gx, pose[1] - gy) < dl.POS_TOL + 0.005   # the base frame, though it turns about its axles
    if gyaw is not None:
        assert abs(dl.wrap(pose[2] - gyaw)) < dl.YAW_TOL + 1e-3


def test_reverse_backs_up_without_turning_round():
    log = []
    pose, phase = _drive((1.0, 0.0, 0.0), (0.4, 0.0), None, reverse=True, log=log)
    assert phase == "done" and abs(pose[0] - 0.4) < 0.025 and abs(dl.wrap(pose[2])) < dl.YAW_TOL + 1e-3
    assert max(abs(dl.wrap(yaw)) for _, yaw in log) < 0.05 and min(v for v, _ in log) < -0.2   # it backed up, facing +x


def test_slipping_wheels_still_get_there():   # closed loop on the true pose, not on the commands
    pose, phase = _drive((0.0, 0.0, 0.0), (1.0, 1.0), 0.0, slip=0.7)
    assert phase == "done" and math.hypot(pose[0] - 1, pose[1] - 1) < 0.03


def test_the_path_check_sweeps_the_real_outline():
    fp = (-0.25, -0.27, 0.25, 0.27)
    start = (0.0, 0.0, 0.0)
    table = {"name": "table", "xy": (1.5, 0.0), "half": (0.2, 0.3), "yaw": 0.0, "z0": 0.0}      # front edge x 1.3
    shelf = {**table, "name": "shelf", "z0": 0.5}                                                 # above the base
    parcel = {"name": "parcel", "xy": (0.0, 0.0), "half": (0.05, 0.05), "yaw": 0.0, "z0": 0.18}   # riding on it
    go = lambda x, y, yaw: dl.sweep(start, *dl.plan(start, (x, y), yaw, CENTER), CENTER)
    assert dl.blocker(go(0.85, 0.0, 0.0), fp, [shelf, parcel, table], 0.24) is None   # up to it: front 1.10 + margin < 1.30
    assert dl.blocker(go(2.5, 0.0, 0.0), fp, [shelf, parcel, table], 0.24) == "table"  # through it
    assert dl.blocker(go(0.95, 0.0, 0.0), fp, [table], 0.24) is None                   # front 1.20 + margin < 1.30
    assert dl.blocker(go(0.95, 0.0, math.pi), fp, [table], 0.24) == "table"            # a half turn there swings the rear corner into it
    assert dl.blocker(go(0.0, 2.0, None), fp, [table], 0.24) is None                   # off to the side
    # the container program's cases (mobi_drive.py): from (0.8, 0.2) facing +y
    s2 = (0.8, 0.2, math.pi / 2)
    leg = lambda x, y, yaw: dl.sweep(s2, *dl.plan(s2, (x, y), yaw, CENTER), CENTER)
    assert dl.blocker(leg(0.8, -1.0, -math.pi / 2), fp, [table], 0.24) is None   # turns 19 cm clear of it
    assert dl.blocker(leg(2.5, 0.2, 0.0), fp, [table], 0.24) == "table"
    poses = go(1.0, 0.5, math.pi / 2)
    assert poses[0] == pytest.approx(start) and poses[-1] == pytest.approx((1.0, 0.5, math.pi / 2), abs=1e-6)


def test_remaining_is_metres_driving_and_radians_turning():
    pose, target = (0.0, 0.0, 0.0), (1.1, 0.0)
    assert dl.remaining("go", pose, target, 0.0, CENTER) == pytest.approx(1.0)
    assert dl.remaining("face", pose, (0.1, 1.0), 0.0, CENTER) == pytest.approx(math.pi / 2)
    assert dl.remaining("turn", pose, target, -math.pi / 2, CENTER) == pytest.approx(math.pi / 2)


def test_stuck_means_no_progress_for_two_seconds():
    assert not dl.stuck([(0.0, 1.0), (1.0, 0.9), (2.5, 0.8)], 2.5)
    assert dl.stuck([(0.0, 1.0), (1.0, 0.999), (2.5, 0.998)], 2.5)
