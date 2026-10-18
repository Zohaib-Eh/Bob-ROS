import numpy as np
import trimesh

from app.importers.step_importer import import_cad


def test_gripper_step_imports_three_separate_bodies(gripper_step_path):
    bodies, _ = import_cad(gripper_step_path)
    by_name = {b.name: b for b in bodies}
    assert set(by_name) == {"gripper_base", "finger_l", "finger_r"}
    assert all(b.status == "ok" for b in bodies)
    for n in ("finger_l", "finger_r"):  # L-shaped: a single convex hull would fill the corner
        m = trimesh.Trimesh(np.array(by_name[n].vertices), np.array(by_name[n].triangles))
        assert not m.is_convex
    xl = np.array(by_name["finger_l"].vertices)[:, 0].max()
    xr = np.array(by_name["finger_r"].vertices)[:, 0].min()
    assert abs((xr - xl) - 0.05) < 1e-6  # flat inner pads 50 mm apart; the feet turn outward
