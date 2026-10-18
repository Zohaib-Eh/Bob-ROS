import numpy as np
import pytest

from app.export.mobile import drive_info, fit_wheel
from app.models import Mobile
from tests.ros_helpers import _boxes_body, _wheel_body, mobile_arm


def test_a_cylinder_wheel_fits_round_and_a_box_does_not():
    w = _wheel_body("w", (0.1, 0.25, 0.08))
    fit = fit_wheel(w.vertices, (0.0, 1.0, 0.0), (0.1, 0.25, 0.08))
    assert fit.round and fit.radius == pytest.approx(0.08, rel=0.01) and fit.width == pytest.approx(0.04)
    assert fit.center == pytest.approx((0.1, 0.25, 0.08))
    box = _boxes_body("b", [((0.0, 0.23, 0.0), (0.16, 0.27, 0.16))])
    assert not fit_wheel(box.vertices, (0.0, 1.0, 0.0), (0.08, 0.25, 0.08)).round


def test_drive_info_reads_sides_size_ground_and_the_caster():
    links, joints, bodies, _, mobile = mobile_arm()
    d = drive_info(links, joints, bodies, mobile)
    assert d.drive == "diff"
    assert (d.left, d.right) == (["j_wheel_l"], ["j_wheel_r"])
    assert d.radius == pytest.approx(0.08, rel=0.01) and d.separation == pytest.approx(0.5)
    assert d.ground == pytest.approx(0.0, abs=1e-3)
    assert len(d.casters) == 1
    x, y, z, r = d.casters[0]
    assert (x, y) == pytest.approx((-0.2, 0.0), abs=0.01) and z - r == pytest.approx(d.ground, abs=1e-6)
    lo, hi = np.array(d.footprint[:2]), np.array(d.footprint[2:])   # chassis and wheels, not the arm
    assert lo == pytest.approx((-0.25, -0.27), abs=0.01) and hi == pytest.approx((0.25, 0.27), abs=0.01)
    assert d.center == pytest.approx((0.1, 0.0)) and d.height == pytest.approx(0.24, abs=1e-3)   # the arm's bolted-on base box is part of it


def test_skid_steer_and_the_mistakes_a_user_can_make():
    links, joints, bodies, _, mobile = mobile_arm(arm=False, skid=True)
    d = drive_info(links, joints, bodies, mobile)
    assert d.drive == "skid" and len(d.left) == 2 and d.casters == []
    turned = [j.model_copy(update={"axis": (1.0, 0.0, 0.0)}) if j.id == "j_wheel_l0" else j for j in joints]
    with pytest.raises(ValueError, match="along y"):
        drive_info(links, turned, bodies, mobile)
    with pytest.raises(ValueError, match="left"):    # the sides swapped
        drive_info(links, joints, bodies, Mobile(left_joint_ids=mobile.right_joint_ids, right_joint_ids=mobile.left_joint_ids))
    with pytest.raises(ValueError, match="not a joint on the base"):
        drive_info(links, joints, bodies, Mobile(left_joint_ids=["nope"], right_joint_ids=mobile.right_joint_ids))
