"""attach() refuses an object the tool is not touching (names the distance), and accepts one just past the flange."""
import numpy as np
from robot import Robot, UnknownObject


def beyond_flange(r, dist):
    """Point `dist` metres out from the tool frame along the flange direction (away from the arm)."""
    q = r._current()
    T = r._kin.fk(q)
    a = T[:3, :3] @ np.asarray(r.chain.joints[-1].axis, float)
    if sum(float(a @ (F[:3, 3] - T[:3, 3])) for F in r._joint_frames(q)[:-1]) > 0:
        a = -a
    return tuple(T[:3, 3] + a * dist)


r = Robot()
r.add_box("far", (0.04, 0.04, 0.04), beyond_flange(r, 0.18))      # nearest face 0.14-0.16 m away
try:
    r.attach("far")
except UnknownObject as exc:
    gap = float(str(exc).split(" is ")[1].split(" m ")[0])
    assert "from the tool" in str(exc) and gap > 0.1, exc
else:
    raise AssertionError("attach should refuse an object ~0.16 m from the tool")
r.add_box("near", (0.04, 0.04, 0.04), beyond_flange(r, 0.025))    # nearest face at most 5 mm away
r.attach("near")
print("attach_reach ok")
