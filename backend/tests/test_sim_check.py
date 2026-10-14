import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.sim import check, child
from tests.ros_helpers import _body, _joint, _link

client = TestClient(app)


def _req(max_effort=None, hand_density=1000.0):
    links = [_link("base"), _link("arm"), _link("hand").model_copy(update={"density_kg_m3": hand_density})]
    j2 = _joint("j2", "arm", "hand", (0.5, 0.0, 0.2), (0.0, 1.0, 0.0))
    if max_effort is not None:
        j2 = j2.model_copy(update={"max_effort": max_effort})
    joints = [_joint("j1", "base", "arm", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0)), j2]
    bodies = {"base": [_body("base", (0.0, 0.0, 0.0))], "arm": [_body("arm", (0.25, 0.0, 0.2))],
              "hand": [_body("hand", (0.7, 0.0, 0.2))]}
    return links, joints, bodies


def test_suggested_motors_hold_pose():
    assert check.physics_check("bot", *_req()) == {"ok": True, "kind": "ok", "message": "Holds pose."}


def test_a_weak_motor_sags_and_names_its_link():
    out = check.physics_check("bot", *_req(max_effort=0.01))
    assert out["ok"] is False and out["kind"] == "sag" and out["link"] == "hand" and out["joint"] == "hand_joint"
    assert out["has"] == 0.01 and out["needed"] > 0.01 and "hand sags" in out["message"]


def test_a_single_link_robot_holds_pose():
    out = check.physics_check("bot", [_link("base")], [], {"base": [_body("base", (0, 0, 0))]})
    assert out["ok"] is True


def test_a_zero_mass_link_is_reported_not_raised():
    out = check.physics_check("bot", *_req(hand_density=0.0))
    assert out["ok"] is False and out["kind"] in ("load", "unstable") and "hand" in out["message"]


def test_check_and_suggest_routes():
    links, joints, bodies = _req()
    body = {"robot_name": "bot", "links": [l.model_dump() for l in links], "joints": [j.model_dump() for j in joints],
            "bodies_by_link": {k: [b.model_dump() for b in v] for k, v in bodies.items()}}
    r = client.post("/simulate/check", json=body)
    assert r.status_code == 200 and r.json()["ok"] is True
    s = client.post("/suggest-efforts", json=body)
    assert s.status_code == 200 and set(s.json()) == {"j1", "j2"}


def _boom():
    import os
    os._exit(3)  # a hard crash, like a native library abort


def test_a_crashed_child_raises_child_failed():
    with pytest.raises(child.ChildFailed):
        child.run_in_child(_boom, timeout=30)
