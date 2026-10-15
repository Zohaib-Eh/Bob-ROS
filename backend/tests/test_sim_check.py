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


def test_a_very_strong_motor_on_a_light_link_still_holds():  # review I-1
    assert check.physics_check("bot", *_req(max_effort=10000))["ok"] is True


def test_a_tiny_arm_holds_with_suggested_motors():  # review I-1: 1 cm links, minimum effort 1 N·m
    links = [_link("base"), _link("arm"), _link("hand")]
    joints = [_joint("j1", "base", "arm", (0.0, 0.0, 0.02), (0.0, 0.0, 1.0)),
              _joint("j2", "arm", "hand", (0.05, 0.0, 0.02), (0.0, 1.0, 0.0))]
    bodies = {"base": [_body("base", (0.0, 0.0, 0.0), 0.01)], "arm": [_body("arm", (0.025, 0.0, 0.02), 0.01)],
              "hand": [_body("hand", (0.07, 0.0, 0.02), 0.01)]}
    out = check.physics_check("bot", links, joints, bodies)
    assert out["ok"] is True, out


def test_a_blow_up_names_the_link_that_caused_it():  # review I-2
    import io
    import re
    import zipfile
    from app.export.urdf_writer import export_package
    z = zipfile.ZipFile(io.BytesIO(export_package("bot", *_req())))
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if "/meshes/" in n}
    xml = z.read("bot/mjcf/bot.xml").decode()
    # a near-massless hand under a strong motor: MuJoCo hits bad accelerations and resets the state
    xml = re.sub(r'(<body name="hand"[^>]*>\s*)<inertial[^>]*/>',
                 r'\1<inertial pos="0.2 0 0" mass="1e-9" diaginertia="1e-15 1e-15 1e-15" />', xml)
    xml = re.sub(r'<position name="hand_joint_motor"[^>]*/>', '<position name="hand_joint_motor" joint="hand_joint" kp="1e6" />', xml)
    out = check.hold_check(xml, meshes)
    assert out["kind"] == "unstable" and out["link"] == "hand", out
    assert re.search(r"after 0.00[1-9] s", out["message"]), out["message"]  # a real time, not "0.0 s"


def test_building_models_from_two_threads_is_safe():  # review I-3
    import threading
    from app.export.urdf_writer import export_package
    errors = []

    def work(dx):
        links, joints, bodies = _req()
        bodies = {k: [b.model_copy(update={"vertices": [(x + dx, y, z) for x, y, z in b.vertices]}) for b in v]
                  for k, v in bodies.items()}
        for _ in range(15):
            try:
                export_package("bot", links, joints, bodies)
            except Exception as exc:  # noqa: BLE001 - any failure is the finding
                errors.append(repr(exc))
    ts = [threading.Thread(target=work, args=(dx,)) for dx in (0.0, 0.3)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    assert errors == []


def test_a_timed_out_check_says_so(monkeypatch):  # review M-4
    from app.routes import sim_routes

    def slow(*a, **k):
        raise child.ChildFailed("timed out")
    monkeypatch.setattr(sim_routes, "run_in_child", slow)
    links, joints, bodies = _req()
    body = {"robot_name": "bot", "links": [l.model_dump() for l in links], "joints": [j.model_dump() for j in joints],
            "bodies_by_link": {k: [b.model_dump() for b in v] for k, v in bodies.items()}}
    out = client.post("/simulate/check", json=body).json()
    assert out["kind"] == "crash" and "longer than 60 s" in out["message"]
