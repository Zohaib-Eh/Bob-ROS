import json
import math

import pytest
import trimesh

from app.models import Body
from app.validate.sweep import reachable, sweep_limits


def body(name, mesh):
    return Body(id=name, name=name, vertices=[tuple(map(float, v)) for v in mesh.vertices],
                triangles=[tuple(map(int, f)) for f in mesh.faces])


def box(name, lo, hi):
    m = trimesh.creation.box(extents=[h - l for l, h in zip(lo, hi)])
    m.apply_translation([(l + h) / 2 for l, h in zip(lo, hi)])
    return body(name, m)


Z = (0.0, 0.0, 1.0)
O = (0.0, 0.0, 0.0)


def test_hinge_stops_one_degree_before_the_post_on_both_sides():
    # arm along +x pivoting about z at the origin; a post stands near +y at radius 0.75..0.85
    arm = box("arm", (0, -0.02, 0.4), (1, 0.02, 0.6))
    post = box("post", (-0.05, 0.75, 0), (0.05, 0.85, 1))
    r = sweep_limits("revolute", O, Z, [arm], [("post", [post])])
    # first contact: post corner (±0.05, 0.75) meets the arm's leading edge (half-width 0.02)
    contact_up = math.degrees(math.atan2(0.75, 0.05) - math.asin(0.02 / math.hypot(0.05, 0.75)))
    contact_down = -math.degrees(2 * math.pi - math.atan2(0.75, -0.05) - math.asin(0.02 / math.hypot(0.05, 0.75)))
    assert abs(math.degrees(r["upper"]) - (contact_up - 1)) < 0.5
    assert abs(math.degrees(r["lower"]) - (contact_down + 1)) < 0.5
    assert r["blocked_by"] == {"lower": "post", "upper": "post"}
    assert r["free"] is False and r["at_rest_overlap"] == []


def test_resting_contact_is_not_a_collision_and_a_free_spin_is_free():
    plate = box("plate", (-1, -1, -0.1), (1, 1, 0))
    puck = box("puck", (-0.2, -0.2, 0), (0.2, 0.2, 0.1))   # sits on the plate
    r = sweep_limits("revolute", O, Z, [puck], [("plate", [plate])])
    assert r["free"] is True
    assert (r["lower"], r["upper"]) == (-math.pi, math.pi)


def test_interference_at_rest_is_reported_and_no_limits_are_given():
    plate = box("plate", (-1, -1, -0.1), (1, 1, 0))
    puck = box("puck", (-0.2, -0.2, -0.001), (0.2, 0.2, 0.1))   # 1 mm into the plate
    r = sweep_limits("revolute", O, Z, [puck], [("plate", [plate])])
    assert r["at_rest_overlap"] == ["plate"]
    assert r["lower"] is None and r["upper"] is None


def test_slider_stops_one_millimetre_before_the_wall():
    # travel is limited to the slider's own box diagonal (0.173 m), so the wall sits within it
    wall = box("wall", (0.15, -1, -1), (0.25, 1, 1))
    slider = box("slider", (0, -0.05, -0.05), (0.1, 0.05, 0.05))
    r = sweep_limits("prismatic", O, (1.0, 0.0, 0.0), [slider], [("wall", [wall])])
    # contact after ~0.05 m (box-corner normals shrink ~0.29 mm per axis), minus the 1 mm margin and ≤0.25 mm bisection slack
    assert 0.0485 <= r["upper"] <= 0.0495
    assert r["blocked_by"] == {"lower": None, "upper": "wall"}
    assert r["lower"] == pytest.approx(-math.sqrt(3 * 0.1 ** 2))   # full travel: the slider's box diagonal


def test_no_obstacles_means_free_or_full_travel():
    part = box("part", (0, 0, 0), (0.1, 0.1, 0.1))
    assert sweep_limits("revolute", O, Z, [part], [])["free"] is True
    r = sweep_limits("prismatic", O, Z, [part], [])
    assert r["lower"] == pytest.approx(-r["upper"]) and r["upper"] > 0


def test_open_meshes_still_collide():
    m = trimesh.creation.box(extents=(0.1, 2, 2))
    m.apply_translation((0.2, 0, 0))
    m.update_faces([i for i in range(len(m.faces)) if m.face_normals[i][0] > -0.5])   # drop the face towards the slider
    wall = body("wall", m)
    slider = box("slider", (0, -0.05, -0.05), (0.1, 0.05, 0.05))
    r = sweep_limits("prismatic", O, (1.0, 0.0, 0.0), [slider], [("wall", [wall])])
    assert r["blocked_by"]["upper"] == "wall"


def test_reachable_keeps_only_obstacles_the_swing_can_reach():
    # moving centre at radius 1 around z: it sweeps a circle in the z = 0 plane
    assert reachable((1, 0, 0), 0.1, (0, 1, 0), 0.05, O, Z, "revolute", 0)
    assert not reachable((1, 0, 0), 0.1, (0, 0, 0), 0.1, O, Z, "revolute", 0)
    assert not reachable((1, 0, 0), 0.1, (0, 1, 5), 0.1, O, Z, "revolute", 0)
    assert reachable((0, 0, 0), 0.05, (0.9, 0.05, 0), 0.01, O, (1, 0, 0), "prismatic", 1.0)
    assert not reachable((0, 0, 0), 0.05, (2, 0, 0), 0.1, O, (1, 0, 0), "prismatic", 1.0)


def test_bad_input_raises():
    part = box("part", (0, 0, 0), (1, 1, 1))
    with pytest.raises(ValueError, match="revolute and prismatic"):
        sweep_limits("fixed", O, Z, [part], [])
    with pytest.raises(ValueError, match="axis"):
        sweep_limits("revolute", O, (0, 0, 0), [part], [])
    empty = Body(id="e", name="e", vertices=[], triangles=[])
    with pytest.raises(ValueError, match="no geometry"):
        sweep_limits("revolute", O, Z, [empty], [])


from fastapi.testclient import TestClient

from app.main import app

client = TestClient(app)


def _payload(kind="prismatic", axis=(1.0, 0.0, 0.0)):
    wall = box("wall", (0.15, -1, -1), (0.25, 1, 1))
    slider = box("slider", (0, -0.05, -0.05), (0.1, 0.05, 0.05))
    return {"joint": {"type": kind, "origin_xyz": [0, 0, 0], "axis": list(axis)},
            "moving": [slider.model_dump()], "obstacles": [{"link_id": "L-wall", "bodies": [wall.model_dump()]}]}


def test_route_returns_limits_and_the_blocking_link():
    r = client.post("/sweep-limits", json=_payload())
    assert r.status_code == 200
    assert r.json()["blocked_by"]["upper"] == "L-wall"
    assert 0.0485 <= r.json()["upper"] <= 0.0495


def test_route_rejects_bad_requests():
    assert client.post("/sweep-limits", json=_payload(kind="fixed")).status_code == 422
    assert client.post("/sweep-limits", json=_payload(axis=(0, 0, 0))).status_code == 422
    nan = _payload()
    nan["joint"]["axis"] = [float("nan"), 0, 0]
    assert client.post("/sweep-limits", content=json.dumps(nan), headers={"content-type": "application/json"}).status_code == 422
    bad = _payload()
    del bad["joint"]["axis"]
    assert client.post("/sweep-limits", json=bad).status_code == 422
