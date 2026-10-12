# Collision Sweep and Drive Joint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the user find joint limits from geometry (a collision sweep on the backend) and drive the moving part through the joint in 3D, Fusion-style, to check and adjust them.

**Architecture:** A pure-Python sweep module (`app/validate/sweep.py`) uses `python-fcl` boolean collision on a moving mesh shrunk inward by 0.5 mm, so resting contact is ignored; a new route `/sweep-limits` exposes it. On the frontend, pure helpers (`ui/drive.ts`) build the request and format results; the Viewer gains a drive drag and a warning tint; the Joint tool gets "Find limits from geometry", a drive slider, a 3D pose label and "Set lower/upper here".

**Tech Stack:** FastAPI, trimesh, python-fcl 0.7.0.11 (PyPI wheel), pytest; React + TypeScript, three.js, Vitest, Playwright.

**Spec:** `docs/superpowers/specs/2026-10-08-collision-sweep-design.md`

## Global Constraints

- Branch `collision-sweep`, main checkout. Backend tests: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`. Frontend: `cd frontend && npm test`, `npx tsc --noEmit`, `npm run e2e`.
- New backend dependency: `python-fcl==0.7.0.11` only (pip into the `cad-to-urdf` env; it is not on conda-forge).
- Tolerance 0.5 mm (moving side shrunk by it); margins 1° (revolute) and 1 mm (slider); revolute coarse step 3°, search to ±360°, bisection to 0.25°; slider step max(1 mm, range/200), bisection to 0.25 mm, range = moving mesh box diagonal.
- The sweep never runs automatically; driving never calls the backend.
- Drive pose is a preview only: URDF zero stays the CAD pose; the pose resets when the tool closes or the joint's axis/origin/motion/parts change.
- Write files as UTF-8.

## Review Focus

- **A sweep result arriving after the user changed the draft** (axis flipped, motion changed, other part picked) must be ignored, not written into the new draft. Test: Task 5 `a result for an outdated draft is ignored`.
- **No obstacles in reach** (single moving link, or everything far away) — revolute reports free, slider reports full travel; no crash. Test: Task 1 `no obstacles means free or full travel`.
- **Open (non-watertight) meshes** — approximate parts are open shells; collisions must still be found. Test: Task 1 `open meshes still collide`.
- **"Set lower/upper here" that would invert the limits** — disabled. Test: Task 5 `set-here never makes lower exceed upper`.
- **Tool closed mid-drive** — preview cleared, warning tint cleared, drive callback unregistered. Test: Task 5 `closing the tool clears the pose, tint and drive`.

---

### Task 1: Backend sweep module

**Files:**
- Create: `backend/app/validate/sweep.py`
- Modify: `backend/pyproject.toml` (dependency)
- Test: `backend/tests/test_sweep.py`

**Interfaces:**
- Produces:
  - `reachable(moving_center, moving_radius, obstacle_center, obstacle_radius, origin, axis, kind: str, travel: float) -> bool`
  - `sweep_limits(kind: str, origin, axis, moving: list[Body], obstacles: list[tuple[str, list[Body]]]) -> dict` returning `{"lower": float|None, "upper": float|None, "free": bool, "blocked_by": {"lower": str|None, "upper": str|None}, "at_rest_overlap": list[str]}`; raises `ValueError` for an unsupported kind, a zero axis, or moving parts without geometry.

- [ ] **Step 1: Install the dependency**

Run: `~/miniforge3/envs/cad-to-urdf/python.exe -m pip install python-fcl==0.7.0.11`
Expected: `Successfully installed python-fcl-0.7.0.11` (or already satisfied).

In `backend/pyproject.toml` add `"python-fcl==0.7.0.11",` to `dependencies` after `"trimesh>=4.2",`.

- [ ] **Step 2: Write the failing tests** — `backend/tests/test_sweep.py`

```python
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
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_sweep.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'app.validate.sweep'`.

- [ ] **Step 4: Implement** — `backend/app/validate/sweep.py`

```python
"""Joint limits from geometry: swing the moving parts until they would hit something.

fcl reports the same penetration depth for touching and deeply overlapping meshes, so
depth can't tell resting contact from a collision. Instead the moving side is shrunk
inward by TOL and only boolean collision is used: touching or <TOL overlap is free,
anything deeper is a hit.
"""
import math

import fcl
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix, translation_matrix

from app.models import Body

TOL = 0.0005                      # m
STEP_RAD = math.radians(3)
FINE_RAD = math.radians(0.25)
MARGIN_RAD = math.radians(1)
STEP_M_MIN = 0.001
FINE_M = 0.00025
MARGIN_M = 0.001


def _mesh(bodies: list[Body]) -> trimesh.Trimesh | None:
    vs, fs, offset = [], [], 0
    for b in bodies:
        if not b.vertices or not b.triangles:
            continue
        vs.append(np.asarray(b.vertices, dtype=float))
        fs.append(np.asarray(b.triangles, dtype=int) + offset)
        offset += len(b.vertices)
    if not vs:
        return None
    return trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=False)


def _object(vertices, faces) -> fcl.CollisionObject:
    bvh = fcl.BVHModel()
    bvh.beginModel(len(vertices), len(faces))
    bvh.addSubModel(np.asarray(vertices, dtype=float), np.asarray(faces, dtype=int))
    bvh.endModel()
    return fcl.CollisionObject(bvh)


def _sphere(m: trimesh.Trimesh) -> tuple[np.ndarray, float]:
    lo, hi = m.bounds
    return (lo + hi) / 2, float(np.linalg.norm(hi - lo)) / 2


def _collides(a: fcl.CollisionObject, b: fcl.CollisionObject) -> bool:
    return fcl.collide(a, b, fcl.CollisionRequest(), fcl.CollisionResult()) > 0


def reachable(moving_center, moving_radius, obstacle_center, obstacle_radius, origin, axis,
              kind: str, travel: float) -> bool:
    """Can the moving part's bounding sphere come near the obstacle's during the sweep?"""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    o, mc, oc = (np.asarray(p, dtype=float) for p in (origin, moving_center, obstacle_center))
    reach = moving_radius + obstacle_radius
    if kind == "prismatic":
        t = float(np.clip(np.dot(oc - mc, a), -travel, travel))
        return float(np.linalg.norm(oc - (mc + t * a))) <= reach
    h = float(np.dot(mc - o, a))
    c0 = o + h * a                               # centre of the circle the moving centre sweeps
    rc = float(np.linalg.norm(mc - c0))
    q = oc - c0
    z = float(np.dot(q, a))
    rho = float(np.linalg.norm(q - z * a))
    return math.hypot(rho - rc, z) <= reach


def sweep_limits(kind: str, origin, axis, moving: list[Body],
                 obstacles: list[tuple[str, list[Body]]]) -> dict:
    if kind not in ("revolute", "prismatic"):
        raise ValueError(f"only revolute and prismatic joints can be swept, not {kind}")
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        raise ValueError("joint axis must have nonzero length")
    a = a / n
    o = np.asarray(origin, dtype=float)
    mv = _mesh(moving)
    if mv is None:
        raise ValueError("the moving parts have no geometry")
    mobj = _object(mv.vertices - mv.vertex_normals * TOL, mv.faces)
    mc, mr = _sphere(mv)
    travel = float(np.linalg.norm(mv.extents))

    obs = []
    for link_id, bodies in obstacles:
        m = _mesh(bodies)
        if m is not None:
            oc, orad = _sphere(m)
            obs.append((link_id, _object(m.vertices, m.faces), oc, orad))

    mobj.setTransform(fcl.Transform())
    overlap = [lid for lid, obj, _, _ in obs if _collides(mobj, obj)]
    if overlap:
        return {"lower": None, "upper": None, "free": False,
                "blocked_by": {"lower": None, "upper": None}, "at_rest_overlap": overlap}
    obs = [x for x in obs if reachable(mc, mr, x[2], x[3], o, a, kind, travel)]

    def hit(value: float) -> str | None:
        T = translation_matrix(a * value) if kind == "prismatic" else rotation_matrix(value, a, o)
        mobj.setTransform(fcl.Transform(T[:3, :3], T[:3, 3]))
        for lid, obj, _, _ in obs:
            if _collides(mobj, obj):
                return lid
        return None

    if kind == "revolute":
        step, fine, margin, full = STEP_RAD, FINE_RAD, MARGIN_RAD, 2 * math.pi
    else:
        step, fine, margin, full = max(STEP_M_MIN, travel / 200), FINE_M, MARGIN_M, travel

    def search(sign: int) -> tuple[float, str] | None:
        prev, v = 0.0, step
        while v <= full + 1e-12:
            lid = hit(sign * v)
            if lid is not None:
                lo, hi = prev, v
                while hi - lo > fine:
                    mid = (lo + hi) / 2
                    if hit(sign * mid) is None:
                        lo = mid
                    else:
                        hi = mid
                return sign * max(lo - margin, 0.0), lid
            prev, v = v, v + step
        return None

    up, down = search(+1), search(-1)
    if kind == "revolute" and up is None:
        return {"lower": -math.pi, "upper": math.pi, "free": True,
                "blocked_by": {"lower": None, "upper": None}, "at_rest_overlap": []}
    upper, upper_by = up if up else (full, None)
    lower, lower_by = down if down else (-full, None)
    return {"lower": lower, "upper": upper, "free": False,
            "blocked_by": {"lower": lower_by, "upper": upper_by}, "at_rest_overlap": []}
```

- [ ] **Step 5: Run the tests and the full suite**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass (201 + 8 new).

- [ ] **Step 6: Commit**

```bash
git add backend/app/validate/sweep.py backend/tests/test_sweep.py backend/pyproject.toml
git commit -m "feat: collision sweep finds joint limits from geometry"
```

---

### Task 2: `/sweep-limits` route

**Files:**
- Modify: `backend/app/routes/joint_routes.py`
- Test: `backend/tests/test_sweep.py` (append)

**Interfaces:**
- Consumes: `sweep_limits` (Task 1).
- Produces: `POST /sweep-limits` with body `{ joint: { type: "revolute"|"prismatic", origin_xyz, axis }, moving: Body[], obstacles: [{ link_id, bodies: Body[] }] }` → the Task 1 dict; 422 on a bad type (pydantic), zero axis or no moving geometry.

- [ ] **Step 1: Write the failing tests** (append to `backend/tests/test_sweep.py`)

```python
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
    bad = _payload()
    del bad["joint"]["axis"]
    assert client.post("/sweep-limits", json=bad).status_code == 422
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q tests/test_sweep.py`
Expected: FAIL — 404 from `/sweep-limits`.

- [ ] **Step 3: Implement** — in `backend/app/routes/joint_routes.py`:

Change `from fastapi import APIRouter` to `from fastapi import APIRouter, HTTPException`, add `from app.validate.sweep import sweep_limits`, and append:

```python
class SweepJoint(BaseModel):
    type: Literal["revolute", "prismatic"]
    origin_xyz: tuple[float, float, float]
    axis: tuple[float, float, float]


class SweepObstacle(BaseModel):
    link_id: str
    bodies: list[Body]


class SweepRequest(BaseModel):
    joint: SweepJoint
    moving: list[Body]
    obstacles: list[SweepObstacle] = []


class SweepBlockers(BaseModel):
    lower: str | None
    upper: str | None


class SweepResponse(BaseModel):
    lower: float | None
    upper: float | None
    free: bool
    blocked_by: SweepBlockers
    at_rest_overlap: list[str]


@router.post("/sweep-limits", response_model=SweepResponse)
def sweep_limits_route(request: SweepRequest) -> SweepResponse:
    try:
        result = sweep_limits(request.joint.type, request.joint.origin_xyz, request.joint.axis,
                              request.moving, [(o.link_id, o.bodies) for o in request.obstacles])
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc
    return SweepResponse(**result)
```

- [ ] **Step 4: Run the full suite**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass.

- [ ] **Step 5: Time it on the ABB** (not committed) — save as `<scratchpad>/sweep_abb.py` and run with the backend env:

```python
import json, math, os, sys, tempfile, time
import yaml
import app  # noqa
from app.importers.pipeline import import_path
from app.importers.placement import combine_bodies
from app.validate.sweep import sweep_limits

exp = yaml.safe_load(open("corpus/expected/irb1300.yaml"))
bodies = import_path("../corpus/irb1300_asm.stp", ".stp", 0.001)[0]
by = {b.name: b for b in bodies}
links = {l["name"]: [by[n] for n in l["bodies"]] for l in exp["links"]}
order = [l["name"] for l in exp["links"]]
for j in exp["joints"]:
    child = order.index(j["child"])
    moving = [b for name in order[child:] for b in links[name]]
    obstacles = [(name, links[name]) for name in order[:child]]
    t = time.time()
    r = sweep_limits("revolute", (0, 0, 0), j["axis"], moving, obstacles)
    print(j["parent"], "->", j["child"], round(time.time() - t, 2), "s",
          r["free"], r["at_rest_overlap"], r["lower"] and round(math.degrees(r["lower"])), r["upper"] and round(math.degrees(r["upper"])))
```

Run from `backend/`: `OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe <scratchpad>/sweep_abb.py`
Expected: each line under 3 s. (The origin here is a placeholder, so the limits themselves are not meaningful — this step only measures speed on real meshes; record the numbers in the ledger.)

- [ ] **Step 6: Commit**

```bash
git add backend/app/routes/joint_routes.py backend/tests/test_sweep.py
git commit -m "feat: /sweep-limits endpoint"
```

---

### Task 3: Frontend sweep client and drive helpers

**Files:**
- Modify: `frontend/src/api/types.ts`, `frontend/src/api/client.ts`, `frontend/src/viewer/overlays.ts`, `frontend/src/ui/JointTool.tsx` (move `movingBodies` out), `frontend/src/ui/glossary.ts`
- Create: `frontend/src/ui/drive.ts`
- Test: `frontend/src/ui/drive.test.ts`

**Interfaces:**
- Produces:
  - `types.ts`: `interface SweepResult { lower: number | null; upper: number | null; free: boolean; blocked_by: { lower: string | null; upper: string | null }; at_rest_overlap: string[] }`
  - `client.ts`: `sweepLimits(joint: { type: "revolute" | "prismatic"; origin_xyz: Vec3; axis: Vec3 }, moving: Body[], obstacles: { link_id: string; bodies: Body[] }[]): Promise<SweepResult>`
  - `overlays.ts`: `unwrapStep(prev: number, next: number): number` (shortest signed angle from prev to next)
  - `drive.ts`: `movingBodies(m, d): string[]` (moved from JointTool, unchanged), `interface SweepRequest`, `sweepRequest(m, d): SweepRequest | null`, `driveRange(m, d): [number, number]`, `beyondSide(d, value): "lower" | "upper" | null`, `fmtJointValue(v: number, slider: boolean): string`, `sweepSummary(r: SweepResult, slider: boolean, name: (id: string) => string): string`
  - glossary terms `sweep`, `drive`.

- [ ] **Step 1: Write the failing tests** — `frontend/src/ui/drive.test.ts`

```ts
import { describe, expect, it } from "vitest";
import type { SweepResult } from "../api/types";
import { historyReducer } from "../model/history";
import { emptyDraft, setMotion, type JointDraft } from "../model/jointDraft";
import { boxBody } from "../model/testBodies";
import { unwrapStep } from "../viewer/overlays";
import { beyondSide, driveRange, fmtJointValue, sweepRequest, sweepSummary } from "./drive";
import { imported } from "./testing";

const model = () => {
  const h = imported([boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1]), boxBody("tip", [2, 0, 0], [3, 1, 1])]);
  const [base, arm, tip] = h.present.links.map((l) => l.id);
  const m = historyReducer(h, { type: "addJoint", joint: { id: "j2", parent_link_id: arm, child_link_id: tip, type: "fixed",
    origin_xyz: [2, 0.5, 0.5], origin_rpy: [0, 0, 0], axis: null, limit_lower: null, limit_upper: null, source: "manual" } }).present;
  return { m, base, arm, tip };
};
const draft = (fixed: string, moving: string, extra: Partial<JointDraft> = {}): JointDraft =>
  ({ ...setMotion({ ...emptyDraft(fixed, moving), origin: [1, 0.5, 0.5], axis: [0, 0, 1] }, "revolute"), ...extra });

describe("sweepRequest", () => {
  it("moves the child and everything below it; every other link is an obstacle", () => {
    const { m, base, arm } = model();
    const r = sweepRequest(m, draft(base, arm))!;
    expect(r.joint).toEqual({ type: "revolute", origin_xyz: [1, 0.5, 0.5], axis: [0, 0, 1] });
    expect(r.moving.map((b) => b.id).sort()).toEqual(["arm", "tip"]);
    expect(r.obstacles.map((o) => o.link_id)).toEqual([base]);
  });
  it("sweeps a no-limits revolute as revolute, a slider as prismatic, and nothing for rigid or no axis", () => {
    const { m, base, arm } = model();
    expect(sweepRequest(m, draft(base, arm, { noLimits: true }))!.joint.type).toBe("revolute");
    expect(sweepRequest(m, setMotion(draft(base, arm), "slider"))!.joint.type).toBe("prismatic");
    expect(sweepRequest(m, setMotion(draft(base, arm), "rigid"))).toBeNull();
    expect(sweepRequest(m, draft(base, arm, { axis: null }))).toBeNull();
  });
});

describe("drive helpers", () => {
  it("drive range: a full turn, widened by limits; a slider spans the moving link's size", () => {
    const { m, base, arm } = model();
    expect(driveRange(m, draft(base, arm))).toEqual([-Math.PI, Math.PI]);
    expect(driveRange(m, draft(base, arm, { lower: -1, upper: 5 }))).toEqual([-Math.PI, 5]);
    const [lo, hi] = driveRange(m, setMotion(draft(base, arm), "slider"));
    expect(hi).toBeCloseTo(Math.sqrt(3));
    expect(lo).toBeCloseTo(-Math.sqrt(3));
  });
  it("beyondSide compares the pose with the draft's limits; never for no limits or rigid", () => {
    const { base, arm } = model();
    const d = draft(base, arm, { lower: -1, upper: 1 });
    expect(beyondSide(d, 0.5)).toBeNull();
    expect(beyondSide(d, 1.2)).toBe("upper");
    expect(beyondSide(d, -1.2)).toBe("lower");
    expect(beyondSide({ ...d, noLimits: true }, 3)).toBeNull();
    expect(beyondSide(setMotion(d, "rigid"), 3)).toBeNull();
  });
  it("formats poses and sweep results", () => {
    expect(fmtJointValue(1, false)).toBe("+57°");
    expect(fmtJointValue(-0.0183, true)).toBe("-18 mm");
    const name = (id: string) => ({ L1: "upper_arm", L2: "base" } as Record<string, string>)[id] ?? id;
    const r: SweepResult = { lower: -1.62, upper: 2.25, free: false, blocked_by: { lower: "L1", upper: null }, at_rest_overlap: [] };
    expect(sweepSummary(r, false, name)).toBe("Stops at -93° (hits upper_arm) and +129° (no contact).");
    expect(sweepSummary({ ...r, free: true }, false, name)).toMatch(/^Spins freely/);
    expect(sweepSummary({ ...r, at_rest_overlap: ["L1", "L2"] }, false, name)).toMatch(/overlap where they sit.*upper_arm, base/);
  });
  it("unwrapStep takes the short way round", () => {
    expect(unwrapStep(3.1, -3.1)).toBeCloseTo(2 * Math.PI - 6.2);
    expect(unwrapStep(0.1, 0.3)).toBeCloseTo(0.2);
  });
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd frontend && npm test -- src/ui/drive`
Expected: FAIL — cannot resolve `./drive`.

- [ ] **Step 3: Implement**

Append to `frontend/src/api/types.ts`:

```ts
export interface SweepResult {
  lower: number | null;
  upper: number | null;
  free: boolean;
  blocked_by: { lower: string | null; upper: string | null };
  at_rest_overlap: string[];
}
```

In `frontend/src/api/client.ts` add `SweepResult` to the type import and append:

```ts
export async function sweepLimits(
  joint: { type: "revolute" | "prismatic"; origin_xyz: Vec3; axis: Vec3 },
  moving: Body[], obstacles: { link_id: string; bodies: Body[] }[],
): Promise<SweepResult> {
  return (await send("/sweep-limits", json({ joint, moving, obstacles }))).json();
}
```

Append to `frontend/src/viewer/overlays.ts`:

```ts
/** Signed angle from prev to next, the short way round (for dragging past ±180°). */
export const unwrapStep = (prev: number, next: number) => Math.atan2(Math.sin(next - prev), Math.cos(next - prev));
```

`frontend/src/ui/drive.ts`:

```ts
import type { Body, SweepResult, Vec3 } from "../api/types";
import { linkBox } from "../model/geometry";
import { draftPreviewJoint, type JointDraft } from "../model/jointDraft";
import { directJoints } from "../model/tree";
import type { ModelState } from "../model/types";
import { subtreeLinks } from "../viewer/preview";
import { deg } from "./format";

/** Bodies that move with the draft's joint: the child subtree seen from the base, or — when the pair
 *  isn't connected to the base yet — the child link and everything joined to it, never crossing the parent. */
export function movingBodies(m: ModelState, d: JointDraft): string[] {
  const j = draftPreviewJoint(d);
  if (!j) return [];
  const others = m.joints.filter((x) => x.id !== j.id);
  let links = subtreeLinks(directJoints(m.links, [...others, j], m.rootLinkId).joints, j.id);
  if (!links.length) {
    links = [j.child_link_id];
    const seen = new Set([j.parent_link_id, j.child_link_id]);
    for (let i = 0; i < links.length; i++) {
      for (const o of others) {
        const next = o.parent_link_id === links[i] ? o.child_link_id : o.child_link_id === links[i] ? o.parent_link_id : null;
        if (next && !seen.has(next)) { seen.add(next); links.push(next); }
      }
    }
  }
  return m.links.filter((l) => links.includes(l.id)).flatMap((l) => l.body_ids);
}

export interface SweepRequest {
  joint: { type: "revolute" | "prismatic"; origin_xyz: Vec3; axis: Vec3 };
  moving: Body[];
  obstacles: { link_id: string; bodies: Body[] }[];
}

export function sweepRequest(m: ModelState, d: JointDraft): SweepRequest | null {
  const j = draftPreviewJoint(d);
  if (!j || !j.axis || (d.motion !== "revolute" && d.motion !== "slider")) return null;
  const moving = new Set(movingBodies(m, d));
  return {
    joint: { type: d.motion === "slider" ? "prismatic" : "revolute", origin_xyz: j.origin_xyz, axis: j.axis },
    moving: [...moving].map((id) => m.bodies[id]),
    obstacles: m.links.filter((l) => !l.body_ids.some((b) => moving.has(b)))
      .map((l) => ({ link_id: l.id, bodies: l.body_ids.map((id) => m.bodies[id]) })),
  };
}

/** Slider/number range for driving: a full turn, or ± the moving link's size, widened to include the limits. */
export function driveRange(m: ModelState, d: JointDraft): [number, number] {
  const lim = d.noLimits ? [0, 0] : [d.lower ?? 0, d.upper ?? 0];
  if (d.motion === "slider") {
    const l = m.links.find((x) => x.id === d.moving);
    const b = l ? linkBox(l, m.bodies) : null;
    const size = b ? Math.hypot(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]) : 0.1;
    return [Math.min(-size, lim[0]), Math.max(size, lim[1])];
  }
  return [Math.min(-Math.PI, lim[0]), Math.max(Math.PI, lim[1])];
}

export function beyondSide(d: JointDraft, value: number): "lower" | "upper" | null {
  if (d.noLimits || (d.motion !== "revolute" && d.motion !== "slider")) return null;
  if (d.lower !== null && value < d.lower - 1e-9) return "lower";
  if (d.upper !== null && value > d.upper + 1e-9) return "upper";
  return null;
}

export const fmtJointValue = (v: number, slider: boolean) =>
  `${v > 0 ? "+" : ""}${slider ? `${(v * 1000).toFixed(0)} mm` : `${deg(v).toFixed(0)}°`}`;

export function sweepSummary(r: SweepResult, slider: boolean, name: (id: string) => string): string {
  if (r.at_rest_overlap.length) {
    return `These parts overlap where they sit, so limits can't be found: ${r.at_rest_overlap.map(name).join(", ")}.`;
  }
  if (r.free) return "Spins freely: no collision in a full turn.";
  const side = (v: number | null, by: string | null) =>
    `${v === null ? "?" : fmtJointValue(v, slider)} (${by ? `hits ${name(by)}` : "no contact"})`;
  return `Stops at ${side(r.lower, r.blocked_by.lower)} and ${side(r.upper, r.blocked_by.upper)}.`;
}
```

In `frontend/src/ui/JointTool.tsx` delete the exported `movingBodies` function (and its doc comment), delete the now-unused imports `directJoints` and `subtreeLinks` (keep `jointMotion`, `previewRange`), and add `import { movingBodies } from "./drive";`.

In `frontend/src/ui/glossary.ts` add before `} as const`:

```ts
  sweep: ["Find limits from geometry", "Swings the moving part until it would hit another part, then stops 1° (or 1 mm) short. Parts that only touch where they sit don't count."],
  drive: ["Drive", "Move the joint by hand to check it: drag the moving part in the 3D view or use the slider. This only previews — the URDF keeps the pose from your CAD file."],
```

- [ ] **Step 4: Run tests and the type check**

Run: `cd frontend && npm test && npx tsc --noEmit`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add frontend/src
git commit -m "feat: sweep client and drive helpers"
```

---

### Task 4: Viewer — drive drag and warning tint

**Files:**
- Modify: `frontend/src/viewer/Viewer.ts`

**Interfaces:**
- Consumes: `unwrapStep` (Task 3), existing `angleAroundAxis`, `rayPlaneHit`, `closestParamOnAxis`.
- Produces:
  - `type DriveSpec = { joint: Joint; bodyIds: string[]; value: number; onChange: (v: number) => void }` (exported)
  - `onDrive(d: DriveSpec | null): void` — pressing on one of `bodyIds` (when no limit handle is under the pointer) drags the joint: revolute by the pointer's angle around the axis (unwrapped), prismatic by the closest point on the axis; calls `onChange(value)` while dragging; orbiting is off during the drag.
  - `setWarning(bodyIds: string[]): void` — red tint (`0xdc2626`) that overrides other tints.

This task has no unit test (WebGL); it is exercised by Task 5's component tests through a fake viewer and by Task 6's e2e and manual ABB check.

- [ ] **Step 1: Implement** — in `frontend/src/viewer/Viewer.ts`:

1. Add `unwrapStep` to the `./overlays` import.
2. After `type LimitCb = …;` add:

```ts
export type DriveSpec = { joint: Joint; bodyIds: string[]; value: number; onChange: (v: number) => void };
```

3. After `const APPROX_COLOR = 0xd97706;` add `const WARNING_COLOR = 0xdc2626;`.
4. New fields after `private dragging …`:

```ts
  private drive: DriveSpec | null = null;
  private driving: { last: number } | null = null;
  private warning: string[] = [];
```

5. In `applyLooks()` replace `mat.color.setHex(this.tints[id] ?? PART_COLOR);` with
   `mat.color.setHex(this.warning.includes(id) ? WARNING_COLOR : this.tints[id] ?? PART_COLOR);`
6. After `onLimitDrag(…)` add:

```ts
  onDrive(d: DriveSpec | null) {
    this.drive = d;
    if (!d && this.driving) { this.driving = null; this.controls.enabled = true; }
  }

  setWarning(bodyIds: string[]) { this.warning = bodyIds; this.applyLooks(); }

  /** The pointer's position along the joint: angle around the axis, or distance along it. */
  private jointParam(e: PointerEvent, j: Joint): number | null {
    if (!j.axis) return null;
    const r = this.ray(e).ray;
    const ro = r.origin.toArray() as Vec3, rd = r.direction.toArray() as Vec3;
    if (j.type === "prismatic") return closestParamOnAxis(ro, rd, j.origin_xyz, j.axis);
    const p = rayPlaneHit(ro, rd, j.origin_xyz, j.axis);
    return p ? angleAroundAxis(p, j.origin_xyz, j.axis) : null;
  }
```

7. Replace the body of `private drag(e)` with:

```ts
  private drag(e: PointerEvent) {
    const { which, joint } = this.dragging!;
    const value = this.jointParam(e, joint);
    if (value !== null) this.limitCb?.(which, value);
  }
```

8. Replace `onDown`, `onMove`, `onUp` with:

```ts
  private onDown = (e: PointerEvent) => {
    this.down = { x: e.clientX, y: e.clientY };
    const glyph = this.glyphs.find((g) => g.state === "active");
    if (this.limitCb && glyph) {
      const handles: Object3D[] = [];
      this.overlay.traverse((c) => { if (c.userData.handle) handles.push(c); });
      const h = this.ray(e).intersectObjects(handles, false)[0];
      if (h) {
        this.dragging = { which: h.object.userData.handle, joint: glyph.joint };
        this.controls.enabled = false;
        return;
      }
    }
    if (this.drive) {
      const hit = this.hitBody(this.ray(e));
      const p = hit && this.drive.bodyIds.includes(hit.bodyId) ? this.jointParam(e, this.drive.joint) : null;
      if (p !== null) { this.driving = { last: p }; this.controls.enabled = false; }
    }
  };

  private onMove = (e: PointerEvent) => {
    if (this.dragging) { this.drag(e); return; }
    if (this.driving && this.drive) {
      const p = this.jointParam(e, this.drive.joint);
      if (p !== null) {
        const step = this.drive.joint.type === "prismatic" ? p - this.driving.last : unwrapStep(this.driving.last, p);
        this.driving.last = p;
        this.drive.value += step;
        this.drive.onChange(this.drive.value);
      }
      return;
    }
    if (this.hoverCb && !this.down) this.pendingHover = e;
  };

  private onUp = (e: PointerEvent) => {
    const down = this.down;
    this.down = null;
    if (this.dragging || this.driving) { this.dragging = null; this.driving = null; this.controls.enabled = true; return; }
    if (!down || !isClick(down, { x: e.clientX, y: e.clientY })) return;
    this.pickCb(this.hitBody(this.ray(e)), e.shiftKey);
  };
```

- [ ] **Step 2: Run tests and the type check**

Run: `cd frontend && npm test && npx tsc --noEmit`
Expected: all pass.

- [ ] **Step 3: Commit**

```bash
git add frontend/src/viewer/Viewer.ts
git commit -m "feat: drag the moving part to drive a joint; warning tint"
```

---

### Task 5: Joint tool — Find limits and Drive

**Files:**
- Modify: `frontend/src/ui/JointTool.tsx`, `frontend/src/ui/theme.css`
- Test: `frontend/src/ui/steps/drive.test.tsx`

**Interfaces:**
- Consumes: Task 3 helpers, `sweepLimits`, Task 4 `DriveSpec`, `onDrive`, `setWarning`.
- Produces test ids: `find-limits`, `sweep-status`, `use-no-limits`, `drive-slider`, `drive-value`, `set-lower-here`, `set-upper-here`, `reset-drive`, `drive-label`.

- [ ] **Step 1: Write the failing tests** — `frontend/src/ui/steps/drive.test.tsx`

```tsx
import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import type { Joint, SweepResult } from "../../api/types";
import { historyReducer } from "../../model/history";
import { membership, pairKey } from "../../model/pairs";
import { boxBody } from "../../model/testBodies";
import type { Viewer } from "../../viewer/Viewer";
import JointTool from "../JointTool";
import { Harness, imported, state } from "../testing";
import ConnectStep from "./ConnectStep";

afterEach(() => vi.restoreAllMocks());

function start() {
  const h = imported([boxBody("block", [0, 0, 0], [1, 1, 1]), boxBody("peg", [1, 0, 0], [2, 1, 1])]);
  const [la, lb] = h.present.links;
  const joint: Joint = { id: "s1", parent_link_id: la.id, child_link_id: lb.id, type: "cylindrical", origin_xyz: [1, 0.5, 0.5],
    origin_rpy: [0, 0, 0], axis: [1, 0, 0], limit_lower: null, limit_upper: null, source: "suggested" };
  return historyReducer(h, { type: "setSuggestion", suggestion: { key: pairKey(la.id, lb.id), linkA: la.id, linkB: lb.id,
    membership: membership(la, lb), status: "review", result: { joint, reason: null, evidence: null } } });
}
const fakeViewer = () => ({ setPreview: vi.fn(), onDrive: vi.fn(), setWarning: vi.fn(), focus: vi.fn(),
  screenPoint: () => ({ x: 10, y: 10 }), onCameraChange: () => () => {} }) as unknown as Viewer;
function open(viewer: Viewer | null = fakeViewer()) {
  const h = start();
  render(<Harness start={h} step="connect" viewer={viewer} ui={(ctx) => (
    <>
      <ConnectStep ctx={ctx} />
      {ctx.jointDraft && <JointTool ctx={ctx} />}
    </>
  )} />);
  fireEvent.click(screen.getByTestId("next-joint"));
  return { viewer, block: h.present.links[0].id };
}
const result = (r: Partial<SweepResult>): SweepResult =>
  ({ lower: -1, upper: 1.5, free: false, blocked_by: { lower: null, upper: null }, at_rest_overlap: [], ...r });

describe("Find limits from geometry", () => {
  it("fills the limits from the sweep and names what blocks each side", async () => {
    const { block } = open();
    const spy = vi.spyOn(client, "sweepLimits").mockResolvedValue(result({ blocked_by: { lower: block, upper: null } }));
    await act(async () => { fireEvent.click(screen.getByTestId("find-limits")); });
    expect(spy).toHaveBeenCalledWith({ type: "revolute", origin_xyz: [1, 0.5, 0.5], axis: [1, 0, 0] },
      [expect.objectContaining({ id: "peg" })], [{ link_id: block, bodies: [expect.objectContaining({ id: "block" })] }]);
    expect(state().jointDraft).toMatchObject({ lower: -1, upper: 1.5 });
    expect(screen.getByTestId("sweep-status").textContent).toBe("Stops at -57° (hits block) and +86° (no contact).");
  });

  it("a free spin offers No limits", async () => {
    open();
    vi.spyOn(client, "sweepLimits").mockResolvedValue(result({ free: true, lower: -Math.PI, upper: Math.PI }));
    await act(async () => { fireEvent.click(screen.getByTestId("find-limits")); });
    fireEvent.click(screen.getByTestId("use-no-limits"));
    expect(state().jointDraft.noLimits).toBe(true);
  });

  it("parts overlapping at rest leave the limits alone", async () => {
    const { block } = open();
    vi.spyOn(client, "sweepLimits").mockResolvedValue(result({ lower: null, upper: null, at_rest_overlap: [block] }));
    await act(async () => { fireEvent.click(screen.getByTestId("find-limits")); });
    expect(screen.getByTestId("sweep-status").textContent).toMatch(/overlap where they sit.*block/);
    expect(state().jointDraft).toMatchObject({ lower: -Math.PI, upper: Math.PI });
  });

  it("a result for an outdated draft is ignored", async () => {
    open();
    let resolve!: (r: SweepResult) => void;
    vi.spyOn(client, "sweepLimits").mockReturnValue(new Promise((r) => { resolve = r; }));
    fireEvent.click(screen.getByTestId("find-limits"));
    expect(screen.getByTestId("find-limits").textContent).toMatch(/Sweeping/);
    fireEvent.click(screen.getByTestId("flip"));
    await act(async () => { resolve(result({ lower: -0.2, upper: 0.2 })); });
    expect(state().jointDraft).toMatchObject({ lower: -Math.PI, upper: Math.PI });
    expect(screen.queryByTestId("sweep-status")).toBeNull();
  });

  it("is not offered for Rigid", () => {
    open();
    fireEvent.click(screen.getByTestId("motion-rigid"));
    expect(screen.queryByTestId("find-limits")).toBeNull();
    expect(screen.queryByTestId("drive-slider")).toBeNull();
  });
});

describe("Drive", () => {
  it("the slider poses the moving part, the label shows it, and the viewer can drive it", () => {
    const { viewer } = open();
    expect(viewer!.onDrive).toHaveBeenLastCalledWith(expect.objectContaining({ bodyIds: ["peg"], value: 0 }));
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    expect(viewer!.setPreview).toHaveBeenLastCalledWith(["peg"], expect.anything());
    expect(screen.getByTestId("drive-label").textContent).toBe("+57°");
    expect((screen.getByTestId("drive-value") as HTMLInputElement).value).toBe("57.3");
  });

  it("past a limit the part turns red and the label names the blocker", async () => {
    const { viewer, block } = open();
    vi.spyOn(client, "sweepLimits").mockResolvedValue(result({ lower: -0.5, upper: 0.5, blocked_by: { lower: null, upper: block } }));
    await act(async () => { fireEvent.click(screen.getByTestId("find-limits")); });
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    expect(screen.getByTestId("drive-label").textContent).toBe("+57° · beyond upper limit (hits block)");
    expect(viewer!.setWarning).toHaveBeenLastCalledWith(["peg"]);
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "0.2" } });
    expect(viewer!.setWarning).toHaveBeenLastCalledWith([]);
  });

  it("Set lower/upper here copy the pose", () => {
    open();
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    fireEvent.click(screen.getByTestId("set-upper-here"));
    expect(state().jointDraft.upper).toBe(1);
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "-0.5" } });
    fireEvent.click(screen.getByTestId("set-lower-here"));
    expect(state().jointDraft.lower).toBe(-0.5);
  });

  it("set-here never makes lower exceed upper", () => {
    open();
    fireEvent.change(screen.getByTestId("limit-upper"), { target: { value: "10" } });
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    expect((screen.getByTestId("set-lower-here") as HTMLButtonElement).disabled).toBe(true);
    expect((screen.getByTestId("set-upper-here") as HTMLButtonElement).disabled).toBe(false);
  });

  it("closing the tool clears the pose, tint and drive", async () => {
    const { viewer } = open();
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    fireEvent.click(screen.getByTestId("cancel"));
    await waitFor(() => expect(viewer!.onDrive).toHaveBeenLastCalledWith(null));
    expect(viewer!.setPreview).toHaveBeenLastCalledWith([], null);
    expect(viewer!.setWarning).toHaveBeenLastCalledWith([]);
  });
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd frontend && npm test -- src/ui/steps/drive`
Expected: FAIL — `find-limits` not found.

- [ ] **Step 3: Implement** — in `frontend/src/ui/JointTool.tsx`:

1. Imports: add `Loader2, Radar` to the lucide import; change `import { useEffect, useRef, useState } from "react";` to `import { useEffect, useMemo, useReducer, useRef, useState } from "react";`; add

```ts
import { sweepLimits } from "../api/client";
import type { SweepResult } from "../api/types";
import { beyondSide, driveRange, fmtJointValue, movingBodies, sweepRequest, sweepSummary } from "./drive";
```

(`movingBodies` replaces the separate import added in Task 3.)

2. After `const opened = useRef(…);` and its effect, add state and effects:

```ts
  type Sweep = { state: "idle" } | { state: "running" } | { state: "done"; result: SweepResult } | { state: "error"; message: string };
  const [drive, setDrive] = useState(0);
  const [sweep, setSweep] = useState<Sweep>({ state: "idle" });
  const [, relayout] = useReducer((n: number) => n + 1, 0);
  // what the pose and a sweep result depend on; when it changes both are stale
  const shape = d ? JSON.stringify([d.id, d.fixed, d.moving, d.origin, d.axis, d.motion]) : "";
  const live = useRef({ d, shape });
  live.current = { d, shape };
  useEffect(() => { setDrive(0); setSweep({ state: "idle" }); }, [shape]);

  const pj = d ? draftPreviewJoint(d) : null;
  const canDrive = !!pj?.axis && (d?.motion === "revolute" || d?.motion === "slider");
  const bodyIds = useMemo(() => (d && canDrive ? movingBodies(model, d) : []), [model, d, canDrive]);
  const side = d ? beyondSide(d, drive) : null;

  // the drive pose is the preview, except while Animate runs
  useEffect(() => {
    if (!viewer || animating) return;
    if (!canDrive || !pj || drive === 0) viewer.setPreview([], null);
    else viewer.setPreview(bodyIds, jointMotion(pj, drive));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [viewer, animating, drive, canDrive, bodyIds, shape]);
  useEffect(() => {
    viewer?.onDrive(canDrive && pj ? { joint: pj, bodyIds, value: drive, onChange: setDrive } : null);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [viewer, canDrive, bodyIds, drive, shape]);
  useEffect(() => { viewer?.setWarning(side ? bodyIds : []); }, [viewer, side, bodyIds]);
  useEffect(() => () => { viewer?.onDrive(null); viewer?.setWarning([]); viewer?.setPreview([], null); }, [viewer]);
  useEffect(() => viewer?.onCameraChange(relayout), [viewer]);

  const runSweep = async () => {
    const req = d ? sweepRequest(model, d) : null;
    if (!req || sweep.state === "running") return;
    const asked = shape;
    setSweep({ state: "running" });
    try {
      const result = await sweepLimits(req.joint, req.moving, req.obstacles);
      if (live.current.shape !== asked) return;   // the joint changed while sweeping
      setSweep({ state: "done", result });
      const cur = live.current.d;
      if (cur && !result.free && !result.at_rest_overlap.length && result.lower !== null && result.upper !== null) {
        setJointDraft({ ...cur, lower: result.lower, upper: result.upper });
      }
    } catch (e) {
      if (live.current.shape === asked) setSweep({ state: "error", message: (e as Error).message });
    }
  };
```

3. After the existing `const canAnimate = …;` line add:

```ts
  const [driveLo, driveHi] = driveRange(model, d);
  const blocker = sweep.state === "done" && side ? sweep.result.blocked_by[side] : null;
  const label = `${fmtJointValue(drive, slider)}${side ? ` · beyond ${side} limit${blocker ? ` (hits ${linkName(blocker)})` : ""}` : ""}`;
  const labelAt = pj && drive !== 0 ? viewer?.screenPoint(pj.origin_xyz) : null;
```

4. Wrap the returned JSX in a fragment and add the label after the tool: change `return (\n    <div className="floating jointtool"` to `return (\n    <>\n    <div className="floating jointtool"`, and at the end change the final `    </div>\n  );` to:

```tsx
    </div>
    {labelAt && <div className="drive-label" data-testid="drive-label" style={{ left: labelAt.x, top: labelAt.y }}>{label}</div>}
    </>
  );
```

5. Insert after the `No limits` checkbox block (`{d.motion === "revolute" && (… no-limits …)}`):

```tsx
      {canDrive && (
        <div className="drive">
          <div className="row">
            <button className="btn" data-testid="find-limits" disabled={sweep.state === "running"} onClick={() => void runSweep()}>
              {sweep.state === "running" ? <><Loader2 size={14} className="spin" />Sweeping…</> : <><Radar size={14} />Find limits from geometry</>}
            </button>
            <Info term="sweep" />
          </div>
          {sweep.state === "done" && (
            <div className="small" data-testid="sweep-status">
              {sweepSummary(sweep.result, slider, (id) => model.links.find((l) => l.id === id)?.name ?? id)}
              {sweep.result.free && !d.noLimits && (
                <button className="btn" data-testid="use-no-limits" onClick={() => set({ noLimits: true })}>Use No limits</button>
              )}
            </div>
          )}
          {sweep.state === "error" && <div className="error-text" data-testid="sweep-status">{sweep.message}</div>}
          <div className="label">Drive <Info term="drive" /></div>
          <div className="row">
            <input type="range" data-testid="drive-slider" min={driveLo} max={driveHi} step="any" value={drive}
              style={{ flex: 1 }} onChange={(e) => setDrive(Number(e.target.value))} />
            <input className="field" data-testid="drive-value" type="number" step="any" style={{ width: 72 }}
              value={shown(drive)} onChange={(e) => setDrive(parsed(e.target.value) ?? 0)} />
          </div>
          {limited && (
            <div className="row">
              <button className="btn" data-testid="set-lower-here" disabled={d.upper !== null && drive > d.upper}
                onClick={() => set({ lower: drive })}>Set lower here</button>
              <button className="btn" data-testid="set-upper-here" disabled={d.lower !== null && drive < d.lower}
                onClick={() => set({ upper: drive })}>Set upper here</button>
              <button className="btn btn-ghost" data-testid="reset-drive" disabled={drive === 0} onClick={() => setDrive(0)}>Reset</button>
            </div>
          )}
        </div>
      )}
```

6. In `close()` nothing changes: the unmount effect clears pose, tint and drive.

Append to `frontend/src/ui/theme.css`:

```css
.drive { display: flex; flex-direction: column; gap: 6px; padding-top: 6px; border-top: 1px solid var(--border); }
.drive-label { position: absolute; z-index: 15; transform: translate(-50%, calc(-100% - 10px)); padding: 2px 8px; border-radius: 10px; background: var(--panel); border: 1px solid var(--border); box-shadow: var(--shadow); font: 600 12px var(--font); white-space: nowrap; pointer-events: none; }
.spin { animation: spin 0.9s linear infinite; }
@keyframes spin { to { transform: rotate(360deg); } }
```

- [ ] **Step 4: Run tests and the type check**

Run: `cd frontend && npm test && npx tsc --noEmit`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add frontend/src
git commit -m "feat: find limits from geometry and drive the joint in the joint tool"
```

---

### Task 6: End-to-end check and manual ABB run

**Files:**
- Modify: `frontend/e2e/smoke.spec.ts`

- [ ] **Step 1: Extend the smoke test** — in `frontend/e2e/smoke.spec.ts`, after the line `await expect(page.getByTestId("motion-revolute")).toHaveClass(/active/);` insert:

```ts
  await page.getByTestId("find-limits").click();
  await expect(page.getByTestId("sweep-status")).toBeVisible({ timeout: 30_000 });
  await expect(page.getByTestId("sweep-status")).not.toHaveClass(/error-text/);
  await page.getByTestId("drive-slider").fill("0.5");
  await expect(page.getByTestId("drive-label")).toContainText("°");
```

- [ ] **Step 2: Run everything**

Run: `cd frontend && npm test && npx tsc --noEmit && npm run e2e` and `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass (e2e: 1 passed).

- [ ] **Step 3: Manual check on the ABB model** (not committed; headless with GPU flags or by the user)

Import `corpus/irb1300_asm.stp`, Group (auto), Connect each joint as Revolute, and for each: Find limits (record time and result), drag the moving part in 3D through its range, confirm the red tint and "beyond … limit" label past a limit, Set lower/upper here, OK. Export still passes its checklist.

- [ ] **Step 4: Commit**

```bash
git add frontend/e2e/smoke.spec.ts
git commit -m "test: e2e covers find limits and drive"
```
