# Collision Sweep and Drive Joint — Design

**Date:** 2026-10-08
**Builds on:** `docs/superpowers/specs/2026-10-07-cad-to-urdf-design.md` §7.2 (collision sweep, auto-derived limits) and the guided UI (`docs/superpowers/specs/2026-10-08-guided-ui-design.md`, Joint tool §6.3).
**Scope:** one backend endpoint, a "Find limits from geometry" control, and Fusion-style Drive Joint (drag the moving part in 3D) in the Joint tool. No change to export or to existing endpoints.

## 1. Purpose and success

Joint limits today are typed in or left at the −180°…+180° / 0…100 mm defaults, so most exported revolute joints can swing through solid metal. The sweep turns the joint through its range against the rest of the robot and stops at first contact, giving limits a simulator can trust and showing what blocks each direction.

Success criteria:

- On the ABB IRB1300 (after grouping), "Find limits from geometry" on each revolute joint returns limits that, when previewed, stop short of visible contact, and names the link that blocks each side.
- A joint that can turn a full circle without contact is reported as free, with one click to "No limits".
- Parts that overlap where they sit are reported, not silently ignored; the limits are left unchanged.
- One sweep on the ABB finishes in under 3 s on the user's machine.
- The user can grab the moving part in 3D (or use the drive slider) and swing it through the joint, see where the limits are, see when the pose is beyond a limit, and set either limit from the current pose.

## 2. Decisions (from the design conversation)

| Topic | Decision |
|---|---|
| Trigger | A button in the Joint tool ("Find limits from geometry"); nothing runs automatically. |
| Collision engine | `python-fcl` (PyPI wheel `python-fcl==0.7.0.11`, cp311 win_amd64 available; not on conda-forge). |
| Resting contact | Mated CAD parts touch at rest. The moving side is shrunk inward by the tolerance before testing, so touching surfaces don't count and only real digging-in does. |
| Tolerance / margin | Tolerance 0.5 mm. Reported limits back off 1° (revolute) or 1 mm (slider) from first contact. |
| What moves | The child link and every link already connected below it (same set the motion preview moves). |
| Obstacles | Every other link with geometry that isn't ignored. |
| Drive Joint | Drag the moving part in 3D or use a drive slider; pose is a preview only (URDF zero stays the CAD pose); beyond-limit poses tint the moving part red using the draft's limits and the sweep's blocker names; "Set lower/upper here" take the pose. No backend calls while driving. |

### Why shrinking, not penetration depth (verified 2026-10-08)

A probe against `python-fcl` 0.7.0.11 showed mesh-to-mesh contacts report `penetration_depth = 1.0` whether two boxes touch or overlap by 0.1 m, and `distance` is 0 for any intersection. Depth cannot separate resting contact from collision. Offsetting every moving vertex inward along its vertex normal by the tolerance and using fcl's boolean collide does: touching (0 mm) and 0.2 mm overlap → no hit; 1 mm and 10 mm overlap → hit. With the BVH built once and only the transform changed per step, 120 steps on an 82k-triangle pair took 3 ms.

## 3. Backend

New module `backend/app/validate/sweep.py` and route `POST /sweep-limits` in `backend/app/routes/joint_routes.py`.

Request:

```json
{
  "joint": { "type": "revolute" | "prismatic", "origin_xyz": [x,y,z], "axis": [x,y,z] },
  "moving": [Body, ...],
  "obstacles": [{ "link_id": "...", "bodies": [Body, ...] }, ...]
}
```

Response:

```json
{
  "lower": -1.62, "upper": 2.25,
  "free": false,
  "blocked_by": { "lower": "<link id>" | null, "upper": "<link id>" | null },
  "at_rest_overlap": ["<link id>", ...]
}
```

Algorithm:

1. Combine `moving` bodies into one mesh; offset each vertex by `-normal × 0.0005` (trimesh vertex normals). Build one fcl BVH for it and one per obstacle link (raw, not shrunk).
2. **At rest:** collide the shrunk moving mesh with each obstacle at the identity transform. Any hit → that link id goes into `at_rest_overlap`; when the list is non-empty, return it with `lower`/`upper` = null and stop.
3. **Pre-filter** (`reachable(moving_sphere, obstacle_sphere, joint, range)`, a pure function): skip obstacles whose bounding sphere can't reach the swept volume (for revolute: the torus swept by the moving mesh's bounding sphere around the axis; for prismatic: the capsule along the axis over ±range).
4. **Sweep each direction** (positive, then negative):
   - Revolute: angles in 3° steps from 0 up to ±360° in each direction (so the free arc between two hits on the same obstacle is found whole, e.g. −40° … +317°). At the first hit, bisect between the last free and first hit angle to 0.25°. Limit = last free angle − 1° (towards 0, never past 0). No hit in the positive direction over a full 360° → `free: true`, `lower = -π`, `upper = π`.
   - Prismatic: travel up to the moving mesh's bounding-box diagonal in each direction, steps of 1 mm or 1/200 of that range (whichever is larger), bisection to 0.25 mm, limit backs off 1 mm. No hit → limit is the full range on that side (`free` is never true for prismatic).
   - Transform per step: rotation about `axis` through `origin_xyz` (revolute) or translation along `axis` (prismatic), via `CollisionObject.setTransform`.
5. `blocked_by` names the obstacle link hit first in each direction.

Errors: 422 for a missing or zero axis, unknown joint type (only `revolute`/`prismatic` are swept), or a moving set with no geometry. Bodies with no vertices are skipped.

Dependency: add `python-fcl==0.7.0.11` to `backend/pyproject.toml`; install into the `cad-to-urdf` env with pip.

## 4. Frontend

API client: `sweepLimits(joint, moving: Body[], obstacles: { link_id: string; bodies: Body[] }[]): Promise<SweepResult>`.

Joint tool (Limits section, shown for Revolute and Slider when an axis is set):

- **"Find limits from geometry"** button with an ⓘ ("Swings the moving part until it would hit another part, then stops 1° short."). While running: spinner and "Sweeping…"; the rest of the tool stays usable; a second click while running is ignored.
- Moving bodies = `movingBodies(model, draft)` (existing). Obstacles = every other link's non-ignored bodies.
- Result handling:
  - limits found → fill lower/upper; the 3D arc updates; a line under the fields: "Stops at −93° (hits *upper arm*) and +129° (hits *base*)" (mm for sliders; "no contact" for an unblocked side).
  - `free` → "Spins freely: no collision in a full turn" and a **Use No limits** button that ticks No limits.
  - `at_rest_overlap` → "These parts overlap where they sit, so limits can't be found: *bolt_3*, *cover*." Limits unchanged.
  - request error → message under the button, limits unchanged.
- A result is cleared when the draft's axis, origin, motion or moving part changes (it no longer applies).
- Glossary: add terms `sweep` and `drive`.

### Drive Joint (Fusion-style)

- Shown in the Joint tool for Revolute and Slider once an axis and origin are set.
- **Drag in 3D:** pressing on a moving-side part (the amber link and everything below it) and dragging turns it about the axis (revolute: the pointer's angle around the axis, unwrapped so it can go past ±180°) or slides it along the axis (slider: the point on the axis closest to the pointer ray). Orbiting is suspended while dragging; limit handles keep priority.
- **Drive slider + number field** (`°` or `mm`): range −180°…+180° widened to include the limits for revolute; ± the moving link's box diagonal widened to include the limits for slider.
- **Pose:** applied as the motion preview. It stays while the tool is open and resets when the tool closes, the draft changes to another joint, or the axis/origin/motion/moving part changes. Animate overrides it while running and returns to it after.
- **Label in 3D** at the joint origin: "+42°" / "+18 mm"; past a limit: "beyond upper limit (hits *upper arm*)" — the blocker name only when a sweep produced it.
- **Beyond a limit** (pose below `lower` or above `upper`; never for No limits): the moving parts are tinted red.
- **"Set lower here" / "Set upper here":** copy the pose into that limit (disabled when it would make lower > upper).

## 5. Testing

- Backend unit (`backend/tests/test_sweep.py`): two boxes with a hinge → limits within 0.5° of the analytic contact angle minus the margin; touching parts at rest are not an overlap; 1 mm interference at rest is; a free-spinning part reports `free`; a slider into a wall stops 1 mm short; `reachable` returns false for an obstacle outside the swept torus/capsule and true for one inside it; route returns 422 for a missing axis.
- Backend corpus: on the ABB (grouped per `corpus/expected/irb1300.yaml`), each expected revolute joint returns finite limits or `free`, no `at_rest_overlap` for the joint pair, and finishes in < 3 s per joint (logged, not asserted in CI).
- Frontend pure helpers: sweep request (moving vs obstacle bodies), drive range, beyond-limit side, sweep summary text, angle unwrapping.
- Frontend: Joint tool with a mocked `sweepLimits` — button fills limits and shows the blocker line; free → Use No limits ticks it; overlap message leaves limits; changing the axis clears the result; button hidden for Rigid/Planar; the drive slider moves the moving parts through the viewer, the label shows the pose, past a limit the parts are tinted red with the blocker named, Set lower/upper here copy the pose, and the pose resets when the tool closes.
- e2e: on peg-in-hole, Find limits returns a result (any outcome but an error) and the drive slider changes the pose.

## 6. Out of scope

Auto-running the sweep, live collision checks while driving, driving several joints at once, self-collision between non-adjacent moving links, sweeping planar joints, continuous collision detection between steps (the 3° step can miss a part thinner than the step's arc at large radius; bisection only refines a found hit), physics checks (separate spec, §7.3).
