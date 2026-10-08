# Import Robustness, User Fixes, and Model Corpus — Design

**Date:** 2026-10-08
**Builds on:** `docs/superpowers/specs/2026-10-07-cad-to-urdf-design.md` (the base spec) and the `backend-mvp` branch.
**Scope:** backend only. The frontend gets its own spec afterwards and consumes the API defined here.

## 1. Why

Running a real model (ABB IRB1300 STEP, Creo export) through the MVP showed the importer silently dropping the robot's four arm links: they are stored as open surface shells (`SHELL_BASED_SURFACE_MODEL`), and the importer only reads solids. The base spec's rule — never fail or guess silently — was violated at import.

The software must work on arbitrary CAD/mesh models from any exporter. That cannot be guaranteed per file, so this design guarantees instead that:

1. Every part in an uploaded file is either imported, imported with a named warning, or reported as failed with a reason. Nothing disappears.
2. Every problem the user sees comes with at least one fix they can apply.
3. A corpus of real models measures how well the software generalises and catches regressions.

## 2. Success criteria

- The ABB IRB1300 file imports all its parts, with the four arm links present and marked `repaired` or `approximate`.
- A generated model with an unrepairable part imports the rest of the model, lists the bad part with a reason, and offers fixes.
- The corpus runner produces a scorecard for every model with expectations and fails on regression against a committed baseline.

## 3. Import: repair, accounting, per-part status

### 3.1 STEP / IGES / BREP

The importer walks the shape in this order:

1. **Solids** — imported as today.
2. **Loose shells** (shells not inside any solid) — sewn (`BRepBuilderAPI_Sewing`) and made into a solid (`BRepBuilderAPI_MakeSolid` + `ShapeFix_Solid`).
   - Sewing leaves no free edges → status `repaired`, warning `shell_repaired`.
   - Free edges remain → status `approximate`, warning `open_geometry` with the free-edge count in the message.
3. **Loose faces** (faces in no shell) — grouped by connectivity, sewn and solidified the same way, same statuses.
4. **Accounting check** — count every face in the file; if any face is not in a returned body, add the file-level warning `unaccounted_faces` with the count.

**Part names** come from STEP product names (via XCAF) where available, e.g. `PEDESTAL04557S`; otherwise `solid_N` / `shell_N`. Names are display-only; `Body.id` stays the stable key.

### 3.2 STL / OBJ

- Not watertight → status `approximate`, warning `open_geometry` (was a hard 422). The mesh still loads and displays.
- Inside-out (negative volume) → flipped, warning `inverted_mesh`, status stays `ok`.
- Unreadable or empty → the request fails with 422 naming the uploaded file (unchanged).
- Region growing currently rejects non-watertight meshes. For `approximate` meshes the import calls it with the watertight guard relaxed (`allow_open=True`), so open meshes still get faces. If the mesh has no shared edges at all (unwelded triangle soup), the body gets no faces and warning `degenerate`, and the import continues. Rule: the import never fails because of one bad part.

### 3.3 Status and warnings model

New fields on `Body` (add-only):

```
status: Literal["ok", "repaired", "approximate", "failed"] = "ok"
warnings: list[Warning] = []
```

```
Warning:
  code: str            # machine-readable, from the table below
  message: str         # human-readable, names the part
  fixes: list[Literal["set_mass", "replace", "ignore", "reexport"]]
```

A `failed` body carries its name, status and warnings but empty `vertices`/`triangles`/`faces`.

The `/import` response gains a file-level `warnings: list[Warning]` alongside `bodies` and `applied_scale`.

| code | level | when | fixes |
|---|---|---|---|
| `shell_repaired` | part | loose shell sewn closed | — (info) |
| `open_geometry` | part | gaps remain after repair / non-watertight mesh | set_mass, replace, ignore |
| `inverted_mesh` | part | mesh was inside-out and was flipped | — (info) |
| `degenerate` | part | zero volume, empty geometry, or could not be meshed | replace, ignore |
| `unaccounted_faces` | file | faces in the file not in any returned body | reexport |
| `placed_by_bbox` | part | replacement aligned by bounding-box centre (§4.2) | replace |
| `position_unverified` | part | replacement for a part with no geometry (§4.2) | replace |
| `unused_body` | export | a body sent to `/export` belongs to no link (§4.3) | — (info) |

`reexport` messages include advice keyed to the exporter named in the STEP header (`FILE_NAME` originating system), e.g. "Creo: export STEP AP214/AP242 with solids enabled", with a generic fallback.

## 4. User fixes

### 4.1 Mass override and approximate parts (on `/export`)

New optional `Link` fields (add-only):

```
mass_override_kg: float | None = None      # finite, > 0
accept_approximate_mass: bool = False
```

- Mass properties stay server-computed (base-spec ruling: client `mass_kg`/`center_of_mass`/`inertia` are ignored).
- With `mass_override_kg` set: compute COM and inertia from geometry at the link's density, then scale the inertia tensor by `mass_override_kg / computed_mass`. The COM is unchanged. (Correct for uniform density.)
- A link containing an `approximate` body must have either `mass_override_kg` or `accept_approximate_mass: true`; otherwise export fails with 422 naming the link and the body.
- A link containing a `failed` body fails export with 422 naming both, since there is no geometry to export.
- Material choice in the frontend maps to the existing `density_kg_m3`.

### 4.2 Replace a part — `POST /replace-part`

Multipart form fields: `file`, `body_id` (the original's id), optional `bbox_min`/`bbox_max` (the original's box, absent if it failed with no geometry), optional `scale`.

- The file is imported with the same code as `/import` (same statuses and warnings). Multiple solids in the file are combined into one body.
- Placement:
  - Original box given, and the replacement's box overlaps it within tolerance → keep coordinates.
  - Original box given, no overlap → translate so the box centres coincide, warning `placed_by_bbox` ("check rotation").
  - No original box → keep coordinates, warning `position_unverified`.
- Returns one `Body` carrying the original `body_id`, so links that reference it keep working.

### 4.3 Ignore or merge a part

No new API. A body in no link is not exported; a body listed under a neighbouring link is merged into it. `/export` still responds with the zip, plus a response header `X-Export-Warnings` holding a JSON list of `Warning` objects; it includes an `unused_body` warning for every body sent in `bodies_by_link` whose id is in no link's `body_ids`, so leaving a part out is always visible.

### 4.4 Scale

Unchanged: re-import with the `scale` query parameter. The applied scale is already returned.

## 5. Joint detection honesty — `/detect-joints`

Response shape changes from `Joint | null` to:

```
{ joint: Joint | null,
  reason: "no_overlap" | "no_contact_faces" | null,
  evidence: { pairs: [{kind, radius}], candidate_count: int, tolerance: float } | null }
```

- Bounding boxes don't overlap → `joint: null, reason: "no_overlap"` (today returns bare `null`).
- Boxes overlap but no matching faces → `joint: null, reason: "no_contact_faces"` (today returns a `fixed` joint).
- Otherwise → the suggested joint plus evidence: the matched pairs' kinds and radii, how many candidate pairs there were, and the tolerance used.

This is the base spec's §6c ("every suggestion carries the matched face pair and tolerance used").

## 6. Model corpus and scorecard

- `corpus/` (repo root, git-ignored): model files.
- `backend/corpus/expected/<name>.yaml` (committed): expectations for one model.

```yaml
file: irb1300_asm.stp
sha256: <hash>              # ties expectations to the exact file
scale: 0.001                # optional; mesh/brep only
parts: {min: 18}
statuses: {failed: 0}       # optional per-status upper bounds
links:                      # optional; enables joint checks
  - {name: base, bodies: [solid_0, PEDESTAL04557S]}
joints:
  - {parent: base, child: link1, type: [revolute, planar], axis: [0, 0, 1]}
```

`type` lists acceptable types. Axes match up to sign within 5°. Bodies are referenced by name.

- `backend/corpus/run.py` runs the real pipeline per model: import → mass per link → detect-joints per expected pair → export. It prints one line per model, e.g.
  `irb1300 | parts 18/≥18 | failed 0 | joints 6/6 type 2/6 axis 6/6 | warn 4 | export ok | 47s`
  and writes a JSON report.
- `backend/corpus/baseline.json` (committed): last accepted scores. The run exits non-zero if any model scores worse. `--update-baseline` rewrites it after an intended improvement.
- An expectations file whose model file is missing from `corpus/`, or whose sha256 doesn't match, is skipped with a note.
- The corpus is not part of `pytest` (it is slow and needs licensed files). It runs as a separate command before merges.
- Starting set: ABB IRB1300 (expectations from the 2026-10-08 probe); generated models that are always present — peg-in-hole STL meshed fine and coarse, and a STEP file with an open shell — written by a generator script into `corpus/`.

## 7. Errors

| Situation | Result |
|---|---|
| File unreadable | 422 naming the file |
| Unsupported extension | 400 |
| One part bad, rest fine | 200; that part `failed`/`approximate`, the rest imported |
| Export: link has an `approximate` body, no override, not accepted | 422 naming link and body |
| Export: link has a `failed` body | 422 naming link and body |
| `/replace-part` file unreadable | 422 |
| `mass_override_kg` not finite or ≤ 0 | 422 (model validation) |

All existing validation stays (name rules, URDF validity, structure, finite values, server-side mass).

## 8. Testing

TDD; each test is written and seen failing before the fix.

- A generated STEP file (one solid + one open shell) imports 2 bodies; the shell body is `repaired` or `approximate` with the matching warning.
- Accounting: an import that drops faces produces `unaccounted_faces` (exercised by forcing an unhandled face into the shape).
- STEP product names appear as body names.
- Non-watertight STL imports as `approximate` (200, not 422); inside-out STL gets `inverted_mesh`.
- Mass override: exported mass equals the override and inertia equals computed inertia × ratio; COM unchanged.
- Export rejects an `approximate` link without override or acceptance, and passes with either.
- Export rejects a link containing a `failed` body.
- `/replace-part`: one test per placement case (overlap keeps, no-overlap centres with `placed_by_bbox`, no box gives `position_unverified`); the returned body keeps the original id.
- `/detect-joints`: `no_overlap`, `no_contact_faces`, and a positive case with filled evidence.
- Corpus: `run.py` on the generated models produces the scorecard line format, the JSON report, and exit codes 0 (at baseline) and non-zero (regressed baseline).

## 9. Out of scope

- Frontend (next spec).
- Suggesting revolute instead of planar for planar contacts between links.
- Axis-consistent region merging for coarse STL cylinders.
- A lock around concurrent STEP imports; an upload size limit.
- Spherical and prismatic detection.
- User-entered inertia tensors (the mass override scales computed inertia only).
