# Frontend — Design

**Date:** 2026-10-08
**Builds on:** `docs/superpowers/specs/2026-10-07-cad-to-urdf-design.md` (base spec, §2–§4) and `docs/superpowers/specs/2026-10-08-import-robustness-design.md` (part status, warnings, fixes, `/replace-part`, `/detect-joints` evidence).
**Scope:** the web frontend plus two small add-only backend endpoints it needs. Local use only.

## 1. Purpose and success

A hobbyist takes a CAD/mesh assembly (e.g. a GrabCAD STEP) to a downloaded URDF package without writing JSON: import, fix problem parts, group parts into links, review suggested joints, verify motion, export.

Success criteria:

- On the ABB IRB1300 STEP (the reference model): import shows 18 parts; after merging hardware, 6 joint suggestions appear; the user confirms them, resolves the approximate parts with a mass override or acceptance, previews each joint's motion, and downloads a zip — no JSON editing.
- Every backend warning or error is visible in the UI next to the part, link or joint it concerns, with its available fixes.
- Save project → close tab → open project restores all work.
- Every edit can be undone and redone.

## 2. Decisions (from the design conversation)

| Topic | Decision |
|---|---|
| Where it runs | Locally only: backend on localhost:8000, frontend dev server on localhost:5173. No hosting work. |
| Layout | One workspace: 3D view centre, links/parts tree left, details panel right, top bar. |
| Joint suggestions | Automatic background detection for every link pair whose bounding boxes touch. |
| Initial grouping | One link per part; the user merges. |
| UI technology | React + TypeScript (Vite) for panels; plain three.js for the 3D view, driven from one viewer module. |

## 3. Architecture

```
frontend/
  package.json, vite.config.ts      # dev server proxies /api/* -> http://localhost:8000
  src/api/        client.ts, types.ts   # typed calls to /import, /replace-part, /detect-joints,
                                         # /link-mass, /export; types mirror backend models
  src/model/      store.ts (reducer + context), actions, pure helpers:
                  tree.ts (tree direction), pairs.ts (touching pairs), smallParts.ts,
                  errors.ts (422 -> per-item errors), project.ts (save/open), history.ts (undo/redo)
  src/viewer/     scene.ts (three.js scene, meshes, picking, highlight, joint arrows),
                  gizmo.ts (TransformControls for joint origin/axis), preview.ts (motion preview)
  src/ui/         TopBar, PartsTree, DetailsPanel (part / link / joint / overview views),
                  StatusBar (detection progress), Banner (errors)
```

- `npm run dev` starts the frontend; a repo-root script starts backend and frontend together.
- The browser calls `/api/...`; Vite forwards to the backend, so no CORS setup.
- The frontend holds no CAD or joint-inference logic. Its geometry is limited to drawing meshes, bounding boxes for the touching-pairs pre-check, and applying a joint transform for the motion preview.

### 3.1 Model store

One store (`useReducer` + React context, no state library):

- `bodies: Record<id, Body>` and `fileWarnings` — from `/import` and `/replace-part`; meshes are immutable once received.
- `links: Link[]` — initially one per body; `rootLinkId`.
- `joints: Joint[]` — accepted or manual.
- `suggestions: Record<pairKey, Suggestion>` — detection results awaiting review, including "no suggestion" results with their reason; `rejected` pairs.
- `selection`, `detectionQueue` (pending / running / done per pair), `linkMass: Record<linkId, result | error>`.

The viewer subscribes to the store and dispatches selection actions; panels dispatch edit actions; only `src/api` calls the backend.

### 3.2 Undo / redo

Every edit action (merge, move, ignore, rename, set base, accept/reject/edit joint, set mass/density, replace part) pushes the previous editable state — `links`, `joints`, `suggestions` status, `rootLinkId`, and the `bodies` map reference — onto a history stack. Meshes are shared by reference, never copied. Ctrl+Z / Ctrl+Y (and toolbar buttons) move through history. Selection, detection progress, and banners are not history. History is capped at 100 steps. Undo of an action that changed links re-queues the affected detection pairs.

### 3.3 Save / open project

"Save project" downloads one JSON file containing the store's persistent part: bodies (with meshes), file warnings, links, joints, suggestions (with evidence and rejections), root link, and a format version. "Open project" restores it and clears history. Opening a file with an unknown format version shows an error and changes nothing.

## 4. Workspace

### 4.1 Top bar

- **Import** — file picker (STEP/STP/IGES/IGS/BREP/STL/OBJ), scale field shown for STL/OBJ/BREP (default 0.001), applied scale displayed after import. Further imports add parts to the current project.
- **Undo / Redo**, **Save project**, **Open project**.
- **Export** — robot name field (validated live against `^[A-Za-z0-9_][A-Za-z0-9_-]*$`), then download. Disabled while the tree is invalid, with the reason as a tooltip.

### 4.2 Left panel — links and parts tree

- Links with their parts; each part shows a status icon: ok, repaired, approximate (⚠), failed (✖). File-level warnings sit at the top of the panel.
- Multi-select parts → "Merge into link"; drag a part onto another link to move it.
- Link rename, validated live against the export name rule; duplicate names flagged.
- One link marked ⚓ base; default = link with the largest bounding-box volume; changeable.
- **Small parts group:** links whose bounding-box volume is under 1% of the largest link's and that touch exactly one other link are listed together with "Merge all into neighbours". Suggestions involving only these links are collapsed under the group rather than listed individually.
- Errors from validation or export are pinned to the link or joint they name.

### 4.3 Centre — 3D view

- Orbit / pan / zoom; click to select a part (shift-click adds); selection highlighted; parts tinted by status (approximate amber, failed red outline at the original bounding box).
- Ignored parts drawn greyed.
- Accepted joints drawn as an arrow along the axis at the origin; suggestions as dashed arrows.

### 4.4 Right panel — details of the selection

- **Part:** status, warnings, and one button per fix in each warning's `fixes`:
  - Set mass → focuses the link's mass override field.
  - Replace → file picker, calls `/replace-part` with `body_id`, `name`, and the part's bounding box (omitted if it has no geometry); the result replaces the body in place, keeping its link.
  - Ignore → removes the part from its link (undoable); greyed in the view.
  - Re-export → shows the warning's advice text.
- **Link:** name, density (with a few material presets), mass override, "accept approximate mass" checkbox, and the result of `/link-mass` (mass, centre of mass) or its error.
- **Joint / suggestion:**
  - Type, axis, origin, limits; for suggestions, the evidence (matched pair kinds and radii, candidate count, tolerance, ⚠ ambiguous).
  - Accept / Reject / Edit. Accepting a `cylindrical` or `spherical` suggestion requires choosing a URDF type (revolute / continuous / prismatic); revolute and prismatic require limits.
  - Edit attaches the gizmo to the joint origin. Snap buttons: "axis along selected cylinder face", "axis along selected face normal".
  - **Motion preview slider:** moves the child link and every link below it in the tree about the joint (rotation for revolute/continuous, translation for prismatic). Revolute/prismatic sweep between the limits; continuous sweeps a full turn. Display-only; it never changes the model. Releasing it returns to the zero pose.
- **Nothing selected:** overview listing suggestions (needs review / accepted / rejected / no suggestion) and "Add manual joint between selected links" (creates a `manual` joint at the centre of the two links' bounding-box overlap — or, if the boxes don't overlap, midway between the two box centres — with axis +Z, type fixed, then opens Edit).

### 4.5 Tree direction

Parent/child is never chosen by hand. The app runs a breadth-first walk from the base link over accepted joints; each joint's parent is the side reached first. A cycle, a link with no path to the base, or two joints between the same pair is an error pinned to the link(s) involved; Export stays disabled until none remain.

## 5. Background joint detection

- After any change to links, the app computes all link pairs whose part bounding boxes overlap within 1e-4 m (the backend's contact tolerance) and queues pairs that are new or whose links changed. A running request for a changed pair is cancelled (`AbortController`).
- One request at a time (the backend work per pair is heavy; the host is memory-constrained).
- Each result becomes a suggestion with its evidence; `joint: null` results are kept as "no suggestion" with the reason, so the user can add a manual joint there.
- Status bar: "Checking joints: 3 of 9" with Cancel. Cancel stops the queue; a "Resume" button restarts it.
- Rejected suggestions stay rejected for that pair until either link's membership changes.

## 6. Backend additions (add-only)

1. **`/detect-joints` accepts whole links.** Request gains optional `bodies_a: list[Body]` and `bodies_b: list[Body]`; exactly one of `body_a`/`bodies_a` and of `body_b`/`bodies_b` must be given (422 otherwise). Lists are combined server-side (reusing `combine_bodies`) before the existing contact check, matching, and classification. The response shape is unchanged.
2. **`POST /link-mass`.** Request `{link: Link, bodies: list[Body]}`; response `{mass_kg, center_of_mass, inertia}` computed exactly as `/export` does (same status guard, override scaling, density check, body_ids filtering), or 422 with the same message export would give.

Both get pytest tests before the frontend uses them.

## 7. Export

The frontend builds `links` (with `body_ids`), `joints` (parent/child from §4.5, `source` confirmed or manual), and `bodies_by_link` keyed by link id, then posts to `/export`. On 200 it downloads `<robot>.zip` and shows any `X-Export-Warnings` entries (e.g. "screw_3 is in no link and was not exported"). On 422 the message is split on `; ` and each piece containing a link id (`Link 'x'`) or joint id (`Joint x`) is pinned to that item; anything else goes to the banner.

## 8. Errors

- Network / 5xx errors: a dismissible banner; the store is unchanged.
- Import failure (400/422): banner naming the file; existing project untouched.
- Every failed request produces a visible message; nothing fails silently.

## 9. Testing

TDD throughout.

- **Vitest unit tests** for pure logic in `src/model/`: tree direction (valid tree, cycle, disconnected link, duplicate pair), touching pairs, small-parts grouping, detection re-queue on link change, 422 message → per-item errors, project save/open round-trip (and unknown version rejected), undo/redo (merge then undo restores links; redo reapplies; history cap; undo re-queues pairs), motion preview transform (child subtree rotated about the axis by the slider angle; analytic check on a point).
- **pytest** for the backend additions: link-level `/detect-joints` on the peg-in-hole model with the block split into two bodies gives the same cylindrical joint; exactly-one-of validation; `/link-mass` matches a unit box's analytic mass and inertia and returns export's 422 messages.
- **One Playwright end-to-end smoke test** against the real backend: import the generated peg-in-hole STEP → a suggestion appears → accept as continuous → preview slider moves the peg → undo/redo the accept → export → the downloaded zip contains the URDF and both meshes. (Costs one Playwright browser download, ~300 MB.)
- **Manual checklist** for the ABB file, run once before merge: §1's first success criterion.

## 10. Out of scope

- Hosting / deployment.
- Physics sanity check and collision sweep (base spec §7.2–§7.3) — next spec, after this frontend works.
- Confidence scoring beyond the evidence the backend returns.
- Auto joint detection across separately uploaded files (base spec §6b); manual joints cover it.
