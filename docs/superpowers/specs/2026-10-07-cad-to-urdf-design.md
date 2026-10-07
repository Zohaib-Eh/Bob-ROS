# CAD-to-URDF Hobbyist App — Design

Date: 2026-10-07
Status: approved (design conversation), pending writing-plans handoff

## 1. Purpose and users

Hobbyist roboticists download part/assembly files from sources like GrabCAD and
need a working URDF to simulate or 3D-print-and-build a robot. Today that means
either a CAD-tool-specific exporter (SolidWorks/Onshape only, requires that
exact tool) or hand-writing URDF joints by eyeballing coordinates. This app
takes arbitrary CAD/mesh files, lets the user confirm or adjust automatically
suggested joints, validates them, and exports a working URDF package.

Success criteria:
- A user can drop in a single multi-part STEP assembly (as downloaded from
  GrabCAD) and get correct joint suggestions for most simple mated joints
  (hinges, sliders) without manually placing a gizmo.
- A user can instead drop in several independent per-part files (STL/OBJ/STEP,
  no relative placement), name parent/child, and get a suggested joint.
- Every exported URDF has passed a structural check and a physics sanity
  check before download; failures are explained per-joint, not just pass/fail.
- No install required to use it: open a web page, upload, download a zip.

Out of scope for this spec (deferred, future specs): MuJoCo/Gazebo scene
export, and any natural-language/prompt-driven simulation generation layer.
This spec's backend is architected so those are additive later (same service,
new endpoints), not a rewrite.

## 2. Scope: must-have vs nice-to-have

**Must-have (MVP — app is not useful without these):**
- Import single-file STEP/IGES/BREP assembly, split into separate solids.
- Import separate per-part files (STL/OBJ/STEP), one per upload. For bare
  mesh formats (STL/OBJ) this includes face reconstruction (§6a-prime) —
  without it those formats have no faces to detect joints from at all.
- Group bodies into links (user-driven merge, e.g. bolted brackets into one
  rigid link).
- Manual joint placement via gizmo — always available, the guaranteed
  fallback when nothing else fires.
- Auto joint-type detection for the single-file case (§6a) — this is the
  core value proposition over existing tools (confirmed by the competitor
  check: nothing shipped does this today).
- Structural validation gate (§7.1) — cheap, blocks obviously-broken exports.
- URDF export with correct mesh scale, mass, and inertia (§8).

**Nice-to-have (real value, ship after MVP works end-to-end):**
- Auto joint detection for the separate-file case (§6b) — heuristic,
  unverified as prior art, higher risk of being wrong; manual placement
  covers this case in the meantime.
- Collision-sweep validation + auto-derived limits (§7.2).
- Headless physics sanity check (§7.3) — needs a pybullet/mujoco subprocess,
  heavier infra than the rest of the backend; valuable but not blocking.
- Suggestion confidence/explanation display (§6c) — matters more once
  auto-detection is actually running broadly.
- Joint motion preview slider in the UI.
- Per-link mass override (vs. density-only) input.

**Explicitly deferred — see §11, not part of this spec's build at all.**

## 3. Architecture

Two deployed pieces:

- **Frontend** — static web app (Vite + TypeScript + three.js). Holds no CAD
  or joint-inference logic. Renders what the backend returns, collects user
  choices, calls the backend, shows results.
- **Backend** — FastAPI service (Python). Owns all processing: CAD/mesh
  import, face classification, joint inference, validation, URDF export.
  Stateless between requests; the frontend resends the current model state
  (or a session id) on each call.

```
browser (three.js viewer, upload + joint-review UI)
    |  HTTP/JSON (+ multipart file upload)
    v
FastAPI backend
    - pythonocc-core  (STEP/IGES/BREP import, exact analytic surface types)
    - trimesh         (STL/OBJ/GLTF import, mesh ops, inertia)
    - python-fcl       (collision sweep)
    - pybullet/mujoco  (headless instability check, run in a subprocess)
```

No database, no accounts. A run's state lives for the duration of a browser
session; nothing persists server-side after the response is sent.

## 4. Repository structure

```
/frontend
  src/viewer/        3D scene, body/link selection, gizmos, limit-slider preview
  src/ui/            upload, link-grouping panel, joint-review panel, export button
  src/api/           typed client for backend endpoints
/backend
  app/import/        pythonocc-core loader (STEP/IGES/BREP), trimesh loader (STL/OBJ/GLTF)
  app/segment/       mesh-only face reconstruction: region growing + plane/
                     cylinder/sphere fitting (STL/OBJ/GLTF input only)
  app/joints/        face classification, face-pair matching, IPV/IMG joint-type lookup
  app/validate/      collision sweep (limit derivation + interference check),
                     headless pybullet/mujoco instability check
  app/export/        URDF + mesh zip builder, structural tree checks
  tests/             analytic-shape fixtures (cube/cylinder/sphere), sample
                     GrabCAD assemblies (hinge, slider) as end-to-end fixtures
docs/superpowers/specs/   this file and future design docs
```

## 5. Data model

- `Body` — one imported solid: id, name, triangle mesh, source face list (each
  face tagged with its analytic type — plane/cylinder/sphere/other — plus the
  fitted axis/radius/normal/center for that type). Each face also carries an
  `origin` tag: `brep` (exact, straight from pythonocc-core, STEP/IGES/BREP
  input) or `segmented` (approximate, reconstructed from a bare mesh — see
  §6a-prime). Downstream joint detection treats `segmented` faces as lower
  confidence than `brep` ones.
- `Link` — one or more `Body` grouped by the user; name, member body ids,
  density or user-entered mass override, computed mass/center-of-mass/inertia.
- `Joint` — parent link id, child link id, type (revolute / continuous /
  prismatic / cylindrical / planar / spherical / fixed), origin (xyz + rpy),
  axis, limits (lower/upper/velocity/effort), and a `source` flag:
  `suggested` (not yet confirmed), `confirmed`, or `manual`.
- Model is a tree: exactly one root link, every other link reachable through
  exactly one parent joint, no cycles. Enforced at validation time, not just
  assumed.

## 6. Joint detection

### 6a-prime. Face reconstruction for mesh-only input (STL/OBJ/GLTF)

STEP/IGES/BREP bodies get exact analytic faces for free from pythonocc-core
(`GeomAbs_Plane`/`Cylinder`/`Sphere`, with no fitting needed — see §6a). A
bare mesh has none of that: STL/OBJ/GLTF are triangle soup with no face
boundaries and no surface-type information at all. Both the single-file and
separate-file detection paths (§6a, §6b) need faces to operate on, so
mesh-only bodies go through a reconstruction step first:

1. **Region growing**: build triangle adjacency (free from the mesh's own
   connectivity — two triangles sharing an edge are adjacent) and grow
   connected regions where adjacent triangles either (a) have near-identical
   normals (a planar region), or (b) have normals that stay consistent with a
   shared candidate cylinder axis as the region grows (a cylindrical region).
2. **Fit per region**: plane via PCA on vertex positions (smallest eigenvalue
   ≈ 0 confirms planarity, that eigenvector is the normal); cylinder via PCA
   on the region's triangle normals (the eigenvector with smallest normal
   variance is the axis; radius from least-squares distance-to-axis) — the
   method documented in Wu et al. 2018 (*Math. Problems in Engineering*,
   DOI 10.1155/2018/8904653); sphere via algebraic least-squares fit.
3. Each resulting face is tagged `origin: segmented` and carries a fit
   residual (how well the triangles actually matched the fitted primitive).
   A region that doesn't fit any primitive within tolerance is left
   untyped — it contributes no candidate face, it does not get forced into
   the nearest-fitting type.
4. Segmented faces feed into §6a/§6b exactly like `brep` faces, but a
   `segmented` match with poor residual lowers the suggestion's confidence
   (§6c) and is more readily routed to "no suggestion, place manually"
   (§9) rather than risk a wrong auto-placed joint.

This is the same primitive-fitting math whether the face came from a region
grown on a bare mesh or was already isolated by OCCT — one classifier, two
ways of getting its input.

### 6a. Single assembled file (parts already touching/mated)

1. Backend already has, per body, each face's exact analytic surface type and
   parameters from pythonocc-core (`GeomAbs_Plane` / `GeomAbs_Cylinder` /
   `GeomAbs_Sphere`, with axis/radius/normal/center) — no curve-fitting
   approximation needed, OCCT gives this directly.
2. For every pair of bodies in contact (bounding-box overlap as a cheap
   pre-filter, then face-distance check), find face pairs that are coincident
   (planar, same normal, touching) or concentric (cylindrical/spherical, same
   axis/center, matching radius within tolerance).
3. Classify the joint from the matched constraint set using the IPV/IMG table
   (Kim et al. 2005; independent principal vectors → rotational DOF, the
   intersection-of-mating-geometry dimensionality → translational DOF):

   | Joint | IPV | IMG | Face-pair pattern |
   |---|---|---|---|
   | Spherical | 0 | Point | matched point contact |
   | Revolute | 1 | Point | matched cylindrical axis ⊥ matched planar contact |
   | Cylindrical | 1 | Line | matched cylindrical axis, no perpendicular plane lock |
   | Planar | 1 | Plane | matched planar contact only |
   | Prismatic | 2 | Line | matched cylindrical axis ∥ matched planar contact (sliding) |
   | Fixed | — | — | no relative-motion-permitting face pair found |

4. Result is a `suggested` joint with computed origin/axis. Never
   auto-applied — surfaced in the joint-review panel for the user to confirm,
   edit, or reject.

### 6b. Separate per-part files (no relative placement, user names parent/child)

1. Per body, find candidate "mating" faces: the largest cylindrical face and
   the largest planar face (by area), same analytic-type extraction as §6a.
2. Across the named parent/child pair, match candidate faces by type and size
   (radius within tolerance for cylinders, comparable area for planes) — this
   is an extension of documented SolidWorks SmartMates logic (same-feature-
   type matching) applied across independent files instead of within one
   session.
3. If a match is found, compute the rigid transform that aligns the matched
   faces, place the child relative to the parent accordingly, and run the same
   IPV/IMG classification as §6a on the resulting (now-synthetic) contact.
4. If no confident match is found, return "no suggestion" — the user places
   the joint manually via gizmo. The app never guesses silently.

### 6c. Confidence and overrides

Every suggestion carries the matched face pair and tolerance used, shown to
the user so a bad suggestion is explainable, not a black box. The user can
accept, edit any field, or discard and place manually at any point.

## 7. Validation

Runs automatically whenever a joint is confirmed/edited, and again as a gate
immediately before export.

1. **Structural gate** (cheap, instant): exactly one root, no cycles, no
   orphan links, every non-leaf link has nonzero inertia. Equivalent to
   `check_urdf`'s tree checks, reimplemented directly rather than shelling out.
2. **Collision sweep**: move the child link through the joint's candidate
   range in small steps, mesh-mesh intersection test (`python-fcl`/`trimesh`)
   against the parent and geometrically-nearby links at each step. Used both
   to derive/confirm sensible limits (first point of contact) and to flag
   self-collision.
3. **Headless physics sanity check**: load the current export as a URDF into
   PyBullet (or MuJoCo) headless, step N frames with zero control input,
   check for NaN/divergence. Catches bad inertia or a flipped/degenerate axis
   that the structural and collision checks wouldn't see.

A failure in any layer blocks export **for that joint only**; other
links/joints remain exportable, and the reason (not just pass/fail) is shown
next to the offending joint.

## 8. Export

Builds a URDF XML file plus a `meshes/` folder (STL, with the correct
mm→m scale already applied) packaged as a ROS-package-shaped zip, so it drops
directly into a ROS workspace or stands alone. Each exported mesh/link also
gets the inertial origin set to match its visual/collision origin (not the
joint origin) to avoid the documented silent-wrong-physics mistake.

## 9. Error handling

- Corrupt/unsupported file on import: structured error for that file, the
  rest of the batch still loads.
- No face-pair match in detection (§6a/§6b): explicit "no suggestion," not a
  wrong guess.
- Validation failure: scoped to the one joint, with the specific cause shown.

## 10. Testing

- Backend unit tests against analytic primitives (cube, cylinder, sphere)
  with known-correct joint classification and inertia, as a correctness
  baseline independent of any real CAD file. Run against both a STEP version
  (exercises the `brep` face path) and a watertight-STL version of the same
  shapes (exercises the §6a-prime region-growing/fitting path) — the two
  input paths are different code and need separate coverage.
- A small set of real sample assemblies (simple hinge, simple slider) as
  end-to-end fixtures covering import → detect → validate → export.
- Frontend smoke test: an exported URDF round-trips through a URDF parser
  without error.

## 11. Explicitly deferred (future specs)

- MuJoCo/Gazebo scene/world export beyond the headless sanity check already
  described in §7.3 (that check loads into the engine transiently; it does
  not produce a saved simulation package).
- Any prompt-driven/natural-language code generation or simulation authoring
  layer. Confirmed via research to be active 2026 research territory (e.g.
  SR-Platform, arXiv 2605.14700) with no reliable shipped consumer product —
  not scoped until the above is built and working.
