# Workbenches, MuJoCo model and physics check: design

Date: 2026-10-14

## Goal

The app reads like professional CAD software, and every robot gets a MuJoCo model with a quick physics sanity check.

- **Two workbenches.** Build is where you assemble the robot (Parts → Group → Connect → Review). Simulate is where you test it: today's Program mode, plus the physics check.
- **A menu bar** (File, Edit, View) replaces the loose buttons in the stepper bar.
- **No download prompts.** The last Build step ends in "Continue to Simulate →". The zip lives under File ▸ Export.
- **Motor strength per joint**, auto-suggested from the masses and editable, so physics is meaningful.
- **Physics check.** The arm holds its home pose under gravity for 2 s of simulated time and gets a pass, or a fail with the cause.

This is part 1 of MuJoCo. Part 2 ("C", a separate spec) runs Program-mode programs on the MuJoCo arm with a Physics on/off toggle. It also adds friction, damping, speed limits, contact settings and a live MuJoCo view. None of those are in this spec.

Out of scope: a Train/RL workbench (added when RL exists); import of URDF/MJCF/project files (its own spec after MuJoCo C); View items beyond the three below; grippers.

## 1. Workbenches and menu bar

**Layout.** A top bar replaces today's stepper header:

```
CAD → URDF   File  Edit  View        [ Build | Simulate ]        (progress strips)
```

- In **Build**, the step tabs (Parts, Group, Connect, Review) show in a second row, as today.
- **Simulate** is today's Program mode screen. Its "Back to robot" button is replaced by the workbench switch. While a task runs the switch is disabled, with the existing "A task is running…" tooltip.
- Simulate is enabled when the Review checklist passes, the same rule as today's Program step. Disabled, it says why ("Fix the Review checklist first.").

**Menus** (new `frontend/src/ui/MenuBar.tsx`, plain buttons and a popover, no library):

| Menu | Items | Source |
|---|---|---|
| File | New project, Open project…, Save project, Import CAD…, Export ▸ URDF + MuJoCo (.zip) | Open, Save and Import move here from the stepper bar; Export moves here from the Review step |
| Edit | Undo, Redo (keep Ctrl+Z / Ctrl+Y) | moves from the stepper bar's icons |
| View | Grid ✓, Part edges ✓, Fit to view | new `Viewer.setGrid(on)`, `Viewer.setEdges(on)`; Fit is `viewer.setView("home")` |

- Grid and Part edges are remembered per browser (localStorage, wrapped in try/catch, default on).
- Menus close on outside click and Escape, and open with keyboard focus (buttons with `aria-haspopup`/`aria-expanded`).
- "New project" asks for confirmation when the model has links.

**Steps.** The `export` step is labelled **Review**; its id stays `export` to keep the diff small. Its primary button becomes **Continue to Simulate →**: same enabled rule as today's Export button, and it calls `goTo("program")`. The download button leaves the panel. The `program` step id stays internal; the Build step row no longer shows it.

## 2. Motor strength (max torque)

**Model.** `Joint` gains `max_effort: float | null` (backend `models.py`, frontend `api/types.ts`).
- `null` means auto: the backend fills in the suggested value.
- Units: N·m for revolute and continuous joints, N for prismatic.
- Old project files load unchanged: the field is missing, so it's null. No project version bump.

**Suggestion.** New `backend/app/sim/torque.py`, using MuJoCo's own inverse dynamics instead of hand-written dynamics:
- `required_efforts(mjcf_xml, samples=500, seed=0) -> dict[joint_name, float]` compiles the model.
- For `samples` random poses within joint limits (continuous joints: full circle), with qvel = qacc = 0, it calls `mj_inverse`. Per joint it takes the largest |qfrc_inverse|, which is the torque needed to hold against gravity in the worst pose found.
- Suggested `max_effort = 2 × that`, rounded up to 2 significant figures, minimum 1.
- Endpoint `POST /suggest-efforts` (body = the export request) returns `{joint_id: suggested}`. It runs in the child-process pattern of `/import`, because MuJoCo's compile holds the GIL.

**UI.** In the Connect step's joint panel, add a "Max torque" field (or "Max force" for prismatic), with units:
- Empty means auto. The placeholder shows the suggestion, e.g. `auto · 140 N·m`.
- Typing a number sets `max_effort`; clearing it returns to auto.
- Suggestions are fetched when the Connect step opens and after joints change (debounced 500 ms). A failed fetch leaves the placeholder as `auto`.

**Export.** `urdf_writer` writes `effort="<max_effort or suggested>"` instead of the hard-coded 100, so ROS and MuJoCo agree. `export_package` computes the suggestions it needs (in-process: one MuJoCo compile plus 500 inverse-dynamics calls, about 1–2 s, briefly holding the GIL; accepted for export, unlike the 40 s OCCT read), and the zip carries them.

## 3. MuJoCo model (MJCF)

New `backend/app/sim/mjcf.py`: `build_mjcf(robot, description_dir) -> str` (MJCF XML text).
1. Read `<robot>/urdf/<robot>.urdf`. Rewrite `package://<robot>/meshes/x.stl` to `../meshes/x.stl`, and add MuJoCo's URDF extension `<mujoco><compiler meshdir="../meshes" strippath="false" balanceinertia="true" discardvisual="true"/></mujoco>`.
2. Compile with `mujoco.MjModel.from_xml_string` (assets from `meshes/`), then save with `mj_saveLastXML` to text. The root link stays fixed to the world, as MuJoCo's URDF import does.
3. Post-process the saved XML with `ElementTree`:
   - one `<position joint=… kp=… forcerange="-E E" forcelimited="true"/>` actuator per non-fixed joint, where `E` = that joint's effort and `kp = E / 0.05` (full torque at 0.05 rad error);
   - `<contact><exclude body1 body2/>` for every pair `disabled_pairs(chain, meshes, samples=0)` marks `Adjacent` or `Default` (touching at home pose);
   - `<option timestep="0.002"/>`.

The export zip gains `<robot>/mjcf/<robot>.xml` (meshes referenced as `../meshes/`). `workspace_from_zip` already extracts the description package, so the ROS workspace gets the MJCF for free, ready for part C.

Dependency: `mujoco` (pip wheel), added to `backend/pyproject.toml` and installed into the conda env.

## 4. Physics check

New `backend/app/sim/check.py`: `hold_check(mjcf_xml, efforts, seconds=2.0) -> dict`. It runs in a spawned child process (the pattern from `import_routes.py`).

1. **Compile.** On failure: `{"ok": false, "kind": "load", "message": "Model won't load: <MuJoCo's error, with body names mapped to link names>"}`.
2. **Hold.** Set qpos to home (0 for every joint, clamped into limits), actuator targets to home, and step `seconds / timestep` times.
3. **Blew up.** If any qpos/qvel is non-finite or `data.warning` reports instability:
   `{"ok": false, "kind": "unstable", "time": t, "link": <link with the largest velocity>, "message": "Simulation blew up after 0.3 s: check <link>'s mass and inertia."}`
4. **Sag.** After the run, any joint whose |q − home| > 5° (revolute) or 5 mm (prismatic):
   `{"ok": false, "kind": "sag", "joint", "link": child link, "deviation", "needed": required effort, "has": effort, "message": "<link> sags 18°: needs ~140 N·m, has 100."}`
   Report the worst one.
5. Otherwise `{"ok": true, "message": "Holds pose."}`.

`POST /simulate/check` (body = the export request) builds the MJCF, runs `hold_check` in the child process and returns the result. A child crash gives `{"ok": false, "kind": "crash", "message": "The physics check crashed."}`, never a hung request.

**UI.** In Simulate, a "Physics" card above the chat column shows the result:
- ✓ green, or ✗ red with the message and the link name highlighted in the viewer;
- a **Run check** button.

It runs by itself when Simulate opens and the model changed since the last check (keyed by a hash of the export request). While running it shows "Checking physics…" with the elapsed timer. Errors from the request show in the card, not as a banner.

## Testing

- **Backend `sim`.**
  - A two-link arm fixture holds pose when efforts are suggested.
  - The same arm with effort 0.1 gives `sag` naming the right joint and link.
  - Zero mass on a link gives `load` or `unstable` with the link named.
  - `required_efforts` is larger for the shoulder than the wrist.
  - The MJCF has one actuator per moving joint and excludes the adjacent pairs.
- **Backend routes.** `/simulate/check` and `/suggest-efforts` return the shapes above. The URDF effort equals the explicit `max_effort` or the suggestion. Old projects without `max_effort` still export.
- **Frontend.**
  - Menu items call the same actions as the old buttons.
  - The workbench switch is gated by the Review checklist.
  - Review's primary button goes to Simulate and there's no download button in the step.
  - View toggles call `setGrid`/`setEdges` and persist.
  - The Max torque field: empty → null, a number → value; the placeholder shows the suggestion.
  - The Physics card renders ok, sag and crash results.
- **Real-app check:** irb1300 holds pose with suggested torques. Setting link2's torque to 10 fails with a sag message naming link2.

## Findings (Task 8, real irb1300)

- The irb1300 exports fine (64 s including the STEP import), but **MuJoCo rejects it**: `link5`'s inertia has negative eigenvalues (URDF `ixx = -0.00273`, `iyy = -0.00146`). As designed, the export falls back to effort 100 for every joint and ships no `mjcf/`. The Physics card reports "Model won't load: … inertia must have positive eigenvalues … link5".
- Cause, traced: `compute_link_mass_properties` integrates signed tetrahedra over the merged link mesh. `link5`'s mesh gives eigenvalues (−0.00107, −0.00094, 0.00085) × 10⁻³ kg·m², a non-physical tensor from mixed-orientation or overlapping shells inside the link. It is a pre-existing mass-properties bug that ROS/MoveIt never exercised, because they don't simulate dynamics. `link4` and `link6` are positive definite.
- Next step (its own fix, outside this plan): make link mass properties robust to badly oriented or overlapping part shells, e.g. per-part properties summed with the parallel-axis theorem, fixing winding per part. Then re-run the irb1300 hold check.
