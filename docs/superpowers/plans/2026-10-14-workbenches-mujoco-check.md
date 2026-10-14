# Workbenches, MuJoCo Model and Physics Check Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build | Simulate workbenches with a File/Edit/View menu bar, per-joint motor strength, a MuJoCo model in every export, and a 2 s "holds pose" physics check in Simulate.

**Architecture:**
- **Backend.** A new `app/sim` package turns our URDF into MJCF with MuJoCo's own URDF importer and adds position motors. It computes the torque each joint needs with `mj_inverse` and runs the hold check. `export_package` uses it, so every zip and every ROS workspace carries `mjcf/<robot>.xml`. The two new endpoints run their work in a spawned child process, because MuJoCo's compile holds the GIL.
- **Frontend.**
  - A `MenuBar` replaces the loose stepper buttons. The workbench switch shows Build (today's steps) or Simulate (today's Program mode plus a Physics card).
  - Review's primary button validates on the server and goes to Simulate. The download lives only under File ▸ Export.

**Tech Stack:** Python 3.11, FastAPI, `mujoco>=3.2` (pip wheel), numpy; React 18 + TypeScript, vitest.

**Spec:** `docs/superpowers/specs/2026-10-14-workbenches-mujoco-check-design.md`

## Global Constraints

- Backend tests: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest <path> -q`. Frontend: `cd frontend && npx vitest run <path>` and `npx tsc --noEmit`.
- Units: `max_effort` is N·m for revolute/continuous and N for prismatic; `null` = auto.
- Suggested effort = 2 × worst static gravity torque over 500 random poses (seed 0), rounded **up** to 2 significant figures, minimum 1.
- Motors: MuJoCo `<position>` actuator per non-fixed joint, `kp = E / 0.05`, `dampratio="1"`, `forcerange="-E E"`, `forcelimited="true"`.
- Hold check: 2.0 s simulated time, `timestep 0.002`. Sag threshold: 5° (revolute/continuous) or 5 mm (prismatic).
- Check result shape: `{"ok": bool, "kind": "ok"|"load"|"unstable"|"sag"|"crash", "message": str, "link"?: str, "joint"?: str, "needed"?: float, "has"?: float}`.
- Internal step ids stay: `export` (labelled "Review") and `program` (shown as the Simulate workbench).
- Copy: "Continue to Simulate →", "Fix the Review checklist first.", "Switch to Build to change the robot.", "Checking physics…", "Holds pose.".
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **Joints of type `fixed` and `continuous`.** Fixed joints get no motor and no torque entry. Continuous joints are sampled over the full circle and get a motor. Test in Task 1.
2. **A link with zero mass, or a robot with a single link.** The zero-mass case gets a `load`/`unstable` result naming the link, never a 500. The single link has no joints: "Holds pose." with no motors. Test in Task 3.
3. **An old project file with no `max_effort`.** It loads and exports with suggested efforts. Test in Task 2.
4. **The physics check child process crashes or takes >60 s.** The result is `kind: "crash"`; the request never hangs. Test in Task 3.
5. **Typing a non-number or a negative value in Max torque.** It's treated as auto (null) and never sent as NaN or a negative effort. Test in Task 5.

---

## File map

| File | Responsibility |
|---|---|
| `backend/app/sim/__init__.py` | package marker |
| `backend/app/sim/mjcf.py` | `build_mjcf(urdf, meshes, efforts)`, `compile_mjcf(xml, meshes)` |
| `backend/app/sim/torque.py` | `required_efforts(model)`, `suggest(required)` |
| `backend/app/sim/check.py` | `hold_check(xml, meshes)`; `physics_check(request dicts)` (runs in the child) |
| `backend/app/sim/child.py` | `run_in_child(fn, *args, timeout)`: spawn one process, return its result |
| `backend/app/routes/sim_routes.py` | `POST /suggest-efforts`, `POST /simulate/check` |
| `backend/app/models.py` | `Joint.max_effort` |
| `backend/app/export/urdf_writer.py` | effort per joint; MJCF in the zip; `suggest_efforts(...)` |
| `frontend/src/ui/MenuBar.tsx` (new) | brand, File/Edit/View menus, workbench switch, import progress strip |
| `frontend/src/ui/Stepper.tsx` | only step tabs + joint-check progress |
| `frontend/src/viewer/Viewer.ts` | `setGrid`, `setEdges` |
| `frontend/src/ui/JointTool.tsx` + `model/jointDraft.ts` | Max torque field, `maxEffort` in the draft |
| `frontend/src/ui/useEffortSuggestions.ts` (new) | debounced `/suggest-efforts` fetch |
| `frontend/src/ui/steps/ExportStep.tsx` | "Review": Continue to Simulate → |
| `frontend/src/ui/program/PhysicsCard.tsx` (new) | physics check card |
| `frontend/src/ui/program/ProgramMode.tsx` | MenuBar instead of Back button; PhysicsCard |

---

### Task 1: MuJoCo model and required torques

**Files:**
- Modify: `backend/pyproject.toml` (add `"mujoco>=3.2"`)
- Create: `backend/app/sim/__init__.py`, `backend/app/sim/mjcf.py`, `backend/app/sim/torque.py`
- Test: `backend/tests/test_sim_mjcf.py`

**Interfaces:**
- Produces: `build_mjcf(urdf: str, meshes: dict[str, bytes], efforts: dict[str, float] | None) -> str`.
  - `meshes` is keyed by bare file name `"<link>.stl"`; `efforts` is keyed by URDF joint name. `None` = no motors (used to measure torques).
  - Returns MJCF text with `<compiler meshdir="../meshes"/>`.
  - Bodies touching at home pose get `<contact><exclude>`.
- Produces: `compile_mjcf(xml: str, meshes: dict[str, bytes]) -> mujoco.MjModel` (strips the meshdir and loads with assets).
- Produces: `required_efforts(model: mujoco.MjModel, samples: int = 500, seed: int = 0) -> dict[str, float]` (joint name → worst |gravity torque|), `suggest(required: dict[str, float]) -> dict[str, float]`.

- [ ] **Step 1: Install MuJoCo.** Add `"mujoco>=3.2",` to `dependencies` in `backend/pyproject.toml`, then:

Run: `~/miniforge3/envs/cad-to-urdf/python.exe -m pip install "mujoco>=3.2"` and `~/miniforge3/envs/cad-to-urdf/python.exe -c "import mujoco; print(mujoco.__version__)"`
Expected: a version ≥ 3.2 printed.

- [ ] **Step 2: Write the failing tests** in `backend/tests/test_sim_mjcf.py`:

```python
import xml.etree.ElementTree as ET
import zipfile

import pytest

from app.sim.mjcf import build_mjcf, compile_mjcf
from app.sim.torque import required_efforts, suggest
from tests.ros_helpers import two_joint_zip


def _parts(tmp_path):
    z = zipfile.ZipFile(two_joint_zip(tmp_path))
    urdf = z.read("bot/urdf/bot.urdf").decode()
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if n.startswith("bot/meshes/")}
    return urdf, meshes


def test_mjcf_keeps_links_and_adds_one_motor_per_moving_joint(tmp_path):
    urdf, meshes = _parts(tmp_path)
    xml = build_mjcf(urdf, meshes, {"arm_joint": 50.0, "hand_joint": 20.0})
    root = ET.fromstring(xml)
    assert root.find("compiler").get("meshdir") == "../meshes"
    bodies = {b.get("name") for b in root.iter("body")}
    assert {"arm", "hand"} <= bodies
    acts = {a.get("joint"): a for a in root.find("actuator")}
    assert set(acts) == {"arm_joint", "hand_joint"}
    assert acts["arm_joint"].get("forcerange") == "-50 50" and float(acts["arm_joint"].get("kp")) == pytest.approx(1000)
    m = compile_mjcf(xml, meshes)
    assert m.nu == 2 and m.opt.timestep == pytest.approx(0.002)


def test_no_efforts_means_no_motors(tmp_path):
    urdf, meshes = _parts(tmp_path)
    assert compile_mjcf(build_mjcf(urdf, meshes, None), meshes).nu == 0


def test_required_torque_is_larger_nearer_the_base(tmp_path):
    urdf, meshes = _parts(tmp_path)
    req = required_efforts(compile_mjcf(build_mjcf(urdf, meshes, None), meshes))
    # arm_joint turns about z (vertical): gravity needs ~no torque there; hand_joint about y lifts the hand
    assert req["hand_joint"] > 0.1 and req["arm_joint"] < 0.01 * req["hand_joint"]


def test_links_touching_at_home_do_not_collide(tmp_path):
    import io
    import mujoco
    from app.export.urdf_writer import export_package
    from tests.ros_helpers import _body, _joint, _link
    links = [_link("base"), _link("arm"), _link("hand")]
    joints = [_joint("j1", "base", "arm", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0)),
              _joint("j2", "arm", "hand", (0.5, 0.0, 0.2), (0.0, 1.0, 0.0))]
    bodies = {"base": [_body("base", (0.0, 0.0, 0.0))], "arm": [_body("arm", (0.25, 0.0, 0.2))],
              "hand": [_body("hand", (0.05, 0.0, 0.0))]}  # hand overlaps base, which is not its parent
    z = zipfile.ZipFile(io.BytesIO(export_package("bot", links, joints, bodies)))
    urdf = z.read("bot/urdf/bot.urdf").decode()
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if n.startswith("bot/meshes/")}
    xml = build_mjcf(urdf, meshes, None)
    assert any({e.get("body1"), e.get("body2")} == {"base", "hand"} for e in ET.fromstring(xml).iter("exclude"))
    m = compile_mjcf(xml, meshes)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert d.ncon == 0


def test_suggest_doubles_and_rounds_up_to_two_significant_figures():
    assert suggest({"a": 70.1, "b": 0.2, "c": 0.0}) == {"a": 150.0, "b": 1.0, "c": 1.0}
```

- [ ] **Step 3: Run to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_sim_mjcf.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.sim'`.

- [ ] **Step 4: Implement `app/sim/mjcf.py`**

```python
"""URDF -> MJCF with MuJoCo's own URDF importer, plus position motors."""
import os
import re
import tempfile
import xml.etree.ElementTree as ET

import mujoco

TIMESTEP = 0.002
HOLD_ERROR = 0.05  # rad (or m) of error at which a motor gives full torque: kp = E / HOLD_ERROR


def _urdf_for_mujoco(urdf: str) -> str:
    """Bare mesh names (assets are passed by name), keep every link as a body, keep inertias as given."""
    urdf = re.sub(r'filename="package://[^"]*/meshes/([^"]+)"', r'filename="\1"', urdf)
    ext = ('<mujoco><compiler strippath="true" fusestatic="false" discardvisual="true" '
           'balanceinertia="false"/></mujoco>')
    return re.sub(r"(<robot[^>]*>)", r"\1" + ext, urdf, count=1)


def compile_mjcf(xml: str, meshes: dict[str, bytes]) -> mujoco.MjModel:
    xml = xml.replace('meshdir="../meshes"', 'meshdir=""')
    return mujoco.MjModel.from_xml_string(xml, meshes)


def build_mjcf(urdf: str, meshes: dict[str, bytes], efforts: dict[str, float] | None) -> str:
    model = mujoco.MjModel.from_xml_string(_urdf_for_mujoco(urdf), meshes)
    fd, path = tempfile.mkstemp(suffix=".xml")
    os.close(fd)
    try:
        mujoco.mj_saveLastXML(path, model)
        root = ET.parse(path).getroot()
    finally:
        os.unlink(path)
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", "../meshes")
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(1, option)
    option.set("timestep", str(TIMESTEP))
    if efforts:
        act = root.find("actuator")
        if act is None:
            act = ET.SubElement(root, "actuator")
        for j in root.iter("joint"):
            name = j.get("name")
            if name in efforts:
                e = float(efforts[name])
                ET.SubElement(act, "position", {
                    "name": f"{name}_motor", "joint": name, "kp": repr(e / HOLD_ERROR), "dampratio": "1",
                    "forcelimited": "true", "forcerange": f"{-e:g} {e:g}"})
    _exclude_touching_at_home(root, model)
    return ET.tostring(root, encoding="unicode")


def _exclude_touching_at_home(root, model: mujoco.MjModel) -> None:
    """Bodies whose collision shapes already touch at home pose never collide (MuJoCo already skips
    parent/child pairs). Spec section 3 named the fcl pairs; MuJoCo's own contacts at home are the same idea
    on the shapes MuJoCo actually collides (convex hulls), so no hold check fails on built-in overlap."""
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    pairs = set()
    for c in d.contact[:d.ncon]:
        a, b = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]) for g in (c.geom1, c.geom2))
        if a and b and a != b:
            pairs.add(tuple(sorted((a, b))))
    if pairs:
        contact = root.find("contact")
        if contact is None:
            contact = ET.SubElement(root, "contact")
        for a, b in sorted(pairs):
            ET.SubElement(contact, "exclude", {"body1": a, "body2": b})
```

- [ ] **Step 5: Implement `app/sim/torque.py`**

```python
"""Static gravity torque each joint needs, from MuJoCo's inverse dynamics."""
import math

import mujoco
import numpy as np


def required_efforts(model: mujoco.MjModel, samples: int = 500, seed: int = 0) -> dict[str, float]:
    """Joint name -> largest |torque| (N·m, or N for slides) needed to hold still against gravity
    over `samples` random poses within the joint limits (full circle for unlimited hinges)."""
    m = model
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT  # no contacts/limits: gravity only
    d = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    joints = [j for j in range(m.njnt) if m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)]
    worst = {j: 0.0 for j in joints}
    for _ in range(samples):
        for j in joints:
            lo, hi = m.jnt_range[j] if m.jnt_limited[j] else (-math.pi, math.pi)
            d.qpos[m.jnt_qposadr[j]] = rng.uniform(lo, hi)
        d.qvel[:] = 0
        d.qacc[:] = 0
        mujoco.mj_inverse(m, d)
        for j in joints:
            worst[j] = max(worst[j], abs(float(d.qfrc_inverse[m.jnt_dofadr[j]])))
    return {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j): v for j, v in worst.items()}


def suggest(required: dict[str, float]) -> dict[str, float]:
    """2x the need, rounded up to 2 significant figures, at least 1."""
    out = {}
    for name, need in required.items():
        x = 2 * need
        if x <= 1:
            out[name] = 1.0
            continue
        step = 10 ** (math.floor(math.log10(x)) - 1)
        out[name] = float(math.ceil(x / step - 1e-9) * step)
    return out
```

- [ ] **Step 6: Run the tests**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_sim_mjcf.py -q`
Expected: PASS. If MuJoCo renames bodies or merges links, check `fusestatic="false"` reached the compiler. A saved MJCF should list `base`, `arm`, `hand` bodies (the root link may be merged into `worldbody`; the tests only require `arm` and `hand`).

- [ ] **Step 7: Commit**

```bash
git add backend/pyproject.toml backend/app/sim backend/tests/test_sim_mjcf.py
git commit -m "feat: MuJoCo model from our URDF with position motors; required torques from inverse dynamics"
```

---

### Task 2: Motor strength in the model and the export

**Files:**
- Modify: `backend/app/models.py:88-102` (`Joint.max_effort`)
- Modify: `backend/app/export/urdf_writer.py` (`_add_joint_element`, `export_urdf`, `export_package`, new `suggest_efforts`)
- Test: `backend/tests/test_urdf_writer.py`, `backend/tests/test_sim_mjcf.py`

**Interfaces:**
- Consumes: `build_mjcf`, `compile_mjcf`, `required_efforts`, `suggest` (Task 1).
- Produces: `Joint.max_effort: float | None = None` (validated `> 0` when set).
- Produces: `export_urdf(robot_name, links, joints, bodies_by_link, efforts: dict[str, float] | None = None)`, where `efforts` is keyed by joint **id**. A missing entry writes `effort="100"`, as before, for callers that don't pass efforts.
- Produces: `suggest_efforts(robot_name, links, joints, bodies_by_link) -> dict[str, float]` (joint id → suggested value; moving joints only).
- Produces: `export_package(...)` zips add `<robot>/mjcf/<robot>.xml`. URDF efforts = `max_effort` or the suggestion.

- [ ] **Step 1: Write the failing tests.** Append to `backend/tests/test_sim_mjcf.py`:

```python
from app.export.urdf_writer import export_package, suggest_efforts
from tests.ros_helpers import _body, _joint, _link


def _arm(max_effort=None):
    links = [_link("base"), _link("arm"), _link("hand")]
    j1 = _joint("j1", "base", "arm", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0))
    j2 = _joint("j2", "arm", "hand", (0.5, 0.0, 0.2), (0.0, 1.0, 0.0))
    if max_effort is not None:
        j2 = j2.model_copy(update={"max_effort": max_effort})
    bodies = {"base": [_body("base", (0.0, 0.0, 0.0))], "arm": [_body("arm", (0.25, 0.0, 0.2))],
              "hand": [_body("hand", (0.7, 0.0, 0.2))]}
    return links, [j1, j2], bodies


def _effort(zip_bytes, joint_name):
    import io
    z = zipfile.ZipFile(io.BytesIO(zip_bytes))
    root = ET.fromstring(z.read("bot/urdf/bot.urdf"))
    return float(next(j for j in root.iter("joint") if j.get("name") == joint_name).find("limit").get("effort"))


def test_export_uses_suggested_efforts_and_ships_an_mjcf():
    links, joints, bodies = _arm()
    data = export_package("bot", links, joints, bodies)
    sug = suggest_efforts("bot", links, joints, bodies)
    assert set(sug) == {"j1", "j2"} and _effort(data, "hand_joint") == sug["j2"]
    import io
    z = zipfile.ZipFile(io.BytesIO(data))
    mjcf = z.read("bot/mjcf/bot.xml").decode()
    assert 'joint="hand_joint"' in mjcf and 'meshdir="../meshes"' in mjcf


def test_an_explicit_max_effort_wins():
    links, joints, bodies = _arm(max_effort=7.5)
    assert _effort(export_package("bot", links, joints, bodies), "hand_joint") == 7.5


def test_an_old_project_joint_without_max_effort_still_exports():
    from app.models import Joint
    raw = _arm()[1][1].model_dump()
    raw.pop("max_effort", None)
    assert Joint.model_validate(raw).max_effort is None


def test_max_effort_must_be_positive():
    from pydantic import ValidationError
    from app.models import Joint
    with pytest.raises(ValidationError):
        Joint.model_validate(_arm()[1][1].model_dump() | {"max_effort": -1})
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_sim_mjcf.py -q`
Expected: FAIL (`cannot import name 'suggest_efforts'`).

- [ ] **Step 3: `Joint.max_effort`** in `models.py`, after `ambiguous`:

```python
    max_effort: float | None = Field(default=None, gt=0)  # N·m (N for prismatic); None = auto
```

(Import `Field` from pydantic if `models.py` doesn't already.)

- [ ] **Step 4: Efforts in `urdf_writer.py`.**
  - `_add_joint_element(robot, joint, name, links_by_id, frames, effort: float = 100.0)` writes `"effort": f"{effort:g}"`.
  - `export_urdf(..., efforts: dict[str, float] | None = None)` passes `efforts.get(joint.id, 100.0) if efforts else 100.0`.
  - Then add:

```python
from app.sim.mjcf import build_mjcf, compile_mjcf
from app.sim.torque import required_efforts, suggest


def _prepared(robot_name, links, joints, bodies_by_link):
    """Validated links (with mass properties), used bodies and per-link STL bytes."""
    bodies_by_link = _used_bodies(links, bodies_by_link)
    errors = _part_status_errors(links, bodies_by_link)
    if errors:
        raise ValueError("Cannot export URDF -- " + "; ".join(errors))
    links = [_with_mass_properties(l, bodies_by_link.get(l.id, [])) for l in links]
    meshes = {}
    for link in links:
        parts = [trimesh.Trimesh(vertices=b.vertices, faces=b.triangles, process=False)
                 for b in bodies_by_link.get(link.id, []) if b.vertices]
        if parts:
            meshes[f"{link.name}.stl"] = trimesh.util.concatenate(parts).export(file_type="stl")
    return links, bodies_by_link, meshes


def _suggested(robot_name, links, joints, bodies_by_link, meshes) -> dict[str, float]:
    urdf = export_urdf(robot_name, links, joints, bodies_by_link)
    by_name = suggest(required_efforts(compile_mjcf(build_mjcf(urdf, meshes, None), meshes)))
    names = _joint_names(joints, {l.id: l for l in links})
    return {j.id: by_name[names[j.id]] for j in joints if names[j.id] in by_name}


def suggest_efforts(robot_name, links, joints, bodies_by_link) -> dict[str, float]:
    links, bodies_by_link, meshes = _prepared(robot_name, links, joints, bodies_by_link)
    return _suggested(robot_name, links, joints, bodies_by_link, meshes)
```

Rewrite `export_package` to use them:

```python
def export_package(robot_name, links, joints, bodies_by_link) -> bytes:
    links, bodies_by_link, meshes = _prepared(robot_name, links, joints, bodies_by_link)
    sug = _suggested(robot_name, links, joints, bodies_by_link, meshes)
    efforts = {j.id: j.max_effort or sug[j.id] for j in joints if j.id in sug}
    urdf = export_urdf(robot_name, links, joints, bodies_by_link, efforts)
    names = _joint_names(joints, {l.id: l for l in links})
    mjcf = build_mjcf(urdf, meshes, {names[jid]: e for jid, e in efforts.items()})
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr(f"{robot_name}/urdf/{robot_name}.urdf", urdf)
        z.writestr(f"{robot_name}/mjcf/{robot_name}.xml", mjcf)
        z.writestr(f"{robot_name}/package.xml", package_xml(robot_name))
        z.writestr(f"{robot_name}/CMakeLists.txt", cmakelists(robot_name))
        for name, stl in meshes.items():
            z.writestr(f"{robot_name}/meshes/{name}", stl)
    return buf.getvalue()
```

`export_urdf` runs structural validation first. If the structure is invalid, `_suggested` raises the same `ValueError` the export raised before, so the error text is unchanged.

- [ ] **Step 5: Run the sim tests, then every export/ROS test** (the zip layout and URDF changed):

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_sim_mjcf.py tests/test_urdf_writer.py tests/test_end_to_end.py tests/test_ros_cli.py tests/test_ros_moveit_config.py tests/test_ros_kinematics_pkg.py tests/test_ros_program_pkg.py tests/test_program_session.py -q`
Expected: PASS. Fix any test that pinned the zip file list or `effort="100"` to the new behaviour, and ledger each change.

- [ ] **Step 6: Commit**

```bash
git add backend/app/models.py backend/app/export/urdf_writer.py backend/tests
git commit -m "feat: per-joint motor strength (auto-suggested); exports carry a MuJoCo model"
```

---

### Task 3: Physics check and the two endpoints

**Files:**
- Create: `backend/app/sim/check.py`, `backend/app/sim/child.py`, `backend/app/routes/sim_routes.py`
- Modify: `backend/app/main.py` (include the router)
- Test: `backend/tests/test_sim_check.py`

**Interfaces:**
- Consumes: Task 1 + 2 functions; `ExportRequest` from `app.routes.export_routes`.
- Produces: `hold_check(xml: str, meshes: dict[str, bytes], seconds: float = 2.0) -> dict` (result shape in Global Constraints).
- Produces: `physics_check(robot_name, links, joints, bodies_by_link) -> dict` (export, then `hold_check` on the zip's MJCF).
- Produces: `run_in_child(fn, *args, timeout: float = 60.0)` → `fn`'s return value; raises `ChildFailed` on crash or timeout.
- Produces: `POST /suggest-efforts` → `{joint_id: number}` (422 with the export error text on invalid models); `POST /simulate/check` → the result dict (always 200; problems are results).

- [ ] **Step 1: Write the failing tests** in `backend/tests/test_sim_check.py`:

```python
import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.sim import check, child
from tests.ros_helpers import _body, _joint, _link

client = TestClient(app)


def _req(max_effort=None, hand_mass_density=1000.0):
    links = [_link("base"), _link("arm"), _link("hand").model_copy(update={"density_kg_m3": hand_mass_density})]
    j2 = _joint("j2", "arm", "hand", (0.5, 0.0, 0.2), (0.0, 1.0, 0.0))
    if max_effort is not None:
        j2 = j2.model_copy(update={"max_effort": max_effort})
    joints = [_joint("j1", "base", "arm", (0.0, 0.0, 0.2), (0.0, 0.0, 1.0)), j2]
    bodies = {"base": [_body("base", (0.0, 0.0, 0.0))], "arm": [_body("arm", (0.25, 0.0, 0.2))],
              "hand": [_body("hand", (0.7, 0.0, 0.2))]}
    return links, joints, bodies


def test_suggested_motors_hold_pose():
    out = check.physics_check("bot", *_req())
    assert out == {"ok": True, "kind": "ok", "message": "Holds pose."}


def test_a_weak_motor_sags_and_names_its_link():
    out = check.physics_check("bot", *_req(max_effort=0.01))
    assert out["ok"] is False and out["kind"] == "sag" and out["link"] == "hand" and out["joint"] == "hand_joint"
    assert out["has"] == 0.01 and out["needed"] > 0.01 and "hand sags" in out["message"]


def test_a_single_link_robot_holds_pose():
    out = check.physics_check("bot", [_link("base")], [], {"base": [_body("base", (0, 0, 0))]})
    assert out["ok"] is True


def test_a_zero_mass_link_is_reported_not_raised():
    out = check.physics_check("bot", *_req(hand_mass_density=0.0))
    assert out["ok"] is False and out["kind"] in ("load", "unstable") and "hand" in out["message"]


def test_check_route_runs_in_a_child_and_returns_the_result():
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


def test_a_crashed_child_is_a_crash_result():
    with pytest.raises(child.ChildFailed):
        child.run_in_child(_boom, timeout=30)
```

Note: if `density_kg_m3=0.0` is rejected by the `Link` model (validation), use `mass_kg=0.0` on the link instead, keep the same assertion, and ledger the switch.

- [ ] **Step 2: Run to verify they fail**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_sim_check.py -q`
Expected: FAIL (`cannot import name 'check'`).

- [ ] **Step 3: `app/sim/child.py`**

```python
"""Run one function in a fresh spawned process: MuJoCo's compile and OCCT hold the GIL."""
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool

_spawn = multiprocessing.get_context("spawn")


class ChildFailed(Exception):
    pass


def run_in_child(fn, *args, timeout: float = 60.0):
    pool = ProcessPoolExecutor(max_workers=1, mp_context=_spawn)
    try:
        return pool.submit(fn, *args).result(timeout=timeout)
    except BrokenProcessPool as exc:
        raise ChildFailed("crashed") from exc
    except TimeoutError as exc:
        raise ChildFailed("timed out") from exc
    finally:
        for p in list(getattr(pool, "_processes", {}).values()):  # a timed-out child must not linger
            if p.is_alive():
                p.kill()
        pool.shutdown(wait=False, cancel_futures=True)
```

- [ ] **Step 4: `app/sim/check.py`**

```python
"""The physics check: hold home pose with the motors for a couple of seconds."""
import io
import math
import zipfile

import mujoco
import numpy as np

from app.sim.mjcf import compile_mjcf
from app.sim.torque import required_efforts

SAG_HINGE = math.radians(5)
SAG_SLIDE = 0.005


def _name(m, kind, i) -> str:
    return mujoco.mj_id2name(m, kind, i) or f"#{i}"


def hold_check(xml: str, meshes: dict[str, bytes], seconds: float = 2.0) -> dict:
    try:
        m = compile_mjcf(xml, meshes)
    except ValueError as exc:  # MuJoCo reports compile errors as ValueError
        return {"ok": False, "kind": "load", "message": f"Model won't load: {exc}"}
    d = mujoco.MjData(m)
    home = np.zeros(m.nq)
    for j in range(m.njnt):  # home = 0, clamped into the limits
        if m.jnt_limited[j]:
            a = m.jnt_qposadr[j]
            home[a] = min(max(0.0, m.jnt_range[j][0]), m.jnt_range[j][1])
    d.qpos[:] = home
    for a in range(m.nu):
        d.ctrl[a] = home[m.jnt_qposadr[m.actuator_trnid[a][0]]]
    for step in range(int(round(seconds / m.opt.timestep))):
        mujoco.mj_step(m, d)
        if not (np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))) or d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number:
            speeds = [np.linalg.norm(d.cvel[b]) if np.all(np.isfinite(d.cvel[b])) else np.inf for b in range(1, m.nbody)]
            link = _name(m, mujoco.mjtObj.mjOBJ_BODY, 1 + int(np.argmax(speeds))) if speeds else "the robot"
            t = round((step + 1) * m.opt.timestep, 2)
            return {"ok": False, "kind": "unstable", "link": link,
                    "message": f"Simulation blew up after {t} s: check {link}'s mass and inertia."}
    worst = None
    for j in range(m.njnt):
        t = m.jnt_type[j]
        if t not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        a = m.jnt_qposadr[j]
        dev = abs(d.qpos[a] - home[a])
        limit = SAG_HINGE if t == mujoco.mjtJoint.mjJNT_HINGE else SAG_SLIDE
        if dev > limit and (worst is None or dev / limit > worst[1] / worst[2]):
            worst = (j, dev, limit)
    if worst is None:
        return {"ok": True, "kind": "ok", "message": "Holds pose."}
    j, dev, _ = worst
    joint = _name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    link = _name(m, mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[j])
    hinge = m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
    has = next((float(m.actuator_forcerange[a][1]) for a in range(m.nu) if m.actuator_trnid[a][0] == j), 0.0)
    needed = required_efforts(compile_mjcf(xml, meshes)).get(joint, 0.0)
    unit = "N·m" if hinge else "N"
    amount = f"{math.degrees(dev):.0f}°" if hinge else f"{dev * 1000:.0f} mm"
    return {"ok": False, "kind": "sag", "joint": joint, "link": link, "needed": round(needed, 3), "has": has,
            "message": f"{link} sags {amount}: needs ~{needed:.3g} {unit}, has {has:g}."}


def physics_check(robot_name, links, joints, bodies_by_link) -> dict:
    """Runs in a child process: export (which builds the MJCF) then hold."""
    from app.export.urdf_writer import export_package
    try:
        data = export_package(robot_name, links, joints, bodies_by_link)
    except ValueError as exc:
        return {"ok": False, "kind": "load", "message": str(exc)}
    z = zipfile.ZipFile(io.BytesIO(data))
    meshes = {n.split("/")[-1]: z.read(n) for n in z.namelist() if "/meshes/" in n}
    return hold_check(z.read(f"{robot_name}/mjcf/{robot_name}.xml").decode(), meshes)
```

A zero-mass link may fail in `export_package` itself, or in MuJoCo's compile (as `load`), or as `unstable`. All three are results, never a 500. If `export_package` raises for zero mass, its text names the link, which satisfies the test.

- [ ] **Step 5: `app/routes/sim_routes.py`, plus `app.include_router(sim_routes.router)` in `main.py`** (next to the other routers):

```python
from fastapi import APIRouter, HTTPException

from app.routes.export_routes import ExportRequest
from app.sim import check
from app.sim.child import ChildFailed, run_in_child

router = APIRouter()


@router.post("/suggest-efforts")
def suggest_efforts(req: ExportRequest) -> dict:
    from app.export.urdf_writer import suggest_efforts as suggest
    try:
        return run_in_child(suggest, req.robot_name, req.links, req.joints, req.bodies_by_link)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    except ChildFailed:
        raise HTTPException(503, "Could not compute motor strengths.")


@router.post("/simulate/check")
def simulate_check(req: ExportRequest) -> dict:
    try:
        return run_in_child(check.physics_check, req.robot_name, req.links, req.joints, req.bodies_by_link)
    except ChildFailed:
        return {"ok": False, "kind": "crash", "message": "The physics check crashed."}
```

Check `export_routes.ExportRequest`'s field names (`robot_name`, `links`, `joints`, `bodies_by_link`) before relying on them; the existing `/program/session` route uses the same model.

- [ ] **Step 6: Run the tests**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_sim_check.py tests/test_sim_mjcf.py -q`
Expected: PASS. `test_a_weak_motor_sags…` depends on the 0.01 N·m motor being far below the hand's need (~0.4 N·m for a 1 kg cube at 0.2 m). If the sag lands on `arm` instead, look at the joint axis, not the threshold.

- [ ] **Step 7: Commit**

```bash
git add backend/app/sim backend/app/routes/sim_routes.py backend/app/main.py backend/tests/test_sim_check.py
git commit -m "feat: physics check (hold home pose 2 s) and suggest-efforts endpoints, run in a child process

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: Menu bar, workbench switch and View toggles

**Files:**
- Create: `frontend/src/ui/MenuBar.tsx`
- Modify: `frontend/src/ui/Stepper.tsx` (drop brand, Import/Open/Save/Undo/Redo and the import strip; drop the `program` tab; label `export` "Review")
- Modify: `frontend/src/viewer/Viewer.ts` (`setGrid`, `setEdges`)
- Modify: `frontend/src/App.tsx` (render `MenuBar` above `Stepper` and above `ProgramMode`)
- Modify: `frontend/src/ui/program/ProgramMode.tsx` (remove the Back to robot button; render `<MenuBar ctx={ctx} lock={busyWhy} />` above its header)
- Modify: `frontend/src/ui/theme.css`
- Test: `frontend/src/ui/menubar.test.tsx` (new), update `frontend/src/ui/shell.test.tsx` and `program/programMode.test.tsx`

**Interfaces:**
- Produces: `simulateBlocked(model, robotName): string | null` in `frontend/src/model/checklist.ts`. It returns `"Fix the Review checklist first."` when any checklist item is `!ok && !warning`, else null.
- Produces: `<MenuBar ctx={Ctx} lock?: string | null />`.
  - `lock` disables the workbench switch with that tooltip; Simulate passes its "task is running" text.
  - In Simulate, New, Open and Import CAD are disabled with the title "Switch to Build to change the robot."
- Produces: `Viewer.setGrid(on: boolean)`, `Viewer.setEdges(on: boolean)`. Both remember the flag, so bodies set later follow it.
- Produces: `VIEW_PREFS` in localStorage key `cad-to-urdf.view` = `{"grid": bool, "edges": bool}`.

- [ ] **Step 1: Write the failing tests** in `frontend/src/ui/menubar.test.tsx`:

```tsx
import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../api/client";
import { boxBody } from "../model/testBodies";
import MenuBar from "./MenuBar";
import { Harness, imported, state } from "./testing";

afterEach(() => { vi.restoreAllMocks(); localStorage.clear(); });
const bar = (start = imported([])) => render(<Harness start={start} ui={(ctx) => <MenuBar ctx={ctx} />} />);
const open = (menu: string) => fireEvent.click(screen.getByRole("button", { name: menu }));

describe("MenuBar", () => {
  it("File ▸ Import CAD imports with progress, like the old button", async () => {
    const spy = vi.spyOn(client, "importFile").mockResolvedValue({ bodies: [boxBody("x", [0, 0, 0], [1, 1, 1])], applied_scale: 0.001, warnings: [] });
    bar();
    open("File");
    const file = new File(["x"], "a.stl");
    await act(async () => { fireEvent.change(screen.getByTestId("import-input"), { target: { files: [file] } }); });
    expect(spy).toHaveBeenCalledWith(file, 0.001, expect.any(Function));
    expect(state().links).toHaveLength(1);
  });

  it("Edit ▸ Undo undoes", async () => {
    bar(imported([boxBody("x", [0, 0, 0], [1, 1, 1])]));
    open("Edit");
    expect(screen.getByRole("menuitem", { name: /undo/i })).toHaveProperty("disabled", true);
  });

  it("View toggles the grid and remembers it", () => {
    bar();
    open("View");
    fireEvent.click(screen.getByRole("menuitemcheckbox", { name: /grid/i }));
    expect(JSON.parse(localStorage.getItem("cad-to-urdf.view")!)).toMatchObject({ grid: false });
  });

  it("Simulate is disabled until the Review checklist passes, and says why", () => {
    bar();
    const sim = screen.getByRole("button", { name: "Simulate" });
    expect(sim).toHaveProperty("disabled", true);
    expect(sim.getAttribute("title")).toBe("Fix the Review checklist first.");
  });

  it("Escape closes an open menu", () => {
    bar();
    open("File");
    fireEvent.keyDown(document, { key: "Escape" });
    expect(screen.queryByRole("menu")).toBeNull();
  });
});
```

In `shell.test.tsx`, the Stepper tests that used `import-input`, `save-project`, `undo`/`redo` move to the MenuBar tests above: delete those cases from the Stepper `describe`. Add one assertion that the Stepper shows a `step-export` tab labelled "Review" and no `step-program` tab.

- [ ] **Step 2: Run to verify they fail**

Run: `cd frontend && npx vitest run src/ui/menubar.test.tsx`
Expected: FAIL (`Cannot find module './MenuBar'`).

- [ ] **Step 3: `Viewer.setGrid` / `setEdges`.** Fields `private showGrid = true; private showEdges = true;`. In the code that creates the grid (line ~165), set `this.grid.visible = this.showGrid`. Where edges `LineSegments` are created (line ~131), set `.visible = this.showEdges` and keep them in an `edges: LineSegments[]` list (cleared where bodies are cleared). Then:

```ts
  setGrid(on: boolean) { this.showGrid = on; if (this.grid) this.grid.visible = on; this.render(); }
  setEdges(on: boolean) { this.showEdges = on; for (const e of this.edges) e.visible = on; this.render(); }
```

(Use whatever the class's existing redraw call is in place of `this.render()`; check how `setPreview` triggers a redraw.)

- [ ] **Step 4: `simulateBlocked`** in `model/checklist.ts`:

```ts
export function simulateBlocked(model: ModelState, robotName: string): string | null {
  return exportChecklist(model, robotName).some((i) => !i.ok && !i.warning) ? "Fix the Review checklist first." : null;
}
```

- [ ] **Step 5: `MenuBar.tsx`.** Move `onImport`, `onOpen`, the import strip state (`busy`, `imp`, `elapsed`, `IMPORT_LABEL`) and the hidden file inputs out of `Stepper.tsx` unchanged.

```tsx
import { useEffect, useRef, useState } from "react";
import { exportRobot, importFile, type ImportProgress } from "../api/client";
import { simulateBlocked } from "../model/checklist";
import { buildExportRequest } from "../model/exportRequest";
import { emptyModel } from "../model/types";
import { loadProject, saveProject } from "../model/project";
import type { Ctx } from "./ctx";
import { downloadBlob } from "./download";
import { fmtElapsed, useElapsed } from "./useElapsed";

const IMPORT_LABEL = { upload: "Uploading", read: "Reading file", parts: "Building parts", faces: "Finding faces" } as const;
const VIEW_KEY = "cad-to-urdf.view";
const IN_SIM = "Switch to Build to change the robot.";

function loadView(): { grid: boolean; edges: boolean } {
  try { return { grid: true, edges: true, ...JSON.parse(localStorage.getItem(VIEW_KEY) ?? "{}") }; }
  catch { return { grid: true, edges: true }; }
}

export default function MenuBar({ ctx, lock = null }: { ctx: Ctx; lock?: string | null }) {
  const { model, dispatch, step, goTo, notify, scale, viewer, robotName } = ctx;
  const sim = step === "program";
  const [menu, setMenu] = useState<"File" | "Edit" | "View" | null>(null);
  const [busy, setBusy] = useState(false);
  const [imp, setImp] = useState<ImportProgress | null>(null);
  const elapsed = useElapsed(busy);
  const [view, setView] = useState(loadView);
  const importRef = useRef<HTMLInputElement>(null);
  const openRef = useRef<HTMLInputElement>(null);
  const blocked = simulateBlocked(model, robotName);

  useEffect(() => { viewer?.setGrid(view.grid); viewer?.setEdges(view.edges); }, [viewer, view]);
  useEffect(() => {
    if (!menu) return;
    const close = (e: Event) => { if (e instanceof KeyboardEvent ? e.key === "Escape" : !(e.target as Element).closest?.(".menubar")) setMenu(null); };
    document.addEventListener("keydown", close);
    document.addEventListener("mousedown", close);
    return () => { document.removeEventListener("keydown", close); document.removeEventListener("mousedown", close); };
  }, [menu]);

  const toggle = (k: "grid" | "edges") => {
    const next = { ...view, [k]: !view[k] };
    setView(next);
    try { localStorage.setItem(VIEW_KEY, JSON.stringify(next)); } catch { /* private window: not remembered */ }
  };
  const onImport = async (file: File) => { /* moved verbatim from Stepper: setBusy, importFile(file, scale, setImp), dispatch, goTo("parts"), notify on error, finally setBusy(false); setImp(null) */ };
  const onOpen = async (file: File) => { /* moved verbatim from Stepper */ };
  const onExport = async () => {
    const req = buildExportRequest(model);
    try {
      const { blob } = await exportRobot(robotName, req.links, req.joints, req.bodiesByLink);
      downloadBlob(blob, `${robotName}.zip`);
    } catch (e) { notify(`Export failed: ${(e as Error).message}`); }
  };
  const onNew = () => {
    if (model.links.length && !window.confirm("Start a new project? Unsaved changes are lost.")) return;
    dispatch({ type: "load", model: emptyModel() });
    goTo("parts");
  };
  const item = (label: string, run: () => void, disabled = false, title?: string) => (
    <button role="menuitem" className="menu-item" disabled={disabled} title={title} onClick={() => { setMenu(null); run(); }}>{label}</button>
  );
  const head = (name: "File" | "Edit" | "View") => (
    <button className={`menu-head${menu === name ? " open" : ""}`} aria-haspopup="menu" aria-expanded={menu === name}
      onClick={() => setMenu(menu === name ? null : name)}>{name}</button>
  );

  return (
    <header className="menubar">
      <span className="brand">CAD → URDF</span>
      <span className="menu">{head("File")}
        {menu === "File" && <div role="menu" className="menu-pop">
          {item("New project", onNew, sim, sim ? IN_SIM : undefined)}
          {item("Open project…", () => openRef.current?.click(), sim, sim ? IN_SIM : undefined)}
          {item("Save project", () => downloadBlob(new Blob([saveProject(model)], { type: "application/json" }), "project.json"))}
          {item("Import CAD…", () => importRef.current?.click(), sim || busy, sim ? IN_SIM : undefined)}
          {item("Export URDF + MuJoCo (.zip)", () => void onExport(), !!blocked, blocked ?? undefined)}
        </div>}
      </span>
      <span className="menu">{head("Edit")}
        {menu === "Edit" && <div role="menu" className="menu-pop">
          {item("Undo  Ctrl+Z", () => dispatch({ type: "undo" }), !ctx.canUndo)}
          {item("Redo  Ctrl+Y", () => dispatch({ type: "redo" }), !ctx.canRedo)}
        </div>}
      </span>
      <span className="menu">{head("View")}
        {menu === "View" && <div role="menu" className="menu-pop">
          <button role="menuitemcheckbox" aria-checked={view.grid} className="menu-item" onClick={() => toggle("grid")}>{view.grid ? "✓ " : ""}Grid</button>
          <button role="menuitemcheckbox" aria-checked={view.edges} className="menu-item" onClick={() => toggle("edges")}>{view.edges ? "✓ " : ""}Part edges</button>
          {item("Fit to view", () => viewer?.setView("home"))}
        </div>}
      </span>
      <input ref={importRef} data-testid="import-input" type="file" hidden accept=".step,.stp,.iges,.igs,.brep,.stl,.obj"
        onChange={(e) => { const f = e.target.files?.[0]; e.target.value = ""; if (f) void onImport(f); }} />
      <input ref={openRef} data-testid="open-input" type="file" hidden accept=".json"
        onChange={(e) => { const f = e.target.files?.[0]; e.target.value = ""; if (f) void onOpen(f); }} />
      <span className="workbench" role="group" aria-label="Workbench">
        <button className={`wb${!sim ? " current" : ""}`} disabled={!!lock} title={lock ?? undefined} onClick={() => goTo("export")}>Build</button>
        <button className={`wb${sim ? " current" : ""}`} disabled={!!blocked || !!lock} title={blocked ?? lock ?? undefined}
          onClick={() => goTo("program")}>Simulate</button>
      </span>
      <span className="spacer" />
      {busy && imp && (
        <span className="progress import-progress" data-testid="import-progress">
          {IMPORT_LABEL[imp.stage]}{imp.total > 0 ? ` ${imp.done} / ${imp.total}` : "…"}
          {imp.total > 0 && <progress value={imp.done} max={imp.total} />}
          <span className="muted">{fmtElapsed(elapsed)}</span>
        </span>
      )}
    </header>
  );
}
```

Replace the two `/* moved verbatim */` bodies with the exact bodies of `onImport` and `onOpen` from today's `Stepper.tsx`. They are moved, not rewritten. (Check `emptyModel`'s exact export name in `model/types.ts` and whether a `"load"` action with an empty model is how "new" should reset; reuse it if the Harness `imported([])` path shows an existing reset.)

Check `exportRobot`'s return shape in `api/client.ts` (`{ blob, warnings }` per ExportStep) before using it.

The Edit menu's Ctrl+Z / Ctrl+Y keyboard shortcuts already exist (`shortcuts.ts`); leave them.

- [ ] **Step 6: Wire it.**
  - In `App.tsx`, render `<MenuBar ctx={ctx} />` above `<Stepper …/>` inside `.app`.
  - In the `step === "program"` branch, ProgramMode renders its own `<MenuBar ctx={ctx} lock={busyWhy} />` as its first child, and its "Back to robot" button is deleted. Change existing tests that clicked "Back to robot" to click the "Build" workbench button, and that asserted its disabled title to assert the Build button's.
  - In `Stepper.tsx`, delete the moved code, the `program` entry from `STEPS`, and rename `{ id: "export", label: "Export" }` to `label: "Review"`.

CSS (`theme.css`):

```css
.menubar { display: flex; align-items: center; gap: 4px; padding: 4px 10px; background: var(--chrome); border-bottom: 1px solid var(--border); }
.menu { position: relative; }
.menu-head { background: none; border: 0; padding: 4px 8px; border-radius: var(--radius); color: var(--text); font: inherit; cursor: pointer; }
.menu-head:hover, .menu-head.open { background: var(--panel); }
.menu-pop { position: absolute; top: 100%; left: 0; z-index: 60; min-width: 220px; padding: 4px; background: var(--panel); border: 1px solid var(--border); border-radius: var(--radius); box-shadow: var(--shadow); display: grid; }
.menu-item { text-align: left; background: none; border: 0; padding: 6px 10px; border-radius: 4px; color: var(--text); font: inherit; cursor: pointer; }
.menu-item:hover:not(:disabled) { background: var(--panel-2); }
.menu-item:disabled { color: var(--muted); cursor: default; }
.workbench { display: inline-flex; margin-left: 12px; border: 1px solid var(--border); border-radius: var(--radius); overflow: hidden; }
.wb { background: var(--panel); border: 0; padding: 4px 12px; font: inherit; color: var(--text); cursor: pointer; }
.wb.current { background: var(--accent, #2563eb); color: #fff; }
.wb:disabled { color: var(--muted); cursor: default; }
```

(Check `--accent` exists in `theme.css`; use the variable the primary button uses if it's named differently.)

- [ ] **Step 7: Run all frontend tests + type check**

Run: `cd frontend && npx vitest run && npx tsc --noEmit`
Expected: PASS. Fix moved-button tests per Step 1/6 and ledger each.

- [ ] **Step 8: Commit**

```bash
git add frontend/src
git commit -m "feat: File/Edit/View menu bar and Build | Simulate workbench switch

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: Max torque field

**Files:**
- Modify: `frontend/src/api/types.ts` (`Joint.max_effort?: number | null`)
- Modify: `frontend/src/model/jointDraft.ts` (`maxEffort: number | null` in `JointDraft`, `emptyDraft`, `draftFromJoint`, `draftToJoint`, `draftPreviewJoint`)
- Create: `frontend/src/ui/useEffortSuggestions.ts`
- Modify: `frontend/src/api/client.ts` (`suggestEfforts`)
- Modify: `frontend/src/ui/JointTool.tsx` (the field, below Limits)
- Test: `frontend/src/model/jointDraft.test.ts` (or the existing draft test file), `frontend/src/ui/steps/connect.test.tsx`

**Interfaces:**
- Consumes: `POST /suggest-efforts` (Task 3).
- Produces: `suggestEfforts(robotName, links, joints, bodiesByLink, signal?): Promise<Record<string, number>>`.
- Produces: `useEffortSuggestions(ctx): Record<string, number>`. Debounced 500 ms on `model.joints`/`model.links`; aborts stale requests; `{}` on error.
- Produces: `parseEffort(text: string): number | null`. Finite and > 0, else null.

- [ ] **Step 1: Write the failing tests**

```ts
// in the jointDraft test file
import { draftFromJoint, draftToJoint, parseEffort } from "./jointDraft";
it("carries max_effort through the draft; missing means auto", () => {
  const j = { id: "j", parent_link_id: "a", child_link_id: "b", type: "revolute" as const, origin_xyz: [0, 0, 0] as [number, number, number],
    origin_rpy: [0, 0, 0] as [number, number, number], axis: [0, 0, 1] as [number, number, number], limit_lower: -1, limit_upper: 1, source: "manual" as const };
  expect(draftFromJoint(j).maxEffort).toBeNull();
  expect(draftToJoint({ ...draftFromJoint(j), maxEffort: 12 }).max_effort).toBe(12);
  expect(draftToJoint(draftFromJoint(j)).max_effort).toBeNull();
});
it("parseEffort keeps only finite positive numbers", () => {
  expect(parseEffort("140")).toBe(140);
  for (const bad of ["", "abc", "-5", "0", "NaN", "Infinity"]) expect(parseEffort(bad)).toBeNull();
});
```

In `connect.test.tsx`, add a test, following the file's existing pattern for opening the JointTool on a joint:
- mock `client.suggestEfforts` to resolve `{ [jointId]: 140 }`;
- open the joint and expect `screen.getByTestId("max-effort")` to have placeholder `auto · 140 N·m`;
- type `"50"` and expect the draft to carry `maxEffort: 50`, via whatever the file uses to inspect the draft (e.g. `state()`).

- [ ] **Step 2: Run to verify they fail**

Run: `cd frontend && npx vitest run src/model src/ui/steps/connect.test.tsx`
Expected: FAIL (`maxEffort` undefined, `parseEffort` not exported, no `max-effort` field).

- [ ] **Step 3: Implement**
  - Add `max_effort?: number | null;` to `Joint` in `types.ts`.
  - In `jointDraft.ts`:
    - add `maxEffort: number | null;` to `JointDraft`;
    - set `maxEffort: null` in `emptyDraft`;
    - set `maxEffort: j.max_effort ?? null` in `draftFromJoint`;
    - set `max_effort: d.maxEffort` in `draftToJoint` and `draftPreviewJoint`;
    - add the parser:

```ts
export function parseEffort(text: string): number | null {
  const v = Number(text.trim());
  return text.trim() !== "" && Number.isFinite(v) && v > 0 ? v : null;
}
```

`client.ts`:

```ts
export async function suggestEfforts(robotName: string, links: Link[], joints: Joint[],
  bodiesByLink: Record<string, Body[]>, signal?: AbortSignal): Promise<Record<string, number>> {
  return (await send("/suggest-efforts", { ...json({ robot_name: robotName, links, joints, bodies_by_link: bodiesByLink }), signal })).json();
}
```

`useEffortSuggestions.ts`:

```ts
import { useEffect, useState } from "react";
import { suggestEfforts } from "../api/client";
import { buildExportRequest } from "../model/exportRequest";
import type { Ctx } from "./ctx";

/** Suggested motor strength per joint id, refreshed 500 ms after the joints or links change. */
export function useEffortSuggestions(ctx: Pick<Ctx, "model" | "robotName">): Record<string, number> {
  const { model, robotName } = ctx;
  const [s, setS] = useState<Record<string, number>>({});
  useEffect(() => {
    if (!model.joints.length) { setS({}); return; }
    const ctl = new AbortController();
    const t = setTimeout(() => {
      const req = buildExportRequest(model);
      suggestEfforts(robotName, req.links, req.joints, req.bodiesByLink, ctl.signal).then(setS).catch(() => {});
    }, 500);
    return () => { clearTimeout(t); ctl.abort(); };
  }, [model.joints, model.links, robotName]);  // eslint-disable-line react-hooks/exhaustive-deps
  return s;
}
```

In `JointTool.tsx`, below the Limits block. It applies to revolute (limited or not) and slider motions; fixed joints have no motor:

```tsx
const suggestions = useEffortSuggestions(ctx);
// ...
{(d.motion === "revolute" || d.motion === "slider") && (
  <label className="row">
    {slider ? "Max force" : "Max torque"}
    <input className="field" data-testid="max-effort" type="number" step="any" min="0" style={{ width: 110 }}
      placeholder={suggestions[d.id] ? `auto · ${suggestions[d.id]} ${slider ? "N" : "N·m"}` : "auto"}
      value={d.maxEffort ?? ""} onChange={(e) => set({ maxEffort: parseEffort(e.target.value) })} />
  </label>
)}
```

(Use the motion names `jointDraft.ts` actually defines. `slider` is already a local in `JointTool` per line ~148.)

- [ ] **Step 4: Run tests + type check**

Run: `cd frontend && npx vitest run && npx tsc --noEmit`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add frontend/src
git commit -m "feat: Max torque per joint, pre-filled with the backend's suggestion

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: Review step goes to Simulate

**Files:**
- Modify: `frontend/src/ui/steps/ExportStep.tsx:50-115`
- Test: `frontend/src/ui/steps/export.test.tsx`

**Interfaces:**
- Consumes: `exportRobot` (server-side validation, unchanged); `goTo("program")`.

- [ ] **Step 1: Write the failing test** (add to `export.test.tsx`, reusing its existing fixture for a valid model):

```tsx
it("Continue to Simulate validates on the server and opens Simulate without downloading", async () => {
  const exp = vi.spyOn(client, "exportRobot").mockResolvedValue({ blob: new Blob(["z"]), warnings: [] });
  const dl = vi.spyOn(download, "downloadBlob");
  // render a valid model on the export step as the other tests in this file do
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /continue to simulate/i })); });
  expect(exp).toHaveBeenCalled();
  expect(dl).not.toHaveBeenCalled();
  expect(state().step).toBe("program");
  expect(screen.queryByText(/export urdf \(\.zip\)/i)).toBeNull();
});
```

(Import `* as download from "../download"` in the test.) Existing tests that click `data-testid="export"` keep the test id; update their expectations from "downloads" to "goes to Simulate".

- [ ] **Step 2: Run to verify it fails**

Run: `cd frontend && npx vitest run src/ui/steps/export.test.tsx`
Expected: FAIL (no "Continue to Simulate" button).

- [ ] **Step 3: Implement.**
  - In `onExport`, replace `downloadBlob(blob, …)` with `goTo("program")`. Keep the warnings `notify`, and drop the unused `blob`.
  - Button content: `{busy ? "Checking…" : <>Continue to Simulate <ArrowRight size={14} /></>}` (import `ArrowRight`, drop `Download`).
  - Remove the `downloadBlob` import if it's now unused.

- [ ] **Step 4: Run tests**

Run: `cd frontend && npx vitest run && npx tsc --noEmit`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add frontend/src
git commit -m "feat: Review ends in Continue to Simulate; downloads live under File ▸ Export

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 7: Physics card in Simulate

**Files:**
- Create: `frontend/src/ui/program/PhysicsCard.tsx`
- Modify: `frontend/src/api/client.ts` (`physicsCheck`)
- Modify: `frontend/src/ui/program/ProgramMode.tsx` (render the card at the top of the chat column)
- Modify: `frontend/src/ui/theme.css`
- Test: `frontend/src/ui/program/physics.test.tsx` (new)

**Interfaces:**
- Consumes: `POST /simulate/check` (Task 3).
- Produces: `type PhysicsResult = { ok: boolean; kind: "ok" | "load" | "unstable" | "sag" | "crash"; message: string; link?: string; joint?: string; needed?: number; has?: number }`; `physicsCheck(robotName, links, joints, bodiesByLink): Promise<PhysicsResult>`.
- Produces: `<PhysicsCard ctx={Ctx} />`. It runs once per distinct export request (key = `JSON.stringify(buildExportRequest(model))` plus the robot name), and has a Run check button. On a failing result with `link`, it calls `viewer.focus(bodyIds of that link)`.

- [ ] **Step 1: Write the failing tests** in `physics.test.tsx`:

```tsx
import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import { boxBody } from "../../model/testBodies";
import { Harness, imported } from "../testing";
import PhysicsCard from "./PhysicsCard";

afterEach(() => vi.restoreAllMocks());
const ui = () => render(<Harness start={imported([boxBody("base", [0, 0, 0], [1, 1, 1])])} step="program" ui={(ctx) => <PhysicsCard ctx={ctx} />} />);

describe("PhysicsCard", () => {
  it("runs on open and shows a pass", async () => {
    const spy = vi.spyOn(client, "physicsCheck").mockResolvedValue({ ok: true, kind: "ok", message: "Holds pose." });
    ui();
    expect(await screen.findByText("Holds pose.")).toBeTruthy();
    expect(spy).toHaveBeenCalledTimes(1);
  });

  it("shows a sag with its message and can run again", async () => {
    const spy = vi.spyOn(client, "physicsCheck").mockResolvedValue(
      { ok: false, kind: "sag", link: "base", joint: "j", needed: 140, has: 100, message: "base sags 18°: needs ~140 N·m, has 100." });
    ui();
    expect(await screen.findByText(/sags 18°/)).toBeTruthy();
    expect(screen.getByTestId("physics-card").className).toMatch(/bad/);
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /run check/i })); });
    expect(spy).toHaveBeenCalledTimes(2);
  });

  it("a failed request shows in the card, not as a banner", async () => {
    vi.spyOn(client, "physicsCheck").mockRejectedValue(new Error("Cannot reach the backend"));
    ui();
    expect(await screen.findByText(/Cannot reach the backend/)).toBeTruthy();
    expect(document.querySelector(".banner")).toBeNull();
  });

  it("shows Checking physics… with a timer while it runs", async () => {
    vi.spyOn(client, "physicsCheck").mockImplementation(() => new Promise(() => {}));
    ui();
    expect(await screen.findByText(/Checking physics…/)).toBeTruthy();
  });
});
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd frontend && npx vitest run src/ui/program/physics.test.tsx`
Expected: FAIL (`Cannot find module './PhysicsCard'`).

- [ ] **Step 3: Implement.** In `client.ts`:

```ts
export type PhysicsResult = { ok: boolean; kind: "ok" | "load" | "unstable" | "sag" | "crash"; message: string;
  link?: string; joint?: string; needed?: number; has?: number };
export async function physicsCheck(robotName: string, links: Link[], joints: Joint[],
  bodiesByLink: Record<string, Body[]>): Promise<PhysicsResult> {
  return (await send("/simulate/check", json({ robot_name: robotName, links, joints, bodies_by_link: bodiesByLink }))).json();
}
```

`PhysicsCard.tsx`:

```tsx
import { Check, Loader2, Play, X } from "lucide-react";
import { useEffect, useMemo, useRef, useState } from "react";
import { physicsCheck, type PhysicsResult } from "../../api/client";
import { buildExportRequest } from "../../model/exportRequest";
import type { Ctx } from "../ctx";
import { fmtElapsed, useElapsed } from "../useElapsed";

/** Holds the arm at home pose under gravity for 2 s in MuJoCo and says whether it stays put. */
export default function PhysicsCard({ ctx }: { ctx: Ctx }) {
  const { model, robotName, viewer } = ctx;
  const req = useMemo(() => buildExportRequest(model), [model]);
  const key = useMemo(() => robotName + JSON.stringify(req), [robotName, req]);
  const [result, setResult] = useState<PhysicsResult | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [running, setRunning] = useState(false);
  const elapsed = useElapsed(running);
  const lastKey = useRef<string | null>(null);
  const token = useRef(0);

  async function run() {
    const t = ++token.current;
    setRunning(true); setError(null);
    try {
      const r = await physicsCheck(robotName, req.links, req.joints, req.bodiesByLink);
      if (t !== token.current) return;
      setResult(r);
      if (!r.ok && r.link) viewer?.focus(model.links.find((l) => l.name === r.link)?.body_ids ?? []);
    } catch (e) { if (t === token.current) setError((e as Error).message); }
    if (t === token.current) setRunning(false);
  }
  useEffect(() => { if (lastKey.current !== key) { lastKey.current = key; void run(); } }, [key]);  // eslint-disable-line react-hooks/exhaustive-deps

  const bad = !!error || (result && !result.ok);
  return (
    <div className={`card physics-card${bad ? " bad" : result?.ok ? " ok" : ""}`} data-testid="physics-card">
      <div className="row">
        <strong>Physics</strong>
        {running ? <span className="muted"><Loader2 size={13} className="spin" /> Checking physics… {fmtElapsed(elapsed)}</span>
          : error ? <span><X size={13} /> {error}</span>
          : result ? <span>{result.ok ? <Check size={13} /> : <X size={13} />} {result.message}</span> : null}
        <span className="spacer" />
        <button className="btn btn-ghost" disabled={running} onClick={() => void run()}><Play size={13} />Run check</button>
      </div>
    </div>
  );
}
```

In `ProgramMode.tsx`, render `<PhysicsCard ctx={ctx} />` as the first child of `<section className="chat-column" …>`. CSS:

```css
.physics-card { margin: 8px; padding: 6px 10px; }
.physics-card.ok { border-color: var(--success); }
.physics-card.bad { border-color: var(--danger); }
.physics-card .row { gap: 8px; align-items: center; }
```

- [ ] **Step 4: Run all frontend tests + type check.** Existing ProgramMode tests now also render the card. Add `vi.spyOn(client, "physicsCheck").mockResolvedValue({ ok: true, kind: "ok", message: "Holds pose." })` to that file's `beforeEach`, so no real fetch happens.

Run: `cd frontend && npx vitest run && npx tsc --noEmit`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add frontend/src
git commit -m "feat: Physics card in Simulate runs the hold check and points at the failing link

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: Real-app check

**Files:** none (verification; fixes go in new commits).

- [ ] **Step 1: Full suites.**

Run: `cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q` (≈3 min, run in the background), and `cd frontend && npx vitest run && npx tsc --noEmit && npm run build`.
Expected: all PASS.

- [ ] **Step 2: Real robot through the API.** With the backend running, POST the irb1300 project's export request to `/simulate/check`. (Build it from `corpus/irb1300_asm.stp` via `/import`, then reuse the corpus expected joints in `backend/corpus/expected/irb1300.yaml` if a helper exists there. Otherwise ask the user for a saved `project.json`.)
Expected: `"Holds pose."`. Then set `link2`'s joint `max_effort` to 10 and expect a `sag` naming that link. Record the suggested efforts per joint and the check's wall time in the spec as a "Findings" section.

- [ ] **Step 3: Restart the dev servers** (backend + `npm run dev`) so the user can try:
  - File/Edit/View;
  - Build ↔ Simulate;
  - Review → Continue to Simulate;
  - the Max torque placeholder;
  - the Physics card.

- [ ] **Step 4: Commit findings**

```bash
git add docs/superpowers/specs/2026-10-14-workbenches-mujoco-check-design.md
git commit -m "docs: MuJoCo check findings on irb1300

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
