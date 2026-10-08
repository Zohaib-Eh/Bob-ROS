# Import Robustness, User Fixes, and Model Corpus Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make import never drop geometry silently, give every problem part a status and a fix, add the fix inputs to the API, and add a model corpus with a regression scorecard.

**Architecture:** The STEP/IGES/BREP importer gains a second pass that sews loose shells and loose faces into solids and an accounting check over all faces; every `Body` carries a `status` and `warnings`. Upload handling moves into one shared pipeline used by `/import` and the new `/replace-part`. `/export` gains mass override and approximate-part guards; `/detect-joints` returns a reason and evidence. A standalone corpus runner scores real models against committed expectations.

**Tech Stack:** Python 3.11 (conda env `cad-to-urdf`), FastAPI, pydantic 2, pythonocc-core 7.8.1, trimesh 5.1.1, numpy, PyYAML 6, pytest.

**Spec:** `docs/superpowers/specs/2026-10-08-import-robustness-design.md` (builds on `docs/superpowers/specs/2026-10-07-cad-to-urdf-design.md`).

## Global Constraints

- All lengths are meters internally.
- `Body`, `Face`, `Link`, `Joint` field names and shapes are final; only add fields, never rename or reshape.
- Face `origin` is exactly `"brep"` or `"segmented"`.
- Nothing disappears silently: every part is imported, imported with a named warning, or listed as `failed` with a reason.
- One bad part never fails a whole import; only an unreadable file returns 422.
- Mass/COM/inertia are always computed server-side; client `mass_kg`/`center_of_mass`/`inertia` are ignored.
- Every function that computes geometry is tested against a known analytic shape.
- Run Python as `~/miniforge3/envs/cad-to-urdf/python.exe` from `backend/`. Set `OPENBLAS_NUM_THREADS=1` for ad-hoc scripts (tests get it from `app/__init__.py`).
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

- **A STEP product that mixes a solid and a loose shell in one part** — both must come back as bodies; covered in Task 2 (`test_compound_with_solid_and_shell_in_one_part`).
- **A replacement file that itself fails** (unreadable geometry, zero triangles) — `/replace-part` must return the body as `failed` with the original id, not 500; covered in Task 6 (`test_replace_part_with_unmeshable_file_returns_failed_body`).
- **Mass override on a part whose volume can't be measured** (open mesh with ≤0 computed volume) — the user explicitly set the mass, so export must succeed using a bounding-box approximation, not 422; covered in Task 5 (`test_mass_override_on_unmeasurable_geometry_uses_bbox`).
- **Non-ASCII part names** (STEP product names like `Ünterteil`) in the `X-Export-Warnings` header — must not crash the response; covered in Task 5 (`test_export_warning_header_is_ascii_safe`).
- **Malformed `/replace-part` form fields** (`bbox_min="a,b"`, `nan`) — 422, not 500; covered in Task 6 (`test_replace_part_malformed_bbox_is_422`).

---

## File Structure

```
backend/
  app/
    models.py                       # + PartWarning, Body.status/warnings, Link.mass_override_kg/accept_approximate_mass
    importers/
      step_importer.py              # + import_cad(): solids, sewn loose shells/faces, accounting, exporter advice, XCAF names
      mesh_importer.py              # open meshes -> approximate; inverted -> warning
      pipeline.py           (new)   # import_path(): one import entry point for any extension; segment_mesh_body()
      placement.py          (new)   # combine_bodies(), place_replacement()
    segment/region_growing.py       # + allow_open flag
    export/urdf_writer.py           # mass override, approximate/failed guards, unused_body_warnings()
    routes/
      upload.py             (new)   # save_upload(): temp-file handling shared by routes
      import_routes.py              # uses pipeline + upload; returns file-level warnings
      replace_routes.py     (new)   # POST /replace-part
      joint_routes.py               # {joint, reason, evidence}
      export_routes.py              # X-Export-Warnings header
    main.py                         # register replace router
  corpus/
    __init__.py             (new)
    generate.py             (new)   # writes generated models into a corpus dir
    run.py                  (new)   # scorecard runner
    expected/*.yaml         (new)   # committed expectations
    baseline.json           (new)   # committed scores
  tests/
    conftest.py                     # + mixed-topology BREP fixture
    test_models.py, test_step_importer.py, test_mesh_importer.py, test_region_growing.py,
    test_pipeline.py (new), test_placement.py (new), test_urdf_writer.py, test_end_to_end.py,
    test_corpus_runner.py (new)
corpus/                             # repo root, git-ignored: model files
```

---

### Task 1: Status, warnings, and mass-override fields on the models

**Files:**
- Modify: `backend/app/models.py`
- Test: `backend/tests/test_models.py`

**Interfaces:**
- Produces:
  - `Fix = Literal["set_mass", "replace", "ignore", "reexport"]`
  - `class PartWarning(_Model): code: str; message: str; fixes: list[Fix] = []`
  - `Body.status: Literal["ok", "repaired", "approximate", "failed"] = "ok"`, `Body.warnings: list[PartWarning] = []`
  - `Link.mass_override_kg: float | None = None` (must be > 0 when set), `Link.accept_approximate_mass: bool = False`
  - `STATUS_ORDER = {"ok": 0, "repaired": 1, "approximate": 2, "failed": 3}` (worst-of ordering, used by Task 6)

- [ ] **Step 1: Write the failing tests** (append to `backend/tests/test_models.py`)

```python
import math
import pytest
from pydantic import ValidationError
from app.models import Body, Link, PartWarning, STATUS_ORDER


def test_body_defaults_to_ok_with_no_warnings():
    b = Body(id="b", name="n", vertices=[], triangles=[])
    assert b.status == "ok"
    assert b.warnings == []


def test_body_carries_status_and_warnings():
    w = PartWarning(code="open_geometry", message="p: 4 open edges", fixes=["set_mass", "replace", "ignore"])
    b = Body(id="b", name="p", vertices=[], triangles=[], status="approximate", warnings=[w])
    assert b.warnings[0].fixes == ["set_mass", "replace", "ignore"]


def test_unknown_status_or_fix_rejected():
    with pytest.raises(ValidationError):
        Body(id="b", name="n", vertices=[], triangles=[], status="broken")
    with pytest.raises(ValidationError):
        PartWarning(code="x", message="m", fixes=["delete_everything"])


def test_link_mass_override_defaults():
    l = Link(id="l", name="l", body_ids=[], density_kg_m3=1000.0)
    assert l.mass_override_kg is None
    assert l.accept_approximate_mass is False


@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf])
def test_link_mass_override_must_be_finite_positive(bad):
    with pytest.raises(ValidationError):
        Link(id="l", name="l", body_ids=[], density_kg_m3=1000.0, mass_override_kg=bad)


def test_status_order_is_worst_last():
    assert sorted(STATUS_ORDER, key=STATUS_ORDER.get) == ["ok", "repaired", "approximate", "failed"]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_models.py -v`
Expected: FAIL with `ImportError: cannot import name 'PartWarning'`

- [ ] **Step 3: Implement** (in `backend/app/models.py`)

Add after `_check_direction`:

```python
Fix = Literal["set_mass", "replace", "ignore", "reexport"]
Status = Literal["ok", "repaired", "approximate", "failed"]
STATUS_ORDER = {"ok": 0, "repaired": 1, "approximate": 2, "failed": 3}


class PartWarning(_Model):
    code: str
    message: str
    fixes: list[Fix] = []
```

Add to `Body` (after `faces`):

```python
    status: Status = "ok"
    warnings: list[PartWarning] = []
```

Add to `Link` (after `inertia`), plus a validator:

```python
    mass_override_kg: float | None = None
    accept_approximate_mass: bool = False

    @model_validator(mode="after")
    def _check_override(self):
        if self.mass_override_kg is not None and not self.mass_override_kg > 0:
            raise ValueError("mass_override_kg must be > 0")
        return self
```

- [ ] **Step 4: Run tests to verify they pass, then the full suite**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass, no warnings.

- [ ] **Step 5: Commit**

```bash
git add backend/app/models.py backend/tests/test_models.py
git commit -m "feat: add part status, warnings, and link mass override to models"
```

---

### Task 2: CAD import sews loose shells/faces and accounts for every face

**Files:**
- Modify: `backend/app/importers/step_importer.py`
- Modify: `backend/tests/conftest.py` (new fixture)
- Test: `backend/tests/test_step_importer.py`

**Interfaces:**
- Consumes: `Body`, `PartWarning` (Task 1).
- Produces:
  - `import_cad(file_path: str, brep_scale: float = 0.001) -> tuple[list[Body], list[PartWarning]]` — bodies plus file-level warnings.
  - `import_step(file_path, brep_scale=0.001) -> list[Body]` keeps its signature: `return import_cad(file_path, brep_scale)[0]`.
  - Body names: solids `solid_N`, sewn shells `shell_N`, sewn loose faces `surface_N` (N = index in the returned list). Task 8 replaces these with STEP product names where available.
  - Warning codes produced: `shell_repaired`, `open_geometry`, `degenerate` (part-level); `unaccounted_faces` (file-level, fixes `["reexport"]`).
  - `_reexport_advice(file_path: str) -> str` — exporter-specific advice from the STEP header.

- [ ] **Step 1: Add the fixture** (append to `backend/tests/conftest.py`)

A BREP file is used because `breptools.Write` preserves topology exactly (a STEP round-trip may convert shells to solids); BREP goes through the same `import_cad` code path.

```python
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SHELL
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Shell


def _faces(shape):
    ex, out = TopExp_Explorer(shape, TopAbs_FACE), []
    while ex.More():
        out.append(ex.Current()); ex.Next()
    return out


@pytest.fixture
def mixed_topology_brep_path(tmp_path) -> str:
    """1 m units (import with brep_scale=1.0). Contains, left to right along x:
    a solid box at x=0, a closed shell (box at x=3, not a solid), an open shell
    (box at x=6 with its 6th face missing), and one loose face (a box face at x=9)."""
    b = BRep_Builder()
    solid = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape()
    closed_shell = TopExp_Explorer(
        BRepPrimAPI_MakeBox(gp_Pnt(3, 0, 0), 1.0, 1.0, 1.0).Shape(), TopAbs_SHELL).Current()
    open_shell = TopoDS_Shell()
    b.MakeShell(open_shell)
    for f in _faces(BRepPrimAPI_MakeBox(gp_Pnt(6, 0, 0), 1.0, 1.0, 1.0).Shape())[:5]:
        b.Add(open_shell, f)
    loose_face = _faces(BRepPrimAPI_MakeBox(gp_Pnt(9, 0, 0), 1.0, 1.0, 1.0).Shape())[0]
    comp = TopoDS_Compound()
    b.MakeCompound(comp)
    for s in (solid, closed_shell, open_shell, loose_face):
        b.Add(comp, s)
    path = str(tmp_path / "mixed.brep")
    breptools.Write(comp, path)
    return path
```

- [ ] **Step 2: Write the failing tests** (append to `backend/tests/test_step_importer.py`)

```python
import trimesh
import numpy as np
from app.importers import step_importer
from app.importers.step_importer import import_cad


def _by_x(bodies):
    return sorted(bodies, key=lambda b: min(v[0] for v in b.vertices) if b.vertices else 1e9)


def test_mixed_topology_imports_every_part(mixed_topology_brep_path):
    bodies, warnings = import_cad(mixed_topology_brep_path, brep_scale=1.0)
    assert len(bodies) == 4
    solid, closed, open_, loose = _by_x(bodies)
    assert solid.status == "ok" and solid.warnings == []
    assert closed.status == "repaired"
    assert [w.code for w in closed.warnings] == ["shell_repaired"]
    assert open_.status == "approximate"
    assert open_.warnings[0].code == "open_geometry"
    assert "4 open edges" in open_.warnings[0].message      # a 1x1 box missing one face
    assert open_.warnings[0].fixes == ["set_mass", "replace", "ignore"]
    assert loose.status == "approximate"
    assert warnings == []                                    # every face accounted for


def test_repaired_shell_has_analytic_volume(mixed_topology_brep_path):
    bodies, _ = import_cad(mixed_topology_brep_path, brep_scale=1.0)
    closed = _by_x(bodies)[1]
    m = trimesh.Trimesh(np.array(closed.vertices), np.array(closed.triangles), process=False)
    assert m.is_watertight
    assert abs(m.volume - 1.0) < 1e-6


def test_repaired_shell_faces_are_brep_planes_with_outward_normals(mixed_topology_brep_path):
    bodies, _ = import_cad(mixed_topology_brep_path, brep_scale=1.0)
    closed = _by_x(bodies)[1]
    assert {f.kind for f in closed.faces} == {"plane"}
    assert {f.origin for f in closed.faces} == {"brep"}
    centroid = np.array([3.5, 0.5, 0.5])
    verts, tris = np.array(closed.vertices), np.array(closed.triangles)
    for f in closed.faces:
        center = verts[tris[f.triangle_indices].ravel()].mean(axis=0)
        assert np.dot(f.normal, center - centroid) > 0


def test_dropped_faces_produce_unaccounted_warning(mixed_topology_brep_path, monkeypatch):
    monkeypatch.setattr(step_importer, "_loose_faces", lambda shape: [])
    bodies, warnings = import_cad(mixed_topology_brep_path, brep_scale=1.0)
    assert len(bodies) == 3
    assert [w.code for w in warnings] == ["unaccounted_faces"]
    assert "1 face" in warnings[0].message
    assert warnings[0].fixes == ["reexport"]


def test_import_step_keeps_list_signature(box_step_path):
    bodies = import_step(box_step_path)
    assert isinstance(bodies, list) and len(bodies) == 1


def test_compound_with_solid_and_shell_in_one_part(tmp_path):
    """Review Focus: a product mixing a solid and a loose shell keeps both."""
    b = BRep_Builder()
    shell = TopExp_Explorer(BRepPrimAPI_MakeBox(gp_Pnt(3, 0, 0), 1, 1, 1).Shape(), TopAbs_SHELL).Current()
    inner = TopoDS_Compound(); b.MakeCompound(inner)
    b.Add(inner, BRepPrimAPI_MakeBox(1, 1, 1).Shape()); b.Add(inner, shell)
    outer = TopoDS_Compound(); b.MakeCompound(outer); b.Add(outer, inner)
    path = str(tmp_path / "nested.brep"); breptools.Write(outer, path)
    bodies, warnings = import_cad(path, brep_scale=1.0)
    assert sorted(x.status for x in bodies) == ["ok", "repaired"]
    assert warnings == []


@pytest.mark.parametrize("system,expected", [
    ("CREO PARAMETRIC BY PTC INC", "Creo"),
    ("SOLIDWORKS 2024", "SOLIDWORKS"),
    ("Autodesk Fusion", "Fusion"),
    ("Onshape", "Onshape"),
    ("Some Unknown CAD", "solid bodies"),
])
def test_reexport_advice_names_exporter(tmp_path, system, expected):
    p = tmp_path / "x.step"
    p.write_text(f"ISO-10303-21;\nHEADER;\nFILE_NAME('x','t',(''),(''),'{system}','{system}','');\nENDSEC;\n")
    assert expected in step_importer._reexport_advice(str(p))
```

These tests use `BRep_Builder`, `BRepPrimAPI_MakeBox`, `TopExp_Explorer`, `TopAbs_SHELL`, `TopoDS_Compound`, `breptools`, `gp_Pnt`, `pytest` and `import_step` — add any missing imports at the top of the test file:

```python
import pytest
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.BRepTools import breptools
from OCC.Core.gp import gp_Pnt
from OCC.Core.TopAbs import TopAbs_SHELL
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Compound
from app.importers.step_importer import import_step
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_step_importer.py -v`
Expected: FAIL with `ImportError: cannot import name 'import_cad'`

- [ ] **Step 4: Implement** (in `backend/app/importers/step_importer.py`)

Add imports:

```python
import re
import trimesh
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing
from OCC.Core.ShapeFix import ShapeFix_Solid
from OCC.Core.TopAbs import TopAbs_SHELL
from OCC.Core.TopoDS import TopoDS_Shell, topods
from app.models import PartWarning
```

Guard `_extract_mesh` against empty triangulations — insert before the "Merge coincident vertices" block:

```python
    if not verts:
        return [], [], per_face
```

Add helpers and `import_cad`; replace `import_step`'s body:

```python
_SEW_TOLERANCE = 1e-5  # meters


def _subshapes(shape, kind, avoid=None) -> list:
    ex = TopExp_Explorer(shape, kind) if avoid is None else TopExp_Explorer(shape, kind, avoid)
    out = []
    while ex.More():
        out.append(ex.Current()); ex.Next()
    return out


def _loose_faces(shape) -> list:
    """Faces that are in no shell (shells inside solids are shells too)."""
    return _subshapes(shape, TopAbs_FACE, TopAbs_SHELL)


def _sew_to_solids(shapes) -> list:
    """Sew shells/faces together and turn each resulting shell into a solid.
    Open shells still become (open) solids; the mesh check decides the status."""
    sew = BRepBuilderAPI_Sewing(_SEW_TOLERANCE)
    for s in shapes:
        sew.Add(s)
    sew.Perform()
    sewn = sew.SewedShape()
    shells = _subshapes(sewn, TopAbs_SHELL)
    if not shells:  # a single sewn face comes back as a face, not a shell
        shell = TopoDS_Shell()
        BRep_Builder().MakeShell(shell)
        for f in _subshapes(sewn, TopAbs_FACE):
            BRep_Builder().Add(shell, f)
        shells = [shell]
    solids = []
    for sh in shells:
        fix = ShapeFix_Solid(BRepBuilderAPI_MakeSolid(topods.Shell(sh)).Solid())
        fix.Perform()
        solids.append(fix.Solid())
    return solids


def _open_edge_count(vertices, triangles) -> int:
    mesh = trimesh.Trimesh(np.asarray(vertices), np.asarray(triangles), process=False)
    return len(trimesh.grouping.group_rows(mesh.edges_sorted, require_count=1))


def _make_body(solid, name: str, repaired: bool) -> Body:
    vertices, triangles, per_face = _extract_mesh(solid)
    if not triangles:
        return Body(id=str(uuid.uuid4()), name=name, vertices=[], triangles=[], status="failed",
                    warnings=[PartWarning(code="degenerate", fixes=["replace", "ignore"],
                                          message=f"{name}: could not be meshed")])
    faces = [_face_to_model(f, ids) for f, ids in per_face if ids]
    status, warnings = "ok", []
    if repaired:
        n_open = _open_edge_count(vertices, triangles)
        if n_open == 0:
            status = "repaired"
            warnings = [PartWarning(code="shell_repaired", fixes=[],
                                    message=f"{name}: surface shell sewn into a closed solid")]
        else:
            status = "approximate"
            warnings = [PartWarning(code="open_geometry", fixes=["set_mass", "replace", "ignore"],
                                    message=f"{name}: {n_open} open edges remain after repair; "
                                            "mass is approximate")]
    return Body(id=str(uuid.uuid4()), name=name, vertices=vertices, triangles=triangles,
                faces=faces, status=status, warnings=warnings)


_EXPORTER_ADVICE = [
    (("CREO", "PRO/ENGINEER"), "Creo: export STEP AP214 or AP242 with 'Solids' enabled."),
    (("SOLIDWORKS",), "SOLIDWORKS: Save As STEP AP214, with 'Export as solid/surface geometry' set to solid."),
    (("FUSION", "AUTODESK"), "Fusion: export the design as STEP from File > Export (bodies, not meshes)."),
    (("ONSHAPE",), "Onshape: export as STEP AP242 and leave 'Export as single part' off."),
    (("FREECAD",), "FreeCAD: export solids (Part objects), not mesh or surface objects, to STEP."),
]


def _reexport_advice(file_path: str) -> str:
    try:
        with open(file_path, "r", errors="ignore") as f:
            header = f.read(8192).upper()
    except OSError:
        header = ""
    for keys, advice in _EXPORTER_ADVICE:
        if any(k in header for k in keys):
            return advice
    return "Re-export the model from your CAD tool as STEP with solid bodies."


def import_cad(file_path: str, brep_scale: float = 0.001) -> tuple[list[Body], list[PartWarning]]:
    """Import STEP/IGES/BREP (by extension) in meters: every solid, plus loose shells
    and loose faces sewn into solids. File-level warnings report faces that ended up
    in no body. `brep_scale` applies to unitless .brep only."""
    if not (math.isfinite(brep_scale) and brep_scale > 0):
        raise ValueError(f"brep_scale must be finite and positive, got {brep_scale}")
    shape = _read_shape(file_path, brep_scale)
    total_faces = len(_subshapes(shape, TopAbs_FACE))

    parts = [(s, "solid", False) for s in _subshapes(shape, TopAbs_SOLID)]
    for sh in _subshapes(shape, TopAbs_SHELL, TopAbs_SOLID):
        parts += [(s, "shell", True) for s in _sew_to_solids([sh])]
    loose = _loose_faces(shape)
    if loose:
        parts += [(s, "surface", True) for s in _sew_to_solids(loose)]

    bodies = [_make_body(s, f"{prefix}_{i}", repaired) for i, (s, prefix, repaired) in enumerate(parts)]
    accounted = sum(len(_subshapes(s, TopAbs_FACE)) for s, _, _ in parts)
    warnings = []
    if accounted < total_faces:
        missing = total_faces - accounted
        warnings.append(PartWarning(
            code="unaccounted_faces", fixes=["reexport"],
            message=f"{missing} face{'s' if missing != 1 else ''} in the file did not end up "
                    f"in any part. {_reexport_advice(file_path)}"))
    if not bodies:
        raise ValueError("No geometry found in CAD file")
    return bodies, warnings


def import_step(file_path: str, brep_scale: float = 0.001) -> list[Body]:
    """Bodies only; see import_cad for file-level warnings."""
    return import_cad(file_path, brep_scale)[0]
```

Note the message for one missing face reads `"1 face in the file ..."` — the test checks `"1 face"`.

- [ ] **Step 5: Run tests to verify they pass, then the full suite**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass, no warnings. Existing STEP tests (box volume 6.0, peg-in-hole 2 bodies, outward normals) must still pass unchanged.

- [ ] **Step 6: Commit**

```bash
git add backend/app/importers/step_importer.py backend/tests/conftest.py backend/tests/test_step_importer.py
git commit -m "feat: sew loose shells/faces into parts and account for every face on CAD import"
```

---

### Task 3: Open and inverted meshes import with a status instead of failing

**Files:**
- Modify: `backend/app/importers/mesh_importer.py`
- Modify: `backend/app/segment/region_growing.py`
- Test: `backend/tests/test_mesh_importer.py`, `backend/tests/test_region_growing.py`

**Interfaces:**
- Consumes: `Body`, `PartWarning` (Task 1).
- Produces:
  - `import_mesh(file_path, scale=0.001) -> tuple[Body, float]` (unchanged signature). Non-watertight → `status="approximate"` + `open_geometry` warning (fixes `["set_mass","replace","ignore"]`). Inside-out → flipped + `inverted_mesh` warning (fixes `[]`), status stays `ok`. Unreadable/empty still raises `ValueError("could not read mesh ...")`. Body name = file stem (e.g. `broken` for `broken.stl`).
  - `grow_regions(vertices, triangles, angle_threshold_deg=15.0, allow_open=False)` — with `allow_open=True` the watertight guard is skipped.

- [ ] **Step 1: Write the failing tests**

Replace `test_non_watertight_mesh_raises_clear_error` in `backend/tests/test_mesh_importer.py` with:

```python
def test_non_watertight_mesh_imports_as_approximate(non_watertight_stl_path):
    body, scale = import_mesh(non_watertight_stl_path)
    assert body.status == "approximate"
    assert body.name == "broken"
    w = body.warnings[0]
    assert w.code == "open_geometry"
    assert "3 open edges" in w.message          # fixture drops one triangle -> 3 boundary edges
    assert w.fixes == ["set_mass", "replace", "ignore"]
    assert len(body.triangles) == 11


def test_inverted_mesh_is_flipped_with_warning(tmp_path):
    import trimesh
    m = trimesh.creation.box(extents=(1000.0, 1000.0, 1000.0))
    m.invert()
    p = str(tmp_path / "inside_out.stl"); m.export(p)
    body, _ = import_mesh(p)
    assert body.status == "ok"
    assert [w.code for w in body.warnings] == ["inverted_mesh"]
    assert body.warnings[0].fixes == []


def test_clean_mesh_has_no_warnings(box_stl_path):
    body, _ = import_mesh(box_stl_path)
    assert body.status == "ok" and body.warnings == []
```

If the existing inverted-mesh test in this file asserts only on volume/normals, keep it; the new test adds the warning assertion.

Append to `backend/tests/test_region_growing.py`:

```python
def test_open_mesh_grows_regions_when_allowed():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    mesh.update_faces(mesh.face_normals[:, 2] < 0.5)   # remove the +z side (2 triangles)
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles, allow_open=True)
    assert len(regions) == 5
    assert sorted(i for r in regions for i in r) == list(range(10))
```

The existing tests `test_open_mesh_raises_watertight_error` and `test_unwelded_triangle_soup_raises_watertight_error` stay unchanged (default `allow_open=False`).

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_mesh_importer.py tests/test_region_growing.py -v`
Expected: FAIL — `ValueError: Mesh is not watertight` in the approximate test; `TypeError: unexpected keyword argument 'allow_open'`.

- [ ] **Step 3: Implement**

`backend/app/segment/region_growing.py` — add the parameter and condition the guard:

```python
def grow_regions(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    angle_threshold_deg: float = 15.0,
    allow_open: bool = False,
) -> list[list[int]]:
    ...
    if not allow_open and not mesh.is_watertight:
        raise ValueError("mesh must be watertight to grow regions")
```

`backend/app/importers/mesh_importer.py` — replace everything from the watertight check to the end:

```python
import os
from app.models import Body, PartWarning

    ...
    name = os.path.splitext(os.path.basename(file_path))[0] or "mesh"
    status, warnings = "ok", []
    if not mesh.is_watertight:
        n_open = len(trimesh.grouping.group_rows(mesh.edges_sorted, require_count=1))
        status = "approximate"
        warnings.append(PartWarning(
            code="open_geometry", fixes=["set_mass", "replace", "ignore"],
            message=f"{name}: mesh is not closed ({n_open} open edges); mass is approximate"))
    elif mesh.volume < 0:  # inward-wound file: flip so normals/volume are outward
        mesh.invert()
        warnings.append(PartWarning(
            code="inverted_mesh", fixes=[],
            message=f"{name}: mesh was inside-out and has been flipped"))

    mesh.apply_scale(scale)
    body = Body(
        id=str(uuid.uuid4()), name=name,
        vertices=[tuple(v) for v in mesh.vertices.tolist()],
        triangles=[tuple(t) for t in mesh.faces.tolist()],
        faces=[], status=status, warnings=warnings,
    )
    return body, scale
```

(The inversion check only runs for closed meshes: an open mesh's signed volume is not meaningful.)

- [ ] **Step 4: Fix the end-to-end test that expected 422 for open meshes**

In `backend/tests/test_end_to_end.py`, replace `test_import_non_watertight_names_upload_not_temp_path` with:

```python
def test_import_non_watertight_is_approximate_not_422(non_watertight_stl_path):
    with open(non_watertight_stl_path, "rb") as f:
        r = client.post("/import", files={"file": ("broken.stl", f.read())})
    assert r.status_code == 200
    body = r.json()["bodies"][0]
    assert body["status"] == "approximate"
    assert body["warnings"][0]["code"] == "open_geometry"
    assert "tmp" not in body["warnings"][0]["message"].lower()
```

This test passes only after Task 4 wires `allow_open` into `/import`. In this task, mark it `@pytest.mark.xfail(reason="wired in Task 4", strict=True)`; Task 4 removes the marker.

- [ ] **Step 5: Run the full suite**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass (1 xfailed), no warnings.

- [ ] **Step 6: Commit**

```bash
git add backend/app/importers/mesh_importer.py backend/app/segment/region_growing.py backend/tests/test_mesh_importer.py backend/tests/test_region_growing.py backend/tests/test_end_to_end.py
git commit -m "feat: import open meshes as approximate and flag inverted meshes"
```

---

### Task 4: One import pipeline; /import returns statuses and file-level warnings

**Files:**
- Create: `backend/app/importers/pipeline.py`
- Create: `backend/app/routes/upload.py`
- Modify: `backend/app/routes/import_routes.py`
- Test: `backend/tests/test_pipeline.py` (new), `backend/tests/test_end_to_end.py`

**Interfaces:**
- Consumes: `import_cad` (Task 2), `import_mesh` + `grow_regions(..., allow_open=)` (Task 3), `fit_face` (existing), `PartWarning`.
- Produces:
  - `pipeline.STEP_EXTENSIONS = {".step", ".stp", ".iges", ".igs", ".brep"}`, `pipeline.MESH_EXTENSIONS = {".stl", ".obj"}`
  - `pipeline.segment_mesh_body(body: Body) -> Body` — fills `faces` via region growing + fitting; open meshes use `allow_open=True`; a mesh with no shared edges (triangle soup) gets `faces=[]` and a `degenerate` warning (fixes `["replace","ignore"]`), keeping its status.
  - `pipeline.import_path(path: str, suffix: str, scale: float) -> tuple[list[Body], float, list[PartWarning]]` — `(bodies, applied_scale, file_warnings)`; raises `ValueError` for unreadable files and unsupported suffixes.
  - `upload.save_upload(file: UploadFile) -> tuple[str, str]` — writes the upload to a temp file, returns `(tmp_path, suffix)`; raises `HTTPException(400)` for unsupported suffixes before writing. Caller deletes the file.
  - `upload.run_import(file: UploadFile, scale: float) -> tuple[list[Body], float, list[PartWarning]]` — save, `import_path`, map errors to `HTTPException(422, "<filename>: <reason>")`, always delete the temp file.
  - `/import` response: `{"bodies": [...], "applied_scale": float, "warnings": [PartWarning...]}`.

- [ ] **Step 1: Write the failing tests**

`backend/tests/test_pipeline.py`:

```python
import numpy as np
import trimesh
from app.importers.pipeline import import_path, segment_mesh_body
from app.models import Body


def _body(mesh, status="ok"):
    return Body(id="b", name="m", vertices=[tuple(v) for v in mesh.vertices.tolist()],
                triangles=[tuple(t) for t in mesh.faces.tolist()], status=status)


def test_segment_closed_box_gives_six_planes():
    b = segment_mesh_body(_body(trimesh.creation.box(extents=(1, 1, 1))))
    assert len(b.faces) == 6 and {f.kind for f in b.faces} == {"plane"}
    assert {f.origin for f in b.faces} == {"segmented"}


def test_segment_open_box_still_gets_faces():
    m = trimesh.creation.box(extents=(1, 1, 1))
    m.update_faces(m.face_normals[:, 2] < 0.5)          # remove the +z side
    b = segment_mesh_body(_body(m, status="approximate"))
    assert len(b.faces) == 5
    assert b.status == "approximate"


def test_segment_triangle_soup_is_degenerate_not_error():
    rng = np.random.default_rng(0)
    verts = rng.random((30, 3)).tolist()
    tris = [(i, i + 1, i + 2) for i in range(0, 30, 3)]          # 10 unconnected triangles
    b = segment_mesh_body(Body(id="b", name="soup", vertices=[tuple(v) for v in verts],
                               triangles=tris, status="approximate"))
    assert b.faces == []
    assert b.warnings[-1].code == "degenerate"
    assert b.warnings[-1].fixes == ["replace", "ignore"]


def test_import_path_step_returns_unit_scale(cylinder_step_path):
    bodies, scale, warnings = import_path(cylinder_step_path, ".step", 0.001)
    assert scale == 1.0 and len(bodies) == 1 and warnings == []


def test_import_path_brep_uses_scale(mixed_topology_brep_path):
    bodies, scale, warnings = import_path(mixed_topology_brep_path, ".brep", 1.0)
    assert scale == 1.0 and len(bodies) == 4


def test_import_path_stl_segments(box_stl_path):
    bodies, scale, _ = import_path(box_stl_path, ".stl", 0.001)
    assert scale == 0.001 and len(bodies[0].faces) == 6
```

Append to `backend/tests/test_end_to_end.py`:

```python
def test_import_response_has_file_level_warnings(box_stl_path):
    r = client.post("/import", files={"file": ("box.stl", open(box_stl_path, "rb").read())})
    assert r.status_code == 200
    assert r.json()["warnings"] == []
    assert r.json()["bodies"][0]["status"] == "ok"


def test_import_mixed_brep_lists_every_part(mixed_topology_brep_path):
    r = client.post("/import?scale=1.0",
                    files={"file": ("mixed.brep", open(mixed_topology_brep_path, "rb").read())})
    assert r.status_code == 200
    assert sorted(b["status"] for b in r.json()["bodies"]) == ["approximate", "approximate", "ok", "repaired"]
```

Remove the `xfail` marker from `test_import_non_watertight_is_approximate_not_422` (added in Task 3).

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_pipeline.py tests/test_end_to_end.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.importers.pipeline'`

- [ ] **Step 3: Implement**

`backend/app/importers/pipeline.py`:

```python
import trimesh
import numpy as np

from app.importers.mesh_importer import import_mesh
from app.importers.step_importer import import_cad
from app.models import Body, PartWarning
from app.segment.primitive_fit import fit_face
from app.segment.region_growing import grow_regions

STEP_EXTENSIONS = {".step", ".stp", ".iges", ".igs", ".brep"}
MESH_EXTENSIONS = {".stl", ".obj"}


def segment_mesh_body(body: Body) -> Body:
    """Reconstruct faces for a mesh body. Never raises for bad geometry: a mesh
    with no shared edges gets no faces and a `degenerate` warning instead."""
    mesh = trimesh.Trimesh(np.asarray(body.vertices), np.asarray(body.triangles), process=False)
    if len(body.triangles) > 1 and len(mesh.face_adjacency) == 0:
        return body.model_copy(update={"faces": [], "warnings": body.warnings + [PartWarning(
            code="degenerate", fixes=["replace", "ignore"],
            message=f"{body.name}: triangles share no edges, so no faces could be found")]})
    regions = grow_regions(body.vertices, body.triangles, allow_open=body.status != "ok")
    faces = [fit_face(body.vertices, body.triangles, r) for r in regions]
    return body.model_copy(update={"faces": faces})


def import_path(path: str, suffix: str, scale: float) -> tuple[list[Body], float, list[PartWarning]]:
    if suffix in STEP_EXTENSIONS:
        if suffix == ".brep":  # unitless: assumed mm unless overridden, like STL
            bodies, warnings = import_cad(path, brep_scale=scale)
            return bodies, scale, warnings
        bodies, warnings = import_cad(path)
        return bodies, 1.0, warnings  # STEP/IGES carry their own unit
    if suffix in MESH_EXTENSIONS:
        body, applied = import_mesh(path, scale=scale)
        return [segment_mesh_body(body)], applied, []
    raise ValueError(f"Unsupported file type: {suffix}")
```

`backend/app/routes/upload.py`:

```python
import os
import tempfile

from fastapi import HTTPException, UploadFile

from app.importers.pipeline import MESH_EXTENSIONS, STEP_EXTENSIONS, import_path
from app.models import Body, PartWarning


def save_upload(file: UploadFile) -> tuple[str, str]:
    suffix = os.path.splitext(file.filename or "")[1].lower()
    if suffix not in STEP_EXTENSIONS | MESH_EXTENSIONS:
        raise HTTPException(400, f"Unsupported file type: {suffix}")
    content = file.file.read()
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        tmp.write(content)
        return tmp.name, suffix


def run_import(file: UploadFile, scale: float) -> tuple[list[Body], float, list[PartWarning]]:
    tmp_path, suffix = save_upload(file)
    try:
        return import_path(tmp_path, suffix, scale)
    except ValueError as exc:
        raise HTTPException(422, f"{file.filename}: {exc}")
    except Exception as exc:  # corrupt uploads can fail anywhere in OCCT/trimesh;
        # library messages may carry the temp path, so name only the error type
        raise HTTPException(422, f"{file.filename}: could not import file ({type(exc).__name__})")
    finally:
        os.unlink(tmp_path)
```

`backend/app/routes/import_routes.py` — replace the whole file:

```python
from fastapi import APIRouter, UploadFile

from app.routes.upload import run_import

router = APIRouter()


@router.post("/import")
def import_file(file: UploadFile, scale: float = 0.001):  # sync: runs in threadpool
    bodies, applied_scale, warnings = run_import(file, scale)
    return {
        "bodies": [b.model_dump() for b in bodies],
        "applied_scale": applied_scale,
        "warnings": [w.model_dump() for w in warnings],
    }
```

- [ ] **Step 4: Run the full suite**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass, no xfail left, no warnings. Existing `/import` tests (400 unsupported, 422 corrupt, nan scale 422, brep scale) must still pass.

- [ ] **Step 5: Commit**

```bash
git add backend/app/importers/pipeline.py backend/app/routes/upload.py backend/app/routes/import_routes.py backend/tests/test_pipeline.py backend/tests/test_end_to_end.py
git commit -m "feat: shared import pipeline; /import returns part statuses and file warnings"
```

---

### Task 5: Export honours mass override and blocks unconfirmed approximate parts

**Files:**
- Modify: `backend/app/export/urdf_writer.py`
- Modify: `backend/app/routes/export_routes.py`
- Test: `backend/tests/test_urdf_writer.py`, `backend/tests/test_end_to_end.py`

**Interfaces:**
- Consumes: `Link.mass_override_kg`, `Link.accept_approximate_mass`, `Body.status`, `PartWarning` (Task 1); `compute_link_mass_properties` (existing).
- Produces:
  - `export_package(...)` (same signature) now raises `ValueError` listing every link with a `failed` body, and every link with an `approximate` body that has neither `mass_override_kg` nor `accept_approximate_mass`.
  - Mass override: COM from geometry; inertia = computed × (override / computed mass); mass = override. If geometry volume can't be computed and an override is set: COM = vertex bounding-box centre, inertia = solid box of the bounding-box extents at the override mass.
  - `unused_body_warnings(links, bodies_by_link) -> list[PartWarning]` — one `unused_body` warning (fixes `[]`) per body listed under a `bodies_by_link` key that is not a link id.
  - `/export` response header `X-Export-Warnings`: JSON list of warning dicts, ASCII-escaped.

  Clarification of spec §4.3: a body "in no link" means a body sent under a `bodies_by_link` key that matches no link's `id` (geometry is keyed by link id; that key is what the frontend uses for "ignore").

- [ ] **Step 1: Write the failing tests** (append to `backend/tests/test_urdf_writer.py`)

```python
import io, json, zipfile
import numpy as np
import pytest
import trimesh
import xml.etree.ElementTree as ET
from app.export.urdf_writer import export_package, unused_body_warnings
from app.models import Body, Link


def _box_body(bid, status="ok", extents=(1.0, 1.0, 1.0)):
    m = trimesh.creation.box(extents=extents)
    return Body(id=bid, name=bid, vertices=[tuple(v) for v in m.vertices.tolist()],
                triangles=[tuple(t) for t in m.faces.tolist()], status=status)


def _single_link_inertial(override=None, accept=False, status="ok"):
    link = Link(id="a", name="a", body_ids=["b"], density_kg_m3=1000.0,
                mass_override_kg=override, accept_approximate_mass=accept)
    data = export_package("r", [link], [], {"a": [_box_body("b", status)]})
    urdf = zipfile.ZipFile(io.BytesIO(data)).read("r/urdf/r.urdf")
    return ET.fromstring(urdf).find("link").find("inertial")


def test_mass_override_scales_inertia():
    el = _single_link_inertial(override=5.0)
    assert float(el.find("mass").attrib["value"]) == pytest.approx(5.0)
    # unit cube: I = m/6 * s^2 -> 5/6 at 5 kg (computed was 1000 kg -> 1000/6, ratio 1/200)
    assert float(el.find("inertia").attrib["ixx"]) == pytest.approx(5.0 / 6.0)
    com = [float(x) for x in el.find("origin").attrib["xyz"].split()]
    assert np.allclose(com, 0.0, atol=1e-9)                  # COM unchanged by the override


def test_approximate_part_without_override_or_accept_is_rejected():
    with pytest.raises(ValueError, match=r"Link 'a'.*approximate.*'b'"):
        _single_link_inertial(status="approximate")


@pytest.mark.parametrize("kw", [{"override": 2.0}, {"accept": True}])
def test_approximate_part_with_override_or_accept_exports(kw):
    el = _single_link_inertial(status="approximate", **kw)
    assert float(el.find("mass").attrib["value"]) > 0


def test_failed_part_is_rejected():
    link = Link(id="a", name="a", body_ids=["b"], density_kg_m3=1000.0, accept_approximate_mass=True)
    failed = Body(id="b", name="b", vertices=[], triangles=[], status="failed")
    with pytest.raises(ValueError, match=r"Link 'a'.*'b'.*failed"):
        export_package("r", [link], [], {"a": [failed]})


def test_mass_override_on_unmeasurable_geometry_uses_bbox():
    """Review Focus: user set the mass on an open part whose volume is <= 0."""
    # a flat 2 x 1 quad in the z=0 plane: signed volume is exactly 0
    body = Body(id="b", name="b", status="approximate",
                vertices=[(0, 0, 0), (2, 0, 0), (2, 1, 0), (0, 1, 0)],
                triangles=[(0, 1, 2), (0, 2, 3)])
    link = Link(id="a", name="a", body_ids=["b"], density_kg_m3=1000.0, mass_override_kg=6.0)
    data = export_package("r", [link], [], {"a": [body]})
    el = ET.fromstring(zipfile.ZipFile(io.BytesIO(data)).read("r/urdf/r.urdf")).find("link").find("inertial")
    assert float(el.find("mass").attrib["value"]) == pytest.approx(6.0)
    # bbox of the remaining quad; box inertia ixx = m/12 * (dy^2 + dz^2) over the vertex bbox
    v = np.array(body.vertices); d = v.max(0) - v.min(0)
    assert float(el.find("inertia").attrib["izz"]) == pytest.approx(6.0 / 12 * (d[0] ** 2 + d[1] ** 2))


def test_unused_body_warnings_name_bodies_under_unknown_keys():
    link = Link(id="a", name="a", body_ids=["b"], density_kg_m3=1000.0)
    ws = unused_body_warnings([link], {"a": [_box_body("b")], "spare": [_box_body("screw")]})
    assert [w.code for w in ws] == ["unused_body"]
    assert "screw" in ws[0].message
```

Append to `backend/tests/test_end_to_end.py`:

```python
def _box_json(bid, name=None):
    m = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    return {"id": bid, "name": name or bid, "vertices": m.vertices.tolist(), "triangles": m.faces.tolist()}


def test_export_warning_header_lists_unused_bodies():
    r = client.post("/export", json={
        "robot_name": "r",
        "links": [{"id": "a", "name": "a", "body_ids": ["b"], "density_kg_m3": 1000.0}],
        "joints": [],
        "bodies_by_link": {"a": [_box_json("b")], "spare": [_box_json("s")]}})
    assert r.status_code == 200
    ws = json.loads(r.headers["X-Export-Warnings"])
    assert [w["code"] for w in ws] == ["unused_body"]


def test_export_warning_header_is_ascii_safe():
    """Review Focus: non-ASCII part names must not break the response header."""
    r = client.post("/export", json={
        "robot_name": "r",
        "links": [{"id": "a", "name": "a", "body_ids": ["b"], "density_kg_m3": 1000.0}],
        "joints": [],
        "bodies_by_link": {"a": [_box_json("b")], "spare": [_box_json("s", "Ünterteil")]}})
    assert r.status_code == 200
    r.headers["X-Export-Warnings"].encode("ascii")
    assert "Ünterteil" in json.loads(r.headers["X-Export-Warnings"])[0]["message"]


def test_export_approximate_part_is_422_naming_link_and_part():
    body = dict(_box_json("b"), status="approximate")
    r = client.post("/export", json={
        "robot_name": "r",
        "links": [{"id": "a", "name": "a", "body_ids": ["b"], "density_kg_m3": 1000.0}],
        "joints": [], "bodies_by_link": {"a": [body]}})
    assert r.status_code == 422
    assert "'a'" in r.json()["detail"] and "'b'" in r.json()["detail"]
```

Add `import json` at the top of `test_end_to_end.py` if missing.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_urdf_writer.py tests/test_end_to_end.py -v`
Expected: FAIL with `ImportError: cannot import name 'unused_body_warnings'`

- [ ] **Step 3: Implement** (in `backend/app/export/urdf_writer.py`)

Add `PartWarning` to the models import, then replace `_with_mass_properties` and `export_package`:

```python
def _bbox_mass_properties(bodies: list[Body], mass: float):
    """Fallback when the user set a mass but the geometry has no measurable
    volume: uniform solid box over the vertex bounding box."""
    v = np.array([p for b in bodies for p in b.vertices], dtype=float)
    lo, hi = v.min(axis=0), v.max(axis=0)
    dx, dy, dz = hi - lo
    inertia = [mass / 12 * (dy**2 + dz**2), mass / 12 * (dx**2 + dz**2),
               mass / 12 * (dx**2 + dy**2), 0.0, 0.0, 0.0]
    return tuple(((lo + hi) / 2).tolist()), inertia


def _with_mass_properties(link: Link, bodies: list[Body]) -> Link:
    """Mass/COM/inertia always come from geometry + density, never the client.
    A user mass override rescales the computed inertia (uniform density)."""
    override = link.mass_override_kg
    try:
        mass, com, inertia = compute_link_mass_properties(bodies, link.density_kg_m3)
    except ValueError as exc:
        if override is None or not any(b.vertices for b in bodies):
            raise ValueError(f"Link '{link.id}': {exc}") from exc
        com, inertia = _bbox_mass_properties(bodies, override)
        mass = override
    else:
        if override is not None:
            ratio = override / mass
            mass, inertia = override, [x * ratio for x in inertia]
    return link.model_copy(update={"mass_kg": mass, "center_of_mass": com, "inertia": inertia})


def _part_status_errors(links: list[Link], bodies_by_link: dict[str, list[Body]]) -> list[str]:
    errors = []
    for l in links:
        for b in bodies_by_link.get(l.id, []):
            if b.status == "failed":
                errors.append(f"Link '{l.id}' contains part '{b.name}' that failed to import; "
                              "replace it or remove it from the link.")
            elif (b.status == "approximate" and l.mass_override_kg is None
                  and not l.accept_approximate_mass):
                errors.append(f"Link '{l.id}' contains approximate part '{b.name}'; set "
                              "mass_override_kg or accept_approximate_mass.")
    return errors


def unused_body_warnings(links: list[Link], bodies_by_link: dict[str, list[Body]]) -> list[PartWarning]:
    link_ids = {l.id for l in links}
    return [
        PartWarning(code="unused_body", fixes=[],
                    message=f"Part '{b.name}' is in no link and was not exported.")
        for key, bodies in bodies_by_link.items() if key not in link_ids
        for b in bodies
    ]


def export_package(
    robot_name: str,
    links: list[Link],
    joints: list[Joint],
    bodies_by_link: dict[str, list[Body]],
) -> bytes:
    errors = _part_status_errors(links, bodies_by_link)
    if errors:
        raise ValueError("Cannot export URDF -- " + "; ".join(errors))
    links = [_with_mass_properties(l, bodies_by_link.get(l.id, [])) for l in links]
    ... # rest unchanged (export_urdf + zip)
```

`backend/app/routes/export_routes.py` — import `json` and `unused_body_warnings`, and set the header:

```python
    warnings = unused_body_warnings(request.links, request.bodies_by_link)
    return Response(
        content=data,
        media_type="application/zip",
        headers={
            "Content-Disposition": f'attachment; filename="{request.robot_name}.zip"',
            "X-Export-Warnings": json.dumps([w.model_dump() for w in warnings]),  # ensure_ascii
        },
    )
```

- [ ] **Step 4: Run the full suite**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass, no warnings.

- [ ] **Step 5: Commit**

```bash
git add backend/app/export/urdf_writer.py backend/app/routes/export_routes.py backend/tests/test_urdf_writer.py backend/tests/test_end_to_end.py
git commit -m "feat: mass override, approximate-part guard, and unused-part warnings on export"
```

---

### Task 6: POST /replace-part

**Files:**
- Create: `backend/app/importers/placement.py`
- Create: `backend/app/routes/replace_routes.py`
- Modify: `backend/app/main.py`
- Test: `backend/tests/test_placement.py` (new), `backend/tests/test_end_to_end.py`

**Interfaces:**
- Consumes: `run_import` (Task 4), `Body`, `PartWarning`, `STATUS_ORDER` (Task 1).
- Produces:
  - `combine_bodies(bodies: list[Body], body_id: str, name: str) -> Body` — concatenates geometry (triangle and face indices shifted), status = worst, warnings = all, in order.
  - `place_replacement(body: Body, bbox_min: Vec3 | None, bbox_max: Vec3 | None, tol: float = 1e-4) -> Body` — placement rules from spec §4.2; adds `placed_by_bbox` (fixes `["replace"]`) or `position_unverified` (fixes `["replace"]`).
  - `POST /replace-part` multipart: `file`, `body_id` (str), `name` (str, default `"replacement"`), `bbox_min`/`bbox_max` (optional `"x,y,z"` strings, both or neither), `scale` (float, default 0.001). Returns one `Body` (JSON) with `id == body_id`. Malformed bbox → 422.

- [ ] **Step 1: Write the failing tests**

`backend/tests/test_placement.py`:

```python
import numpy as np
import trimesh
from app.importers.placement import combine_bodies, place_replacement
from app.models import Body, Face, PartWarning


def _box(bid, offset=(0, 0, 0), status="ok"):
    m = trimesh.creation.box(extents=(1, 1, 1)); m.apply_translation(offset)
    return Body(id=bid, name=bid, vertices=[tuple(v) for v in m.vertices.tolist()],
                triangles=[tuple(t) for t in m.faces.tolist()], status=status,
                faces=[Face(kind="plane", origin="segmented", normal=(0, 0, 1), center=(0, 0, 0),
                            residual=0.0, triangle_indices=[0, 1])])


def test_combine_shifts_indices_and_takes_worst_status():
    a, b = _box("a"), _box("b", (5, 0, 0), status="approximate")
    b = b.model_copy(update={"warnings": [PartWarning(code="open_geometry", message="m")]})
    c = combine_bodies([a, b], "orig", "part")
    assert c.id == "orig" and c.name == "part"
    assert len(c.vertices) == 16 and len(c.triangles) == 24
    assert c.faces[1].triangle_indices == [12, 13]
    assert max(i for t in c.triangles for i in t) == 15
    assert c.status == "approximate" and [w.code for w in c.warnings] == ["open_geometry"]


def test_overlapping_replacement_keeps_coordinates():
    r = place_replacement(_box("r", (0.2, 0, 0)), (-0.5, -0.5, -0.5), (0.5, 0.5, 0.5))
    assert np.allclose(np.array(r.vertices).min(0), [-0.3, -0.5, -0.5])
    assert r.warnings == []


def test_non_overlapping_replacement_is_centred_with_warning():
    r = place_replacement(_box("r", (100, 0, 0)), (9.5, -0.5, -0.5), (10.5, 0.5, 0.5))
    v = np.array(r.vertices)
    assert np.allclose((v.min(0) + v.max(0)) / 2, [10, 0, 0])
    assert np.allclose(r.faces[0].center, (-90, 0, 0))     # faces move with the body
    assert [w.code for w in r.warnings] == ["placed_by_bbox"]
    assert r.warnings[0].fixes == ["replace"]


def test_no_original_box_keeps_coordinates_unverified():
    r = place_replacement(_box("r", (100, 0, 0)), None, None)
    assert np.allclose(np.array(r.vertices).min(0), [99.5, -0.5, -0.5])
    assert [w.code for w in r.warnings] == ["position_unverified"]
```

Face centers move with the body: the box moves by `(-90, 0, 0)` (centre 100 → 10), so the fixture's face center at the origin moves to `(-90, 0, 0)`.

Append to `backend/tests/test_end_to_end.py`:

```python
def _replace(data: bytes, filename: str, **form):
    return client.post("/replace-part", files={"file": (filename, data)},
                       data={"body_id": "orig", "name": "part", **form})


def test_replace_part_keeps_original_id_and_places(box_stl_path):
    # box_mm.stl is 1000x1000x2000 mm centred at origin -> 1x1x2 m; original sat at x=10
    r = _replace(open(box_stl_path, "rb").read(), "box.stl",
                 bbox_min="9.5,-0.5,-1.0", bbox_max="10.5,0.5,1.0")
    assert r.status_code == 200
    body = r.json()
    assert body["id"] == "orig" and body["name"] == "part"
    xs = [v[0] for v in body["vertices"]]
    assert abs((min(xs) + max(xs)) / 2 - 10.0) < 1e-9
    assert [w["code"] for w in body["warnings"]] == ["placed_by_bbox"]


def test_replace_part_malformed_bbox_is_422(box_stl_path):
    """Review Focus: malformed form fields are 422, not 500."""
    data = open(box_stl_path, "rb").read()
    for bad in ({"bbox_min": "a,b", "bbox_max": "1,1,1"},
                {"bbox_min": "nan,0,0", "bbox_max": "1,1,1"},
                {"bbox_min": "0,0,0"}):
        assert _replace(data, "box.stl", **bad).status_code == 422


def test_replace_part_with_unmeshable_file_returns_failed_body(tmp_path):
    """Review Focus: a replacement that itself fails comes back as `failed`, not 500."""
    from OCC.Core.BRep import BRep_Builder
    from OCC.Core.BRepTools import breptools
    from OCC.Core.TopoDS import TopoDS_Compound
    comp = TopoDS_Compound(); BRep_Builder().MakeCompound(comp)
    p = str(tmp_path / "empty.brep"); breptools.Write(comp, p)
    r = _replace(open(p, "rb").read(), "empty.brep")
    assert r.status_code in (200, 422)
    if r.status_code == 200:
        assert r.json()["status"] == "failed" and r.json()["id"] == "orig"
    else:
        assert "empty.brep" in r.json()["detail"]
```

(An empty compound has no geometry at all, so `import_cad` raises "No geometry found" → 422 naming the file. Either outcome is a clean, named failure; the assertion pins "never 500".)

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_placement.py tests/test_end_to_end.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.importers.placement'`

- [ ] **Step 3: Implement**

`backend/app/importers/placement.py`:

```python
import numpy as np

from app.models import STATUS_ORDER, Body, PartWarning, Vec3


def combine_bodies(bodies: list[Body], body_id: str, name: str) -> Body:
    vertices, triangles, faces, warnings = [], [], [], []
    for b in bodies:
        v0, t0 = len(vertices), len(triangles)
        vertices += b.vertices
        triangles += [(a + v0, c + v0, d + v0) for a, c, d in b.triangles]
        faces += [f.model_copy(update={"triangle_indices": [i + t0 for i in f.triangle_indices]})
                  for f in b.faces]
        warnings += b.warnings
    status = max((b.status for b in bodies), key=STATUS_ORDER.get, default="failed")
    return Body(id=body_id, name=name, vertices=vertices, triangles=triangles,
                faces=faces, status=status, warnings=warnings)


def _translate(body: Body, d: np.ndarray) -> Body:
    shift = lambda p: tuple(float(x) for x in np.asarray(p) + d)
    return body.model_copy(update={
        "vertices": [shift(v) for v in body.vertices],
        "faces": [f.model_copy(update={"center": shift(f.center)}) for f in body.faces],
    })


def place_replacement(body: Body, bbox_min: Vec3 | None, bbox_max: Vec3 | None,
                      tol: float = 1e-4) -> Body:
    """Spec §4.2: keep coordinates if the replacement overlaps the original's box;
    otherwise centre it on the original; with no original box, keep and flag."""
    if bbox_min is None or bbox_max is None:
        w = PartWarning(code="position_unverified", fixes=["replace"],
                        message=f"{body.name}: original part had no geometry, so the "
                                "replacement's position could not be checked")
        return body.model_copy(update={"warnings": body.warnings + [w]})
    if not body.vertices:
        return body
    v = np.asarray(body.vertices, dtype=float)
    lo, hi = v.min(axis=0), v.max(axis=0)
    o_lo, o_hi = np.asarray(bbox_min, dtype=float), np.asarray(bbox_max, dtype=float)
    if np.all(lo <= o_hi + tol) and np.all(o_lo <= hi + tol):
        return body
    moved = _translate(body, (o_lo + o_hi) / 2 - (lo + hi) / 2)
    w = PartWarning(code="placed_by_bbox", fixes=["replace"],
                    message=f"{body.name}: placed by bounding-box centre; check its rotation")
    return moved.model_copy(update={"warnings": moved.warnings + [w]})
```

`backend/app/routes/replace_routes.py`:

```python
import math

from fastapi import APIRouter, Form, HTTPException, UploadFile

from app.importers.placement import combine_bodies, place_replacement
from app.routes.upload import run_import

router = APIRouter()


def _vec(s: str | None, field: str):
    if s is None:
        return None
    try:
        v = tuple(float(x) for x in s.split(","))
    except ValueError:
        raise HTTPException(422, f"{field} must be 'x,y,z' numbers")
    if len(v) != 3 or not all(math.isfinite(x) for x in v):
        raise HTTPException(422, f"{field} must be three finite numbers")
    return v


@router.post("/replace-part")
def replace_part(
    file: UploadFile,
    body_id: str = Form(...),
    name: str = Form("replacement"),
    bbox_min: str | None = Form(None),
    bbox_max: str | None = Form(None),
    scale: float = Form(0.001),
):
    lo, hi = _vec(bbox_min, "bbox_min"), _vec(bbox_max, "bbox_max")
    if (lo is None) != (hi is None):
        raise HTTPException(422, "bbox_min and bbox_max must be given together")
    bodies, _, _ = run_import(file, scale)
    body = place_replacement(combine_bodies(bodies, body_id, name), lo, hi)
    return body.model_dump()
```

`backend/app/main.py` — register it:

```python
from app.routes import export_routes, import_routes, joint_routes, replace_routes
...
app.include_router(replace_routes.router)
```

- [ ] **Step 4: Run the full suite**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass, no warnings.

- [ ] **Step 5: Commit**

```bash
git add backend/app/importers/placement.py backend/app/routes/replace_routes.py backend/app/main.py backend/tests/test_placement.py backend/tests/test_end_to_end.py
git commit -m "feat: POST /replace-part with bounding-box placement"
```

---

### Task 7: /detect-joints returns reason and evidence

**Files:**
- Modify: `backend/app/routes/joint_routes.py`
- Test: `backend/tests/test_end_to_end.py`

**Interfaces:**
- Consumes: `bodies_in_contact`, `find_matching_face_pairs`, `_PLANE_DISTANCE_TOLERANCE_DEFAULT` (from `app.joints.face_matching`), `classify_joint`.
- Produces: `POST /detect-joints` response
  `{"joint": Joint | null, "reason": "no_overlap" | "no_contact_faces" | null, "evidence": {"pairs": [{"kind": str, "radius": float | null}], "candidate_count": int, "tolerance": float} | null}`.

- [ ] **Step 1: Update and add tests** (in `backend/tests/test_end_to_end.py`)

Replace the two existing detect tests and update `_confirm_and_export`:

```python
def test_detect_joints_far_apart_bodies_returns_null(box_step_path):
    box = import_step(box_step_path)[0].model_dump()
    r = _detect(box, _shifted(box, 100.0))
    assert r.status_code == 200
    assert r.json() == {"joint": None, "reason": "no_overlap", "evidence": None}


def test_detect_joints_touching_bodies_returns_joint_with_evidence(box_step_path):
    box = import_step(box_step_path)[0].model_dump()  # 1 x 2 x 3, x in [0, 1]
    r = _detect(box, _shifted(box, 1.0))
    assert r.status_code == 200
    out = r.json()
    assert out["joint"]["type"] == "planar" and out["reason"] is None
    assert out["evidence"]["candidate_count"] == 1
    assert out["evidence"]["pairs"] == [{"kind": "plane", "radius": None}]
    assert out["evidence"]["tolerance"] == 1e-4


def test_detect_joints_overlapping_boxes_without_contact_faces_is_null(box_step_path):
    """Boxes overlap (bbox) but no faces mate: an offset box that overlaps in volume."""
    box = import_step(box_step_path)[0].model_dump()
    r = _detect(box, _shifted(box, 0.5))
    assert r.status_code == 200
    assert r.json() == {"joint": None, "reason": "no_contact_faces", "evidence": None}
```

In `_confirm_and_export`, change `joint = r.json()` to `joint = r.json()["joint"]`.

(Overlapping boxes shifted by 0.5 m have parallel same-direction faces, not opposing coplanar ones, so `find_matching_face_pairs` returns no pairs. If it does return pairs on this geometry, shift by `0.5` along x **and** `0.3` along y by extending `_shifted` with a `dy` parameter — the requirement is "boxes overlap, no mating faces".)

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_end_to_end.py -v -k detect`
Expected: FAIL — response is `null` / a bare joint, not the new shape.

- [ ] **Step 3: Implement** — replace `backend/app/routes/joint_routes.py`:

```python
from typing import Literal

from fastapi import APIRouter
from pydantic import BaseModel

from app.joints.face_matching import (
    _PLANE_DISTANCE_TOLERANCE_DEFAULT, bodies_in_contact, find_matching_face_pairs,
)
from app.joints.ipv_img import classify_joint
from app.models import Body, Joint

router = APIRouter()


class DetectJointsRequest(BaseModel):
    body_a: Body
    body_b: Body
    parent_link_id: str
    child_link_id: str


class PairEvidence(BaseModel):
    kind: str
    radius: float | None


class Evidence(BaseModel):
    pairs: list[PairEvidence]
    candidate_count: int
    tolerance: float


class DetectJointsResponse(BaseModel):
    joint: Joint | None
    reason: Literal["no_overlap", "no_contact_faces"] | None
    evidence: Evidence | None


@router.post("/detect-joints", response_model=DetectJointsResponse)
def detect_joints(request: DetectJointsRequest) -> DetectJointsResponse:
    tol = _PLANE_DISTANCE_TOLERANCE_DEFAULT
    if not bodies_in_contact(request.body_a, request.body_b, tol):
        return DetectJointsResponse(joint=None, reason="no_overlap", evidence=None)
    pairs = find_matching_face_pairs(request.body_a, request.body_b, tol)
    if not pairs:  # spec §9: explicit "no suggestion", not a guessed "fixed"
        return DetectJointsResponse(joint=None, reason="no_contact_faces", evidence=None)
    joint = classify_joint(pairs, request.parent_link_id, request.child_link_id)
    evidence = Evidence(
        pairs=[PairEvidence(kind=a.kind, radius=a.radius) for a, _ in pairs],
        candidate_count=len(pairs), tolerance=tol,
    )
    return DetectJointsResponse(joint=joint, reason=None, evidence=evidence)
```

- [ ] **Step 4: Run the full suite**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass, no warnings.

- [ ] **Step 5: Commit**

```bash
git add backend/app/routes/joint_routes.py backend/tests/test_end_to_end.py
git commit -m "feat: /detect-joints reports reason and evidence instead of guessing fixed"
```

---

### Task 8: STEP product names as part names (XCAF)

**Files:**
- Modify: `backend/app/importers/step_importer.py`
- Test: `backend/tests/test_step_importer.py`

**Interfaces:**
- Consumes: `import_cad` internals from Task 2 (`_subshapes`, `_sew_to_solids`, `_make_body`, accounting).
- Produces: for `.step`/`.stp`, `import_cad` reads through `STEPCAFControl_Reader` and names bodies after their STEP product (leaf label name). A product yielding one body gets the bare name; several bodies get `<name>_<k>` (k from 0). Products without a name fall back to Task 2 names. IGES and BREP are unchanged. Body order follows assembly order (depth-first over components); the existing peg-in-hole tests (block first, peg second) pin this.

- [ ] **Step 1: Write the failing test** (append to `backend/tests/test_step_importer.py`)

```python
def test_step_product_names_become_body_names(tmp_path):
    from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
    from OCC.Core.TDocStd import TDocStd_Document
    from OCC.Core.TCollection import TCollection_ExtendedString
    from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool
    from OCC.Core.TDataStd import TDataStd_Name
    from OCC.Core.STEPControl import STEPControl_AsIs

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    asm = tool.NewShape()
    TDataStd_Name.Set(asm, TCollection_ExtendedString("ASSEMBLY"))
    for nm, x in (("BASE_PLATE", 0.0), ("ARM_LINK", 3.0)):
        part = tool.AddShape(BRepPrimAPI_MakeBox(gp_Pnt(x, 0, 0), 1, 1, 1).Shape(), False)
        TDataStd_Name.Set(part, TCollection_ExtendedString(nm))
        tool.AddComponent(asm, part, TopLoc_Location())
    tool.UpdateAssemblies()
    w = STEPCAFControl_Writer()
    Interface_Static.SetCVal("xstep.cascade.unit", "MM")
    Interface_Static.SetCVal("write.step.unit", "M")
    w.Transfer(doc, STEPControl_AsIs)
    path = str(tmp_path / "named.step"); w.Write(path)

    bodies, warnings = import_cad(path)
    assert [b.name for b in bodies] == ["BASE_PLATE", "ARM_LINK"]
    assert warnings == []
    xs = [min(v[0] for v in b.vertices) for b in bodies]
    assert abs(xs[0] - 0.0) < 1e-6 and abs(xs[1] - 3.0) < 1e-6   # meters, placed
```

Add imports at the top of the test file as needed: `from OCC.Core.TopLoc import TopLoc_Location`, `from OCC.Core.Interface import Interface_Static`.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_step_importer.py -v -k product_names`
Expected: FAIL — names are `["solid_0", "solid_1"]`.

- [ ] **Step 3: Implement** (in `backend/app/importers/step_importer.py`)

Add a reader that returns named parts and route STEP through it:

```python
from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Core.TCollection import TCollection_ExtendedString
from OCC.Core.TDataStd import TDataStd_Name
from OCC.Core.TDF import TDF_Label, TDF_LabelSequence
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool


def _label_name(label) -> str | None:
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID(), attr):
        name = attr.Get().ToExtString().strip()
        return name or None
    return None


def _read_step_named(file_path: str) -> list[tuple[str | None, object]]:
    """[(product name, located shape)] for each leaf part, in assembly order, meters."""
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    Interface_Static.SetCVal("xstep.cascade.unit", "M")  # statics exist once a reader exists
    if reader.ReadFile(file_path) != IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("Could not read CAD file")
    tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    parts = []

    def walk(label, loc):
        if tool.IsAssembly(label):
            comps = TDF_LabelSequence()
            tool.GetComponents(label, comps)
            for i in range(1, comps.Length() + 1):
                comp = comps.Value(i)
                ref = TDF_Label()
                tool.GetReferredShape(comp, ref)
                walk(ref, loc.Multiplied(tool.GetLocation(comp)))
        else:
            parts.append((_label_name(label), tool.GetShape(label).Moved(loc)))

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location())
    return parts
```

Then restructure `import_cad` so the per-shape work (solids, loose shells, loose faces, accounting) runs per part:

```python
def _parts_of(shape) -> list[tuple[object, str, bool]]:
    parts = [(s, "solid", False) for s in _subshapes(shape, TopAbs_SOLID)]
    for sh in _subshapes(shape, TopAbs_SHELL, TopAbs_SOLID):
        parts += [(s, "shell", True) for s in _sew_to_solids([sh])]
    loose = _loose_faces(shape)
    if loose:
        parts += [(s, "surface", True) for s in _sew_to_solids(loose)]
    return parts


def import_cad(file_path, brep_scale=0.001):
    ...  # scale validation unchanged
    ext = os.path.splitext(file_path)[1].lower()
    named = (_read_step_named(file_path) if ext in (".step", ".stp")
             else [(None, _read_shape(file_path, brep_scale))])
    bodies, total_faces, accounted = [], 0, 0
    for product, shape in named:
        total_faces += len(_subshapes(shape, TopAbs_FACE))
        parts = _parts_of(shape)
        accounted += sum(len(_subshapes(s, TopAbs_FACE)) for s, _, _ in parts)
        for k, (s, prefix, repaired) in enumerate(parts):
            if product:
                name = product if len(parts) == 1 else f"{product}_{k}"
            else:
                name = f"{prefix}_{len(bodies)}"
            bodies.append(_make_body(s, name, repaired))
    ...  # unaccounted_faces warning and "No geometry" check unchanged
```

**Unit check (required before committing):** `STEPCAFControl_Reader` may scale to the XCAF document's length unit instead of `xstep.cascade.unit`. The existing tests `test_box_mesh_is_watertight_with_volume_6`, `test_step_declared_in_mm_imports_in_meters`, and the cylinder-radius tests catch a wrong unit. If they fail with values off by 1000, add before `reader.Transfer(doc)`:

```python
from OCC.Core.UnitsMethods import UnitsMethods_LengthUnit_Meter
XCAFDoc_DocumentTool.SetLengthUnit(doc, 1.0, UnitsMethods_LengthUnit_Meter)
```

and re-run. Do not change any expected value in an existing test to make it pass.

- [ ] **Step 4: Run the full suite**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass, no warnings. In particular the peg-in-hole tests (2 bodies, block first) and every STEP unit test pass unchanged.

- [ ] **Step 5: Check the real model** (manual, not committed)

Run (the ABB file is at `C:\Users\zohai\Downloads\irb1300_7_12kg-1400-ip40ip67cr-1.snapshot.1\irb1300_7_12kg-1400-ip40ip67cr-_asm.stp`):

```bash
cd backend && OPENBLAS_NUM_THREADS=1 ~/miniforge3/envs/cad-to-urdf/python.exe -c "
import sys; from app.importers.step_importer import import_cad
b, w = import_cad(sys.argv[1]); print(len(b), [x.code for x in w])
for x in b: print(x.name, x.status, len(x.triangles))" "<path above>" 2>/dev/null | grep -v '^\*'
```

Expected: ~18 bodies (14 solids + 4 sewn arm shells), no `unaccounted_faces`, the four large shells `repaired` or `approximate`. Paste the output into the task report.

- [ ] **Step 6: Commit**

```bash
git add backend/app/importers/step_importer.py backend/tests/test_step_importer.py
git commit -m "feat: name STEP parts after their product names"
```

---

### Task 9: Model corpus and regression scorecard

**Files:**
- Modify: `.gitignore` (repo root: add `corpus/`)
- Create: `backend/corpus/__init__.py` (empty)
- Create: `backend/corpus/generate.py`
- Create: `backend/corpus/run.py`
- Create: `backend/corpus/expected/peg_in_hole.yaml`, `open_shell.yaml`, `box_coarse.yaml`, `irb1300.yaml`
- Create: `backend/corpus/baseline.json`
- Test: `backend/tests/test_corpus_runner.py` (new)

**Interfaces:**
- Consumes: `import_path` (Task 4), `combine_bodies` (Task 6), `bodies_in_contact`/`find_matching_face_pairs` (existing), `classify_joint`, `compute_link_mass_properties`, `export_package` (Task 5), `Link`, `Joint`.
- Produces:
  - `python -m corpus.generate <out_dir>` — writes `peg_in_hole.step`, `open_shell.brep`, `box_coarse.stl`.
  - `python -m corpus.run [--corpus-dir DIR] [--expected-dir DIR] [--baseline FILE] [--report FILE] [--update-baseline] [--list-parts NAME]` — exit 0 when no model regressed, 1 otherwise.
  - Expectation keys: `file`, `sha256` (null or omitted for generated models), `generated` (bool), `scale`, `parts: {min}`, `statuses: {<status>: max}`, `links: [{name, bodies: [body names]}]`, `joints: [{parent, child, type: [..], axis: [x,y,z]}]`.
  - Score per model: `{"parts": int, "failed": int, "joints": int, "type_ok": int, "axis_ok": int, "warnings": int, "export_ok": bool, "seconds": float}`. Regression = parts, type_ok, axis_ok lower; failed higher; export_ok True→False (vs baseline; `seconds` and `warnings` not compared).
  - Scorecard line: `<name> | parts P/≥M | failed F | joints J/N type T/N axis A/N | warn W | export ok|FAIL | Ss`.

- [ ] **Step 1: Write the failing test** — `backend/tests/test_corpus_runner.py`:

```python
import json
import re
import shutil
from pathlib import Path

from corpus import generate, run

EXPECTED = Path(__file__).resolve().parents[1] / "corpus" / "expected"
LINE = re.compile(r"^\S+ \| parts \d+/≥\d+ \| failed \d+ \| joints \d+/\d+ type \d+/\d+ axis \d+/\d+ "
                  r"\| warn \d+ \| export (ok|FAIL) \| [\d.]+s$")


def test_scorecard_runs_and_detects_regression(tmp_path, capsys):
    corpus = tmp_path / "corpus"; generate.main([str(corpus)])
    baseline, report = tmp_path / "baseline.json", tmp_path / "report.json"
    args = ["--corpus-dir", str(corpus), "--expected-dir", str(EXPECTED),
            "--baseline", str(baseline), "--report", str(report)]

    assert run.main(args + ["--update-baseline"]) == 0
    out = capsys.readouterr().out
    scored = [l for l in out.splitlines() if " | parts " in l]
    assert len(scored) == 3 and all(LINE.match(l) for l in scored)   # irb1300 skipped (no file)
    assert "irb1300: skipped" in out
    data = json.loads(report.read_text())
    assert data["peg_in_hole"]["type_ok"] == 1 and data["peg_in_hole"]["export_ok"] is True

    assert run.main(args) == 0                                        # at baseline

    b = json.loads(baseline.read_text()); b["peg_in_hole"]["type_ok"] = 5
    baseline.write_text(json.dumps(b))
    assert run.main(args) == 1                                        # regressed
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_corpus_runner.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'corpus'`

- [ ] **Step 3: Implement the generator** — `backend/corpus/generate.py`:

```python
"""Write the always-available generated models into a corpus directory."""
import sys
from pathlib import Path

import app  # noqa: F401  (BLAS thread cap)
import trimesh
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCC.Core.BRepTools import breptools
from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCC.Core.Interface import Interface_Static
from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Writer
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Shell


def _peg_in_hole(path: Path) -> None:
    """Block 0.5x0.5x0.2 m with a r=0.1 through-hole; peg r=0.1 through it (cylindrical)."""
    hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0.25, 0.25, -0.1), gp_Dir(0, 0, 1)), 0.1, 0.4).Shape()
    block = BRepAlgoAPI_Cut(BRepPrimAPI_MakeBox(0.5, 0.5, 0.2).Shape(), hole).Shape()
    peg = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0.25, 0.25, -0.4), gp_Dir(0, 0, 1)), 0.1, 1.0).Shape()
    b, comp = BRep_Builder(), TopoDS_Compound()
    b.MakeCompound(comp); b.Add(comp, block); b.Add(comp, peg)
    w = STEPControl_Writer()
    Interface_Static.SetCVal("xstep.cascade.unit", "MM")
    Interface_Static.SetCVal("write.step.unit", "M")
    w.Transfer(comp, STEPControl_AsIs); w.Write(str(path))


def _open_shell(path: Path) -> None:
    """Meters (brep scale 1.0): a solid box plus a box shell missing one face."""
    b = BRep_Builder()
    shell = TopoDS_Shell(); b.MakeShell(shell)
    ex, n = TopExp_Explorer(BRepPrimAPI_MakeBox(gp_Pnt(3, 0, 0), 1, 1, 1).Shape(), TopAbs_FACE), 0
    while ex.More():
        if n < 5:
            b.Add(shell, ex.Current())
        n += 1; ex.Next()
    comp = TopoDS_Compound(); b.MakeCompound(comp)
    b.Add(comp, BRepPrimAPI_MakeBox(1, 1, 1).Shape()); b.Add(comp, shell)
    breptools.Write(comp, str(path))


def _box_coarse(path: Path) -> None:
    """Coarse 16-sided cylinder in mm (documents the coarse-mesh limit)."""
    trimesh.creation.cylinder(radius=50.0, height=200.0, sections=16).export(str(path))


def main(argv: list[str]) -> int:
    out = Path(argv[0]); out.mkdir(parents=True, exist_ok=True)
    _peg_in_hole(out / "peg_in_hole.step")
    _open_shell(out / "open_shell.brep")
    _box_coarse(out / "box_coarse.stl")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
```

- [ ] **Step 4: Write the generated-model expectations**

`backend/corpus/expected/peg_in_hole.yaml`:

```yaml
file: peg_in_hole.step
generated: true
parts: {min: 2}
statuses: {failed: 0, approximate: 0}
links:
  - {name: block, bodies: [solid_0]}
  - {name: peg, bodies: [solid_1]}
joints:
  - {parent: block, child: peg, type: [cylindrical, revolute], axis: [0, 0, 1]}
```

(After Task 8, unnamed products keep `solid_N` names. If the generated STEP gains product names from the writer, run `python -m corpus.run --corpus-dir <dir> --list-parts peg_in_hole` and use the printed names.)

`backend/corpus/expected/open_shell.yaml`:

```yaml
file: open_shell.brep
generated: true
scale: 1.0
parts: {min: 2}
statuses: {failed: 0}
```

`backend/corpus/expected/box_coarse.yaml`:

```yaml
file: box_coarse.stl
generated: true
scale: 0.001
parts: {min: 1}
statuses: {failed: 0}
```

`backend/corpus/expected/irb1300.yaml` (placeholder; Step 7 fills it in — the test expects it to be skipped when the file is absent):

```yaml
file: irb1300_asm.stp
```

- [ ] **Step 5: Implement the runner** — `backend/corpus/run.py`:

```python
"""Run every model with expectations through the pipeline and score it.
Usage: python -m corpus.run [--corpus-dir DIR] [--expected-dir DIR] [--baseline FILE]
                            [--report FILE] [--update-baseline] [--list-parts NAME]"""
import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import app  # noqa: F401  (BLAS thread cap)
import numpy as np
import yaml

from app.export.urdf_writer import export_package
from app.importers.pipeline import import_path
from app.importers.placement import combine_bodies
from app.joints.face_matching import bodies_in_contact, find_matching_face_pairs
from app.joints.ipv_img import classify_joint
from app.models import Joint, Link

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
LOWER_IS_WORSE = ("parts", "type_ok", "axis_ok")
_AXIS_TOL_DEG = 5.0


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _axis_ok(got, want) -> bool:
    if got is None:
        return False
    a, b = np.asarray(got, float), np.asarray(want, float)
    cos = abs(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return math.degrees(math.acos(min(1.0, cos))) <= _AXIS_TOL_DEG


def _confirmed(joint: Joint, exp: dict) -> Joint:
    """Model the user confirming: first expected type, detected axis/origin."""
    t = exp["type"][0]
    extra = {"limit_lower": -math.pi, "limit_upper": math.pi} if t in ("revolute", "prismatic") else {}
    axis = joint.axis or tuple(exp["axis"])
    return joint.model_copy(update={"type": t, "source": "confirmed", "axis": axis, **extra})


def score_model(exp: dict, path: Path) -> dict:
    t0 = time.time()
    bodies, _, file_warnings = import_path(str(path), path.suffix.lower(), exp.get("scale", 0.001))
    by_name = {b.name: b for b in bodies}
    s = {"parts": len(bodies), "failed": sum(b.status == "failed" for b in bodies),
         "joints": 0, "type_ok": 0, "axis_ok": 0,
         "warnings": len(file_warnings) + sum(len(b.warnings) for b in bodies),
         "export_ok": False, "seconds": 0.0}
    links = {l["name"]: combine_bodies([by_name[n] for n in l["bodies"] if n in by_name],
                                       l["name"], l["name"]) for l in exp.get("links", [])}
    joints = []
    for j in exp.get("joints", []):
        a, b = links.get(j["parent"]), links.get(j["child"])
        if a is None or b is None or not bodies_in_contact(a, b):
            continue
        pairs = find_matching_face_pairs(a, b)
        if not pairs:
            continue
        got = classify_joint(pairs, j["parent"], j["child"])
        s["joints"] += 1
        s["type_ok"] += got.type in j["type"]
        s["axis_ok"] += _axis_ok(got.axis, j["axis"])
        joints.append(_confirmed(got, j))
    if links:
        model_links = [Link(id=n, name=n, body_ids=[n], density_kg_m3=2700.0,
                            accept_approximate_mass=True) for n in links]
        try:
            export_package("corpus_robot", model_links, joints, {n: [b] for n, b in links.items()})
            s["export_ok"] = True
        except ValueError as exc:
            s["export_error"] = str(exc)[:200]
    else:
        s["export_ok"] = True  # nothing to export for parts-only expectations
    s["seconds"] = round(time.time() - t0, 1)
    return s


def _line(name: str, exp: dict, s: dict) -> str:
    n = len(exp.get("joints", []))
    return (f"{name} | parts {s['parts']}/≥{exp.get('parts', {}).get('min', 0)} | failed {s['failed']} "
            f"| joints {s['joints']}/{n} type {s['type_ok']}/{n} axis {s['axis_ok']}/{n} "
            f"| warn {s['warnings']} | export {'ok' if s['export_ok'] else 'FAIL'} | {s['seconds']}s")


def _regressions(name: str, s: dict, base: dict | None) -> list[str]:
    if not base:
        return []
    out = [f"{name}: {k} {base[k]} -> {s[k]}" for k in LOWER_IS_WORSE if s[k] < base[k]]
    if s["failed"] > base["failed"]:
        out.append(f"{name}: failed {base['failed']} -> {s['failed']}")
    if base["export_ok"] and not s["export_ok"]:
        out.append(f"{name}: export ok -> FAIL")
    return out


def _expectation_failures(name: str, exp: dict, s: dict) -> list[str]:
    out = []
    if s["parts"] < exp.get("parts", {}).get("min", 0):
        out.append(f"{name}: parts {s['parts']} < min {exp['parts']['min']}")
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--corpus-dir", default=str(REPO / "corpus"))
    p.add_argument("--expected-dir", default=str(HERE / "expected"))
    p.add_argument("--baseline", default=str(HERE / "baseline.json"))
    p.add_argument("--report", default=str(REPO / "corpus" / "report.json"))
    p.add_argument("--update-baseline", action="store_true")
    p.add_argument("--list-parts")
    a = p.parse_args(argv)
    corpus, baseline_path = Path(a.corpus_dir), Path(a.baseline)

    if a.list_parts:
        exp = yaml.safe_load((Path(a.expected_dir) / f"{a.list_parts}.yaml").read_text())
        path = corpus / exp["file"]
        bodies, _, _ = import_path(str(path), path.suffix.lower(), exp.get("scale", 0.001))
        for b in bodies:
            v = np.asarray(b.vertices) if b.vertices else np.zeros((1, 3))
            print(f"{b.name} | {b.status} | tris {len(b.triangles)} | "
                  f"bbox {v.min(0).round(3).tolist()}..{v.max(0).round(3).tolist()}")
        return 0

    baseline = json.loads(baseline_path.read_text()) if baseline_path.exists() else {}
    scores, problems = {}, []
    for exp_file in sorted(Path(a.expected_dir).glob("*.yaml")):
        name, exp = exp_file.stem, yaml.safe_load(exp_file.read_text())
        path = corpus / exp["file"]
        if not path.exists():
            print(f"{name}: skipped (file {exp['file']} not in {corpus})")
            continue
        if not exp.get("generated") and exp.get("sha256") and _sha256(path) != exp["sha256"]:
            print(f"{name}: skipped (sha256 mismatch)")
            continue
        try:
            s = score_model(exp, path)
        except Exception as exc:  # a crashing model is a result, not a runner crash
            s = {"parts": 0, "failed": 0, "joints": 0, "type_ok": 0, "axis_ok": 0,
                 "warnings": 0, "export_ok": False, "seconds": 0.0,
                 "error": f"{type(exc).__name__}: {exc}"[:200]}
        scores[name] = s
        print(_line(name, exp, s))
        problems += _regressions(name, s, baseline.get(name)) + _expectation_failures(name, exp, s)

    Path(a.report).parent.mkdir(parents=True, exist_ok=True)
    Path(a.report).write_text(json.dumps(scores, indent=2))
    if a.update_baseline:
        baseline.update({k: {kk: v[kk] for kk in ("parts", "failed", "type_ok", "axis_ok", "export_ok")}
                         for k, v in scores.items()})
        baseline_path.write_text(json.dumps(baseline, indent=2, sort_keys=True))
        print(f"baseline updated: {baseline_path}")
        return 0
    for line in problems:
        print("REGRESSION " + line)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest tests/test_corpus_runner.py -v`
Expected: PASS. If `peg_in_hole` body names differ from `solid_0`/`solid_1`, fix the YAML using `--list-parts` (Step 4 note), not the test.

- [ ] **Step 7: Add the ABB model and its expectations**

Add `corpus/` to the repo-root `.gitignore`. Copy the model:

```bash
mkdir -p corpus && cp "/c/Users/zohai/Downloads/irb1300_7_12kg-1400-ip40ip67cr-1.snapshot.1/irb1300_7_12kg-1400-ip40ip67cr-_asm.stp" corpus/irb1300_asm.stp
sha256sum corpus/irb1300_asm.stp
```

List its parts (the placeholder `irb1300.yaml` from Step 4 is enough for `--list-parts`):

```bash
cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m corpus.run --list-parts irb1300 2>/dev/null | grep "|"
```

Map body names to links by bounding box (meters, from the 2026-10-08 probe):

| link | body bbox (min z .. max z) | note |
|---|---|---|
| base | 0.000 .. 0.254 (≈71k tris) **and** -0.460 .. 0.000 (pedestal) | two bodies |
| link1 | 0.252 .. 0.630 (≈32k tris) | sewn shell |
| link2 | 0.454 .. 1.038 (≈40k tris) | sewn shell |
| link3 | 0.864 .. 1.096 (≈15k tris) | sewn shell |
| link4 | 0.979 .. 1.237 (≈34k tris) | sewn shell |
| link5 | 1.138 .. 1.247 (≈15k tris) | wrist solid |
| link6 | 1.186 .. 1.236 (≈1.2k tris) | flange solid |

Small screw/pin bodies are left out of every link. Write `backend/corpus/expected/irb1300.yaml` with the real names from `--list-parts` in place of the `<...>` placeholders and the hash from `sha256sum`:

```yaml
file: irb1300_asm.stp
sha256: <sha256 from above>
parts: {min: 18}
statuses: {failed: 0}
links:
  - {name: base,  bodies: [<base body>, <pedestal body>]}
  - {name: link1, bodies: [<link1 body>]}
  - {name: link2, bodies: [<link2 body>]}
  - {name: link3, bodies: [<link3 body>]}
  - {name: link4, bodies: [<link4 body>]}
  - {name: link5, bodies: [<link5 body>]}
  - {name: link6, bodies: [<link6 body>]}
joints:
  - {parent: base,  child: link1, type: [revolute, planar],             axis: [0, 0, 1]}
  - {parent: link1, child: link2, type: [revolute, planar],             axis: [-0.712, 0.702, 0]}
  - {parent: link2, child: link3, type: [revolute, planar],             axis: [-0.712, 0.702, 0]}
  - {parent: link3, child: link4, type: [revolute, cylindrical],        axis: [-0.676, -0.685, -0.272]}
  - {parent: link4, child: link5, type: [revolute, planar],             axis: [-0.712, 0.702, 0]}
  - {parent: link5, child: link6, type: [revolute, cylindrical, continuous], axis: [-0.676, -0.685, -0.272]}
```

These `<...>` values are data the implementer reads from the command output; they are not code placeholders.

- [ ] **Step 8: Record the baseline over the whole corpus**

```bash
cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m corpus.generate ../corpus
~/miniforge3/envs/cad-to-urdf/python.exe -m corpus.run --update-baseline
~/miniforge3/envs/cad-to-urdf/python.exe -m corpus.run; echo "exit=$?"
```

Expected: four scorecard lines; `irb1300` shows `parts ≥18`, `joints 6/6`, `axis 6/6` (probe result), `export ok`; second run `exit=0`. Paste the scorecard into the task report. If any irb1300 joint is not detected, report it as a finding — do not edit expectations to hide it.

- [ ] **Step 9: Run the full suite**

Run: `cd backend && ~/miniforge3/envs/cad-to-urdf/python.exe -m pytest -q`
Expected: all pass, no warnings.

- [ ] **Step 10: Commit**

```bash
git add .gitignore backend/corpus backend/tests/test_corpus_runner.py
git commit -m "feat: model corpus with expectations and regression scorecard"
```
