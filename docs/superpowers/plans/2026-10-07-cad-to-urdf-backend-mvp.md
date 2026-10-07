# CAD-to-URDF Backend MVP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the Python backend that turns an uploaded CAD/mesh assembly into a validated, exported URDF — the must-have scope from the spec (§2), excluding the frontend and all nice-to-have items.

**Architecture:** A stateless FastAPI service. Two import paths (STEP/IGES/BREP via pythonocc-core giving exact analytic faces; STL/OBJ via trimesh giving raw mesh that then goes through region-growing + primitive fitting to reconstruct faces) converge on one shared `Face` representation, so one joint classifier (face-pair matching + the IPV/IMG lookup table) serves both. A structural validation gate and a URDF+mesh-zip exporter close the loop.

**Tech Stack:** Python 3.11, FastAPI, pythonocc-core 7.8.x, trimesh (with `numpy`/`scipy` for fitting math), pytest.

**Spec:** `docs/superpowers/specs/2026-10-07-cad-to-urdf-design.md`

## Global Constraints

- All lengths are meters internally, matching URDF. STEP files carry their own unit in the file header — read and convert to meters at import, never assume. STL/OBJ carry no unit metadata at all — assume millimeters (the common CAD-export convention) and scale by 0.001, but accept an optional `scale` override on the import request so a user whose file is already in meters isn't silently corrupted.
- Every `Body`/`Link`/`Joint` field name and shape defined in Task 1 is final — later tasks must not rename or reshape them; if a later task needs a new field, add it, don't repurpose an existing one.
- Face `origin` is always exactly `"brep"` or `"segmented"` — no other value, checked by test in every task that produces a `Face`.
- No task silently guesses: a face-pair match with poor confidence, or no match at all, returns an explicit empty/low-confidence result, never a best-effort placement presented as certain.
- Every function that computes geometry must be tested against a known analytic shape (box/cylinder/sphere) with an expected numeric answer, not just "it ran without error."

## Review Focus

- **STL/OBJ with no unit metadata, modeled at a different scale than mm** (e.g. a hobbyist exported in meters or inches) — the default 0.001 scale silently produces a robot 1000x too small; the import response must say what scale it assumed so a wrong guess is visible, not hidden. (Task 3)
- **Non-manifold or open (non-watertight) mesh input** — common from careless STL exports (e.g. 3D-print-oriented files with internal cavities or gaps). Mass/inertia integration and region growing both assume a closed solid; feeding them a bad mesh must produce a clear error, not a silently-wrong mass or a crash. (Task 3, Task 6)
- **Two solids in the same assembly file that are simply not in contact** (unrelated far-apart parts, or two parts that happen to share a bounding-box overlap without any real touching face). Face-pair matching must return no match, not the nearest-available-but-wrong pair. (Task 7)
- **Multiple equally-valid concentric cylindrical face pairs between the same two bodies** (e.g. a bracket fastened by four parallel dowel pins). Matching must not silently pick an arbitrary one; it must pick the most-constraining (largest-radius or largest-contact-area) pair and flag that more than one candidate existed. (Task 7)
- **A link with genuinely zero or negative computed volume** (degenerate geometry, an open shell mistakenly treated as a solid, or a body accidentally left out of a link group). The structural gate must reject this with a specific reason before it ever reaches URDF export, not let a zero-inertia link through. (Task 6, Task 9)

---

## File Structure

```
backend/
  pyproject.toml
  app/
    __init__.py
    main.py                    # FastAPI app + route registration
    models.py                  # Body, Face, Link, Joint pydantic models
    importers/
      __init__.py
      step_importer.py         # pythonocc-core: STEP/IGES/BREP -> Body list, brep faces
      mesh_importer.py         # trimesh: STL/OBJ -> Body, no faces yet
    segment/
      __init__.py
      region_growing.py        # trimesh face_adjacency + angle threshold -> candidate regions
      primitive_fit.py         # region -> Face (plane/cylinder/sphere fit + residual)
    joints/
      __init__.py
      face_matching.py         # coincident/concentric face-pair search between two Bodies
      ipv_img.py                # face-pair set -> Joint (type, origin, axis)
    massprops/
      __init__.py
      link_mass.py             # Body meshes + density -> mass, center_mass, inertia tensor
    validate/
      __init__.py
      structural.py             # tree checks: one root, no cycles, no orphans, nonzero inertia
    export/
      __init__.py
      urdf_writer.py            # Link/Joint model -> URDF XML + mesh zip
    routes/
      __init__.py
      import_routes.py          # POST /import
      joint_routes.py           # POST /detect-joints
      export_routes.py          # POST /export
  tests/
    __init__.py
    conftest.py                 # shared fixtures: programmatically-built box/cylinder/sphere
                                 # solids (pythonocc-core primitives + trimesh.creation),
                                 # and a constructed two-part "hinge" and "slider" assembly
    test_models.py
    test_step_importer.py
    test_mesh_importer.py
    test_region_growing.py
    test_primitive_fit.py
    test_link_mass.py
    test_face_matching.py
    test_ipv_img.py
    test_structural.py
    test_urdf_writer.py
    test_end_to_end.py
```

---

### Task 1: Project scaffold and shared data models

**Files:**
- Create: `backend/pyproject.toml`
- Create: `backend/app/__init__.py`
- Create: `backend/app/models.py`
- Create: `backend/app/main.py`
- Test: `backend/tests/test_models.py`
- Test: `backend/tests/__init__.py` (empty)

**Interfaces:**
- Produces: `Vec3` (tuple of 3 floats), `Face` (fields: `kind: Literal["plane","cylinder","sphere","unclassified"]`, `origin: Literal["brep","segmented"]`, `normal: Vec3 | None`, `axis: Vec3 | None`, `center: Vec3`, `radius: float | None`, `residual: float`, `triangle_indices: list[int]`), `Body` (fields: `id: str`, `name: str`, `vertices: list[Vec3]`, `triangles: list[tuple[int,int,int]]`, `faces: list[Face]`), `Link` (fields: `id: str`, `name: str`, `body_ids: list[str]`, `density_kg_m3: float`, `mass_kg: float | None`, `center_of_mass: Vec3 | None`, `inertia: list[float] | None` as `[ixx,iyy,izz,ixy,ixz,iyz]`), `Joint` (fields: `id: str`, `parent_link_id: str`, `child_link_id: str`, `type: Literal["revolute","continuous","prismatic","cylindrical","planar","spherical","fixed"]`, `origin_xyz: Vec3`, `origin_rpy: Vec3`, `axis: Vec3 | None`, `limit_lower: float | None`, `limit_upper: float | None`, `source: Literal["suggested","confirmed","manual"]`).

- [ ] **Step 1: Write the failing test for the data models**

```python
# backend/tests/test_models.py
from app.models import Face, Body, Link, Joint

def test_face_requires_valid_origin():
    face = Face(
        kind="plane", origin="brep", normal=(0, 0, 1), axis=None,
        center=(0, 0, 0), radius=None, residual=0.0, triangle_indices=[0, 1],
    )
    assert face.origin == "brep"

def test_face_rejects_invalid_origin():
    import pytest
    from pydantic import ValidationError
    with pytest.raises(ValidationError):
        Face(
            kind="plane", origin="guessed", normal=(0, 0, 1), axis=None,
            center=(0, 0, 0), radius=None, residual=0.0, triangle_indices=[0],
        )

def test_body_holds_faces():
    body = Body(id="b1", name="box", vertices=[(0, 0, 0)], triangles=[], faces=[])
    assert body.id == "b1"
    assert body.faces == []

def test_link_defaults_unset_mass_properties_to_none():
    link = Link(id="l1", name="base", body_ids=["b1"], density_kg_m3=1000.0)
    assert link.mass_kg is None
    assert link.center_of_mass is None

def test_joint_defaults_source_to_manual_when_unspecified_is_rejected():
    import pytest
    from pydantic import ValidationError
    with pytest.raises(ValidationError):
        Joint(
            id="j1", parent_link_id="l1", child_link_id="l2", type="revolute",
            origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0), axis=(0, 0, 1),
            limit_lower=None, limit_upper=None,
        )  # missing required `source`
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_models.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app'` (or `app.models` does not exist yet)

- [ ] **Step 3: Write the models**

```python
# backend/app/models.py
from typing import Literal
from pydantic import BaseModel

Vec3 = tuple[float, float, float]

class Face(BaseModel):
    kind: Literal["plane", "cylinder", "sphere", "unclassified"]
    origin: Literal["brep", "segmented"]
    normal: Vec3 | None = None
    axis: Vec3 | None = None
    center: Vec3
    radius: float | None = None
    residual: float
    triangle_indices: list[int]

class Body(BaseModel):
    id: str
    name: str
    vertices: list[Vec3]
    triangles: list[tuple[int, int, int]]
    faces: list[Face] = []

class Link(BaseModel):
    id: str
    name: str
    body_ids: list[str]
    density_kg_m3: float
    mass_kg: float | None = None
    center_of_mass: Vec3 | None = None
    inertia: list[float] | None = None  # [ixx, iyy, izz, ixy, ixz, iyz]

class Joint(BaseModel):
    id: str
    parent_link_id: str
    child_link_id: str
    type: Literal[
        "revolute", "continuous", "prismatic", "cylindrical",
        "planar", "spherical", "fixed",
    ]
    origin_xyz: Vec3
    origin_rpy: Vec3
    axis: Vec3 | None = None
    limit_lower: float | None = None
    limit_upper: float | None = None
    source: Literal["suggested", "confirmed", "manual"]
```

```python
# backend/app/main.py
from fastapi import FastAPI

app = FastAPI(title="cad-to-urdf")

@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}
```

```toml
# backend/pyproject.toml
[project]
name = "cad-to-urdf-backend"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = [
    "fastapi>=0.110",
    "uvicorn>=0.29",
    "pydantic>=2.6",
    "pythonocc-core==7.8.1",
    "trimesh>=4.2",
    "numpy>=1.26",
    "scipy>=1.12",
]

[project.optional-dependencies]
dev = ["pytest>=8.0", "httpx>=0.27"]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_models.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: Add and verify the health route, then commit**

Run: `cd backend && python -c "from app.main import app; from fastapi.testclient import TestClient; print(TestClient(app).get('/health').json())"`
Expected output: `{'status': 'ok'}`

```bash
git add backend/pyproject.toml backend/app backend/tests
git commit -m "feat: scaffold FastAPI backend and core data models"
```

---

### Task 2: STEP/IGES/BREP importer (exact analytic faces)

**Files:**
- Create: `backend/app/importers/__init__.py`
- Create: `backend/app/importers/step_importer.py`
- Create: `backend/tests/conftest.py`
- Test: `backend/tests/test_step_importer.py`

**Interfaces:**
- Consumes: `Body`, `Face`, `Vec3` from `app.models` (Task 1).
- Produces: `import_step(file_path: str) -> list[Body]` in `app.importers.step_importer`. Each returned `Body` has `faces` populated with `origin="brep"`, exact `kind`/`normal`/`axis`/`center`/`radius` from OCCT, and `residual=0.0` (exact, not fitted).

- [ ] **Step 1: Add shared test fixtures that build STEP files programmatically**

```python
# backend/tests/conftest.py
import pytest
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere,
)
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.gp import gp_Trsf, gp_Vec

def _write_step(shape, path: str) -> None:
    writer = STEPControl_Writer()
    writer.Transfer(shape, STEPControl_AsIs)
    writer.Write(path)

@pytest.fixture
def box_step_path(tmp_path) -> str:
    shape = BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape()  # 1m x 2m x 3m box
    path = str(tmp_path / "box.step")
    _write_step(shape, path)
    return path

@pytest.fixture
def cylinder_step_path(tmp_path) -> str:
    shape = BRepPrimAPI_MakeCylinder(0.5, 2.0).Shape()  # radius 0.5m, height 2m
    path = str(tmp_path / "cylinder.step")
    _write_step(shape, path)
    return path

@pytest.fixture
def sphere_step_path(tmp_path) -> str:
    shape = BRepPrimAPI_MakeSphere(0.75).Shape()  # radius 0.75m
    path = str(tmp_path / "sphere.step")
    _write_step(shape, path)
    return path

@pytest.fixture
def peg_in_hole_step_path(tmp_path) -> str:
    """Two solids sharing a concentric cylindrical mate, no perpendicular
    planar lock -- should classify as a cylindrical joint (§6a)."""
    from OCC.Core.TopoDS import TopoDS_Compound
    from OCC.Core.BRep import BRep_Builder

    peg = BRepPrimAPI_MakeCylinder(0.1, 1.0).Shape()  # radius 0.1m, height 1m
    hole_block = BRepPrimAPI_MakeBox(0.5, 0.5, 0.2).Shape()
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(0.0, 0.0, 0.2))  # peg sits flush on top of block
    peg_placed = BRepBuilderAPI_Transform(peg, trsf, True).Shape()

    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    builder.Add(compound, hole_block)
    builder.Add(compound, peg_placed)

    path = str(tmp_path / "peg_in_hole.step")
    _write_step(compound, path)
    return path
```

- [ ] **Step 2: Write the failing importer tests**

```python
# backend/tests/test_step_importer.py
import math
from app.importers.step_importer import import_step

def test_box_import_has_six_planar_brep_faces(box_step_path):
    bodies = import_step(box_step_path)
    assert len(bodies) == 1
    faces = bodies[0].faces
    assert len(faces) == 6
    assert all(f.kind == "plane" for f in faces)
    assert all(f.origin == "brep" for f in faces)
    assert all(f.residual == 0.0 for f in faces)

def test_cylinder_import_has_correct_radius_and_axis(cylinder_step_path):
    bodies = import_step(cylinder_step_path)
    faces = bodies[0].faces
    cyl_faces = [f for f in faces if f.kind == "cylinder"]
    assert len(cyl_faces) == 1
    assert math.isclose(cyl_faces[0].radius, 0.5, rel_tol=1e-6)
    # axis should be parallel to Z (up to sign)
    ax = cyl_faces[0].axis
    assert math.isclose(abs(ax[2]), 1.0, rel_tol=1e-6)

def test_sphere_import_has_correct_radius(sphere_step_path):
    bodies = import_step(sphere_step_path)
    faces = bodies[0].faces
    sph_faces = [f for f in faces if f.kind == "sphere"]
    assert len(sph_faces) == 1
    assert math.isclose(sph_faces[0].radius, 0.75, rel_tol=1e-6)

def test_peg_in_hole_import_returns_two_bodies(peg_in_hole_step_path):
    bodies = import_step(peg_in_hole_step_path)
    assert len(bodies) == 2
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_step_importer.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.importers.step_importer'`

- [ ] **Step 4: Implement the STEP importer**

```python
# backend/app/importers/step_importer.py
import uuid
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Sphere
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopLoc import TopLoc_Location

from app.models import Body, Face, Vec3

_LINEAR_DEFLECTION = 0.001  # meters; fine enough for hobbyist-scale parts


def _read_shape(file_path: str):
    reader = STEPControl_Reader()
    status = reader.ReadFile(file_path)
    if status != IFSelect_RetDone:
        raise ValueError(f"Could not read STEP file: {file_path}")
    reader.TransferRoots()
    return reader.OneShape()


def _face_to_model(face, triangle_indices: list[int]) -> Face:
    adaptor = BRepAdaptor_Surface(face, True)
    surface_type = adaptor.GetType()

    if surface_type == GeomAbs_Plane:
        plane = adaptor.Plane()
        loc = plane.Location()
        normal = plane.Axis().Direction()
        return Face(
            kind="plane", origin="brep",
            normal=(normal.X(), normal.Y(), normal.Z()),
            axis=None, center=(loc.X(), loc.Y(), loc.Z()),
            radius=None, residual=0.0, triangle_indices=triangle_indices,
        )

    if surface_type == GeomAbs_Cylinder:
        cyl = adaptor.Cylinder()
        ax1 = cyl.Axis()
        loc = ax1.Location()
        direction = ax1.Direction()
        return Face(
            kind="cylinder", origin="brep", normal=None,
            axis=(direction.X(), direction.Y(), direction.Z()),
            center=(loc.X(), loc.Y(), loc.Z()),
            radius=cyl.Radius(), residual=0.0,
            triangle_indices=triangle_indices,
        )

    if surface_type == GeomAbs_Sphere:
        sph = adaptor.Sphere()
        loc = sph.Location()
        return Face(
            kind="sphere", origin="brep", normal=None, axis=None,
            center=(loc.X(), loc.Y(), loc.Z()), radius=sph.Radius(),
            residual=0.0, triangle_indices=triangle_indices,
        )

    return Face(
        kind="unclassified", origin="brep", normal=None, axis=None,
        center=(0.0, 0.0, 0.0), radius=None, residual=0.0,
        triangle_indices=triangle_indices,
    )


def _extract_triangles(solid, triangle_offset: int):
    """Triangulate the solid and return (vertices, triangles, per-face
    triangle index ranges) with triangle indices offset to be unique
    within the whole Body."""
    BRepMesh_IncrementalMesh(solid, _LINEAR_DEFLECTION)
    vertices: list[Vec3] = []
    triangles: list[tuple[int, int, int]] = []
    face_triangle_ranges: list[tuple[object, list[int]]] = []

    face_explorer = TopExp_Explorer(solid, TopAbs_FACE)
    while face_explorer.More():
        face = face_explorer.Current()
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation(face, location)
        this_face_indices: list[int] = []
        if triangulation is not None:
            vertex_offset = len(vertices)
            transform = location.Transformation()
            for i in range(1, triangulation.NbNodes() + 1):
                pnt = triangulation.Node(i).Transformed(transform)
                vertices.append((pnt.X(), pnt.Y(), pnt.Z()))
            for i in range(1, triangulation.NbTriangles() + 1):
                tri = triangulation.Triangle(i)
                a, b, c = tri.Get()
                idx = (
                    vertex_offset + a - 1 + triangle_offset,
                    vertex_offset + b - 1 + triangle_offset,
                    vertex_offset + c - 1 + triangle_offset,
                )
                triangles.append(idx)
                this_face_indices.append(len(triangles) - 1 + triangle_offset)
        face_triangle_ranges.append((face, this_face_indices))
        face_explorer.Next()

    return vertices, triangles, face_triangle_ranges


def import_step(file_path: str) -> list[Body]:
    shape = _read_shape(file_path)
    bodies: list[Body] = []

    solid_explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while solid_explorer.More():
        solid = solid_explorer.Current()
        vertices, triangles, face_ranges = _extract_triangles(solid, 0)
        faces = [
            _face_to_model(face, indices)
            for face, indices in face_ranges
            if indices
        ]
        bodies.append(
            Body(
                id=str(uuid.uuid4()), name=f"solid_{len(bodies)}",
                vertices=vertices, triangles=triangles, faces=faces,
            )
        )
        solid_explorer.Next()

    if not bodies:
        raise ValueError(f"No solids found in {file_path}")
    return bodies
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_step_importer.py -v`
Expected: PASS (4 passed). If a pythonocc-core API in Step 4 doesn't match your installed `7.8.1` wheel exactly (OCCT triangulation accessors have changed across releases), check the installed version's `OCC.Core.Poly.Poly_Triangulation` docstring (`python -c "from OCC.Core.Poly import Poly_Triangulation; help(Poly_Triangulation)"`) and adjust the node/triangle accessor calls in `_extract_triangles` to match — the surface-type classification in `_face_to_model` is stable across versions and should not need changes.

- [ ] **Step 6: Commit**

```bash
git add backend/app/importers backend/tests/conftest.py backend/tests/test_step_importer.py
git commit -m "feat: import STEP/IGES/BREP assemblies with exact analytic faces"
```

---

### Task 3: STL/OBJ importer with unit scaling

**Files:**
- Create: `backend/app/importers/mesh_importer.py`
- Modify: `backend/tests/conftest.py` (add mesh fixtures)
- Test: `backend/tests/test_mesh_importer.py`

**Interfaces:**
- Consumes: `Body`, `Vec3` from `app.models`.
- Produces: `import_mesh(file_path: str, scale: float = 0.001) -> tuple[Body, float]` in `app.importers.mesh_importer`. Returns the imported `Body` (no `faces` populated — that is Tasks 4/5) and the scale factor actually applied, so the caller can surface it to the user per the Review Focus item on silent unit mistakes. Raises `ValueError` if the mesh is not watertight.

- [ ] **Step 1: Add mesh fixtures**

```python
# backend/tests/conftest.py  (append)
import trimesh

@pytest.fixture
def box_stl_path(tmp_path) -> str:
    # Built "in mm" (1000x1000x2000 units) to match the real-world STL
    # convention this importer must correct for.
    mesh = trimesh.creation.box(extents=(1000.0, 1000.0, 2000.0))
    path = str(tmp_path / "box_mm.stl")
    mesh.export(path)
    return path

@pytest.fixture
def non_watertight_stl_path(tmp_path) -> str:
    mesh = trimesh.creation.box(extents=(10.0, 10.0, 10.0))
    mesh.faces = mesh.faces[:-1]  # drop one face -> open mesh
    path = str(tmp_path / "broken.stl")
    mesh.export(path)
    return path
```

- [ ] **Step 2: Write the failing tests**

```python
# backend/tests/test_mesh_importer.py
import math
import pytest
from app.importers.mesh_importer import import_mesh

def test_default_scale_converts_mm_to_meters(box_stl_path):
    body, applied_scale = import_mesh(box_stl_path)
    assert math.isclose(applied_scale, 0.001)
    xs = [v[0] for v in body.vertices]
    span_x = max(xs) - min(xs)
    assert math.isclose(span_x, 1.0, rel_tol=1e-6)  # 1000mm -> 1m

def test_scale_override_is_respected(box_stl_path):
    body, applied_scale = import_mesh(box_stl_path, scale=1.0)
    assert applied_scale == 1.0
    xs = [v[0] for v in body.vertices]
    span_x = max(xs) - min(xs)
    assert math.isclose(span_x, 1000.0, rel_tol=1e-6)

def test_non_watertight_mesh_raises_clear_error(non_watertight_stl_path):
    with pytest.raises(ValueError, match="watertight"):
        import_mesh(non_watertight_stl_path)
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_mesh_importer.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.importers.mesh_importer'`

- [ ] **Step 4: Implement the mesh importer**

```python
# backend/app/importers/mesh_importer.py
import uuid
import trimesh

from app.models import Body


def import_mesh(file_path: str, scale: float = 0.001) -> tuple[Body, float]:
    """Load an STL/OBJ/GLTF file. STL/OBJ carry no unit metadata, so the
    hobbyist convention (millimeters) is assumed by default; callers must
    surface `scale` to the user so a wrong assumption is visible, per the
    spec's documented mm/m pitfall."""
    mesh = trimesh.load(file_path, force="mesh")

    if not mesh.is_watertight:
        raise ValueError(
            f"Mesh in {file_path} is not watertight (has holes or "
            "non-manifold edges) -- cannot reliably compute volume/inertia "
            "or segment faces from it."
        )

    mesh.apply_scale(scale)

    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]

    body = Body(
        id=str(uuid.uuid4()),
        name="imported_mesh",
        vertices=vertices,
        triangles=triangles,
        faces=[],
    )
    return body, scale
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_mesh_importer.py -v`
Expected: PASS (3 passed)

- [ ] **Step 6: Commit**

```bash
git add backend/app/importers/mesh_importer.py backend/tests/conftest.py backend/tests/test_mesh_importer.py
git commit -m "feat: import STL/OBJ meshes with explicit mm-to-meter scaling"
```

---

### Task 4: Region growing (mesh -> candidate face regions)

**Files:**
- Create: `backend/app/segment/__init__.py`
- Create: `backend/app/segment/region_growing.py`
- Test: `backend/tests/test_region_growing.py`

**Interfaces:**
- Consumes: raw `vertices`/`triangles` (as plain lists, matching `Body` fields) — this module works on arrays directly rather than a full `Body`, so it is testable without constructing a `Body`.
- Produces: `grow_regions(vertices: list[tuple[float,float,float]], triangles: list[tuple[int,int,int]], angle_threshold_deg: float = 15.0) -> list[list[int]]` in `app.segment.region_growing`, returning a list of regions, each a list of triangle indices.

- [ ] **Step 1: Write the failing tests**

```python
# backend/tests/test_region_growing.py
import trimesh
from app.segment.region_growing import grow_regions

def test_box_produces_six_regions():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    assert len(regions) == 6
    # every triangle accounted for exactly once
    all_tris = sorted(i for region in regions for i in region)
    assert all_tris == list(range(len(triangles)))

def test_cylinder_produces_three_regions():
    mesh = trimesh.creation.cylinder(radius=0.5, height=2.0, sections=64)
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    # one curved side region + two flat end-cap regions
    assert len(regions) == 3
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_region_growing.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.segment.region_growing'`

- [ ] **Step 3: Implement region growing using trimesh's own adjacency**

```python
# backend/app/segment/region_growing.py
import math
import numpy as np
import trimesh


def grow_regions(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    angle_threshold_deg: float = 15.0,
) -> list[list[int]]:
    """Flood-fill triangles into regions, merging across an edge only when
    the dihedral angle between the two adjacent faces is below threshold.
    A fine-tessellated curved surface (e.g. a cylinder's side) has small
    adjacent-triangle angle changes and stays one region; a sharp edge
    (e.g. box corner, cylinder cap boundary) does not get merged across."""
    mesh = trimesh.Trimesh(
        vertices=np.array(vertices), faces=np.array(triangles), process=False
    )
    threshold_rad = math.radians(angle_threshold_deg)

    adjacency = mesh.face_adjacency
    angles = mesh.face_adjacency_angle

    n_faces = len(triangles)
    adjacency_list: list[list[int]] = [[] for _ in range(n_faces)]
    for (a, b), angle in zip(adjacency, angles):
        if angle < threshold_rad:
            adjacency_list[a].append(b)
            adjacency_list[b].append(a)

    visited = [False] * n_faces
    regions: list[list[int]] = []
    for start in range(n_faces):
        if visited[start]:
            continue
        stack = [start]
        region: list[int] = []
        visited[start] = True
        while stack:
            current = stack.pop()
            region.append(current)
            for neighbor in adjacency_list[current]:
                if not visited[neighbor]:
                    visited[neighbor] = True
                    stack.append(neighbor)
        regions.append(region)

    return regions
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_region_growing.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add backend/app/segment/__init__.py backend/app/segment/region_growing.py backend/tests/test_region_growing.py
git commit -m "feat: grow candidate face regions from bare mesh adjacency"
```

---

### Task 5: Primitive fitting (region -> Face)

**Files:**
- Create: `backend/app/segment/primitive_fit.py`
- Test: `backend/tests/test_primitive_fit.py`

**Interfaces:**
- Consumes: `Face` from `app.models`; a region as `list[int]` (triangle indices) plus the owning `vertices`/`triangles` arrays, matching Task 4's output shape.
- Produces: `fit_face(vertices: list[tuple[float,float,float]], triangles: list[tuple[int,int,int]], region: list[int]) -> Face` in `app.segment.primitive_fit`. Always returns a `Face` with `origin="segmented"`; `kind="unclassified"` with a high `residual` if nothing fits within tolerance, per the Global Constraint that this module never silently guesses a confident wrong type.

- [ ] **Step 1: Write the failing tests**

```python
# backend/tests/test_primitive_fit.py
import math
import trimesh
from app.segment.primitive_fit import fit_face
from app.segment.region_growing import grow_regions

def test_box_face_fits_as_plane_with_correct_normal():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    faces = [fit_face(vertices, triangles, r) for r in regions]
    assert all(f.kind == "plane" for f in faces)
    assert all(f.origin == "segmented" for f in faces)
    for f in faces:
        length = math.sqrt(sum(c * c for c in f.normal))
        assert math.isclose(length, 1.0, rel_tol=1e-3)
    # the six normals should be the three axes, each appearing twice (+/-)
    axis_hits = {0: 0, 1: 0, 2: 0}
    for f in faces:
        dominant_axis = max(range(3), key=lambda i: abs(f.normal[i]))
        axis_hits[dominant_axis] += 1
    assert axis_hits == {0: 2, 1: 2, 2: 2}

def test_cylinder_side_fits_with_correct_radius_and_axis():
    mesh = trimesh.creation.cylinder(radius=0.5, height=2.0, sections=64)
    vertices = [tuple(v) for v in mesh.vertices.tolist()]
    triangles = [tuple(t) for t in mesh.faces.tolist()]
    regions = grow_regions(vertices, triangles)
    faces = [fit_face(vertices, triangles, r) for r in regions]
    cyl_faces = [f for f in faces if f.kind == "cylinder"]
    assert len(cyl_faces) == 1
    assert math.isclose(cyl_faces[0].radius, 0.5, rel_tol=0.02)
    dominant_axis = max(range(3), key=lambda i: abs(cyl_faces[0].axis[i]))
    assert dominant_axis == 2  # trimesh.creation.cylinder is Z-aligned
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_primitive_fit.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.segment.primitive_fit'`

- [ ] **Step 3: Implement the fitter**

```python
# backend/app/segment/primitive_fit.py
import numpy as np
from app.models import Face

_PLANE_RESIDUAL_TOLERANCE = 1e-4   # meters, max deviation from fitted plane
_CYLINDER_RESIDUAL_TOLERANCE = 0.02  # relative radius deviation


def _region_points_and_normals(vertices, triangles, region):
    points = []
    normals = []
    verts_arr = np.array(vertices)
    for tri_idx in region:
        a, b, c = triangles[tri_idx]
        pa, pb, pc = verts_arr[a], verts_arr[b], verts_arr[c]
        points.extend([pa, pb, pc])
        normal = np.cross(pb - pa, pc - pa)
        norm = np.linalg.norm(normal)
        if norm > 1e-12:
            normals.append(normal / norm)
    return np.array(points), np.array(normals)


def _try_fit_plane(points: np.ndarray, normals: np.ndarray):
    centroid = points.mean(axis=0)
    centered = points - centroid
    cov = centered.T @ centered
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = eigvecs[:, 0]  # smallest eigenvalue direction
    residual = float(np.sqrt(max(eigvals[0], 0.0) / max(len(points), 1)))
    if residual > _PLANE_RESIDUAL_TOLERANCE:
        return None
    return normal, tuple(centroid), residual


def _try_fit_cylinder(points: np.ndarray, normals: np.ndarray):
    if len(normals) < 3:
        return None
    normal_centroid = normals.mean(axis=0)
    centered_normals = normals - normal_centroid
    cov = centered_normals.T @ centered_normals
    eigvals, eigvecs = np.linalg.eigh(cov)
    axis_dir = eigvecs[:, 0]  # smallest-variance direction = cylinder axis
    axis_dir = axis_dir / np.linalg.norm(axis_dir)

    centroid = points.mean(axis=0)
    # project points onto plane perpendicular to axis, fit a circle there
    arbitrary = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(arbitrary, axis_dir)) > 0.9:
        arbitrary = np.array([0.0, 1.0, 0.0])
    u = np.cross(axis_dir, arbitrary)
    u /= np.linalg.norm(u)
    v = np.cross(axis_dir, u)

    rel = points - centroid
    xs = rel @ u
    ys = rel @ v
    # algebraic circle fit: x^2+y^2 + D*x + E*y + F = 0
    A = np.column_stack([xs, ys, np.ones_like(xs)])
    b = -(xs**2 + ys**2)
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    d, e, f = sol
    cx, cy = -d / 2, -e / 2
    radius_sq = cx**2 + cy**2 - f
    if radius_sq <= 0:
        return None
    radius = float(np.sqrt(radius_sq))

    distances = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2)
    residual = float(np.std(distances) / radius) if radius > 0 else 1.0
    if residual > _CYLINDER_RESIDUAL_TOLERANCE:
        return None

    center_3d = centroid + cx * u + cy * v
    return axis_dir, radius, tuple(center_3d), residual


def fit_face(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    region: list[int],
) -> Face:
    points, normals = _region_points_and_normals(vertices, triangles, region)
    if len(points) == 0:
        return Face(
            kind="unclassified", origin="segmented", center=(0.0, 0.0, 0.0),
            residual=1.0, triangle_indices=region,
        )

    plane_fit = _try_fit_plane(points, normals)
    if plane_fit is not None:
        normal, center, residual = plane_fit
        return Face(
            kind="plane", origin="segmented",
            normal=(float(normal[0]), float(normal[1]), float(normal[2])),
            axis=None, center=center, radius=None, residual=residual,
            triangle_indices=region,
        )

    cylinder_fit = _try_fit_cylinder(points, normals)
    if cylinder_fit is not None:
        axis_dir, radius, center, residual = cylinder_fit
        return Face(
            kind="cylinder", origin="segmented", normal=None,
            axis=(float(axis_dir[0]), float(axis_dir[1]), float(axis_dir[2])),
            center=center, radius=radius, residual=residual,
            triangle_indices=region,
        )

    return Face(
        kind="unclassified", origin="segmented",
        center=tuple(points.mean(axis=0).tolist()), residual=1.0,
        triangle_indices=region,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_primitive_fit.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add backend/app/segment/primitive_fit.py backend/tests/test_primitive_fit.py
git commit -m "feat: fit plane/cylinder primitives to grown mesh regions"
```

---

### Task 6: Link mass properties (volume, center of mass, inertia)

**Files:**
- Create: `backend/app/massprops/__init__.py`
- Create: `backend/app/massprops/link_mass.py`
- Test: `backend/tests/test_link_mass.py`

**Interfaces:**
- Consumes: `Body` from `app.models` (uses `vertices`/`triangles` only).
- Produces: `compute_link_mass_properties(bodies: list[Body], density_kg_m3: float) -> tuple[float, tuple[float,float,float], list[float]]` in `app.massprops.link_mass`, returning `(mass_kg, center_of_mass, [ixx,iyy,izz,ixy,ixz,iyz])`. Raises `ValueError` if the combined mesh has zero or negative volume (Review Focus: degenerate geometry must not silently produce a zero-inertia link).

- [ ] **Step 1: Write the failing tests**

```python
# backend/tests/test_link_mass.py
import math
import pytest
import trimesh
from app.models import Body
from app.massprops.link_mass import compute_link_mass_properties


def _body_from_trimesh(mesh) -> Body:
    return Body(
        id="b1", name="test",
        vertices=[tuple(v) for v in mesh.vertices.tolist()],
        triangles=[tuple(t) for t in mesh.faces.tolist()],
        faces=[],
    )


def test_unit_box_mass_matches_volume_times_density():
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    body = _body_from_trimesh(mesh)
    mass, com, inertia = compute_link_mass_properties([body], density_kg_m3=1000.0)
    assert math.isclose(mass, 1000.0, rel_tol=1e-3)  # 1 m^3 * 1000 kg/m^3
    assert all(math.isclose(c, 0.0, abs_tol=1e-6) for c in com)


def test_known_cube_inertia_matches_closed_form():
    # solid cube inertia about its own center: I = (1/6) * m * s^2 for each axis
    side = 2.0
    mesh = trimesh.creation.box(extents=(side, side, side))
    body = _body_from_trimesh(mesh)
    density = 500.0
    mass, com, inertia = compute_link_mass_properties([body], density_kg_m3=density)
    expected_i = (mass / 6.0) * side**2
    ixx, iyy, izz = inertia[0], inertia[1], inertia[2]
    assert math.isclose(ixx, expected_i, rel_tol=0.02)
    assert math.isclose(iyy, expected_i, rel_tol=0.02)
    assert math.isclose(izz, expected_i, rel_tol=0.02)


def test_degenerate_zero_volume_body_raises():
    # a flat, zero-thickness "box" has zero volume
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 0.0))
    body = _body_from_trimesh(mesh)
    with pytest.raises(ValueError, match="volume"):
        compute_link_mass_properties([body], density_kg_m3=1000.0)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_link_mass.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.massprops.link_mass'`

- [ ] **Step 3: Implement using trimesh's built-in mass properties**

```python
# backend/app/massprops/link_mass.py
import numpy as np
import trimesh
from app.models import Body

_MIN_VOLUME_M3 = 1e-9  # below this, treat geometry as degenerate


def compute_link_mass_properties(
    bodies: list[Body], density_kg_m3: float
) -> tuple[float, tuple[float, float, float], list[float]]:
    """Combine every Body belonging to a Link into one mesh and compute its
    mass properties via signed-tetrahedra volume integration (trimesh's
    built-in implementation), rather than hand-rolling the integral."""
    combined: trimesh.Trimesh | None = None
    for body in bodies:
        mesh = trimesh.Trimesh(
            vertices=np.array(body.vertices),
            faces=np.array(body.triangles),
            process=False,
        )
        combined = mesh if combined is None else combined + mesh

    if combined is None or combined.volume <= _MIN_VOLUME_M3:
        raise ValueError(
            "Link has zero or negligible volume -- check that its member "
            "bodies form a closed solid and were not left empty."
        )

    combined.density = density_kg_m3
    mass = float(combined.mass)
    com = tuple(combined.center_mass.tolist())
    inertia_tensor = combined.moment_inertia  # 3x3, about center of mass
    inertia_vector = [
        float(inertia_tensor[0, 0]), float(inertia_tensor[1, 1]),
        float(inertia_tensor[2, 2]), float(inertia_tensor[0, 1]),
        float(inertia_tensor[0, 2]), float(inertia_tensor[1, 2]),
    ]
    return mass, com, inertia_vector
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_link_mass.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add backend/app/massprops backend/tests/test_link_mass.py
git commit -m "feat: compute link mass, center of mass, and inertia from member bodies"
```

---

### Task 7: Face-pair matching for single-file joint detection

**Files:**
- Create: `backend/app/joints/__init__.py`
- Create: `backend/app/joints/face_matching.py`
- Test: `backend/tests/test_face_matching.py`

**Interfaces:**
- Consumes: `Body`, `Face` from `app.models`.
- Produces: `find_matching_face_pairs(body_a: Body, body_b: Body, distance_tolerance: float = 1e-4) -> list[tuple[Face, Face]]` in `app.joints.face_matching`. Returns an empty list when no faces actually touch (Review Focus: no false match between unrelated bodies). When multiple concentric cylindrical pairs exist, returns them ordered by contact area/radius descending, largest first, so the caller can either use the top one or flag ambiguity when more than one scores highly (Review Focus: no silent arbitrary pick).

- [ ] **Step 1: Write the failing tests**

```python
# backend/tests/test_face_matching.py
import math
from app.models import Body, Face
from app.joints.face_matching import find_matching_face_pairs


def _planar_face(center, normal, radius=None, kind="plane"):
    return Face(
        kind=kind, origin="brep", normal=normal, axis=None,
        center=center, radius=radius, residual=0.0, triangle_indices=[],
    )


def _cylindrical_face(center, axis, radius):
    return Face(
        kind="cylinder", origin="brep", normal=None, axis=axis,
        center=center, radius=radius, residual=0.0, triangle_indices=[],
    )


def _body(faces):
    return Body(id="b", name="n", vertices=[], triangles=[], faces=faces)


def test_coincident_planar_faces_match():
    face_a = _planar_face(center=(0, 0, 0), normal=(0, 0, 1))
    face_b = _planar_face(center=(0, 0, 0), normal=(0, 0, -1))
    pairs = find_matching_face_pairs(_body([face_a]), _body([face_b]))
    assert len(pairs) == 1
    assert pairs[0] == (face_a, face_b)


def test_concentric_cylindrical_faces_match():
    face_a = _cylindrical_face(center=(0, 0, 0), axis=(0, 0, 1), radius=0.1)
    face_b = _cylindrical_face(center=(0, 0, 0.5), axis=(0, 0, 1), radius=0.1)
    pairs = find_matching_face_pairs(_body([face_a]), _body([face_b]))
    assert len(pairs) == 1


def test_unrelated_far_apart_faces_do_not_match():
    face_a = _planar_face(center=(0, 0, 0), normal=(0, 0, 1))
    face_b = _planar_face(center=(10, 10, 10), normal=(0, 0, -1))
    pairs = find_matching_face_pairs(_body([face_a]), _body([face_b]))
    assert pairs == []


def test_multiple_concentric_pairs_ordered_largest_radius_first():
    small = _cylindrical_face(center=(0, 0, 0), axis=(0, 0, 1), radius=0.05)
    small_match = _cylindrical_face(center=(0, 0, 0.1), axis=(0, 0, 1), radius=0.05)
    large = _cylindrical_face(center=(1, 0, 0), axis=(0, 0, 1), radius=0.2)
    large_match = _cylindrical_face(center=(1, 0, 0.1), axis=(0, 0, 1), radius=0.2)
    body_a = _body([small, large])
    body_b = _body([small_match, large_match])
    pairs = find_matching_face_pairs(body_a, body_b)
    assert len(pairs) == 2
    assert pairs[0][0].radius == 0.2  # largest first
    assert pairs[1][0].radius == 0.05
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_face_matching.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.joints.face_matching'`

- [ ] **Step 3: Implement face matching**

```python
# backend/app/joints/face_matching.py
import math
from app.models import Body, Face

_PLANE_DISTANCE_TOLERANCE_DEFAULT = 1e-4   # meters
_PLANE_NORMAL_PARALLEL_TOLERANCE = 1e-3    # dot product closeness to -1
_CYLINDER_AXIS_PARALLEL_TOLERANCE = 1e-3
_CYLINDER_RADIUS_RELATIVE_TOLERANCE = 0.02
_CYLINDER_CENTER_LINE_DISTANCE_TOLERANCE = 1e-3


def _dist(a: tuple, b: tuple) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _dot(a: tuple, b: tuple) -> float:
    return sum(x * y for x, y in zip(a, b))


def _planar_match(face_a: Face, face_b: Face, tol: float) -> bool:
    if face_a.kind != "plane" or face_b.kind != "plane":
        return False
    # opposing normals (the two mating faces point at each other)
    if _dot(face_a.normal, face_b.normal) > -(1 - _PLANE_NORMAL_PARALLEL_TOLERANCE):
        return False
    return _dist(face_a.center, face_b.center) < tol


def _cylindrical_match(face_a: Face, face_b: Face) -> bool:
    if face_a.kind != "cylinder" or face_b.kind != "cylinder":
        return False
    if abs(abs(_dot(face_a.axis, face_b.axis)) - 1.0) > _CYLINDER_AXIS_PARALLEL_TOLERANCE:
        return False
    if face_a.radius is None or face_b.radius is None:
        return False
    radius_diff = abs(face_a.radius - face_b.radius) / max(face_a.radius, 1e-9)
    if radius_diff > _CYLINDER_RADIUS_RELATIVE_TOLERANCE:
        return False
    # perpendicular distance from face_b's center to face_a's axis line
    to_point = tuple(b - a for a, b in zip(face_a.center, face_b.center))
    along_axis = _dot(to_point, face_a.axis)
    perp = [to_point[i] - along_axis * face_a.axis[i] for i in range(3)]
    perp_distance = math.sqrt(sum(p * p for p in perp))
    return perp_distance < _CYLINDER_CENTER_LINE_DISTANCE_TOLERANCE


def _contact_score(face_a: Face, face_b: Face) -> float:
    """Larger is a more-constraining, more-trustworthy match."""
    if face_a.kind == "cylinder":
        return face_a.radius
    return 1.0  # planar matches have no natural size signal here


def find_matching_face_pairs(
    body_a: Body, body_b: Body,
    distance_tolerance: float = _PLANE_DISTANCE_TOLERANCE_DEFAULT,
) -> list[tuple[Face, Face]]:
    pairs: list[tuple[Face, Face]] = []
    for face_a in body_a.faces:
        for face_b in body_b.faces:
            if _planar_match(face_a, face_b, distance_tolerance) or _cylindrical_match(
                face_a, face_b
            ):
                pairs.append((face_a, face_b))

    pairs.sort(key=lambda pair: _contact_score(*pair), reverse=True)
    return pairs
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_face_matching.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add backend/app/joints/__init__.py backend/app/joints/face_matching.py backend/tests/test_face_matching.py
git commit -m "feat: find coincident/concentric face pairs between two bodies"
```

---

### Task 8: IPV/IMG joint classification

**Files:**
- Create: `backend/app/joints/ipv_img.py`
- Test: `backend/tests/test_ipv_img.py`

**Interfaces:**
- Consumes: `Face`, `Joint` from `app.models`; `list[tuple[Face, Face]]` from Task 7's `find_matching_face_pairs`.
- Produces: `classify_joint(matched_pairs: list[tuple[Face, Face]], parent_link_id: str, child_link_id: str) -> Joint` in `app.joints.ipv_img`. Always returns a `Joint` with `source="suggested"`; when `matched_pairs` is empty, returns `type="fixed"` with `axis=None` (no relative-motion-permitting geometry found, per spec §6a step 4).

- [ ] **Step 1: Write the failing tests**

```python
# backend/tests/test_ipv_img.py
import math
from app.models import Face
from app.joints.ipv_img import classify_joint


def _cyl(center, axis, radius):
    return Face(kind="cylinder", origin="brep", normal=None, axis=axis,
                center=center, radius=radius, residual=0.0, triangle_indices=[])


def _plane(center, normal):
    return Face(kind="plane", origin="brep", normal=normal, axis=None,
                center=center, radius=None, residual=0.0, triangle_indices=[])


def test_no_matched_pairs_is_fixed():
    joint = classify_joint([], "parent", "child")
    assert joint.type == "fixed"
    assert joint.axis is None
    assert joint.source == "suggested"


def test_cylindrical_pair_alone_is_cylindrical_joint():
    pair = (_cyl((0, 0, 0), (0, 0, 1), 0.1), _cyl((0, 0, 0.1), (0, 0, 1), 0.1))
    joint = classify_joint([pair], "parent", "child")
    assert joint.type == "cylindrical"
    assert math.isclose(abs(joint.axis[2]), 1.0, rel_tol=1e-6)


def test_cylindrical_plus_perpendicular_planar_is_revolute():
    cyl_pair = (_cyl((0, 0, 0), (0, 0, 1), 0.1), _cyl((0, 0, 0.1), (0, 0, 1), 0.1))
    plane_pair = (_plane((0, 0, 0.2), (0, 0, 1)), _plane((0, 0, 0.2), (0, 0, -1)))
    joint = classify_joint([cyl_pair, plane_pair], "parent", "child")
    assert joint.type == "revolute"


def test_planar_pair_alone_is_planar_joint():
    pair = (_plane((0, 0, 0), (0, 0, 1)), _plane((0, 0, 0), (0, 0, -1)))
    joint = classify_joint([pair], "parent", "child")
    assert joint.type == "planar"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_ipv_img.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.joints.ipv_img'`

- [ ] **Step 3: Implement the IPV/IMG classifier**

```python
# backend/app/joints/ipv_img.py
import uuid
import math
from app.models import Face, Joint

_PERPENDICULAR_DOT_TOLERANCE = 0.05


def _dot(a, b) -> float:
    return sum(x * y for x, y in zip(a, b))


def classify_joint(
    matched_pairs: list[tuple[Face, Face]],
    parent_link_id: str,
    child_link_id: str,
) -> Joint:
    """Classify per the spec's IPV/IMG table (§6a step 3, Kim et al. 2005):
    one matched cylindrical pair -> cylindrical; that plus a perpendicular
    planar lock -> revolute; planar pair alone -> planar; nothing matched
    -> fixed. (Spherical and prismatic combinations are not reachable from
    a single cylinder+plane pair set and are left for a future extension
    once real multi-pair assemblies surface the need.)"""
    cyl_pairs = [p for p in matched_pairs if p[0].kind == "cylinder"]
    plane_pairs = [p for p in matched_pairs if p[0].kind == "plane"]

    if not matched_pairs:
        return Joint(
            id=str(uuid.uuid4()), parent_link_id=parent_link_id,
            child_link_id=child_link_id, type="fixed",
            origin_xyz=(0.0, 0.0, 0.0), origin_rpy=(0.0, 0.0, 0.0),
            axis=None, source="suggested",
        )

    if cyl_pairs:
        cyl_face = cyl_pairs[0][0]
        has_perpendicular_plane = any(
            abs(_dot(cyl_face.axis, p[0].normal)) > (1 - _PERPENDICULAR_DOT_TOLERANCE)
            for p in plane_pairs
        )
        joint_type = "revolute" if has_perpendicular_plane else "cylindrical"
        return Joint(
            id=str(uuid.uuid4()), parent_link_id=parent_link_id,
            child_link_id=child_link_id, type=joint_type,
            origin_xyz=cyl_face.center, origin_rpy=(0.0, 0.0, 0.0),
            axis=cyl_face.axis, source="suggested",
        )

    plane_face = plane_pairs[0][0]
    return Joint(
        id=str(uuid.uuid4()), parent_link_id=parent_link_id,
        child_link_id=child_link_id, type="planar",
        origin_xyz=plane_face.center, origin_rpy=(0.0, 0.0, 0.0),
        axis=plane_face.normal, source="suggested",
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_ipv_img.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add backend/app/joints/ipv_img.py backend/tests/test_ipv_img.py
git commit -m "feat: classify joint type/axis from matched face pairs via IPV/IMG table"
```

---

### Task 9: Structural validation gate

**Files:**
- Create: `backend/app/validate/__init__.py`
- Create: `backend/app/validate/structural.py`
- Test: `backend/tests/test_structural.py`

**Interfaces:**
- Consumes: `Link`, `Joint` from `app.models`.
- Produces: `validate_structure(links: list[Link], joints: list[Joint]) -> list[str]` in `app.validate.structural`. Returns a list of human-readable error strings (empty list means the structure is valid). Never raises — callers decide what to do with a non-empty error list (e.g. the export route blocks).

- [ ] **Step 1: Write the failing tests**

```python
# backend/tests/test_structural.py
from app.models import Link, Joint
from app.validate.structural import validate_structure


def _link(id_, mass=1.0, inertia=(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)):
    return Link(
        id=id_, name=id_, body_ids=[], density_kg_m3=1000.0,
        mass_kg=mass, center_of_mass=(0, 0, 0), inertia=list(inertia),
    )


def _joint(id_, parent, child, type_="fixed"):
    return Joint(
        id=id_, parent_link_id=parent, child_link_id=child, type=type_,
        origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0), axis=(0, 0, 1),
        source="confirmed",
    )


def test_valid_two_link_tree_has_no_errors():
    links = [_link("base"), _link("arm")]
    joints = [_joint("j1", "base", "arm")]
    assert validate_structure(links, joints) == []


def test_two_roots_is_an_error():
    links = [_link("base"), _link("arm1"), _link("arm2")]
    joints = []  # arm1 and arm2 both have no parent -> two roots alongside base
    errors = validate_structure(links, joints)
    assert any("root" in e.lower() for e in errors)


def test_cycle_is_an_error():
    links = [_link("a"), _link("b")]
    joints = [_joint("j1", "a", "b"), _joint("j2", "b", "a")]
    errors = validate_structure(links, joints)
    assert any("cycle" in e.lower() for e in errors)


def test_non_leaf_link_with_zero_inertia_is_an_error():
    zero_inertia_parent = _link("base", inertia=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    child = _link("arm")
    links = [zero_inertia_parent, child]
    joints = [_joint("j1", "base", "arm")]
    errors = validate_structure(links, joints)
    assert any("inertia" in e.lower() for e in errors)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_structural.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.validate.structural'`

- [ ] **Step 3: Implement structural validation**

```python
# backend/app/validate/structural.py
from app.models import Link, Joint


def validate_structure(links: list[Link], joints: list[Joint]) -> list[str]:
    errors: list[str] = []
    link_ids = {link.id for link in links}
    children_of_joint = {joint.child_link_id for joint in joints}
    parent_by_child = {joint.child_link_id: joint.parent_link_id for joint in joints}

    roots = [link.id for link in links if link.id not in children_of_joint]
    if len(roots) == 0:
        errors.append("No root link found (every link has a parent joint).")
    elif len(roots) > 1:
        errors.append(
            f"Multiple root links found ({', '.join(sorted(roots))}); "
            "a URDF tree must have exactly one root."
        )

    for joint in joints:
        if joint.parent_link_id not in link_ids:
            errors.append(
                f"Joint {joint.id} references unknown parent link "
                f"'{joint.parent_link_id}'."
            )
        if joint.child_link_id not in link_ids:
            errors.append(
                f"Joint {joint.id} references unknown child link "
                f"'{joint.child_link_id}'."
            )

    for link in links:
        visited: set[str] = set()
        current = link.id
        while current in parent_by_child:
            if current in visited:
                errors.append(
                    f"Cycle detected in kinematic tree involving link '{current}'."
                )
                break
            visited.add(current)
            current = parent_by_child[current]

    non_leaf_ids = {joint.parent_link_id for joint in joints}
    for link in links:
        if link.id in non_leaf_ids:
            inertia = link.inertia or [0.0] * 6
            if all(abs(v) < 1e-12 for v in inertia[:3]):
                errors.append(
                    f"Link '{link.id}' is not a leaf but has zero inertia; "
                    "every non-leaf link must have nonzero inertia."
                )

    return errors
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_structural.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add backend/app/validate backend/tests/test_structural.py
git commit -m "feat: validate kinematic tree structure and link inertia before export"
```

---

### Task 10: URDF export

**Files:**
- Create: `backend/app/export/__init__.py`
- Create: `backend/app/export/urdf_writer.py`
- Test: `backend/tests/test_urdf_writer.py`

**Interfaces:**
- Consumes: `Link`, `Joint`, `Body` from `app.models`; `validate_structure` from `app.validate.structural` (Task 9).
- Produces: `export_urdf(robot_name: str, links: list[Link], joints: list[Joint], bodies_by_link: dict[str, list[Body]]) -> str` in `app.export.urdf_writer`, returning the URDF XML as a string. Raises `ValueError` (carrying the structural error list in its message) if `validate_structure` reports any errors — export is blocked, per the spec's structural gate.

- [ ] **Step 1: Write the failing tests**

```python
# backend/tests/test_urdf_writer.py
import pytest
import xml.etree.ElementTree as ET
from app.models import Link, Joint, Body
from app.export.urdf_writer import export_urdf


def _link(id_):
    return Link(
        id=id_, name=id_, body_ids=[f"{id_}_body"], density_kg_m3=1000.0,
        mass_kg=2.0, center_of_mass=(0.0, 0.0, 0.1),
        inertia=[0.01, 0.01, 0.01, 0.0, 0.0, 0.0],
    )


def _joint(id_, parent, child):
    return Joint(
        id=id_, parent_link_id=parent, child_link_id=child, type="revolute",
        origin_xyz=(0.0, 0.0, 0.2), origin_rpy=(0.0, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0), limit_lower=-1.57, limit_upper=1.57,
        source="confirmed",
    )


def _body(link_id):
    return Body(id=f"{link_id}_body", name=link_id, vertices=[], triangles=[], faces=[])


def test_valid_model_produces_parseable_urdf_with_expected_joint():
    links = [_link("base"), _link("arm")]
    joints = [_joint("j1", "base", "arm")]
    bodies_by_link = {"base": [_body("base")], "arm": [_body("arm")]}

    xml_str = export_urdf("test_robot", links, joints, bodies_by_link)
    root = ET.fromstring(xml_str)

    assert root.tag == "robot"
    assert root.attrib["name"] == "test_robot"
    link_names = {el.attrib["name"] for el in root.findall("link")}
    assert link_names == {"base", "arm"}

    joint_el = root.find("joint")
    assert joint_el.attrib["type"] == "revolute"
    assert joint_el.find("parent").attrib["link"] == "base"
    assert joint_el.find("child").attrib["link"] == "arm"
    limit_el = joint_el.find("limit")
    assert float(limit_el.attrib["lower"]) == pytest.approx(-1.57)


def test_invalid_structure_blocks_export():
    links = [_link("a"), _link("b")]
    joints = [_joint("j1", "a", "b"), _joint("j2", "b", "a")]  # cycle
    bodies_by_link = {"a": [_body("a")], "b": [_body("b")]}
    with pytest.raises(ValueError, match="[Cc]ycle"):
        export_urdf("test_robot", links, joints, bodies_by_link)


def test_inertial_origin_matches_center_of_mass_not_joint_origin():
    links = [_link("base"), _link("arm")]
    joints = [_joint("j1", "base", "arm")]
    bodies_by_link = {"base": [_body("base")], "arm": [_body("arm")]}
    xml_str = export_urdf("test_robot", links, joints, bodies_by_link)
    root = ET.fromstring(xml_str)
    arm_link = next(el for el in root.findall("link") if el.attrib["name"] == "arm")
    inertial_origin = arm_link.find("inertial").find("origin").attrib["xyz"]
    assert inertial_origin == "0.0 0.0 0.1"  # the link's center_of_mass, not the joint's origin_xyz
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd backend && pytest tests/test_urdf_writer.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'app.export.urdf_writer'`

- [ ] **Step 3: Implement the URDF writer**

```python
# backend/app/export/urdf_writer.py
import xml.etree.ElementTree as ET
from xml.dom import minidom
from app.models import Link, Joint, Body
from app.validate.structural import validate_structure


def _xyz(v: tuple[float, float, float]) -> str:
    return f"{v[0]} {v[1]} {v[2]}"


def _add_link_element(robot: ET.Element, link: Link) -> None:
    link_el = ET.SubElement(robot, "link", {"name": link.name})

    inertial_el = ET.SubElement(link_el, "inertial")
    # Inertial origin MUST match the link's own center of mass, never the
    # joint origin -- a documented pitfall from the spec (§8).
    com = link.center_of_mass or (0.0, 0.0, 0.0)
    ET.SubElement(inertial_el, "origin", {"xyz": _xyz(com), "rpy": "0 0 0"})
    ET.SubElement(inertial_el, "mass", {"value": str(link.mass_kg or 0.0)})
    ixx, iyy, izz, ixy, ixz, iyz = link.inertia or [0.0] * 6
    ET.SubElement(inertial_el, "inertia", {
        "ixx": str(ixx), "iyy": str(iyy), "izz": str(izz),
        "ixy": str(ixy), "ixz": str(ixz), "iyz": str(iyz),
    })

    for geometry_tag in ("visual", "collision"):
        geom_el = ET.SubElement(link_el, geometry_tag)
        ET.SubElement(geom_el, "origin", {"xyz": _xyz(com), "rpy": "0 0 0"})
        geometry = ET.SubElement(geom_el, "geometry")
        ET.SubElement(geometry, "mesh", {"filename": f"meshes/{link.name}.stl"})


def _add_joint_element(robot: ET.Element, joint: Joint, links_by_id: dict[str, Link]) -> None:
    joint_el = ET.SubElement(robot, "joint", {
        "name": f"joint_{joint.id}", "type": joint.type,
    })
    ET.SubElement(joint_el, "parent", {"link": links_by_id[joint.parent_link_id].name})
    ET.SubElement(joint_el, "child", {"link": links_by_id[joint.child_link_id].name})
    ET.SubElement(joint_el, "origin", {
        "xyz": _xyz(joint.origin_xyz), "rpy": _xyz(joint.origin_rpy),
    })
    if joint.axis is not None:
        ET.SubElement(joint_el, "axis", {"xyz": _xyz(joint.axis)})
    if joint.type in ("revolute", "prismatic") and joint.limit_lower is not None:
        ET.SubElement(joint_el, "limit", {
            "lower": str(joint.limit_lower), "upper": str(joint.limit_upper),
            "effort": "100", "velocity": "1.0",
        })


def export_urdf(
    robot_name: str,
    links: list[Link],
    joints: list[Joint],
    bodies_by_link: dict[str, list[Body]],
) -> str:
    errors = validate_structure(links, joints)
    if errors:
        raise ValueError(
            "Cannot export URDF -- structural validation failed: "
            + "; ".join(errors)
        )

    robot = ET.Element("robot", {"name": robot_name})
    links_by_id = {link.id: link for link in links}

    for link in links:
        _add_link_element(robot, link)
    for joint in joints:
        _add_joint_element(robot, joint, links_by_id)

    rough_string = ET.tostring(robot, encoding="unicode")
    return minidom.parseString(rough_string).toprettyxml(indent="  ")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd backend && pytest tests/test_urdf_writer.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add backend/app/export backend/tests/test_urdf_writer.py
git commit -m "feat: export validated link/joint model to URDF XML"
```

---

### Task 11: API routes and end-to-end wiring

**Files:**
- Create: `backend/app/routes/__init__.py`
- Create: `backend/app/routes/import_routes.py`
- Create: `backend/app/routes/joint_routes.py`
- Create: `backend/app/routes/export_routes.py`
- Modify: `backend/app/main.py` (register routers)
- Modify: `backend/tests/conftest.py` (add a full two-part assembly fixture if not already present from Task 2)
- Test: `backend/tests/test_end_to_end.py`

**Interfaces:**
- Consumes: every module from Tasks 1-10.
- Produces: `POST /import` (multipart file upload; STEP/IGES/BREP via `import_step`, STL/OBJ via `import_mesh` + Task 4/5 segmentation chained together; returns JSON list of `Body`), `POST /detect-joints` (JSON body `{body_a, body_b, parent_link_id, child_link_id}`; runs `find_matching_face_pairs` then `classify_joint`; returns a `Joint`), `POST /export` (JSON body `{robot_name, links, joints, bodies_by_link}`; returns the URDF XML string, or HTTP 422 with the structural error list on failure).

- [ ] **Step 1: Write the failing end-to-end test**

```python
# backend/tests/test_end_to_end.py
from fastapi.testclient import TestClient
from app.main import app
from app.importers.step_importer import import_step
from app.joints.face_matching import find_matching_face_pairs
from app.joints.ipv_img import classify_joint
from app.massprops.link_mass import compute_link_mass_properties
from app.models import Link
from app.export.urdf_writer import export_urdf

client = TestClient(app)


def test_peg_in_hole_assembly_imports_detects_and_exports(peg_in_hole_step_path):
    bodies = import_step(peg_in_hole_step_path)
    assert len(bodies) == 2
    body_hole, body_peg = bodies  # order matches construction order in the fixture

    pairs = find_matching_face_pairs(body_hole, body_peg)
    assert len(pairs) >= 1

    joint = classify_joint(pairs, parent_link_id="base", child_link_id="peg")
    assert joint.type in ("cylindrical", "revolute")
    joint.source = "confirmed"

    mass_a, com_a, inertia_a = compute_link_mass_properties([body_hole], 1000.0)
    mass_b, com_b, inertia_b = compute_link_mass_properties([body_peg], 1000.0)

    links = [
        Link(id="base", name="base", body_ids=[body_hole.id], density_kg_m3=1000.0,
             mass_kg=mass_a, center_of_mass=com_a, inertia=inertia_a),
        Link(id="peg", name="peg", body_ids=[body_peg.id], density_kg_m3=1000.0,
             mass_kg=mass_b, center_of_mass=com_b, inertia=inertia_b),
    ]
    joint.parent_link_id, joint.child_link_id = "base", "peg"

    xml_str = export_urdf("peg_robot", links, [joint], {
        "base": [body_hole], "peg": [body_peg],
    })
    assert "<robot" in xml_str
    assert 'type="' + joint.type + '"' in xml_str


def test_health_route_still_works():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}
```

- [ ] **Step 2: Run the test to verify it fails or passes partially**

Run: `cd backend && pytest tests/test_end_to_end.py -v`
Expected: the `test_health_route_still_works` test passes already (Task 1); `test_peg_in_hole_assembly_imports_detects_and_exports` should already pass too, since it only composes Tasks 2/7/8/6/10 directly — this test exists to prove those pieces compose correctly end-to-end, not to drive new production code. If it fails, the bug is in how two already-tested modules are wired together, not in any single module; fix the composition in this test or, if a real defect is found, in the offending module from its own task.

- [ ] **Step 3: Implement the API routes**

```python
# backend/app/routes/import_routes.py
import tempfile
import os
from fastapi import APIRouter, UploadFile, HTTPException

from app.importers.step_importer import import_step
from app.importers.mesh_importer import import_mesh
from app.segment.region_growing import grow_regions
from app.segment.primitive_fit import fit_face
from app.models import Body

router = APIRouter()

_STEP_EXTENSIONS = {".step", ".stp", ".iges", ".igs", ".brep"}
_MESH_EXTENSIONS = {".stl", ".obj"}


@router.post("/import")
async def import_file(file: UploadFile, scale: float = 0.001):
    suffix = os.path.splitext(file.filename or "")[1].lower()
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        tmp.write(await file.read())
        tmp_path = tmp.name

    try:
        if suffix in _STEP_EXTENSIONS:
            bodies = import_step(tmp_path)
            applied_scale = 1.0  # STEP carries its own units; no mm assumption
        elif suffix in _MESH_EXTENSIONS:
            body, applied_scale = import_mesh(tmp_path, scale=scale)
            regions = grow_regions(body.vertices, body.triangles)
            body.faces = [fit_face(body.vertices, body.triangles, r) for r in regions]
            bodies = [body]
        else:
            raise HTTPException(400, f"Unsupported file type: {suffix}")
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    finally:
        os.unlink(tmp_path)

    return {
        "bodies": [b.model_dump() for b in bodies],
        "applied_scale": applied_scale,
    }
```

```python
# backend/app/routes/joint_routes.py
from fastapi import APIRouter
from pydantic import BaseModel

from app.models import Body, Joint
from app.joints.face_matching import find_matching_face_pairs
from app.joints.ipv_img import classify_joint

router = APIRouter()


class DetectJointsRequest(BaseModel):
    body_a: Body
    body_b: Body
    parent_link_id: str
    child_link_id: str


@router.post("/detect-joints", response_model=Joint)
def detect_joints(request: DetectJointsRequest) -> Joint:
    pairs = find_matching_face_pairs(request.body_a, request.body_b)
    return classify_joint(pairs, request.parent_link_id, request.child_link_id)
```

```python
# backend/app/routes/export_routes.py
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from app.models import Link, Joint, Body
from app.export.urdf_writer import export_urdf

router = APIRouter()


class ExportRequest(BaseModel):
    robot_name: str
    links: list[Link]
    joints: list[Joint]
    bodies_by_link: dict[str, list[Body]]


@router.post("/export")
def export(request: ExportRequest) -> dict[str, str]:
    try:
        xml_str = export_urdf(
            request.robot_name, request.links, request.joints,
            request.bodies_by_link,
        )
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    return {"urdf": xml_str}
```

```python
# backend/app/main.py  (replace)
from fastapi import FastAPI
from app.routes import import_routes, joint_routes, export_routes

app = FastAPI(title="cad-to-urdf")
app.include_router(import_routes.router)
app.include_router(joint_routes.router)
app.include_router(export_routes.router)


@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}
```

- [ ] **Step 4: Run the full test suite**

Run: `cd backend && pytest -v`
Expected: all tests across every task pass.

- [ ] **Step 5: Commit**

```bash
git add backend/app/routes backend/app/main.py backend/tests/test_end_to_end.py
git commit -m "feat: wire import/detect-joints/export into FastAPI routes"
```

---

## Self-Review Notes

- **Spec coverage:** §6a-prime (Task 4+5), §6a (Task 7+8), §7.1 structural gate (Task 9), §8 export with mm→m scale and correct inertial origin (Task 10), the data model of §5 (Task 1), mass/inertia needed by §7.1/§8 (Task 6). §6b, §7.2, §7.3, and all frontend/UI items are nice-to-have or out-of-backend-MVP-scope per the spec's own §2 split and are intentionally not tasked here.
- **Placeholder scan:** none found — every step has runnable code or an exact command.
- **Type consistency:** `Body`/`Face`/`Link`/`Joint` field names are defined once in Task 1 and used identically (checked against Task 1's field list) in every later task.
- **Review Focus:** all five items have an owning task and test, as listed above each one.
