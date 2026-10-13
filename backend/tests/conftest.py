import app  # noqa: F401  (sets BLAS thread cap before numpy loads)
import pytest
import trimesh
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere,
)
from OCC.Core.BRepTools import breptools
from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from OCC.Core.IGESControl import IGESControl_Writer
from OCC.Core.Interface import Interface_Static
from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Writer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SHELL
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Shell


def _write_step(shape, path: str, unit: str = "M") -> None:
    """Write shape numerics as-is, declaring `unit` in the file header.
    Statics are process-global (always set explicitly) and only exist once a
    writer is constructed; the writer treats numerics as cascade unit MM and
    just labels the file with write.step.unit."""
    writer = STEPControl_Writer()
    Interface_Static.SetCVal("xstep.cascade.unit", "MM")
    Interface_Static.SetCVal("write.step.unit", unit)
    writer.Transfer(shape, STEPControl_AsIs)
    writer.Write(path)


@pytest.fixture
def box_step_path(tmp_path) -> str:
    path = str(tmp_path / "box.step")
    _write_step(BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape(), path)
    return path


@pytest.fixture
def cylinder_step_path(tmp_path) -> str:
    path = str(tmp_path / "cylinder.step")
    _write_step(BRepPrimAPI_MakeCylinder(0.5, 2.0).Shape(), path)
    return path


@pytest.fixture
def cylinder_mm_step_path(tmp_path) -> str:
    """Cylinder radius 500, height 2000, declared in millimetres."""
    path = str(tmp_path / "cylinder_mm.step")
    _write_step(BRepPrimAPI_MakeCylinder(500.0, 2000.0).Shape(), path, unit="MM")
    return path


@pytest.fixture
def cylinder_iges_path(tmp_path) -> str:
    # writer converts OCCT-internal mm to the declared unit: 500 mm -> 0.5 m
    writer = IGESControl_Writer("M", 1)  # 1 = write solids as BREP
    writer.AddShape(BRepPrimAPI_MakeCylinder(500.0, 2000.0).Shape())
    writer.ComputeModel()
    path = str(tmp_path / "cylinder.iges")
    writer.Write(path)
    return path


@pytest.fixture
def cylinder_brep_path(tmp_path) -> str:
    """BREP is unitless; OCCT-native mm: radius 500 -> 0.5 m."""
    path = str(tmp_path / "cylinder.brep")
    breptools.Write(BRepPrimAPI_MakeCylinder(500.0, 2000.0).Shape(), path)
    return path


@pytest.fixture
def sphere_step_path(tmp_path) -> str:
    path = str(tmp_path / "sphere.step")
    _write_step(BRepPrimAPI_MakeSphere(0.75).Shape(), path)
    return path


@pytest.fixture
def peg_in_hole_step_path(tmp_path) -> str:
    """Block with a through-hole (r=0.1 along Z at (0.25,0.25)) plus a peg
    (r=0.1, h=1.0) passing through it: a cylindrical mate."""
    block = BRepPrimAPI_MakeBox(0.5, 0.5, 0.2).Shape()
    hole = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.25, 0.25, -0.1), gp_Dir(0, 0, 1)), 0.1, 0.4
    ).Shape()
    block = BRepAlgoAPI_Cut(block, hole).Shape()
    peg = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.25, 0.25, -0.4), gp_Dir(0, 0, 1)), 0.1, 1.0
    ).Shape()

    compound = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound)
    builder.Add(compound, block)
    builder.Add(compound, peg)

    path = str(tmp_path / "peg_in_hole.step")
    _write_step(compound, path)
    return path


@pytest.fixture
def holed_plate_step_path(tmp_path) -> str:
    """1x1x0.1 plate with an 8x8 grid of through-holes: ~70 faces, ~7k triangles."""
    plate = BRepPrimAPI_MakeBox(1.0, 1.0, 0.1).Shape()
    for i in range(8):
        for j in range(8):
            hole = BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(0.06 + i * 0.12, 0.06 + j * 0.12, -0.1), gp_Dir(0, 0, 1)),
                0.02, 0.4,
            ).Shape()
            plate = BRepAlgoAPI_Cut(plate, hole).Shape()
    path = str(tmp_path / "plate.step")
    _write_step(plate, path)
    return path


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


from pathlib import Path as _Path

_IRB_STEP = _Path(__file__).resolve().parents[2] / "corpus" / "irb1300_asm.stp"


@pytest.fixture(scope="session")
def irb1300_zip(tmp_path_factory):
    """The ABB IRB 1300 through the real pipeline (~15 s). Skips if the STEP file is absent."""
    if not _IRB_STEP.exists():
        pytest.skip("corpus/irb1300_asm.stp not present")
    import yaml
    from corpus import run
    exp = yaml.safe_load((run.HERE / "expected" / "irb1300.yaml").read_text())
    out = tmp_path_factory.mktemp("irb") / "irb1300.zip"
    s = run.score_model(exp, _IRB_STEP, robot_name="irb1300", zip_out=out)
    assert s["export_ok"], s.get("export_error")
    return out
