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
