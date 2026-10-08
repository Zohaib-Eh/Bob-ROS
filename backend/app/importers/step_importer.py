import math
import os
import uuid

import numpy as np
import trimesh
from OCC.Core.BRep import BRep_Builder, BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing, BRepBuilderAPI_Transform,
)
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepTools import breptools
from OCC.Core.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Sphere
from OCC.Core.gp import gp_Pnt, gp_Trsf
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.IGESControl import IGESControl_Reader
from OCC.Core.Interface import Interface_Static
from OCC.Core.ShapeFix import ShapeFix_Solid
from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TDF import TDF_Label, TDF_LabelSequence
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SHELL, TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import TopoDS_Shape, TopoDS_Shell, topods

from OCC.Core.XCAFApp import XCAFApp_Application
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool

from app.models import Body, Face, PartWarning

_LINEAR_DEFLECTION = 0.001  # meters


def _read_shape(file_path: str, brep_scale: float):
    """Read a CAD file into a shape in meters.

    STEP/IGES carry a unit; Interface_Static (process-global, so set on every
    call) makes OCCT convert to meters. BREP is unitless: scaled by
    `brep_scale` (default 0.001 = OCCT-native millimetres).
    """
    ext = os.path.splitext(file_path)[1].lower()
    if ext in (".step", ".stp", ".iges", ".igs"):
        reader = STEPControl_Reader() if ext in (".step", ".stp") else IGESControl_Reader()
        # statics only exist once a reader has been constructed
        Interface_Static.SetCVal("xstep.cascade.unit", "M")
        if reader.ReadFile(file_path) != IFSelect_RetDone:
            raise ValueError("Could not read CAD file")
        reader.TransferRoots()
        shape = reader.OneShape()
        if shape is None or shape.IsNull():  # e.g. IGES "reads" junk as 0 entities
            raise ValueError("Could not read CAD file")
        return shape
    if ext == ".brep":
        shape = TopoDS_Shape()
        if not breptools.Read(shape, file_path, BRep_Builder()):
            raise ValueError("Could not read BREP file")
        trsf = gp_Trsf()
        trsf.SetScale(gp_Pnt(0, 0, 0), brep_scale)
        return BRepBuilderAPI_Transform(shape, trsf, True).Shape()
    raise ValueError(f"Unsupported CAD file extension: {ext!r}")


def _label_name(label) -> str | None:
    return (label.GetLabelName() or "").strip() or None


def _read_step_named(file_path: str) -> list[tuple[str | None, object]]:
    """[(product name, located shape)] per leaf part of a STEP file, in assembly order, meters."""
    doc = TDocStd_Document("MDTV-XCAF")
    XCAFApp_Application.GetApplication().NewDocument("MDTV-XCAF", doc)
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
    if not parts:
        raise ValueError("Could not read CAD file")
    return parts


def _face_to_model(face, triangle_indices: list[int]) -> Face:
    adaptor = BRepAdaptor_Surface(face, True)
    t = adaptor.GetType()
    if t == GeomAbs_Plane:
        pl = adaptor.Plane()
        loc, n = pl.Location(), pl.Axis().Direction()
        s = -1.0 if face.Orientation() == TopAbs_REVERSED else 1.0  # outward
        return Face(kind="plane", origin="brep", normal=(s * n.X(), s * n.Y(), s * n.Z()),
                    center=(loc.X(), loc.Y(), loc.Z()), residual=0.0,
                    triangle_indices=triangle_indices)
    if t == GeomAbs_Cylinder:
        cyl = adaptor.Cylinder()
        ax = cyl.Axis()
        loc, d = ax.Location(), ax.Direction()
        return Face(kind="cylinder", origin="brep", axis=(d.X(), d.Y(), d.Z()),
                    center=(loc.X(), loc.Y(), loc.Z()), radius=cyl.Radius(),
                    residual=0.0, triangle_indices=triangle_indices)
    if t == GeomAbs_Sphere:
        sph = adaptor.Sphere()
        loc = sph.Location()
        return Face(kind="sphere", origin="brep", center=(loc.X(), loc.Y(), loc.Z()),
                    radius=sph.Radius(), residual=0.0,
                    triangle_indices=triangle_indices)
    return Face(kind="unclassified", origin="brep", center=(0.0, 0.0, 0.0),
                residual=0.0, triangle_indices=triangle_indices)


def _extract_mesh(solid):
    """Triangulate; return (vertices, triangles, [(occ_face, triangle ids)]).
    Winding is outward for every face; coincident vertices are merged."""
    BRepMesh_IncrementalMesh(solid, _LINEAR_DEFLECTION)
    verts: list[tuple[float, float, float]] = []
    tris: list[tuple[int, int, int]] = []
    per_face = []
    ex = TopExp_Explorer(solid, TopAbs_FACE)
    while ex.More():
        face = ex.Current()
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation(face, loc)
        ids: list[int] = []
        if tri is not None:
            base = len(verts)
            trsf = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X(), p.Y(), p.Z()))
            flip = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = (base + k - 1 for k in tri.Triangle(i).Get())
                if flip:
                    b, c = c, b
                ids.append(len(tris))
                tris.append((a, b, c))
        per_face.append((face, ids))
        ex.Next()

    if not verts:
        return [], [], per_face

    # Merge coincident vertices (1e-9 m grid); triangle order/count unchanged.
    arr = np.asarray(verts, dtype=float).reshape(-1, 3)
    _, first, inverse = np.unique(
        np.round(arr / 1e-9).astype(np.int64), axis=0,
        return_index=True, return_inverse=True,
    )
    inverse = inverse.reshape(-1)
    merged = [tuple(map(float, arr[i])) for i in first]
    tris = [tuple(int(inverse[k]) for k in t) for t in tris]
    return merged, tris, per_face


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
    for f in _subshapes(sewn, TopAbs_FACE, TopAbs_SHELL):  # faces sewing left unattached
        shell = TopoDS_Shell()
        BRep_Builder().MakeShell(shell)
        BRep_Builder().Add(shell, f)
        shells.append(shell)
    solids = []
    for sh in shells:
        fix = ShapeFix_Solid(BRepBuilderAPI_MakeSolid(topods.Shell(sh)).Solid())
        fix.Perform()
        solids.append(fix.Solid())
    return solids


def _open_edge_count(vertices, triangles) -> int:
    mesh = trimesh.Trimesh(np.asarray(vertices), np.asarray(triangles), process=False)
    return len(trimesh.grouping.group_rows(mesh.edges_sorted, require_count=1))


def _is_watertight(vertices, triangles) -> bool:
    return trimesh.Trimesh(np.asarray(vertices), np.asarray(triangles), process=False).is_watertight


def _failed_body(name: str, reason: str) -> Body:
    return Body(id=str(uuid.uuid4()), name=name, vertices=[], triangles=[], status="failed",
                warnings=[PartWarning(code="degenerate", fixes=["replace", "ignore"],
                                      message=f"{name}: {reason}")])


def _make_body(solid, name: str, repaired: bool) -> Body:
    vertices, triangles, per_face = _extract_mesh(solid)
    if not triangles:
        return _failed_body(name, "could not be meshed")
    faces = [_face_to_model(f, ids) for f, ids in per_face if ids]
    status, warnings = "ok", []
    if repaired:
        n_open = _open_edge_count(vertices, triangles)
        if n_open == 0 and _is_watertight(vertices, triangles):
            status = "repaired"
            warnings = [PartWarning(code="shell_repaired", fixes=[],
                                    message=f"{name}: surface shell sewn into a closed solid")]
        else:
            status = "approximate"
            warnings = [PartWarning(code="open_geometry", fixes=["set_mass", "replace", "ignore"],
                                    message=f"{name}: {n_open} open edges remain after repair"
                                            f"{' (non-manifold)' if n_open == 0 else ''}; "
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
    ext = os.path.splitext(file_path)[1].lower()
    named = (_read_step_named(file_path) if ext in (".step", ".stp")
             else [(None, _read_shape(file_path, brep_scale))])

    bodies: list[Body] = []
    total_faces = accounted = 0
    for product, shape in named:
        total_faces += len(_subshapes(shape, TopAbs_FACE))
        groups = [([s], "solid", False) for s in _subshapes(shape, TopAbs_SOLID)]
        groups += [([sh], "shell", True) for sh in _subshapes(shape, TopAbs_SHELL, TopAbs_SOLID)]
        loose = _loose_faces(shape)
        if loose:
            groups.append((loose, "surface", True))

        mine: list[Body] = []
        for sources, prefix, repaired in groups:
            n = len(bodies) + len(mine)
            try:
                solids = _sew_to_solids(sources) if repaired else sources
                new = [_make_body(s, f"{prefix}_{n + i}", repaired) for i, s in enumerate(solids)]
                accounted += sum(len(_subshapes(s, TopAbs_FACE)) for s in solids)
            except Exception as e:  # one bad part must not fail the import
                new = [_failed_body(f"{prefix}_{n}", f"import failed ({type(e).__name__})")]
                accounted += sum(len(_subshapes(s, TopAbs_FACE)) for s in sources)
            mine += new
        if product:  # bare name for one body, name_k for several
            for k, b in enumerate(mine):
                new_name = product if len(mine) == 1 else f"{product}_{k}"
                for w in b.warnings:
                    w.message = w.message.replace(f"{b.name}: ", f"{new_name}: ", 1)
                b.name = new_name
        bodies += mine
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
