import os
import uuid

import numpy as np
from OCC.Core.BRep import BRep_Builder, BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepTools import breptools
from OCC.Core.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Sphere
from OCC.Core.gp import gp_Pnt, gp_Trsf
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.IGESControl import IGESControl_Reader
from OCC.Core.Interface import Interface_Static
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import TopoDS_Shape

from app.models import Body, Face

_LINEAR_DEFLECTION = 0.001  # meters
_MM_TO_M = 0.001


def _read_shape(file_path: str):
    """Read a CAD file into a shape in meters.

    STEP/IGES carry a unit; Interface_Static (process-global, so set on every
    call) makes OCCT convert to meters. BREP is unitless: assumed OCCT-native
    millimetres and scaled by 0.001.
    """
    ext = os.path.splitext(file_path)[1].lower()
    if ext in (".step", ".stp", ".iges", ".igs"):
        reader = STEPControl_Reader() if ext in (".step", ".stp") else IGESControl_Reader()
        # statics only exist once a reader has been constructed
        Interface_Static.SetCVal("xstep.cascade.unit", "M")
        if reader.ReadFile(file_path) != IFSelect_RetDone:
            raise ValueError(f"Could not read CAD file: {file_path}")
        reader.TransferRoots()
        return reader.OneShape()
    if ext == ".brep":
        shape = TopoDS_Shape()
        if not breptools.Read(shape, file_path, BRep_Builder()):
            raise ValueError(f"Could not read BREP file: {file_path}")
        trsf = gp_Trsf()
        trsf.SetScale(gp_Pnt(0, 0, 0), _MM_TO_M)
        return BRepBuilderAPI_Transform(shape, trsf, True).Shape()
    raise ValueError(f"Unsupported CAD file extension: {ext!r}")


def _face_to_model(face, triangle_indices: list[int]) -> Face:
    adaptor = BRepAdaptor_Surface(face, True)
    t = adaptor.GetType()
    if t == GeomAbs_Plane:
        pl = adaptor.Plane()
        loc, n = pl.Location(), pl.Axis().Direction()
        return Face(kind="plane", origin="brep", normal=(n.X(), n.Y(), n.Z()),
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


def import_step(file_path: str) -> list[Body]:
    """Import STEP/IGES/BREP (by extension) as one Body per solid, in meters."""
    shape = _read_shape(file_path)
    bodies: list[Body] = []
    ex = TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        vertices, triangles, per_face = _extract_mesh(ex.Current())
        faces = [_face_to_model(f, ids) for f, ids in per_face if ids]
        bodies.append(Body(id=str(uuid.uuid4()), name=f"solid_{len(bodies)}",
                           vertices=vertices, triangles=triangles, faces=faces))
        ex.Next()
    if not bodies:
        raise ValueError(f"No solids found in {file_path}")
    return bodies
