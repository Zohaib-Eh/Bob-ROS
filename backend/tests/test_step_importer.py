import math

import numpy as np
import pytest
import trimesh
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.BRepTools import breptools
from OCC.Core.gp import gp_Pnt
from OCC.Core.Interface import Interface_Static
from OCC.Core.TopAbs import TopAbs_SHELL
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Compound

from app.importers import step_importer
from app.importers.step_importer import import_cad, import_step


def _mesh(body):
    m = trimesh.Trimesh(vertices=body.vertices, faces=body.triangles, process=False)
    return m


def test_box_import_has_six_planar_brep_faces(box_step_path):
    bodies = import_step(box_step_path)
    assert len(bodies) == 1
    faces = bodies[0].faces
    assert len(faces) == 6
    assert all(f.kind == "plane" for f in faces)
    assert all(f.origin == "brep" for f in faces)
    assert all(f.residual == 0.0 for f in faces)


def test_box_mesh_is_watertight_with_volume_6(box_step_path):
    body = import_step(box_step_path)[0]
    m = _mesh(body)
    assert m.is_watertight
    assert math.isclose(m.volume, 6.0, rel_tol=1e-3)
    # face triangle indices must stay valid and cover every triangle once
    idx = sorted(i for f in body.faces for i in f.triangle_indices)
    assert idx == list(range(len(body.triangles)))


def test_cylinder_import_has_correct_radius_and_axis(cylinder_step_path):
    faces = import_step(cylinder_step_path)[0].faces
    cyl_faces = [f for f in faces if f.kind == "cylinder"]
    assert len(cyl_faces) == 1
    assert math.isclose(cyl_faces[0].radius, 0.5, rel_tol=1e-6)
    assert math.isclose(abs(cyl_faces[0].axis[2]), 1.0, rel_tol=1e-6)


def test_cylinder_mesh_volume(cylinder_step_path):
    m = _mesh(import_step(cylinder_step_path)[0])
    assert math.isclose(m.volume, math.pi * 0.25 * 2, rel_tol=0.01)


def test_sphere_import_has_correct_radius(sphere_step_path):
    faces = import_step(sphere_step_path)[0].faces
    sph_faces = [f for f in faces if f.kind == "sphere"]
    assert len(sph_faces) == 1
    assert math.isclose(sph_faces[0].radius, 0.75, rel_tol=1e-6)


def test_peg_in_hole_import_returns_two_bodies(peg_in_hole_step_path):
    bodies = import_step(peg_in_hole_step_path)
    assert len(bodies) == 2
    for b in bodies:
        radii = [f.radius for f in b.faces if f.kind == "cylinder"]
        assert any(math.isclose(r, 0.1, rel_tol=1e-6) for r in radii)


def test_step_declared_in_mm_imports_in_meters(cylinder_mm_step_path):
    faces = import_step(cylinder_mm_step_path)[0].faces
    cyl = [f for f in faces if f.kind == "cylinder"][0]
    assert math.isclose(cyl.radius, 0.5, rel_tol=1e-6)


def test_iges_import_in_meters(cylinder_iges_path):
    faces = import_step(cylinder_iges_path)[0].faces
    cyl = [f for f in faces if f.kind == "cylinder"][0]
    assert math.isclose(cyl.radius, 0.5, rel_tol=1e-6)


def test_brep_import_scaled_to_meters(cylinder_brep_path):
    faces = import_step(cylinder_brep_path)[0].faces
    cyl = [f for f in faces if f.kind == "cylinder"][0]
    assert math.isclose(cyl.radius, 0.5, rel_tol=1e-6)


def test_unknown_extension_raises(tmp_path):
    p = tmp_path / "x.foo"
    p.write_text("x")
    with pytest.raises(ValueError):
        import_step(str(p))


def test_box_plane_normals_point_outward(box_step_path):
    body = import_step(box_step_path)[0]
    verts = trimesh.Trimesh(body.vertices, body.triangles, process=False).triangles_center
    centroid = (0.5, 1.0, 1.5)
    for f in body.faces:
        c = verts[f.triangle_indices].mean(axis=0)
        d = sum(n * (ci - ci0) for n, ci, ci0 in zip(f.normal, c, centroid))
        assert d > 0


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_brep_scale_must_be_finite_positive(cylinder_brep_path, bad):
    with pytest.raises(ValueError, match="scale"):
        import_step(cylinder_brep_path, brep_scale=bad)


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


def test_failing_part_becomes_failed_body_not_exception(mixed_topology_brep_path, monkeypatch, caplog):
    real = step_importer._sew_to_solids
    calls = []

    def flaky(shapes):
        calls.append(1)
        if len(calls) == 2:  # the open shell (second shell group)
            raise RuntimeError("boom")
        return real(shapes)

    monkeypatch.setattr(step_importer, "_sew_to_solids", flaky)
    with caplog.at_level("ERROR", logger="app.importers.step_importer"):
        bodies, warnings = import_cad(mixed_topology_brep_path, brep_scale=1.0)
    assert any(r.exc_info for r in caplog.records)
    assert len(bodies) == 4
    failed = [b for b in bodies if b.status == "failed"]
    assert len(failed) == 1 and failed[0].vertices == []
    assert failed[0].warnings[0].code == "degenerate"
    assert "RuntimeError" in failed[0].warnings[0].message
    assert failed[0].warnings[0].fixes == ["replace", "ignore"]
    assert warnings == []  # failed part's faces still count as accounted


def test_unmeshable_part_is_failed_body(mixed_topology_brep_path, monkeypatch):
    monkeypatch.setattr(step_importer, "_extract_mesh", lambda s: ([], [], []))
    bodies, _ = import_cad(mixed_topology_brep_path, brep_scale=1.0)
    assert {b.status for b in bodies} == {"failed"}


def test_loose_faces_sewn_per_connected_group(tmp_path):
    from app.importers.step_importer import _subshapes
    from OCC.Core.TopAbs import TopAbs_FACE
    f = _subshapes(BRepPrimAPI_MakeBox(1, 1, 1).Shape(), TopAbs_FACE)
    far = _subshapes(BRepPrimAPI_MakeBox(gp_Pnt(9, 0, 0), 1, 1, 1).Shape(), TopAbs_FACE)[0]
    adjacent = next(x for x in f[1:] if _shares_edge(f[0], x))
    b = BRep_Builder(); comp = TopoDS_Compound(); b.MakeCompound(comp)
    for x in (f[0], adjacent, far):
        b.Add(comp, x)
    path = str(tmp_path / "loose.brep"); breptools.Write(comp, path)
    bodies, warnings = import_cad(path, brep_scale=1.0)
    assert sorted(len(x.faces) for x in bodies) == [1, 2]
    assert all(x.name.startswith("surface_") for x in bodies)
    assert warnings == []


def _shares_edge(a, b):
    from OCC.Core.TopAbs import TopAbs_EDGE
    from app.importers.step_importer import _subshapes
    ea = _subshapes(a, TopAbs_EDGE)
    return any(e.IsSame(g) for e in ea for g in _subshapes(b, TopAbs_EDGE))


def test_step_product_names_become_body_names(tmp_path):
    from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
    from OCC.Core.TDocStd import TDocStd_Document
    from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool
    from OCC.Core.TDataStd import TDataStd_Name
    from OCC.Core.STEPControl import STEPControl_AsIs

    from OCC.Core.XCAFApp import XCAFApp_Application

    doc = TDocStd_Document("MDTV-XCAF")
    XCAFApp_Application.GetApplication().NewDocument("MDTV-XCAF", doc)
    tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    asm = tool.NewShape()
    TDataStd_Name.Set(asm, "ASSEMBLY")
    for nm, x in (("BASE_PLATE", 0.0), ("ARM_LINK", 3.0)):
        part = tool.AddShape(BRepPrimAPI_MakeBox(gp_Pnt(x, 0, 0), 1, 1, 1).Shape(), False)
        TDataStd_Name.Set(part, nm)
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


def test_empty_step_product_becomes_failed_body(tmp_path):
    from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
    from OCC.Core.TDocStd import TDocStd_Document
    from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool
    from OCC.Core.TDataStd import TDataStd_Name
    from OCC.Core.STEPControl import STEPControl_AsIs
    from OCC.Core.XCAFApp import XCAFApp_Application

    doc = TDocStd_Document("MDTV-XCAF")
    XCAFApp_Application.GetApplication().NewDocument("MDTV-XCAF", doc)
    tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    asm = tool.NewShape()
    TDataStd_Name.Set(asm, "ASSEMBLY")
    empty = TopoDS_Compound(); BRep_Builder().MakeCompound(empty)
    for nm, shape in (("REAL_BOX", BRepPrimAPI_MakeBox(1, 1, 1).Shape()), ("EMPTY_PART", empty)):
        part = tool.AddShape(shape, False)
        TDataStd_Name.Set(part, nm)
        tool.AddComponent(asm, part, TopLoc_Location())
    tool.UpdateAssemblies()
    w = STEPCAFControl_Writer()
    Interface_Static.SetCVal("write.step.unit", "M")
    w.Transfer(doc, STEPControl_AsIs)
    path = str(tmp_path / "empty_product.step"); w.Write(path)

    bodies, _ = import_cad(path)
    assert [b.name for b in bodies] == ["REAL_BOX", "EMPTY_PART"]
    assert bodies[1].status == "failed"
    assert bodies[1].warnings[0].code == "degenerate"
    assert bodies[1].warnings[0].fixes == ["replace", "ignore"]


def _plane_with_normal(body, n):
    return next(f for f in body.faces if f.kind == "plane" and np.allclose(f.normal, n, atol=1e-9))


def test_plane_face_centre_is_its_hole_centre(tmp_path):
    """A plate's top face around a through-hole: the joint axis passes through the hole, not the
    plane's parametric origin (a corner of the plate)."""
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCC.Core.gp import gp_Ax2, gp_Dir
    from tests.conftest import _write_step
    plate = BRepPrimAPI_MakeBox(0.2, 0.2, 0.05).Shape()
    hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0.13, 0.07, -0.1), gp_Dir(0, 0, 1)), 0.03, 0.3).Shape()
    path = str(tmp_path / "plate.step")
    _write_step(BRepAlgoAPI_Cut(plate, hole).Shape(), path)
    top = _plane_with_normal(import_step(path)[0], (0, 0, 1))
    assert top.center == pytest.approx((0.13, 0.07, 0.05), abs=1e-6)


def test_plane_face_without_circles_is_centred_on_its_area(box_step_path):
    side = _plane_with_normal(import_step(box_step_path)[0], (1, 0, 0))   # 1 x 2 x 3 box
    assert side.center == pytest.approx((1.0, 1.0, 1.5), abs=1e-6)
