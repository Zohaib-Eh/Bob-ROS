import { useEffect, useRef, useState } from "react";
import { exportRobot, importFile, type ImportProgress } from "../api/client";
import { simulateBlocked } from "../model/checklist";
import { buildExportRequest } from "../model/exportRequest";
import { loadProject, saveProject } from "../model/project";
import { emptyModel } from "../model/types";
import type { Ctx } from "./ctx";
import { downloadBlob } from "./download";
import { fmtElapsed, useElapsed } from "./useElapsed";

const IMPORT_LABEL = { upload: "Uploading", read: "Reading file", parts: "Building parts", faces: "Finding faces" } as const;
const VIEW_KEY = "cad-to-urdf.view";
const IN_SIM = "Switch to Build to change the robot.";
type Menu = "File" | "Edit" | "View";

function loadView(): { grid: boolean; edges: boolean } {
  try { return { grid: true, edges: true, ...JSON.parse(localStorage.getItem(VIEW_KEY) ?? "{}") }; }
  catch { return { grid: true, edges: true }; }
}

/** The top bar: File / Edit / View menus and the Build | Simulate workbench switch.
 *  `lock` disables the switch with that reason (Simulate passes it while a task runs). */
export default function MenuBar({ ctx, lock = null }: { ctx: Ctx; lock?: string | null }) {
  const { model, dispatch, step, goTo, notify, scale, viewer, robotName } = ctx;
  const sim = step === "program";
  const [menu, setMenu] = useState<Menu | null>(null);
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
    const onKey = (e: KeyboardEvent) => { if (e.key === "Escape") setMenu(null); };
    const onDown = (e: MouseEvent) => { if (!(e.target as Element).closest?.(".menubar")) setMenu(null); };
    document.addEventListener("keydown", onKey);
    document.addEventListener("mousedown", onDown);
    return () => { document.removeEventListener("keydown", onKey); document.removeEventListener("mousedown", onDown); };
  }, [menu]);

  const toggle = (k: "grid" | "edges") => {
    const next = { ...view, [k]: !view[k] };
    setView(next);
    try { localStorage.setItem(VIEW_KEY, JSON.stringify(next)); } catch { /* private window: not remembered */ }
  };
  const onImport = async (file: File) => {
    setBusy(true);
    try {
      const res = await importFile(file, scale, setImp);
      dispatch({ type: "import", bodies: res.bodies, warnings: res.warnings, source: file.name });
      goTo("parts");
    } catch (e) {
      notify(`Import of ${file.name} failed: ${(e as Error).message}`);
    } finally {
      setBusy(false);
      setImp(null);
    }
  };
  const onOpen = async (file: File) => {
    try {
      dispatch({ type: "load", model: loadProject(await file.text()) });
      ctx.setAutoGrouped(true);   // an opened project is already arranged
    } catch (e) {
      notify((e as Error).message);
    }
  };
  const onExport = async () => {
    const req = buildExportRequest(model);
    try {
      const { blob } = await exportRobot(robotName, req.links, req.joints, req.bodiesByLink, req.gripper);
      downloadBlob(blob, `${robotName}.zip`);
    } catch (e) { notify(`Export failed: ${(e as Error).message}`); }
  };
  const onNew = () => {
    if (model.links.length && !window.confirm("Start a new project? Unsaved changes are lost.")) return;
    dispatch({ type: "load", model: emptyModel() });
    goTo("parts");
  };

  const item = (label: string, run: () => void, disabled = false, title?: string) => (
    <button role="menuitem" className="menu-item" disabled={disabled} title={title}
      onClick={() => { setMenu(null); run(); }}>{label}</button>
  );
  const head = (name: Menu) => (
    <button className={`menu-head${menu === name ? " open" : ""}`} aria-haspopup="menu" aria-expanded={menu === name}
      onClick={() => setMenu(menu === name ? null : name)}>{name}</button>
  );
  const why = sim ? IN_SIM : undefined;

  return (
    <header className="menubar">
      <span className="brand">CAD → URDF</span>
      <span className="menu">{head("File")}
        {menu === "File" && <div role="menu" className="menu-pop">
          {item("New project", onNew, sim, why)}
          {item("Open project…", () => openRef.current?.click(), sim, why)}
          {item("Save project", () => downloadBlob(new Blob([saveProject(model)], { type: "application/json" }), "project.json"))}
          {item("Import CAD…", () => importRef.current?.click(), sim || busy, why)}
          {item("Add part from CAD…", () => importRef.current?.click(), sim || busy || !Object.keys(model.bodies).length,
            sim ? why : "Adds another file's parts to this robot, e.g. a gripper")}
          {item("Export URDF + MuJoCo (.zip)", () => void onExport(), !!blocked, blocked ?? undefined)}
        </div>}
      </span>
      <span className="menu">{head("Edit")}
        {menu === "Edit" && <div role="menu" className="menu-pop">
          {item("Undo (Ctrl+Z)", () => dispatch({ type: "undo" }), !ctx.canUndo)}
          {item("Redo (Ctrl+Y)", () => dispatch({ type: "redo" }), !ctx.canRedo)}
        </div>}
      </span>
      <span className="menu">{head("View")}
        {menu === "View" && <div role="menu" className="menu-pop">
          <button role="menuitemcheckbox" aria-checked={view.grid} className="menu-item" onClick={() => toggle("grid")}>
            {view.grid ? "✓ " : ""}Grid</button>
          <button role="menuitemcheckbox" aria-checked={view.edges} className="menu-item" onClick={() => toggle("edges")}>
            {view.edges ? "✓ " : ""}Part edges</button>
          {item("Fit to view", () => viewer?.setView("home"))}
        </div>}
      </span>
      <input ref={importRef} data-testid="import-input" type="file" hidden disabled={busy}
        accept=".step,.stp,.iges,.igs,.brep,.stl,.obj"
        onChange={(e) => { const f = e.target.files?.[0]; e.target.value = ""; if (f) void onImport(f); }} />
      <input ref={openRef} data-testid="open-input" type="file" hidden accept=".json"
        onChange={(e) => { const f = e.target.files?.[0]; e.target.value = ""; if (f) void onOpen(f); }} />
      <span className="workbench" role="group" aria-label="Workbench">
        <button className={`wb${!sim ? " current" : ""}`} disabled={!!lock} title={lock ?? undefined}
          onClick={() => { if (sim) goTo("export"); }}>Build</button>
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
