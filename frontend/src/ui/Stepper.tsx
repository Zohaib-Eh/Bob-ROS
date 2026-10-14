import { Check, FolderOpen, Redo2, Save, Undo2, Upload } from "lucide-react";
import { useState } from "react";
import { importFile } from "../api/client";
import { exportChecklist, type StepId } from "../model/checklist";
import type { Progress } from "../model/detection";
import { chainDepth } from "../model/nextJoint";
import { partProblems } from "../model/problems";
import { loadProject, saveProject } from "../model/project";
import type { Ctx } from "./ctx";
import { downloadBlob } from "./download";

export const STEPS: { id: StepId; label: string }[] = [
  { id: "parts", label: "Parts" }, { id: "group", label: "Group" },
  { id: "connect", label: "Connect" }, { id: "export", label: "Export" }, { id: "program", label: "Program" },
];

export function stepBadges(ctx: Pick<Ctx, "model" | "robotName">): Record<StepId, number> {
  const m = ctx.model;
  if (!m.links.length) return { parts: 0, group: 0, connect: 0, export: 0, program: 0 };
  const checks = exportChecklist(m, ctx.robotName);
  const depth = chainDepth(m);
  return {
    parts: partProblems(m).filter((p) => p.kind !== "warning").length,
    group: checks.find((c) => c.id === "names")!.linkIds.length,
    connect: m.links.filter((l) => !depth.has(l.id)).length,
    export: checks.filter((c) => !c.ok && !c.warning).length,
    program: 0,
  };
}

interface Props { ctx: Ctx; progress: Progress; onCancel: () => void; onResume: () => void }

export default function Stepper({ ctx, progress, onCancel, onResume }: Props) {
  const { model, dispatch, step, goTo, notify, scale } = ctx;
  const [busy, setBusy] = useState(false);
  const badges = stepBadges(ctx);

  const onImport = async (file: File) => {
    setBusy(true);
    try {
      const res = await importFile(file, scale);
      dispatch({ type: "import", bodies: res.bodies, warnings: res.warnings });
      goTo("parts");
    } catch (e) {
      notify(`Import of ${file.name} failed: ${(e as Error).message}`);
    } finally {
      setBusy(false);
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
  const working = progress.total > 0 && progress.done < progress.total;

  return (
    <header className="stepper">
      <span className="brand">CAD → URDF</span>
      {STEPS.map((s, i) => (
        <span key={s.id} className="row">
          {i > 0 && <span className="step-sep" />}
          <button className={`step-tab${step === s.id ? " current" : ""}`} data-testid={`step-${s.id}`} onClick={() => goTo(s.id)}>
            <span className="num">{i + 1}</span>{s.label}
            {model.links.length > 0 && (badges[s.id] > 0
              ? <span className="badge danger">{badges[s.id]}</span>
              : <Check size={13} className="done" aria-label="done" />)}
          </button>
        </span>
      ))}
      <span className="spacer" />
      {working && (
        <span className="progress">
          Checking joints {progress.done}/{progress.total}
          {progress.paused
            ? <button className="btn btn-ghost" onClick={onResume}>Resume</button>
            : <button className="btn btn-ghost" onClick={onCancel}>Pause</button>}
        </span>
      )}
      <label className={`btn file-btn${busy ? " active" : ""}`} title="Import STEP, IGES, BREP, STL or OBJ">
        <Upload size={14} />{busy ? "Importing…" : "Import"}
        <input data-testid="import-input" type="file" disabled={busy} accept=".step,.stp,.iges,.igs,.brep,.stl,.obj"
          onChange={(e) => { const f = e.target.files?.[0]; e.target.value = ""; if (f) void onImport(f); }} />
      </label>
      <button className="btn btn-icon" data-testid="undo" title="Undo (Ctrl+Z)" aria-label="Undo" disabled={!ctx.canUndo}
        onClick={() => dispatch({ type: "undo" })}><Undo2 size={14} /></button>
      <button className="btn btn-icon" data-testid="redo" title="Redo (Ctrl+Y)" aria-label="Redo" disabled={!ctx.canRedo}
        onClick={() => dispatch({ type: "redo" })}><Redo2 size={14} /></button>
      <button className="btn btn-icon" data-testid="save-project" title="Save project" aria-label="Save project"
        onClick={() => downloadBlob(new Blob([saveProject(model)], { type: "application/json" }), "project.json")}>
        <Save size={14} />
      </button>
      <label className="btn btn-icon file-btn" title="Open project" aria-label="Open project">
        <FolderOpen size={14} />
        <input data-testid="open-input" type="file" accept=".json"
          onChange={(e) => { const f = e.target.files?.[0]; e.target.value = ""; if (f) void onOpen(f); }} />
      </label>
    </header>
  );
}
