import { useState, type Dispatch } from "react";
import { importFile } from "../api/client";
import type { HistoryAction } from "../model/history";
import { loadProject, saveProject } from "../model/project";
import { NAME_RE, type ModelState } from "../model/types";
import { downloadBlob } from "./download";

const NEEDS_SCALE = /\.(stl|obj|brep)$/i;

interface Props {
  model: ModelState;
  dispatch: Dispatch<HistoryAction>;
  canUndo: boolean;
  canRedo: boolean;
  setBanner: (m: string | null) => void;
  robotName: string;
  setRobotName: (n: string) => void;
  onExport: () => void;
  exportDisabledReason: string | null;
}

export default function TopBar(p: Props) {
  const [scale, setScale] = useState(0.001);
  const [lastScale, setLastScale] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const onImport = async (file: File) => {
    setBusy(true);
    try {
      const res = await importFile(file, scale);
      p.dispatch({ type: "import", bodies: res.bodies, warnings: res.warnings });
      setLastScale(NEEDS_SCALE.test(file.name) ? `${file.name}: scale ${res.applied_scale}` : null);
    } catch (e) {
      p.setBanner(`Import of ${file.name} failed: ${(e as Error).message}`);
    } finally {
      setBusy(false);
    }
  };

  const onOpen = async (file: File) => {
    try {
      p.dispatch({ type: "load", model: loadProject(await file.text()) });
    } catch (e) {
      p.setBanner((e as Error).message);
    }
  };

  const nameValid = NAME_RE.test(p.robotName);
  return (
    <div className="topbar">
      <label>Import <input data-testid="import-input" type="file" disabled={busy}
        accept=".step,.stp,.iges,.igs,.brep,.stl,.obj"
        onChange={(e) => { const f = e.target.files?.[0]; e.target.value = ""; if (f) void onImport(f); }} /></label>
      <label>Scale (STL/OBJ/BREP) <input data-testid="scale-input" type="number" step="any" value={scale}
        style={{ width: 80 }} onChange={(e) => setScale(Number(e.target.value))} /></label>
      {busy && <span>Importing…</span>}
      {lastScale && <span>{lastScale}</span>}
      <button data-testid="undo" disabled={!p.canUndo} onClick={() => p.dispatch({ type: "undo" })}>Undo</button>
      <button data-testid="redo" disabled={!p.canRedo} onClick={() => p.dispatch({ type: "redo" })}>Redo</button>
      <button data-testid="save-project"
        onClick={() => downloadBlob(new Blob([saveProject(p.model)], { type: "application/json" }), "project.json")}>
        Save project
      </button>
      <label>Open project <input data-testid="open-input" type="file" accept=".json"
        onChange={(e) => { const f = e.target.files?.[0]; e.target.value = ""; if (f) void onOpen(f); }} /></label>
      <span style={{ marginLeft: "auto" }} />
      <input data-testid="robot-name" value={p.robotName} aria-label="robot name"
        onChange={(e) => p.setRobotName(e.target.value)} />
      {!nameValid && <span className="error">Use letters, digits, _ and - only.</span>}
      <button data-testid="export" disabled={!!p.exportDisabledReason || !nameValid}
        title={p.exportDisabledReason ?? ""} onClick={p.onExport}>Export</button>
    </div>
  );
}
