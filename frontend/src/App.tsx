import { useCallback, useEffect, useMemo, useReducer, useRef, useState } from "react";
import type { Matrix4 } from "three";
import { detectJoints, exportRobot } from "./api/client";
import { DetectionRunner, pairsToDetect, type Progress } from "./model/detection";
import { splitExportError } from "./model/errors";
import { buildExportRequest } from "./model/exportRequest";
import { historyReducer, initialHistory } from "./model/history";
import Banner from "./ui/Banner";
import DetailsPanel from "./ui/DetailsPanel";
import { downloadBlob } from "./ui/download";
import PartsTree from "./ui/PartsTree";
import type { Selection } from "./ui/selection";
import StatusBar from "./ui/StatusBar";
import TopBar from "./ui/TopBar";
import ViewerPane from "./ui/ViewerPane";
import type { Viewer } from "./viewer/Viewer";

export default function App() {
  const [history, dispatch] = useReducer(historyReducer, undefined, initialHistory);
  const model = history.present;
  const [selection, setSelection] = useState<Selection>(null);
  const [banner, setBanner] = useState<string | null>(null);
  const [progress, setProgress] = useState<Progress>({ done: 0, total: 0, running: false, paused: false });
  const [robotName, setRobotName] = useState("robot");
  const [scale, setScale] = useState(0.001);
  const [viewer, setViewer] = useState<Viewer | null>(null);
  const [preview, setPreview] = useState<{ bodyIds: string[]; matrix: Matrix4 } | null>(null);
  const [lastPick, setLastPick] = useState<{ bodyId: string; triangle: number } | null>(null);

  const runner = useRef<DetectionRunner>();
  if (!runner.current) {
    runner.current = new DetectionRunner(
      (a, b, la, lb, signal) => detectJoints(a, b, la, lb, signal),
      {
        onResult: (p, result) => dispatch({ type: "setSuggestion", suggestion: { ...p, result, status: "review" } }),
        onProgress: setProgress,
        onError: (m) => setBanner(`Joint detection paused: ${m}`),
      });
  }
  useEffect(() => {
    runner.current!.update(pairsToDetect(model), (linkId) =>
      (model.links.find((l) => l.id === linkId)?.body_ids ?? []).map((id) => model.bodies[id]));
  }, [model]);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (!(e.ctrlKey || e.metaKey) || (e.target as HTMLElement).tagName === "INPUT") return;
      if (e.key === "z") { e.preventDefault(); dispatch({ type: "undo" }); }
      if (e.key === "y") { e.preventDefault(); dispatch({ type: "redo" }); }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  const bodies = useMemo(() => Object.values(model.bodies), [model.bodies]);
  const onPick = useCallback((hit: { bodyId: string; triangle: number } | null, additive: boolean) => {
    setLastPick(hit);
    setSelection((s) => {
      if (!hit) return null;
      const prev = s?.kind === "bodies" ? s.ids : [];
      return { kind: "bodies", ids: additive ? [...new Set([...prev, hit.bodyId])] : [hit.bodyId] };
    });
  }, []);
  const exportReq = useMemo(() => buildExportRequest(model), [model]);
  const [serverErrors, setServerErrors] = useState<Record<string, string[]>>({});
  useEffect(() => setServerErrors({}), [model]);
  const treeErrors = useMemo(() => {
    const out: Record<string, string[]> = {};
    for (const e of exportReq.errors) for (const id of e.linkIds) (out[id] ??= []).push(e.message);
    return out;
  }, [exportReq]);
  const pinned = useMemo(() => {
    const out: Record<string, string[]> = { ...treeErrors };
    for (const [id, msgs] of Object.entries(serverErrors)) (out[id] ??= []).push(...msgs);
    return out;
  }, [treeErrors, serverErrors]);

  const onExport = async () => {
    try {
      const { blob, warnings } = await exportRobot(robotName, exportReq.links, exportReq.joints, exportReq.bodiesByLink);
      downloadBlob(blob, `${robotName}.zip`);
      setBanner(warnings.length ? `Exported. ${warnings.map((w) => w.message).join(" ")}` : null);
    } catch (e) {
      const { byLink, byJoint, general } = splitExportError((e as Error).message);
      setServerErrors(byLink);
      const linkName = (id: string) => model.links.find((l) => l.id === id)?.name ?? id;
      const jointMsgs = Object.entries(byJoint).flatMap(([id, msgs]) => {
        const j = exportReq.joints.find((x) => x.id === id);
        const label = j ? `${linkName(j.parent_link_id)} ↔ ${linkName(j.child_link_id)}: ` : "";
        return msgs.map((m) => label + m);
      });
      setBanner(["Export failed.", ...jointMsgs, ...general].join(" "));
    }
  };

  const jointArrows = useMemo(() => [
    ...model.joints.map((joint) => ({ joint, dashed: false })),
    ...Object.values(model.suggestions).filter((g) => g.status === "review" && g.result.joint)
      .map((g) => ({ joint: g.result.joint!, dashed: true })),
  ], [model.joints, model.suggestions]);

  return (
    <div className="app">
      <div>
        <TopBar model={model} dispatch={dispatch} canUndo={history.past.length > 0} canRedo={history.future.length > 0}
          setBanner={setBanner} robotName={robotName} setRobotName={setRobotName} scale={scale} setScale={setScale}
          onExport={() => void onExport()}
          exportDisabledReason={exportReq.errors[0]?.message ?? (model.links.length ? null : "Import a model first.")} />
        <Banner message={banner} onClose={() => setBanner(null)} />
      </div>
      <div className="workspace">
        <div className="panel">
          <PartsTree model={model} dispatch={dispatch} selection={selection} setSelection={setSelection} errors={pinned} />
        </div>
        <ViewerPane bodies={bodies} ignored={model.ignored}
          selectedBodyIds={selection?.kind === "bodies" ? selection.ids : []}
          joints={jointArrows} preview={preview} onPick={onPick} onViewer={setViewer} />
        <div className="panel right">
          <DetailsPanel model={model} dispatch={dispatch} selection={selection} setSelection={setSelection}
            setBanner={setBanner} viewer={viewer} setPreview={setPreview} lastPick={lastPick} scale={scale} />
        </div>
      </div>
      <StatusBar progress={progress} onCancel={() => runner.current!.cancel()} onResume={() => runner.current!.resume()} />
    </div>
  );
}
