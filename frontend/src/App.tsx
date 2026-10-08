import { useCallback, useEffect, useMemo, useReducer, useRef, useState } from "react";
import type { Matrix4 } from "three";
import { detectJoints } from "./api/client";
import { DetectionRunner, pairsToDetect, type Progress } from "./model/detection";
import { historyReducer, initialHistory } from "./model/history";
import Banner from "./ui/Banner";
import DetailsPanel from "./ui/DetailsPanel";
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
  const jointArrows = useMemo(() => [
    ...model.joints.map((joint) => ({ joint, dashed: false })),
    ...Object.values(model.suggestions).filter((g) => g.status === "review" && g.result.joint)
      .map((g) => ({ joint: g.result.joint!, dashed: true })),
  ], [model.joints, model.suggestions]);

  return (
    <div className="app">
      <div>
        <TopBar model={model} dispatch={dispatch} canUndo={history.past.length > 0} canRedo={history.future.length > 0}
          setBanner={setBanner} robotName={robotName} setRobotName={setRobotName}
          onExport={() => setBanner("Export is added in Task 11.")} exportDisabledReason={null} />
        <Banner message={banner} onClose={() => setBanner(null)} />
      </div>
      <div className="workspace">
        <div className="panel">
          <PartsTree model={model} dispatch={dispatch} selection={selection} setSelection={setSelection} errors={{}} />
        </div>
        <ViewerPane bodies={bodies} ignored={model.ignored}
          selectedBodyIds={selection?.kind === "bodies" ? selection.ids : []}
          joints={jointArrows} preview={preview} onPick={onPick} onViewer={setViewer} />
        <div className="panel right">
          <DetailsPanel model={model} dispatch={dispatch} selection={selection} setSelection={setSelection}
            setBanner={setBanner} viewer={viewer} setPreview={setPreview} lastPick={lastPick} />
        </div>
      </div>
      <StatusBar progress={progress} onCancel={() => runner.current!.cancel()} onResume={() => runner.current!.resume()} />
    </div>
  );
}
