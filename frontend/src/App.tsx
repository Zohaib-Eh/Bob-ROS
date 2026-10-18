import { useCallback, useEffect, useMemo, useReducer, useRef, useState } from "react";
import { detectJoints } from "./api/client";
import type { StepId } from "./model/checklist";
import { DetectionRunner, pairsToDetect, type Progress } from "./model/detection";
import { historyReducer, initialHistory } from "./model/history";
import { draftForPair, emptyDraft, snapAt, type JointDraft } from "./model/jointDraft";
import { nextJoint } from "./model/nextJoint";
import { partProblems } from "./model/problems";
import Banner from "./ui/Banner";
import RestoreBanner from "./ui/RestoreBanner";
import { useAutosave } from "./ui/useAutosave";
import Browser from "./ui/Browser";
import { glyphsFor, tintsFor } from "./ui/colors";
import type { Ctx, PickedFace, PickMode } from "./ui/ctx";
import InfoCard from "./ui/InfoCard";
import JointTool from "./ui/JointTool";
import MarkerLayer from "./ui/MarkerLayer";
import { looseLines, markersFor, type Marker } from "./ui/markers";
import MenuBar from "./ui/MenuBar";
import ProgramMode from "./ui/program/ProgramMode";
import { routePick } from "./ui/pick";
import { deleteAction, linksOfSelection, selectedBodies, type Selection } from "./ui/selection";
import { shortcutFor } from "./ui/shortcuts";
import StepPanel from "./ui/steps/StepPanel";
import Stepper from "./ui/Stepper";
import ViewCube from "./ui/ViewCube";
import ViewerPane from "./ui/ViewerPane";
import type { Hit, Viewer } from "./viewer/Viewer";

export default function App() {
  const [history, dispatch] = useReducer(historyReducer, undefined, initialHistory);
  const model = history.present;
  const autosave = useAutosave(model, dispatch);
  const [step, goTo] = useState<StepId>("parts");
  const [selection, setSelection] = useState<Selection>(null);
  const [pick, setPick] = useState<PickMode>("select");
  const [jointDraft, setJointDraft] = useState<JointDraft | null>(null);
  const [pickedFace, setPickedFace] = useState<PickedFace | null>(null);
  const [banner, notify] = useState<string | null>(null);
  const [progress, setProgress] = useState<Progress>({ done: 0, total: 0, running: false, paused: false });
  const [robotName, setRobotName] = useState("robot");
  const [scale, setScale] = useState(0.001);
  const [viewer, setViewer] = useState<Viewer | null>(null);
  const [autoGrouped, setAutoGrouped] = useState(false);
  const [hovered, setHoveredBody] = useState<string | null>(null);

  const runner = useRef<DetectionRunner>();
  if (!runner.current) {
    runner.current = new DetectionRunner(
      (a, b, la, lb, signal) => detectJoints(a, b, la, lb, signal),
      {
        onResult: (p, result) => dispatch({ type: "setSuggestion", suggestion: { ...p, result, status: "review" } }),
        onProgress: setProgress,
        onError: (m) => notify(`Joint detection paused: ${m}`),
      });
  }
  useEffect(() => {
    runner.current!.update(pairsToDetect(model), (linkId) =>
      (model.links.find((l) => l.id === linkId)?.body_ids ?? []).map((id) => model.bodies[id]));
  }, [model]);

  // the viewer calls back outside React's render: read the latest state through a ref
  const live = useRef({ pick, model, selection, jointDraft, viewer });
  live.current = { pick, model, selection, jointDraft, viewer };

  const onPick = useCallback((hit: Hit, additive: boolean) => {
    const s = live.current;
    const r = routePick(s.pick, hit, additive, s.model, s.selection, s.jointDraft);
    if (r.selection !== undefined) setSelection(r.selection);
    if (r.draft) setJointDraft(r.draft);
    if (r.face) setPickedFace(r.face);
    if (r.action) dispatch(r.action);
    if (r.mode) setPick(r.mode);
  }, []);

  useEffect(() => {
    if (!viewer) return;
    viewer.onHover((hit) => {
      setHoveredBody(hit?.bodyId ?? null);   // React skips the render when the id is unchanged
      if (live.current.pick !== "origin") return;
      const b = hit ? live.current.model.bodies[hit.bodyId] : undefined;
      viewer.setSnap(b && hit ? snapAt(b, hit.triangle) : null);
    });
    if (pick !== "origin") viewer.setSnap(null);
  }, [viewer, pick]);

  const drafting = jointDraft !== null;
  useEffect(() => {
    viewer?.onLimitDrag(drafting ? (which, value) => setJointDraft((d) => (d ? { ...d, [which]: value } : d)) : null);
  }, [viewer, drafting]);

  useEffect(() => {
    if (step !== "connect") { setJointDraft(null); setPick("select"); viewer?.setPreview([], null); }
  }, [step, viewer]);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const s = shortcutFor(e);
      if (!s) return;
      e.preventDefault();
      const cur = live.current;
      if (s === "undo") dispatch({ type: "undo" });
      else if (s === "redo") dispatch({ type: "redo" });
      else if (s === "joint") { goTo("connect"); setJointDraft((d) => d ?? emptyDraft()); setPick("fixed"); }
      else if (s === "group") {
        const ids = linksOfSelection(cur.model, cur.selection);
        if (ids.length > 1) dispatch({ type: "merge", linkIds: ids });
      } else if (s === "focus") cur.viewer?.focus(selectedBodies(cur.model, cur.selection));
      else if (s === "animate") window.dispatchEvent(new Event("cad:animate"));
      else if (s === "rename") window.dispatchEvent(new Event("cad:rename"));
      else if (s === "delete") {
        const a = deleteAction(cur.model, cur.selection);
        if (a) { dispatch(a); setSelection(null); }
      }
      else if (cur.pick !== "select") setPick("select");
      else if (cur.jointDraft) { setJointDraft(null); cur.viewer?.setPreview([], null); }
      else setSelection(null);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  const bodies = useMemo(() => Object.values(model.bodies), [model.bodies]);
  const selected = useMemo(() => selectedBodies(model, selection), [model, selection]);
  const tints = useMemo(() => tintsFor(step, model, jointDraft), [step, model, jointDraft]);
  const glyphs = useMemo(() => glyphsFor(step, model, jointDraft), [step, model, jointDraft]);
  const markers = useMemo(() => markersFor(step, model), [step, model]);
  const loose = useMemo(() => (step === "connect" ? looseLines(model) : []), [step, model]);
  const approximate = useMemo(() => partProblems(model).filter((p) => p.kind === "approximate").map((p) => p.bodyId), [model]);

  const onMarker = (mk: Marker) => {
    if (mk.kind === "loose" && mk.target?.kind === "link") {
      const n = nextJoint(model, mk.target.id);
      if (n) { setJointDraft(draftForPair(model, n)); return; }
    }
    setSelection(mk.target);
    if (mk.kind === "problem") goTo("parts");
  };

  const ctx: Ctx = { model, dispatch, canUndo: history.past.length > 0, canRedo: history.future.length > 0, undoDepth: history.past.length, step,
    goTo, selection, setSelection, pick, setPick, jointDraft, setJointDraft, pickedFace, setPickedFace, viewer, notify, scale, setScale,
    robotName, setRobotName, autoGrouped, setAutoGrouped, detecting: progress.running, hovered };

  if (step === "program") {
    return (
      <ProgramMode ctx={ctx} preview={(
        <div className="viewer-pane">
          <ViewerPane bodies={bodies} ignored={model.ignored} selected={selected} tints={tints}
            approximate={approximate} glyphs={glyphs} loose={loose} onPick={onPick} onViewer={setViewer} />
          <ViewCube viewer={viewer} />
        </div>
      )} />
    );
  }

  return (
    <div className="app">
      <MenuBar ctx={ctx} />
      <Stepper ctx={ctx} progress={progress} onCancel={() => runner.current!.cancel()} onResume={() => runner.current!.resume()} />
      <Banner message={banner} onClose={() => notify(null)} />
      <RestoreBanner offer={autosave.offer} onRestore={autosave.restore} onDiscard={autosave.discard} />
      <div className="workspace">
        <aside className="panel left"><Browser ctx={ctx} /></aside>
        <main className={`viewport${pick !== "select" ? " picking" : ""}`}>
          <ViewerPane bodies={bodies} ignored={model.ignored} selected={selected} tints={tints}
            approximate={approximate} glyphs={glyphs} loose={loose} onPick={onPick} onViewer={setViewer} />
          <MarkerLayer viewer={viewer} markers={markers} onClick={onMarker} />
          <ViewCube viewer={viewer} />
          {!jointDraft && <InfoCard ctx={ctx} />}
          {jointDraft && step === "connect" && <JointTool ctx={ctx} />}
        </main>
        <aside className="panel right"><StepPanel ctx={ctx} /></aside>
      </div>
    </div>
  );
}
