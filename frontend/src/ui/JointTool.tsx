import { ArrowLeftRight, Check, Crosshair, Loader2, Lock, MoveHorizontal, Pause, Play, Radar, RotateCw, Square, Trash2, X, type LucideIcon } from "lucide-react";
import { useEffect, useMemo, useReducer, useRef, useState } from "react";
import type { Vec3 } from "../api/types";
import {
  draftFromJoint, draftPreviewJoint, draftProblem, draftToJoint, isDefaultLimits, parseEffort, setMotion, type JointDraft, type Motion,
} from "../model/jointDraft";
import { NAME_RE } from "../model/types";
import { jointMotion, previewRange } from "../viewer/preview";
import type { Ctx, PickMode } from "./ctx";
import { deg, rad, shortName } from "./format";
import type { Term } from "./glossary";
import { sweepLimits } from "../api/client";
import { useEffortSuggestions } from "./useEffortSuggestions";
import type { SweepResult } from "../api/types";
import { beyondSide, driveRange, fmtJointValue, movingBodies, sweepRequest, sweepSummary } from "./drive";
import Info from "./Info";

const MOTIONS: { id: Motion; label: string; term: Term; icon: LucideIcon }[] = [
  { id: "rigid", label: "Rigid", term: "rigid", icon: Lock },
  { id: "revolute", label: "Revolute", term: "revolute", icon: RotateCw },
  { id: "slider", label: "Slider", term: "slider", icon: MoveHorizontal },
  { id: "planar", label: "Planar", term: "planar", icon: Square },
];

export default function JointTool({ ctx }: { ctx: Ctx }) {
  const { model, dispatch, jointDraft: d, setJointDraft, pick, setPick, viewer } = ctx;
  const [animating, setAnimating] = useState(false);
  const suggestions = useEffortSuggestions(ctx);
  // what was typed in Max torque, so "0" or "0." on the way to "0.18" is not wiped by the parsed value
  const [effortText, setEffortText] = useState<{ id: string; text: string } | null>(null);
  const stored = d ? model.joints.find((j) => j.id === d.id) ?? null : null;
  const opened = useRef({ id: d?.id ?? null, stored });

  // undo/redo changed or removed the joint being edited: follow the stored joint, or close.
  // A different draft (New joint, another pair) is not an undo: just start tracking it.
  useEffect(() => {
    if (!d) return;
    if (opened.current.id !== d.id) { opened.current = { id: d.id, stored }; return; }
    if (stored === opened.current.stored) return;
    opened.current = { id: d.id, stored };
    setJointDraft(stored ? draftFromJoint(stored) : null);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [stored, d?.id]);

  type Sweep = { state: "idle" } | { state: "running" } | { state: "done"; result: SweepResult } | { state: "error"; message: string };
  const [drive, setDrive] = useState(0);
  const [sweep, setSweep] = useState<Sweep>({ state: "idle" });
  const [, relayout] = useReducer((n: number) => n + 1, 0);
  // what the pose and a sweep result depend on; when it changes both are stale
  const shape = d ? JSON.stringify([d.id, d.fixed, d.moving, d.origin, d.axis, d.motion]) : "";
  const live = useRef({ d, shape });
  live.current = { d, shape };
  useEffect(() => { setDrive(0); setSweep({ state: "idle" }); }, [shape]);

  const pj = d ? draftPreviewJoint(d) : null;
  const canDrive = !!pj?.axis && (d?.motion === "revolute" || d?.motion === "slider");
  const bodyIds = useMemo(() => (d && canDrive ? movingBodies(model, d) : []), [model, d, canDrive]);
  const side = d ? beyondSide(d, drive) : null;

  // the drive pose is the preview, except while Animate runs
  useEffect(() => {
    if (!viewer || animating) return;
    if (!canDrive || !pj || drive === 0) viewer.setPreview([], null);
    else viewer.setPreview(bodyIds, jointMotion(pj, drive));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [viewer, animating, drive, canDrive, bodyIds, shape]);
  useEffect(() => {
    // while picking a part or face, a press on the moving part is a pick, not a drive
    viewer?.onDrive(canDrive && pj && pick === "select" ? { joint: pj, bodyIds, value: drive, onChange: setDrive } : null);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [viewer, canDrive, bodyIds, drive, shape, pick]);
  useEffect(() => { viewer?.setWarning(side ? bodyIds : []); }, [viewer, side, bodyIds]);
  useEffect(() => () => { viewer?.onDrive(null); viewer?.setWarning([]); viewer?.setPreview([], null); }, [viewer]);
  // closed tool: a sweep still in flight must not write its draft back
  useEffect(() => () => { live.current.shape = "\0closed"; }, []);
  useEffect(() => viewer?.onCameraChange(relayout), [viewer]);

  const runSweep = async () => {
    const req = d ? sweepRequest(model, d) : null;
    if (!req || sweep.state === "running") return;
    const asked = shape;
    setSweep({ state: "running" });
    try {
      const result = await sweepLimits(req.joint, req.moving, req.obstacles);
      if (live.current.shape !== asked) return;   // the joint changed while sweeping
      setSweep({ state: "done", result });
      const cur = live.current.d;
      if (cur && !result.free && !result.at_rest_overlap.length && result.lower !== null && result.upper !== null) {
        setJointDraft({ ...cur, lower: result.lower, upper: result.upper, noLimits: false });
      }
    } catch (e) {
      if (live.current.shape === asked) setSweep({ state: "error", message: (e as Error).message });
    }
  };

  // a draft still at the default range finds its limits by itself, once per axis/origin/motion
  const atDefault = !!d && !d.noLimits && isDefaultLimits(d.motion, d.lower, d.upper);
  useEffect(() => {
    if (canDrive && atDefault && sweep.state === "idle") void runSweep();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [shape, canDrive, atDefault, sweep.state]);

  useEffect(() => {
    const toggle = () => setAnimating((a) => !a);
    window.addEventListener("cad:animate", toggle);
    return () => window.removeEventListener("cad:animate", toggle);
  }, []);

  useEffect(() => {
    const j = d ? draftPreviewJoint(d) : null;
    if (!animating || !viewer || !d || !j) return;
    const [lo, hi] = previewRange(j);
    if (lo === hi) return;
    const bodyIds = movingBodies(model, d);
    let raf = 0;
    const t0 = performance.now();
    const tick = (t: number) => {
      const s = 0.5 - 0.5 * Math.cos(((t - t0) / 1000) * Math.PI * 0.8);
      viewer.setPreview(bodyIds, jointMotion(j, lo + (hi - lo) * s));
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => { cancelAnimationFrame(raf); viewer.setPreview([], null); };
  }, [animating, viewer, d, model]);

  if (!d) return null;
  const problem = draftProblem(d);
  const linkName = (id: string | null) => (id ? shortName(model.links.find((l) => l.id === id)?.name ?? id, 22) : null);
  const set = (patch: Partial<JointDraft>) => setJointDraft({ ...d, ...patch });
  const close = () => { setAnimating(false); viewer?.setPreview([], null); setPick("select"); setJointDraft(null); };
  const ok = () => {
    const joint = draftToJoint(d);
    if (stored) dispatch({ type: "updateJoint", joint });
    else if (d.suggestionKey) dispatch({ type: "acceptSuggestion", key: d.suggestionKey, joint });
    else dispatch({ type: "addJoint", joint });
    close();
  };
  const pickBtn = (mode: PickMode, value: string | null, empty: string) => (
    <button className={`btn pick${pick === mode ? " active" : ""}`} data-testid={`pick-${mode}`}
      onClick={() => setPick(pick === mode ? "select" : mode)}>
      <Crosshair size={13} />{pick === mode ? "Click in the 3D view…" : value ?? empty}
    </button>
  );
  const limited = (d.motion === "revolute" && !d.noLimits) || d.motion === "slider";
  const slider = d.motion === "slider";
  const shown = (v: number | null) => (v === null ? "" : String(+(slider ? v * 1000 : deg(v)).toFixed(slider ? 2 : 1)));
  const parsed = (s: string) => (s === "" ? null : slider ? Number(s) / 1000 : rad(Number(s)));
  const movingLink = d.moving ? model.links.find((l) => l.id === d.moving) : undefined;
  const canAnimate = !!d.axis && !!d.origin && (d.motion === "revolute" || d.motion === "slider");
  const [driveLo, driveHi] = driveRange(model, d);
  const blocker = sweep.state === "done" && side ? sweep.result.blocked_by[side] : null;
  const label = `${fmtJointValue(drive, slider)}${side ? ` · beyond ${side} limit${blocker ? ` (hits ${linkName(blocker)})` : ""}` : ""}`;
  const labelAt = pj && drive !== 0 ? viewer?.screenPoint(pj.origin_xyz) : null;

  return (
    <>
    <div className="floating jointtool" data-testid="joint-tool" onPointerDown={(e) => e.stopPropagation()}>
      <div className="row"><strong>Joint</strong><Info term="joint" /><span className="spacer" />
        <button className="btn btn-ghost btn-icon" aria-label="Close joint tool" onClick={close}><X size={14} /></button></div>
      {d.suggestionKey && (
        <div className="hint small" data-testid="suggested">
          ✨ Suggested from contact faces <Info term="suggested" />
          {d.detected && <> · Detector thinks: <b>{d.detected}</b></>}
          {d.ambiguous && <> · ⚠ several matching features</>}
        </div>
      )}
      <div className="field-row"><span><span className="swatch fixed" /> ① Fixed part</span>{pickBtn("fixed", linkName(d.fixed), "Pick in 3D")}</div>
      <div className="field-row"><span><span className="swatch moving" /> ② Moving part</span>{pickBtn("moving", linkName(d.moving), "Pick in 3D")}</div>
      <div className="field-row"><span>③ Origin <Info term="origin" /></span>
        {pickBtn("origin", d.origin ? (d.axis ? "Set — click to change" : "Between the parts") : null, "Click a round or flat face")}</div>

      <div className="label">Motion</div>
      <div className="motion-grid">
        {MOTIONS.map((m) => (
          <span key={m.id} className="motion-cell">
            <button className={`btn motion${d.motion === m.id ? " active" : ""}`} data-testid={`motion-${m.id}`}
              onClick={() => setJointDraft(setMotion(d, m.id))}>
              <m.icon size={14} />{m.label}
            </button>
            <Info term={m.term} />
          </span>
        ))}
      </div>
      <div className="row small muted">
        <span className="btn motion" aria-disabled="true">Cylindrical <Info term="cylindrical" /></span>
        <span className="btn motion" aria-disabled="true">Ball <Info term="ball" /></span>
      </div>

      {limited && (
        <div>
          <div className="label">Limits <Info term="limits" /> <span className="muted small">({slider ? "mm" : "°"} — or drag the handles in 3D)</span></div>
          <div className="row">
            <input className="field" data-testid="limit-lower" type="number" step="any" style={{ width: 80 }}
              value={shown(d.lower)} onChange={(e) => set({ lower: parsed(e.target.value) })} />
            to
            <input className="field" data-testid="limit-upper" type="number" step="any" style={{ width: 80 }}
              value={shown(d.upper)} onChange={(e) => set({ upper: parsed(e.target.value) })} />
          </div>
        </div>
      )}
      {(d.motion === "revolute" || d.motion === "slider") && (
        <label className="row">
          {slider ? "Max force" : "Max torque"}
          <input className="field" data-testid="max-effort" type="number" step="any" min="0" style={{ width: 120 }}
            placeholder={suggestions[d.id] ? `auto · ${suggestions[d.id]} ${slider ? "N" : "N·m"}` : "auto"}
            value={effortText?.id === d.id ? effortText.text : d.maxEffort ?? ""}
            onChange={(e) => { setEffortText({ id: d.id, text: e.target.value }); set({ maxEffort: parseEffort(e.target.value) }); }} />
        </label>
      )}
      {d.motion === "revolute" && (
        <label className="row small">
          <input type="checkbox" data-testid="no-limits" checked={d.noLimits} onChange={(e) => set({ noLimits: e.target.checked })} />
          No limits — spins freely <Info term="continuous" />
        </label>
      )}
      {canDrive && (
        <div className="drive">
          <div className="row">
            <button className="btn" data-testid="find-limits" disabled={sweep.state === "running"} onClick={() => void runSweep()}>
              {sweep.state === "running" ? <><Loader2 size={14} className="spin" />Sweeping…</> : <><Radar size={14} />Find limits from geometry</>}
            </button>
            <Info term="sweep" />
          </div>
          {sweep.state === "done" && (
            <div className="small" data-testid="sweep-status">
              {sweepSummary(sweep.result, slider, (id) => model.links.find((l) => l.id === id)?.name ?? id)}
              {sweep.result.free && !d.noLimits && (
                <button className="btn" data-testid="use-no-limits" onClick={() => set({ noLimits: true })}>Use No limits</button>
              )}
            </div>
          )}
          {sweep.state === "error" && <div className="error-text" data-testid="sweep-status">{sweep.message}</div>}
          <div className="label">Drive <Info term="drive" /></div>
          <div className="row">
            <input type="range" data-testid="drive-slider" min={driveLo} max={driveHi} step="any" value={drive}
              style={{ flex: 1 }} onChange={(e) => setDrive(Number(e.target.value))} />
            <input className="field" data-testid="drive-value" type="number" step="any" style={{ width: 72 }}
              value={shown(drive)} onChange={(e) => setDrive(parsed(e.target.value) ?? 0)} />
          </div>
          {limited && (
            <div className="row">
              <button className="btn" data-testid="set-lower-here" disabled={d.upper !== null && drive > d.upper}
                onClick={() => set({ lower: drive })}>Set lower here</button>
              <button className="btn" data-testid="set-upper-here" disabled={d.lower !== null && drive < d.lower}
                onClick={() => set({ upper: drive })}>Set upper here</button>
              <button className="btn btn-ghost" data-testid="reset-drive" disabled={drive === 0} onClick={() => setDrive(0)}>Reset</button>
            </div>
          )}
        </div>
      )}
      {movingLink && (
        <label className="row small">Name this link
          <input className="field" data-testid="moving-name" key={movingLink.id + movingLink.name} defaultValue={movingLink.name}
            style={{ flex: 1 }} onBlur={(e) => {
              const v = e.target.value;
              const taken = model.links.some((l) => l.id !== movingLink.id && l.name === v);
              if (v !== movingLink.name && NAME_RE.test(v) && !taken) dispatch({ type: "renameLink", linkId: movingLink.id, name: v });
              else e.target.value = movingLink.name;
            }} />
          <Info term="name" />
        </label>
      )}

      <div className="row">
        <button className={`btn${animating ? " active" : ""}`} data-testid="animate" disabled={!canAnimate} title="Animate (Space)"
          onClick={() => setAnimating((a) => !a)}>{animating ? <Pause size={14} /> : <Play size={14} />}Animate</button>
        <button className="btn" data-testid="flip" disabled={!d.axis}
          onClick={() => set({ axis: d.axis!.map((x) => -x) as Vec3 })}><ArrowLeftRight size={14} />Flip</button>
      </div>
      {problem && <div className="muted small" data-testid="problem">{problem}</div>}
      <div className="row">
        <span className="spacer" />
        {stored && <button className="btn" data-testid="remove-joint" onClick={() => { dispatch({ type: "removeJoint", jointId: d.id }); close(); }}><Trash2 size={14} />Remove</button>}
        <button className="btn" data-testid="cancel" onClick={close}>Cancel</button>
        <button className="btn btn-primary" data-testid="ok" disabled={!!problem} onClick={ok}><Check size={14} />OK</button>
      </div>
    </div>
    {labelAt && <div className="drive-label" data-testid="drive-label" style={{ left: labelAt.x, top: labelAt.y }}>{label}</div>}
    </>
  );
}
