import { ArrowLeftRight, Check, Crosshair, Lock, MoveHorizontal, Pause, Play, RotateCw, Square, Trash2, X, type LucideIcon } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import type { Vec3 } from "../api/types";
import {
  draftFromJoint, draftPreviewJoint, draftProblem, draftToJoint, setMotion, type JointDraft, type Motion,
} from "../model/jointDraft";
import { directJoints } from "../model/tree";
import { NAME_RE, type ModelState } from "../model/types";
import { jointMotion, previewRange, subtreeLinks } from "../viewer/preview";
import type { Ctx, PickMode } from "./ctx";
import { deg, rad, shortName } from "./format";
import type { Term } from "./glossary";
import Info from "./Info";

const MOTIONS: { id: Motion; label: string; term: Term; icon: LucideIcon }[] = [
  { id: "rigid", label: "Rigid", term: "rigid", icon: Lock },
  { id: "revolute", label: "Revolute", term: "revolute", icon: RotateCw },
  { id: "slider", label: "Slider", term: "slider", icon: MoveHorizontal },
  { id: "planar", label: "Planar", term: "planar", icon: Square },
];

/** Bodies that move with the draft's joint: the child subtree seen from the base, or — when the pair
 *  isn't connected to the base yet — the child link and everything joined to it, never crossing the parent. */
export function movingBodies(m: ModelState, d: JointDraft): string[] {
  const j = draftPreviewJoint(d);
  if (!j) return [];
  const others = m.joints.filter((x) => x.id !== j.id);
  let links = subtreeLinks(directJoints(m.links, [...others, j], m.rootLinkId).joints, j.id);
  if (!links.length) {
    links = [j.child_link_id];
    const seen = new Set([j.parent_link_id, j.child_link_id]);
    for (let i = 0; i < links.length; i++) {
      for (const o of others) {
        const next = o.parent_link_id === links[i] ? o.child_link_id : o.child_link_id === links[i] ? o.parent_link_id : null;
        if (next && !seen.has(next)) { seen.add(next); links.push(next); }
      }
    }
  }
  return m.links.filter((l) => links.includes(l.id)).flatMap((l) => l.body_ids);
}

export default function JointTool({ ctx }: { ctx: Ctx }) {
  const { model, dispatch, jointDraft: d, setJointDraft, pick, setPick, viewer } = ctx;
  const [animating, setAnimating] = useState(false);
  const stored = d ? model.joints.find((j) => j.id === d.id) ?? null : null;
  const opened = useRef(stored);

  // undo/redo changed or removed the joint being edited: follow the stored joint, or close
  useEffect(() => {
    if (!d || stored === opened.current) return;
    opened.current = stored;
    setJointDraft(stored ? draftFromJoint(stored) : null);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [stored]);

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

  return (
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
      {d.motion === "revolute" && (
        <label className="row small">
          <input type="checkbox" data-testid="no-limits" checked={d.noLimits} onChange={(e) => set({ noLimits: e.target.checked })} />
          No limits — spins freely <Info term="continuous" />
        </label>
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
  );
}
