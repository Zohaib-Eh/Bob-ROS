import { Trash2, Ungroup, X } from "lucide-react";
import { useEffect, useReducer, useState } from "react";
import type { JointType, Vec3 } from "../api/types";
import { bodyBox, linkBox, type Box } from "../model/geometry";
import { draftFromJoint } from "../model/jointDraft";
import type { ModelState } from "../model/types";
import type { Ctx } from "./ctx";
import { deg, fmtLen, shortName } from "./format";
import type { Term } from "./glossary";
import { JointIcon } from "./icons";
import Info from "./Info";
import type { Selection } from "./selection";
import { nameProblem } from "./steps/GroupStep";
import { useLinkMass } from "./useLinkMass";

const STATUS_TEXT = { ok: "Shape OK", repaired: "Shape repaired automatically", approximate: "Approximate shape", failed: "Failed to import" } as const;
const TYPE_TEXT: Partial<Record<JointType, [string, Term]>> = {
  fixed: ["Rigid", "rigid"], revolute: ["Revolute", "revolute"], continuous: ["Revolute, no limits", "continuous"],
  prismatic: ["Slider", "slider"], planar: ["Planar", "planar"],
};
const top = (b: Box): Vec3 => [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, b.max[2]];

function resolve(m: ModelState, s: Selection): { kind: "body" | "link" | "joint"; id: string; at: Vec3 } | null {
  if (s?.kind === "bodies" && s.ids.length === 1) {
    const b = m.bodies[s.ids[0]];
    const box = b ? bodyBox(b) : null;
    return box ? { kind: "body", id: b.id, at: top(box) } : null;
  }
  if (s?.kind === "link") {
    const l = m.links.find((x) => x.id === s.id);
    const box = l ? linkBox(l, m.bodies) : null;
    return l && box ? { kind: "link", id: l.id, at: top(box) } : null;
  }
  if (s?.kind === "joint") {
    const j = m.joints.find((x) => x.id === s.id);
    return j ? { kind: "joint", id: j.id, at: j.origin_xyz } : null;
  }
  return null;
}

/** Details next to the clicked object in 3D (docked top-right when there is no viewer). */
export default function InfoCard({ ctx }: { ctx: Ctx }) {
  const { model, selection, viewer, setSelection } = ctx;
  const [, relayout] = useReducer((n: number) => n + 1, 0);
  useEffect(() => viewer?.onCameraChange(relayout), [viewer]);
  const target = resolve(model, selection);
  if (!target) return null;
  const p = viewer?.screenPoint(target.at);
  const style = p ? { left: p.x, top: p.y } : { right: 130, top: 10, transform: "none" };
  return (
    <div className="floating infocard" style={style} data-testid="infocard" onPointerDown={(e) => e.stopPropagation()}>
      <button className="btn btn-ghost btn-icon close" aria-label="Close" onClick={() => setSelection(null)}><X size={14} /></button>
      {target.kind === "body" && <BodyInfo ctx={ctx} bodyId={target.id} />}
      {target.kind === "link" && <LinkInfo ctx={ctx} linkId={target.id} />}
      {target.kind === "joint" && <JointInfo ctx={ctx} jointId={target.id} />}
    </div>
  );
}

/** Inline name editor: Enter or blur saves, Escape reverts; a problem blocks saving and says why. */
function NameField({ value, testid, problem, onCommit }: {
  value: string; testid: string; problem: (v: string) => string | null; onCommit: (v: string) => void;
}) {
  const [v, setV] = useState(value);
  useEffect(() => setV(value), [value]);
  const err = problem(v);
  const save = () => { if (!err && v !== value) onCommit(v); else setV(value); };
  return (
    <>
      <input className={`field title-field${err ? " invalid" : ""}`} data-testid={testid} value={v} aria-label="Name"
        onChange={(e) => setV(e.target.value)} onBlur={save}
        onKeyDown={(e) => { if (e.key === "Enter") save(); if (e.key === "Escape") { e.stopPropagation(); setV(value); } }} />
      {err && <div className="error-text">{err}</div>}
    </>
  );
}

function BodyInfo({ ctx, bodyId }: { ctx: Ctx; bodyId: string }) {
  const { model, dispatch, goTo, setSelection } = ctx;
  const b = model.bodies[bodyId];
  const link = model.links.find((l) => l.body_ids.includes(bodyId));
  const box = bodyBox(b)!;
  return (
    <>
      <NameField value={b.name} testid={`info-name-${bodyId}`} problem={(v) => (v.trim() ? null : "Give the part a name.")}
        onCommit={(v) => dispatch({ type: "renameBody", bodyId, name: v })} />
      <div className="row"><span className={`dot ${b.status}`} />{STATUS_TEXT[b.status]}
        {b.status === "approximate" && <Info term="approximate" />}</div>
      <div className="muted small">Size {fmtLen(box.max[0] - box.min[0])} × {fmtLen(box.max[1] - box.min[1])} × {fmtLen(box.max[2] - box.min[2])}</div>
      {link && (
        <label className="row" style={{ marginTop: 6 }}>Link <Info term="link" />
          <select className="field" data-testid={`move-${bodyId}`} value={link.id}
            onChange={(e) => dispatch({ type: "moveBody", bodyId, toLinkId: e.target.value })}>
            {model.links.map((l) => <option key={l.id} value={l.id}>{shortName(l.name)}</option>)}
          </select>
        </label>
      )}
      <div className="row" style={{ marginTop: 6 }}>
        {(b.status === "approximate" || b.warnings.length > 0) && <button className="btn" onClick={() => goTo("parts")}>Fix…</button>}
        <button className="btn" data-testid={`info-delete-${bodyId}`} title="Delete from the robot (Delete) — recoverable"
          onClick={() => { dispatch({ type: "ignoreBodies", bodyIds: [bodyId] }); setSelection(null); }}><Trash2 size={14} />Delete</button>
      </div>
    </>
  );
}

function LinkInfo({ ctx, linkId }: { ctx: Ctx; linkId: string }) {
  const { model, dispatch, setSelection } = ctx;
  const link = model.links.find((l) => l.id === linkId)!;
  const { mass, error } = useLinkMass(link, model.bodies, model.links);
  const joints = model.joints.filter((j) => j.parent_link_id === linkId || j.child_link_id === linkId).length;
  return (
    <>
      <div className="row"><NameField value={link.name} testid="info-link-name" problem={(v) => nameProblem(v, linkId, model)}
        onCommit={(v) => dispatch({ type: "renameLink", linkId, name: v })} /><Info term="name" /></div>
      <div className="muted small">{link.body_ids.length} part(s) · {joints} joint(s)</div>
      <div className="small">{error ? <span className="error-text">{error}</span> : mass ? `Mass ${mass.mass_kg.toFixed(3)} kg` : "Computing mass…"}</div>
      <div className="row" style={{ marginTop: 6 }}>
        {link.id === model.rootLinkId
          ? <span className="muted small">This is the base <Info term="base" /></span>
          : <button className="btn" data-testid="make-base" onClick={() => dispatch({ type: "setRoot", linkId })}>Make base</button>}
      </div>
      <div className="row" style={{ marginTop: 6 }}>
        {link.body_ids.length > 1 && (
          <button className="btn" data-testid="info-ungroup" title="One link per part"
            onClick={() => { dispatch({ type: "ungroupLink", linkId }); setSelection(null); }}><Ungroup size={14} />Ungroup</button>
        )}
        <button className="btn" data-testid="info-delete-link" title="Delete the link's parts from the robot — recoverable"
          onClick={() => { dispatch({ type: "ignoreBodies", bodyIds: link.body_ids }); setSelection(null); }}><Trash2 size={14} />Delete</button>
      </div>
    </>
  );
}

function JointInfo({ ctx, jointId }: { ctx: Ctx; jointId: string }) {
  const { model, goTo, setJointDraft } = ctx;
  const j = model.joints.find((x) => x.id === jointId)!;
  const name = (id: string) => shortName(model.links.find((l) => l.id === id)?.name ?? id, 14);
  const [label, term] = TYPE_TEXT[j.type] ?? [j.type, "joint" as Term];
  return (
    <>
      <h3>{name(j.parent_link_id)} → {name(j.child_link_id)}</h3>
      <div className="row"><JointIcon type={j.type} />{label} <Info term={term} /></div>
      {j.limit_lower !== null && j.limit_upper !== null && (
        <div className="muted small">Limits {j.type === "prismatic"
          ? `${(j.limit_lower * 1000).toFixed(0)}…${(j.limit_upper * 1000).toFixed(0)} mm`
          : `${deg(j.limit_lower).toFixed(0)}°…${deg(j.limit_upper).toFixed(0)}°`}</div>
      )}
      <div className="row" style={{ marginTop: 6 }}>
        <button className="btn btn-primary" data-testid="info-edit" onClick={() => { goTo("connect"); setJointDraft(draftFromJoint(j)); }}>Edit</button>
      </div>
    </>
  );
}
