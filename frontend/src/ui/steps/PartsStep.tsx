import { Check, FileUp, Ruler } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { replacePart } from "../../api/client";
import { bodyBox, unionBox } from "../../model/geometry";
import { partProblems, type Problem } from "../../model/problems";
import type { Ctx } from "../ctx";
import { fmtLen, shortName } from "../format";
import Info from "../Info";
import MountCard from "../MountCard";

const REEXPORT = "Re-export this part from your CAD tool as a closed solid (STEP works best), then use Replace.";

export default function PartsStep({ ctx }: { ctx: Ctx }) {
  const { model, dispatch, notify, scale, setScale, selection, setSelection, viewer } = ctx;
  const problems = partProblems(model);
  const live = Object.values(model.bodies).filter((b) => !model.ignored.includes(b.id));
  const box = unionBox(live.map(bodyBox));
  const flagged = new Set(problems.map((p) => p.bodyId));
  const fine = live.filter((b) => !flagged.has(b.id)).length;
  const selected = selection?.kind === "bodies" ? selection.ids : [];

  const replace = async (bodyId: string, file: File) => {
    const body = model.bodies[bodyId];
    try {
      dispatch({ type: "replaceBody", body: await replacePart(file, body.id, body.name, bodyBox(body), scale) });
    } catch (e) {
      notify(`Replace failed: ${(e as Error).message}`);
    }
  };
  const focus = (bodyId: string) => {
    setSelection({ kind: "bodies", ids: [bodyId] });
    viewer?.focus([bodyId]);
  };

  return (
    <section className="step">
      <h2>Parts</h2>
      <p className="muted">Check what came in. Problems are pinned on the model; nothing here blocks you until Export.</p>
      {box && (
        <div className="card" data-testid="size">
          <div className="row">
            <Ruler size={14} /><strong>{fmtLen(box.max[2] - box.min[2])} tall</strong>
            <span className="muted">× {fmtLen(box.max[0] - box.min[0])} × {fmtLen(box.max[1] - box.min[1])}</span>
          </div>
          <label className="row" style={{ marginTop: 6 }}>
            Scale <Info term="scale" />
            <input className="field" data-testid="scale-input" type="number" step="any" value={scale}
              style={{ width: 90 }} onChange={(e) => setScale(Number(e.target.value))} />
          </label>
          <p className="muted small" style={{ marginTop: 6 }}>
            Wrong size? Undo the import (Ctrl+Z), change the scale and import again. STEP and IGES files carry their own units.
          </p>
        </div>
      )}
      {problems.map((p) => (
        <ProblemCard key={`${p.kind}-${p.bodyId}-${p.kind === "warning" ? p.warning.code : ""}`} p={p} ctx={ctx}
          selected={selected.includes(p.bodyId)} onFocus={focus} onReplace={replace} />
      ))}
      <div className="card ok row" data-testid="fine">
        <Check size={14} />{fine} {fine === 1 ? "part looks" : "parts look"} fine.
      </div>
      {model.ignored.length > 0 && (
        <div className="card">
          <h3>Ignored</h3>
          {model.ignored.map((id) => (
            <div className="row" key={id}>
              <span title={model.bodies[id].name}>{shortName(model.bodies[id].name)}</span>
              <span className="spacer" />
              <button className="btn" data-testid={`unignore-${id}`} onClick={() => dispatch({ type: "unignoreBody", bodyId: id })}>Use again</button>
            </div>
          ))}
        </div>
      )}
      <MountCard ctx={ctx} />
    </section>
  );
}

function ProblemCard({ p, ctx, selected, onFocus, onReplace }: {
  p: Problem; ctx: Ctx; selected: boolean; onFocus: (id: string) => void; onReplace: (id: string, f: File) => Promise<void>;
}) {
  const { model, dispatch, goTo, setSelection } = ctx;
  const body = model.bodies[p.bodyId];
  const id = body.id;
  const [advice, setAdvice] = useState(false);
  const [mass, setMass] = useState("");
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => { if (selected) ref.current?.scrollIntoView?.({ block: "nearest" }); }, [selected]);

  const replaceBtn = (label: string, primary: boolean) => (
    <label className={`btn file-btn${primary ? " btn-primary" : ""}`} onClick={(e) => e.stopPropagation()}>
      <FileUp size={14} />{label}
      <input data-testid={`replace-${id}`} type="file" accept=".step,.stp,.iges,.igs,.brep,.stl,.obj"
        onChange={(e) => { const f = e.target.files?.[0]; e.target.value = ""; if (f) void onReplace(id, f); }} />
    </label>
  );
  const ignoreBtn = <button className="btn" data-testid={`ignore-${id}`} onClick={() => dispatch({ type: "ignoreBody", bodyId: id })}>Ignore it</button>;
  const adviceBtn = <button className="btn btn-ghost" data-testid={`advice-${id}`} onClick={() => setAdvice(true)}>How to re-export</button>;
  const kindClass = p.kind === "warning" ? "attention" : "problem";

  return (
    <div ref={ref} className={`card ${kindClass}${selected ? " selected-card" : ""}`} data-testid={`problem-${id}`} onClick={() => onFocus(id)}>
      <h3 title={body.name}><span className={`dot ${body.status}`} />{shortName(body.name, 32)}</h3>
      {p.kind === "failed" && (
        <>
          <p>This part has no usable shape. <Info term="failed" /></p>
          <div className="row">{replaceBtn("Replace with a file…", true)}</div>
          <details onClick={(e) => e.stopPropagation()}><summary>Other options</summary>
            <div className="row">{ignoreBtn}{adviceBtn}</div>
          </details>
        </>
      )}
      {p.kind === "approximate" && (
        <>
          <p>This part's surface has gaps, so its weight is estimated. <Info term="approximate" /></p>
          <button className="btn btn-primary" data-testid={`estimate-${id}`}
            onClick={() => dispatch({ type: "setLinkProps", linkId: p.linkId, props: { accept_approximate_mass: true } })}>
            Use the estimate
          </button>
          <details onClick={(e) => e.stopPropagation()}><summary>Other options</summary>
            <div className="row">
              <input className="field" data-testid={`mass-${id}`} type="number" step="any" min="0" placeholder="Real mass"
                value={mass} style={{ width: 100 }} onChange={(e) => setMass(e.target.value)} /> kg
              <button className="btn" data-testid={`set-mass-${id}`} disabled={!(Number(mass) > 0)}
                onClick={() => dispatch({ type: "setLinkProps", linkId: p.linkId, props: { mass_override_kg: Number(mass) } })}>
                Set mass
              </button>
            </div>
            <p className="muted small">Applies to its whole link: {model.links.find((l) => l.id === p.linkId)?.name}.</p>
            <div className="row">{replaceBtn("Replace…", false)}</div>
          </details>
        </>
      )}
      {p.kind === "warning" && (
        <>
          <p>{p.warning.message}</p>
          <div className="row">
            {p.warning.fixes.includes("replace") && replaceBtn("Replace…", false)}
            {p.warning.fixes.includes("ignore") && ignoreBtn}
            {p.warning.fixes.includes("set_mass") && (
              <button className="btn" onClick={() => {
                const l = model.links.find((x) => x.body_ids.includes(id));
                if (l) { setSelection({ kind: "link", id: l.id }); goTo("group"); }
              }}>Set link mass</button>
            )}
            {p.warning.fixes.includes("reexport") && adviceBtn}
          </div>
        </>
      )}
      {advice && <p className="muted small">{REEXPORT}</p>}
    </div>
  );
}
