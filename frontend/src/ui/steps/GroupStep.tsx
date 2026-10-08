import { Group as GroupIcon, Merge, Undo2 } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { smallLinks } from "../../model/smallParts";
import { DENSITY_PRESETS, NAME_RE, type ModelState } from "../../model/types";
import { hex, LINK_HUES } from "../colors";
import type { Ctx } from "../ctx";
import Info from "../Info";
import { linksOfSelection } from "../selection";
import { useLinkMass } from "../useLinkMass";

export function nameProblem(name: string, linkId: string, m: ModelState): string | null {
  if (!NAME_RE.test(name)) return "Use letters, digits, _ and - only (start with a letter, digit or _).";
  if (m.links.some((l) => l.id !== linkId && l.name === name)) return "Another link already has this name.";
  return null;
}

export default function GroupStep({ ctx }: { ctx: Ctx }) {
  const { model, dispatch, autoGrouped, setAutoGrouped, selection } = ctx;
  const [merged, setMerged] = useState<number | null>(null);
  useEffect(() => {
    if (autoGrouped) return;
    setAutoGrouped(true);
    if (model.joints.length) return;   // merging now could fold a joined link into its neighbour
    const pairs = smallLinks(model.links, model.bodies);
    if (pairs.length) { dispatch({ type: "mergeSmall", pairs }); setMerged(pairs.length); }
    // first visit only
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const depthAtMerge = useRef<number | null>(null);
  useEffect(() => {
    if (merged === null) { depthAtMerge.current = null; return; }
    if (depthAtMerge.current === null) depthAtMerge.current = ctx.undoDepth;
    else if (ctx.undoDepth !== depthAtMerge.current) setMerged(null);   // Undo would now undo something else
  }, [ctx.undoDepth, merged]);
  const small = smallLinks(model.links, model.bodies);
  const selLinks = linksOfSelection(model, selection);

  return (
    <section className="step">
      <h2>Group <Info term="link" /></h2>
      <p className="muted">Parts bolted together move as one link. Each link has its own colour in the 3D view.</p>
      {merged !== null && (
        <div className="card ok" data-testid="auto-merged">
          <div className="row">
            We grouped {merged} small part{merged === 1 ? "" : "s"} (bolts, covers) into the parts they touch.
            <span className="spacer" />
            <button className="btn" data-testid="undo-auto" onClick={() => { dispatch({ type: "undo" }); setMerged(null); }}>
              <Undo2 size={14} />Undo
            </button>
          </div>
        </div>
      )}
      <div className="row">
        <button className="btn" data-testid="group-selected" disabled={selLinks.length < 2} title="Group selected parts (G)"
          onClick={() => dispatch({ type: "merge", linkIds: selLinks })}>
          <GroupIcon size={14} />Group selected <kbd>G</kbd>
        </button>
        {small.length > 0 && (
          <button className="btn" data-testid="merge-small" onClick={() => dispatch({ type: "mergeSmall", pairs: small })}>
            <Merge size={14} />Merge {small.length} small part{small.length === 1 ? "" : "s"}
          </button>
        )}
      </div>
      <p className="muted small">Shift-click parts to select several, or drag a part from the browser onto a link.</p>
      {model.links.map((l, i) => <LinkCard key={l.id} ctx={ctx} linkId={l.id} hue={LINK_HUES[i % LINK_HUES.length]} />)}
    </section>
  );
}

function LinkCard({ ctx, linkId, hue }: { ctx: Ctx; linkId: string; hue: number }) {
  const { model, dispatch, selection, setSelection } = ctx;
  const link = model.links.find((l) => l.id === linkId)!;
  const [name, setName] = useState(link.name);
  useEffect(() => setName(link.name), [link.name]);
  const { mass, error } = useLinkMass(link, model.bodies, model.links);
  const nameError = nameProblem(name, linkId, model);
  const commit = () => {
    if (nameError) setName(link.name);
    else if (name !== link.name) dispatch({ type: "renameLink", linkId, name });
  };
  const preset = Object.entries(DENSITY_PRESETS).find(([, v]) => v === link.density_kg_m3)?.[0] ?? "";
  const isSelected = selection?.kind === "link" && selection.id === linkId;

  return (
    <div className={`card linkcard${isSelected ? " selected-card" : ""}`} data-testid={`linkcard-${linkId}`}
      onDragOver={(e) => e.preventDefault()}
      onDrop={(e) => {
        const b = e.dataTransfer.getData("text/body-id");
        if (b) dispatch({ type: "moveBody", bodyId: b, toLinkId: linkId });
      }}
      onClick={() => setSelection({ kind: "link", id: linkId })}>
      <div className="row">
        <span className="swatch" style={{ background: hex(hue) }} />
        <input className={`field${nameError ? " invalid" : ""}`} data-testid={`name-${linkId}`} value={name}
          aria-label="Link name" style={{ flex: 1 }} onChange={(e) => setName(e.target.value)} onBlur={commit}
          onKeyDown={(e) => { if (e.key === "Enter") (e.target as HTMLInputElement).blur(); }} />
        <Info term="name" />
      </div>
      {nameError && <div className="error-text">{nameError}</div>}
      <div className="row muted small" style={{ marginTop: 4 }}>
        {link.body_ids.length} part{link.body_ids.length === 1 ? "" : "s"}{link.id === model.rootLinkId ? " · base" : ""}
      </div>
      <div className="row" style={{ marginTop: 4 }}>
        <label className="row">Material
          <select className="field" data-testid={`density-${linkId}`} value={preset}
            onChange={(e) => { if (e.target.value) dispatch({ type: "setLinkProps", linkId, props: { density_kg_m3: DENSITY_PRESETS[e.target.value] } }); }}>
            <option value="">Custom ({link.density_kg_m3} kg/m³)</option>
            {Object.keys(DENSITY_PRESETS).map((k) => <option key={k} value={k}>{k}</option>)}
          </select>
        </label>
      </div>
      <div className="row" data-testid={`mass-line-${linkId}`} style={{ marginTop: 4 }}>
        {error ? <span className="error-text">{error}</span>
          : mass ? <>Mass {mass.mass_kg.toFixed(3)} kg <span className="muted small">
              · {link.mass_override_kg !== null ? "set by you" : "estimated from volume"}</span> <Info term="estimated" /></>
          : <span className="muted small">Computing mass…</span>}
      </div>
    </div>
  );
}
