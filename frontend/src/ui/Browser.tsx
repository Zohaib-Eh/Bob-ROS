import { Anchor, RotateCcw, Trash2, Ungroup } from "lucide-react";
import { useEffect, useRef, useState, type KeyboardEvent, type ReactNode } from "react";
import ChainView from "./ChainView";
import { hex, LINK_HUES } from "./colors";
import type { Ctx } from "./ctx";
import { shortName } from "./format";
import { JointIcon } from "./icons";
import { nameProblem } from "./steps/GroupStep";

type Editing = { kind: "part" | "link"; id: string; value: string } | null;

export default function Browser({ ctx }: { ctx: Ctx }) {
  const { model, dispatch, selection, setSelection, step, viewer, hovered } = ctx;
  const [editing, setEditing] = useState<Editing>(null);
  const cancelled = useRef(false);   // removing the focused input fires blur: don't commit after Escape
  const selBodies = selection?.kind === "bodies" ? selection.ids : [];
  const bodies = Object.values(model.bodies);
  const linkName = (id: string) => model.links.find((l) => l.id === id)?.name ?? id;

  const startPart = (id: string) => { cancelled.current = false; setEditing({ kind: "part", id, value: model.bodies[id].name }); };
  const startLink = (id: string) => { cancelled.current = false; setEditing({ kind: "link", id, value: linkName(id) }); };

  // F2 renames whatever is selected
  const live = { selection, startPart, startLink };
  useEffect(() => {
    const onRename = () => {
      const s = live.selection;
      if (s?.kind === "bodies" && s.ids.length === 1) live.startPart(s.ids[0]);
      else if (s?.kind === "link") live.startLink(s.id);
    };
    window.addEventListener("cad:rename", onRename);
    return () => window.removeEventListener("cad:rename", onRename);
  });

  const error = editing?.kind === "link" ? nameProblem(editing.value, editing.id, model) : null;
  const commit = () => {
    if (!editing || cancelled.current) return;
    if (editing.kind === "part") dispatch({ type: "renameBody", bodyId: editing.id, name: editing.value });
    else if (!error && editing.value !== linkName(editing.id)) dispatch({ type: "renameLink", linkId: editing.id, name: editing.value });
    setEditing(null);
  };
  const onKey = (e: KeyboardEvent<HTMLInputElement>) => {
    if (e.key === "Enter" && !error) commit();
    if (e.key === "Escape") { e.stopPropagation(); cancelled.current = true; setEditing(null); }
  };
  const nameOrEditor = (kind: "part" | "link", id: string, label: string): ReactNode =>
    editing?.kind === kind && editing.id === id ? (
      <span className="rename" onClick={(e) => e.stopPropagation()}>
        <input className={`field${error ? " invalid" : ""}`} data-testid="rename-input" autoFocus value={editing.value}
          onChange={(e) => setEditing({ ...editing, value: e.target.value })} onKeyDown={onKey}
          onBlur={() => (error ? setEditing(null) : commit())} />
        {error && <span className="error-text" data-testid="rename-error">{error}</span>}
      </span>
    ) : <span className="name" title={`${label} — double-click or F2 to rename`}>{shortName(label)}</span>;
  const hoverBodies = (ids: string[]) => ({
    onMouseEnter: () => viewer?.setHovered(ids),
    onMouseLeave: () => viewer?.setHovered([]),
  });
  const act = (fn: () => void) => (e: { stopPropagation: () => void }) => { e.stopPropagation(); fn(); };

  return (
    <div className="browser">
      <details open className="browser-section">
        <summary>Parts <span className="badge">{bodies.length}</span></summary>
        {bodies.map((b) => {
          const ignored = model.ignored.includes(b.id);
          return (
            <div key={b.id} role="button" tabIndex={0} data-testid={`part-${b.id}`} draggable={!ignored}
              onDragStart={(e) => e.dataTransfer.setData("text/body-id", b.id)}
              className={`browser-row${selBodies.includes(b.id) ? " selected" : ""}${ignored ? " ignored" : ""}${hovered === b.id ? " hovered" : ""}`}
              {...hoverBodies([b.id])}
              onClick={(e) => setSelection({ kind: "bodies", ids: e.shiftKey ? [...new Set([...selBodies, b.id])] : [b.id] })}
              onDoubleClick={() => { startPart(b.id); viewer?.focus([b.id]); }}>
              <span className={`dot ${b.status}`} />{nameOrEditor("part", b.id, b.name)}
              <span className="spacer" />
              {ignored
                ? <button className="row-action" data-testid={`restore-${b.id}`} title="Use this part again" aria-label="Use again"
                    onClick={act(() => dispatch({ type: "unignoreBody", bodyId: b.id }))}><RotateCcw size={12} /></button>
                : <button className="row-action" data-testid={`delete-part-${b.id}`} title="Delete from the robot (Delete)" aria-label="Delete part"
                    onClick={act(() => dispatch({ type: "ignoreBodies", bodyIds: [b.id] }))}><Trash2 size={12} /></button>}
            </div>
          );
        })}
      </details>
      <details open className="browser-section">
        <summary>Links <span className="badge">{model.links.length}</span></summary>
        {model.links.map((l, i) => (
          <div key={l.id} role="button" tabIndex={0} data-testid={`link-${l.id}`}
            className={`browser-row${selection?.kind === "link" && selection.id === l.id ? " selected" : ""}${hovered && l.body_ids.includes(hovered) ? " hovered" : ""}`}
            {...hoverBodies(l.body_ids)}
            onClick={() => setSelection({ kind: "link", id: l.id })}
            onDoubleClick={() => { startLink(l.id); viewer?.focus(l.body_ids); }}>
            <span className="swatch" style={{ background: step === "group" ? hex(LINK_HUES[i % LINK_HUES.length]) : "transparent" }} />
            {l.id === model.rootLinkId && <Anchor size={12} data-base aria-label="base link" />}
            {nameOrEditor("link", l.id, l.name)}
            <span className="spacer" /><span className="muted">{l.body_ids.length}</span>
            {l.body_ids.length > 1 && (
              <button className="row-action" data-testid={`ungroup-${l.id}`} title="Ungroup: one link per part" aria-label="Ungroup"
                onClick={act(() => dispatch({ type: "ungroupLink", linkId: l.id }))}><Ungroup size={12} /></button>
            )}
            <button className="row-action" data-testid={`delete-link-${l.id}`} title="Delete the link's parts from the robot" aria-label="Delete link"
              onClick={act(() => dispatch({ type: "ignoreBodies", bodyIds: l.body_ids }))}><Trash2 size={12} /></button>
          </div>
        ))}
      </details>
      <details open className="browser-section">
        <summary>Joints <span className="badge">{model.joints.length}</span></summary>
        {model.joints.map((j) => (
          <div key={j.id} role="button" tabIndex={0} data-testid={`jointrow-${j.id}`}
            className={`browser-row${selection?.kind === "joint" && selection.id === j.id ? " selected" : ""}`}
            onClick={() => setSelection({ kind: "joint", id: j.id })}>
            <JointIcon type={j.type} />{shortName(linkName(j.parent_link_id), 12)} → {shortName(linkName(j.child_link_id), 12)}
          </div>
        ))}
        {!model.joints.length && <div className="muted small">No joints yet</div>}
      </details>
      <ChainView ctx={ctx} />
    </div>
  );
}
