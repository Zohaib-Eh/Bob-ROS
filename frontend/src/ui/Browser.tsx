import { Anchor } from "lucide-react";
import ChainView from "./ChainView";
import { hex, LINK_HUES } from "./colors";
import type { Ctx } from "./ctx";
import { shortName } from "./format";
import { JointIcon } from "./icons";

export default function Browser({ ctx }: { ctx: Ctx }) {
  const { model, selection, setSelection, step } = ctx;
  const selBodies = selection?.kind === "bodies" ? selection.ids : [];
  const bodies = Object.values(model.bodies);
  const name = (id: string) => model.links.find((l) => l.id === id)?.name ?? id;
  return (
    <div className="browser">
      <details open className="browser-section">
        <summary>Parts <span className="badge">{bodies.length}</span></summary>
        {bodies.map((b) => (
          <button key={b.id} data-testid={`part-${b.id}`} title={b.name} draggable
            onDragStart={(e) => e.dataTransfer.setData("text/body-id", b.id)}
            className={`browser-row${selBodies.includes(b.id) ? " selected" : ""}${model.ignored.includes(b.id) ? " ignored" : ""}`}
            onClick={(e) => setSelection({ kind: "bodies",
              ids: e.shiftKey ? [...new Set([...selBodies, b.id])] : [b.id] })}>
            <span className={`dot ${b.status}`} />{shortName(b.name)}
          </button>
        ))}
      </details>
      <details open className="browser-section">
        <summary>Links <span className="badge">{model.links.length}</span></summary>
        {model.links.map((l, i) => (
          <button key={l.id} data-testid={`link-${l.id}`} title={l.name}
            className={`browser-row${selection?.kind === "link" && selection.id === l.id ? " selected" : ""}`}
            onClick={() => setSelection({ kind: "link", id: l.id })}>
            <span className="swatch" style={{ background: step === "group" ? hex(LINK_HUES[i % LINK_HUES.length]) : "transparent" }} />
            {l.id === model.rootLinkId && <Anchor size={12} data-base aria-label="base link" />}
            {shortName(l.name)}
            <span className="spacer" /><span className="muted">{l.body_ids.length}</span>
          </button>
        ))}
      </details>
      <details open className="browser-section">
        <summary>Joints <span className="badge">{model.joints.length}</span></summary>
        {model.joints.map((j) => (
          <button key={j.id} data-testid={`jointrow-${j.id}`}
            className={`browser-row${selection?.kind === "joint" && selection.id === j.id ? " selected" : ""}`}
            onClick={() => setSelection({ kind: "joint", id: j.id })}>
            <JointIcon type={j.type} />{shortName(name(j.parent_link_id), 12)} → {shortName(name(j.child_link_id), 12)}
          </button>
        ))}
        {!model.joints.length && <div className="muted small">No joints yet</div>}
      </details>
      <ChainView ctx={ctx} />
    </div>
  );
}
