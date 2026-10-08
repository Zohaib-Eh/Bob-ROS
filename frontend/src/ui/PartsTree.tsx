import type { Dispatch } from "react";
import type { HistoryAction } from "../model/history";
import { smallLinks } from "../model/smallParts";
import { NAME_RE, type ModelState } from "../model/types";
import type { Selection } from "./selection";

const ICON = { ok: "", repaired: "✓", approximate: "⚠", failed: "✖" } as const;

interface Props {
  model: ModelState;
  dispatch: Dispatch<HistoryAction>;
  selection: Selection;
  setSelection: (s: Selection) => void;
  errors: Record<string, string[]>;
}

export default function PartsTree({ model, dispatch, selection, setSelection, errors }: Props) {
  const selectedBodies = selection?.kind === "bodies" ? selection.ids : [];
  const small = smallLinks(model.links, model.bodies);
  const names = model.links.map((l) => l.name);

  const clickPart = (id: string, additive: boolean) =>
    setSelection({ kind: "bodies", ids: additive ? [...new Set([...selectedBodies, id])] : [id] });

  const mergeSelected = () => {
    const linkIds = [...new Set(selectedBodies.map((b) => model.links.find((l) => l.body_ids.includes(b))?.id)
      .filter((x): x is string => !!x))];
    dispatch({ type: "merge", linkIds });
  };

  return (
    <div>
      {model.fileWarnings.map((w, i) => <div key={i} className="status-approximate">⚠ <span>{w.message}</span></div>)}
      <div style={{ display: "flex", gap: 4, margin: "4px 0" }}>
        <button data-testid="merge-selected" disabled={selectedBodies.length < 2} onClick={mergeSelected}>
          Merge into link
        </button>
        <button data-testid="merge-small" disabled={!small.length}
          onClick={() => dispatch({ type: "mergeSmall", pairs: small })}>
          Merge {small.length} small parts into neighbours
        </button>
      </div>
      {model.links.map((l) => {
        const nameError = !NAME_RE.test(l.name) ? "Use letters, digits, _ and - only."
          : names.filter((n) => n === l.name).length > 1 ? "Name already used." : null;
        return (
          <div key={l.id} data-testid={`link-${l.id}`}
            className={selection?.kind === "link" && selection.id === l.id ? "selected" : ""}
            onDragOver={(e) => e.preventDefault()}
            onDrop={(e) => dispatch({ type: "moveBody", bodyId: e.dataTransfer.getData("text/plain"), toLinkId: l.id })}>
            <div onClick={() => setSelection({ kind: "link", id: l.id })}>
              {model.rootLinkId === l.id && <span title="Base link">⚓</span>}{" "}
              <input value={l.name} aria-label="link name"
                onChange={(e) => dispatch({ type: "renameLink", linkId: l.id, name: e.target.value })} />
              {model.rootLinkId !== l.id && (
                <button onClick={() => dispatch({ type: "setRoot", linkId: l.id })} title="Make base">⚓?</button>
              )}
            </div>
            {nameError && <div className="error">{nameError}</div>}
            {(errors[l.id] ?? []).map((m, i) => <div key={i} className="error">{m}</div>)}
            <ul style={{ margin: 0, paddingLeft: 18 }}>
              {l.body_ids.map((id) => {
                const b = model.bodies[id];
                return (
                  <li key={id} data-testid={`part-${id}`} draggable
                    onDragStart={(e) => e.dataTransfer.setData("text/plain", id)}
                    className={`status-${b.status} ${selectedBodies.includes(id) ? "selected" : ""}`}
                    onClick={(e) => clickPart(id, e.shiftKey)}>
                    {ICON[b.status]} {b.name}
                  </li>
                );
              })}
            </ul>
          </div>
        );
      })}
      {model.ignored.length > 0 && (
        <div>
          <div>Ignored</div>
          <ul>{model.ignored.map((id) => (
            <li key={id} data-testid={`part-${id}`} style={{ opacity: 0.5 }} onClick={(e) => clickPart(id, e.shiftKey)}>
              {model.bodies[id].name}
            </li>))}
          </ul>
        </div>
      )}
    </div>
  );
}
