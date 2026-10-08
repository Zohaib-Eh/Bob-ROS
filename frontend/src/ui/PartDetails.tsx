import { useState } from "react";
import { replacePart } from "../api/client";
import type { Fix } from "../api/types";
import { bodyBox } from "../model/geometry";
import type { DetailsProps } from "./DetailsPanel";

const LABEL: Record<Fix, string> = {
  set_mass: "Set mass", replace: "Replace part…", ignore: "Ignore part", reexport: "How to re-export",
};

export default function PartDetails({ model, dispatch, setSelection, setBanner, bodyId }:
  DetailsProps & { bodyId: string }) {
  const body = model.bodies[bodyId];
  const [advice, setAdvice] = useState(false);
  const [busy, setBusy] = useState(false);
  if (!body) return null;
  const link = model.links.find((l) => l.body_ids.includes(bodyId));
  const ignored = model.ignored.includes(bodyId);

  const onReplace = async (file: File) => {
    setBusy(true);
    try {
      const next = await replacePart(file, body.id, body.name, bodyBox(body), 0.001);
      dispatch({ type: "replaceBody", body: next });
    } catch (e) {
      setBanner(`Replace failed: ${(e as Error).message}`);
    } finally {
      setBusy(false);
    }
  };

  const fixes = [...new Set(body.warnings.flatMap((w) => w.fixes))];
  return (
    <div>
      <h3>{body.name}</h3>
      <div className={`status-${body.status}`}>Status: {body.status}</div>
      {body.warnings.map((w, i) => (
        <div key={i}>
          <p>{w.message}</p>
          {advice && w.fixes.includes("reexport") && <p><em>Re-export this part from your CAD tool with the problem above fixed, then use Replace part.</em></p>}
        </div>
      ))}
      {fixes.map((f) => f === "replace" ? (
        <label key={f} data-testid="fix-replace">{LABEL[f]}{" "}
          <input data-testid="replace-input" type="file" disabled={busy}
            accept=".step,.stp,.iges,.igs,.brep,.stl,.obj"
            onChange={(e) => { const file = e.target.files?.[0]; if (file) void onReplace(file); }} />
        </label>
      ) : (
        <button key={f} data-testid={`fix-${f}`} onClick={() => {
          if (f === "ignore") dispatch({ type: "ignoreBody", bodyId });
          if (f === "set_mass" && link) {
            setSelection({ kind: "link", id: link.id });
            setTimeout(() => document.querySelector<HTMLInputElement>("[data-testid=mass-override]")?.focus(), 0);
          }
          if (f === "reexport") setAdvice(true);
        }}>{LABEL[f]}</button>
      ))}
      {ignored && <button data-testid="unignore" onClick={() => dispatch({ type: "unignoreBody", bodyId })}>Use this part again</button>}
      {link && !ignored && <p>In link: <a href="#" onClick={(e) => { e.preventDefault(); setSelection({ kind: "link", id: link.id }); }}>{link.name}</a></p>}
    </div>
  );
}
