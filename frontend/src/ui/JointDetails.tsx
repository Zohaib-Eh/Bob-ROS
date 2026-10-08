import { useEffect, useState } from "react";
import type { Joint, JointType, Vec3 } from "../api/types";
import { directJoints } from "../model/tree";
import { faceAtTriangle } from "../viewer/meshData";
import { jointMotion, previewRange, subtreeLinks } from "../viewer/preview";
import type { DetailsProps } from "./DetailsPanel";

const URDF_TYPES: JointType[] = ["revolute", "continuous", "prismatic", "fixed", "planar"];
const NEEDS_LIMITS = new Set<JointType>(["revolute", "prismatic"]);

export default function JointDetails(p: DetailsProps) {
  const { model, dispatch, selection, viewer, setPreview, lastPick, setSelection } = p;
  const suggestion = selection?.kind === "suggestion" ? model.suggestions[selection.key] : null;
  const stored = selection?.kind === "joint" ? model.joints.find((j) => j.id === selection.id) ?? null : null;
  const source = stored ?? suggestion?.result.joint ?? null;
  const [draft, setDraft] = useState<Joint | null>(source);
  const [editing, setEditing] = useState(false);
  const [preview, setPreviewValue] = useState(0);
  useEffect(() => { setDraft(source); setEditing(false); }, [source?.id]);   // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => {
    if (!editing || !viewer || !draft?.axis) { viewer?.detachGizmo(); return; }
    viewer.attachGizmo(draft.origin_xyz, draft.axis, (origin, axis) =>
      setDraft((d) => (d ? { ...d, origin_xyz: origin, axis } : d)));
    return () => viewer.detachGizmo();
  }, [editing, viewer, draft?.id]);   // eslint-disable-line react-hooks/exhaustive-deps
  if (!draft) return <div>Nothing to show.</div>;

  const typeOk = URDF_TYPES.includes(draft.type);
  const limitsOk = !NEEDS_LIMITS.has(draft.type) || (draft.limit_lower !== null && draft.limit_upper !== null
    && draft.limit_lower <= draft.limit_upper);
  const canSave = typeOk && limitsOk;
  const num = (v: string) => (v === "" ? null : Number(v));

  const snap = (mode: "cylinder" | "normal") => {
    if (!lastPick) return;
    const face = faceAtTriangle(model.bodies[lastPick.bodyId], lastPick.triangle);
    const axis: Vec3 | null = mode === "cylinder" ? face?.axis ?? null : face?.normal ?? null;
    if (axis) setDraft({ ...draft, axis, origin_xyz: mode === "cylinder" && face ? face.center : draft.origin_xyz });
  };

  const onPreview = (value: number) => {
    setPreviewValue(value);
    const { joints } = directJoints(model.links, [...model.joints.filter((j) => j.id !== draft.id), draft], model.rootLinkId);
    const links = subtreeLinks(joints, draft.id);
    const bodyIds = model.links.filter((l) => links.includes(l.id)).flatMap((l) => l.body_ids);
    setPreview({ bodyIds, matrix: jointMotion(draft, value) });
  };
  const [lo, hi] = previewRange(draft);

  return (
    <div>
      <h3>{suggestion ? "Suggested joint" : `Joint (${draft.source})`}</h3>
      <label>Type{" "}
        <select data-testid="joint-type" value={typeOk ? draft.type : ""}
          onChange={(e) => setDraft({ ...draft, type: e.target.value as JointType })}>
          {!typeOk && <option value="">{draft.type} — choose a URDF type</option>}
          {URDF_TYPES.map((t) => <option key={t} value={t}>{t}</option>)}
        </select>
      </label>
      {NEEDS_LIMITS.has(draft.type) && (
        <div>
          <label>Lower <input data-testid="limit-lower" type="number" step="any" value={draft.limit_lower ?? ""}
            onChange={(e) => setDraft({ ...draft, limit_lower: num(e.target.value) })} /></label>
          <label>Upper <input data-testid="limit-upper" type="number" step="any" value={draft.limit_upper ?? ""}
            onChange={(e) => setDraft({ ...draft, limit_upper: num(e.target.value) })} /></label>
        </div>
      )}
      <div>Origin: {draft.origin_xyz.map((x) => x.toFixed(4)).join(", ")}</div>
      <div>Axis: {draft.axis ? draft.axis.map((x) => x.toFixed(3)).join(", ") : "none"}</div>
      {suggestion?.result.evidence && (
        <ul data-testid="evidence">
          {suggestion.result.evidence.pairs.map((e, i) => (
            <li key={i}>{e.kind}{e.radius !== null ? ` r=${(e.radius * 1000).toFixed(1)} mm` : ""}</li>
          ))}
          <li>{suggestion.result.evidence.candidate_count} candidate pair(s), tolerance {suggestion.result.evidence.tolerance} m</li>
        </ul>
      )}
      {draft.ambiguous && <div className="status-approximate">⚠ ambiguous: several matching features</div>}

      <button data-testid="edit" onClick={() => setEditing((x) => !x)}>{editing ? "Done editing" : "Edit"}</button>
      {editing && (
        <>
          <button onClick={() => viewer?.setGizmoMode("translate")}>Move</button>
          <button onClick={() => viewer?.setGizmoMode("rotate")}>Rotate axis</button>
          <button data-testid="snap-cylinder" disabled={!lastPick} onClick={() => snap("cylinder")}>Axis along picked cylinder</button>
          <button data-testid="snap-normal" disabled={!lastPick} onClick={() => snap("normal")}>Axis along picked face normal</button>
        </>
      )}

      {lo !== hi && (
        <label>Preview{" "}
          <input data-testid="preview-slider" type="range" min={lo} max={hi} step={(hi - lo) / 100}
            value={preview} data-preview-value={preview}
            onChange={(e) => onPreview(Number(e.target.value))}
            onMouseUp={() => { setPreviewValue(0); setPreview(null); }} />
        </label>
      )}

      <div style={{ marginTop: 8, display: "flex", gap: 4 }}>
        {suggestion && suggestion.status !== "accepted" && (
          <button data-testid="accept" disabled={!canSave}
            onClick={() => dispatch({ type: "acceptSuggestion", key: suggestion.key, joint: draft })}>Accept</button>
        )}
        {suggestion && suggestion.status === "review" && (
          <button data-testid="reject" onClick={() => dispatch({ type: "rejectSuggestion", key: suggestion.key })}>Reject</button>
        )}
        {stored && (
          <>
            <button data-testid="save-joint" disabled={!canSave}
              onClick={() => dispatch({ type: "updateJoint", joint: draft })}>Save</button>
            <button data-testid="remove-joint" onClick={() => { dispatch({ type: "removeJoint", jointId: draft.id }); setSelection(null); }}>
              Remove
            </button>
          </>
        )}
      </div>
    </div>
  );
}
