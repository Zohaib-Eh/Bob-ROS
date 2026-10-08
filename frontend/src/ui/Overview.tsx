import type { Joint } from "../api/types";
import { linkBox } from "../model/geometry";
import type { DetailsProps } from "./DetailsPanel";

const REASON = { no_overlap: "parts don't overlap", no_contact_faces: "no contact faces found" } as const;

export default function Overview({ model, dispatch, selection, setSelection }: DetailsProps) {
  const sugs = Object.values(model.suggestions);
  const name = (id: string) => model.links.find((l) => l.id === id)?.name ?? id;
  const pairLabel = (a: string, b: string) => `${name(a)} ↔ ${name(b)}`;
  const selectedLinks = selection?.kind === "bodies"
    ? [...new Set(selection.ids.map((b) => model.links.find((l) => l.body_ids.includes(b))?.id).filter(Boolean))] as string[]
    : [];

  const addManual = () => {
    const [a, b] = selectedLinks;
    const A = linkBox(model.links.find((l) => l.id === a)!, model.bodies);
    const B = linkBox(model.links.find((l) => l.id === b)!, model.bodies);
    if (!A || !B) return;
    const lo = [0, 1, 2].map((i) => Math.max(A.min[i], B.min[i]));
    const hi = [0, 1, 2].map((i) => Math.min(A.max[i], B.max[i]));
    const overlaps = [0, 1, 2].every((i) => lo[i] <= hi[i]);
    const origin = [0, 1, 2].map((i) => overlaps ? (lo[i] + hi[i]) / 2
      : ((A.min[i] + A.max[i]) / 2 + (B.min[i] + B.max[i]) / 2) / 2) as [number, number, number];
    const joint: Joint = { id: crypto.randomUUID(), parent_link_id: a, child_link_id: b, type: "fixed",
      origin_xyz: origin, origin_rpy: [0, 0, 0], axis: [0, 0, 1], limit_lower: null, limit_upper: null, source: "manual" };
    dispatch({ type: "addJoint", joint });
    setSelection({ kind: "joint", id: joint.id });
  };

  const section = (title: string, status: "review" | "accepted" | "rejected") => (
    <>
      <h4>{title}</h4>
      {sugs.filter((g) => g.status === status && g.result.joint).map((g) => (
        <div key={g.key} data-testid={`suggestion-${g.key}`} style={{ cursor: "pointer" }}
          onClick={() => setSelection({ kind: "suggestion", key: g.key })}>
          {pairLabel(g.linkA, g.linkB)}: {g.result.joint!.type}{g.result.joint!.ambiguous ? " ⚠" : ""}
        </div>
      ))}
    </>
  );

  return (
    <div>
      {section("Needs review", "review")}
      <h4>Accepted / manual joints</h4>
      {model.joints.map((j) => (
        <div key={j.id} style={{ cursor: "pointer" }} onClick={() => setSelection({ kind: "joint", id: j.id })}>
          {pairLabel(j.parent_link_id, j.child_link_id)}: {j.type} ({j.source})
        </div>
      ))}
      {section("Rejected", "rejected")}
      <h4>No suggestion</h4>
      {sugs.filter((g) => !g.result.joint).map((g) => (
        <div key={g.key}>{pairLabel(g.linkA, g.linkB)}: {REASON[g.result.reason ?? "no_contact_faces"]}</div>
      ))}
      <button data-testid="add-manual-joint" disabled={selectedLinks.length !== 2} onClick={addManual}>
        Add manual joint between selected links
      </button>
    </div>
  );
}
