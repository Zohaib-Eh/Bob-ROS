import { contactCenter, snapAt, type JointDraft } from "../model/jointDraft";
import type { Action } from "../model/reducer";
import type { ModelState } from "../model/types";
import type { Hit } from "../viewer/Viewer";
import type { PickedFace, PickMode } from "./ctx";
import type { Selection } from "./selection";

export interface PickResult { selection?: Selection; draft?: JointDraft; action?: Action; mode?: PickMode; face?: PickedFace }

/** What a click in the 3D view does, by pick mode. Empty space never leaves a pick mode. */
export function routePick(mode: PickMode, hit: Hit, additive: boolean, m: ModelState, selection: Selection,
  draft: JointDraft | null): PickResult {
  if (mode === "select") {
    if (!hit) return { selection: null };
    const prev = additive && selection?.kind === "bodies" ? selection.ids : [];
    return { selection: { kind: "bodies", ids: [...new Set([...prev, hit.bodyId])] } };
  }
  if (mode === "mount-from" || mode === "mount-to") {
    const faceIndex = hit ? (m.bodies[hit.bodyId]?.faces ?? []).findIndex((f) => f.triangle_indices.includes(hit.triangle)) : -1;
    return hit && faceIndex >= 0 ? { face: { mode, bodyId: hit.bodyId, faceIndex }, mode: "select" } : {};
  }
  const link = hit ? m.links.find((l) => l.body_ids.includes(hit.bodyId)) : undefined;
  if (!hit || !link) return {};
  if (mode === "base") return { action: { type: "setRoot", linkId: link.id }, mode: "select" };
  if (!draft) return { mode: "select" };
  if (mode === "fixed" || mode === "moving") {
    const next: JointDraft = { ...draft, [mode]: link.id };
    if (next.fixed && next.moving && next.fixed !== next.moving && !next.origin) {
      next.origin = contactCenter(m, next.fixed, next.moving);
    }
    return { draft: next, mode: mode === "fixed" && !next.moving ? "moving" : "select" };
  }
  const snap = snapAt(m.bodies[hit.bodyId], hit.triangle);
  return snap ? { draft: { ...draft, origin: snap.origin, axis: snap.axis }, mode: "select" } : {};
}
