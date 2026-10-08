import type { Action } from "../model/reducer";
import type { ModelState } from "../model/types";

export type Selection =
  | { kind: "bodies"; ids: string[] }
  | { kind: "link"; id: string }
  | { kind: "joint"; id: string }
  | null;

export function selectedBodies(m: ModelState, s: Selection): string[] {
  if (s?.kind === "bodies") return s.ids;
  if (s?.kind === "link") return m.links.find((l) => l.id === s.id)?.body_ids ?? [];
  if (s?.kind === "joint") {
    const j = m.joints.find((x) => x.id === s.id);
    return j ? m.links.find((l) => l.id === j.child_link_id)?.body_ids ?? [] : [];
  }
  return [];
}

export function linksOfSelection(m: ModelState, s: Selection): string[] {
  if (s?.kind !== "bodies") return [];
  return [...new Set(s.ids.map((b) => m.links.find((l) => l.body_ids.includes(b))?.id)
    .filter((x): x is string => x !== undefined))];
}

/** What the Delete key does: parts and links leave the robot (recoverable), joints are removed. */
export function deleteAction(m: ModelState, s: Selection): Action | null {
  if (s?.kind === "joint") return { type: "removeJoint", jointId: s.id };
  const bodyIds = selectedBodies(m, s);
  return bodyIds.length ? { type: "ignoreBodies", bodyIds } : null;
}
