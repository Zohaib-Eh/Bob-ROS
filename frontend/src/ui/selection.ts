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
