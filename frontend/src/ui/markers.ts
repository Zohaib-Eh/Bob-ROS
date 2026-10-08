import type { Vec3 } from "../api/types";
import type { StepId } from "../model/checklist";
import { bodyBox, linkBox, type Box } from "../model/geometry";
import { chainDepth } from "../model/nextJoint";
import { touchingPairs } from "../model/pairs";
import { partProblems } from "../model/problems";
import type { ModelState } from "../model/types";
import type { Selection } from "./selection";

export interface Marker { id: string; at: Vec3; kind: "problem" | "joint" | "loose"; label: string; target: Selection }

const top = (b: Box): Vec3 => [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, b.max[2]];
const mid = (b: Box): Vec3 => [0, 1, 2].map((i) => (b.min[i] + b.max[i]) / 2) as Vec3;

export function markersFor(step: StepId, m: ModelState): Marker[] {
  const out: Marker[] = [];
  const name = (id: string) => m.links.find((l) => l.id === id)?.name ?? id;
  const pinned = new Set<string>();
  // problems only where the user deals with them; elsewhere the stepper badge is enough
  for (const p of step === "parts" || step === "export" ? partProblems(m) : []) {
    if (p.kind === "failed" || pinned.has(p.bodyId)) continue;   // failed parts have no geometry to pin to
    const b = bodyBox(m.bodies[p.bodyId]);
    if (!b) continue;
    pinned.add(p.bodyId);
    out.push({ id: `problem-${p.bodyId}`, at: top(b), kind: "problem",
      label: p.kind === "approximate" ? "Approximate shape" : p.warning.message, target: { kind: "bodies", ids: [p.bodyId] } });
  }
  if (step === "connect" || step === "export") {
    for (const j of m.joints) {
      out.push({ id: `joint-${j.id}`, at: j.origin_xyz, kind: "joint",
        label: `${name(j.parent_link_id)} → ${name(j.child_link_id)}`, target: { kind: "joint", id: j.id } });
    }
  }
  if (step === "connect" && m.rootLinkId) {
    // only the frontier: loose links that touch the connected chain (the next ones to connect)
    const depth = chainDepth(m);
    const frontier = new Set(touchingPairs(m.links, m.bodies)
      .filter(([a, b]) => depth.has(a.id) !== depth.has(b.id))
      .map(([a, b]) => (depth.has(a.id) ? b.id : a.id)));
    for (const l of m.links) {
      if (!frontier.has(l.id)) continue;
      const b = linkBox(l, m.bodies);
      if (b) out.push({ id: `loose-${l.id}`, at: top(b), kind: "loose", label: "Not connected", target: { kind: "link", id: l.id } });
    }
  }
  return out;
}

/** One dashed line per loose link, to a touching link that is already connected. */
export function looseLines(m: ModelState): [Vec3, Vec3][] {
  if (!m.rootLinkId) return [];
  const depth = chainDepth(m);
  const done = new Set<string>();
  const out: [Vec3, Vec3][] = [];
  for (const [a, b] of touchingPairs(m.links, m.bodies)) {
    const inA = depth.has(a.id), inB = depth.has(b.id);
    if (inA === inB) continue;
    const [inside, loose] = inA ? [a, b] : [b, a];
    if (done.has(loose.id)) continue;
    const A = linkBox(inside, m.bodies), B = linkBox(loose, m.bodies);
    if (A && B) { out.push([mid(A), mid(B)]); done.add(loose.id); }
  }
  return out;
}
