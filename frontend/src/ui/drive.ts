import type { Body, SweepResult, Vec3 } from "../api/types";
import { linkBox } from "../model/geometry";
import { draftPreviewJoint, type JointDraft } from "../model/jointDraft";
import { directJoints } from "../model/tree";
import type { ModelState } from "../model/types";
import { subtreeLinks } from "../viewer/preview";
import { deg } from "./format";

/** Bodies that move with the draft's joint: the child subtree seen from the base, or — when the pair
 *  isn't connected to the base yet — the child link and everything joined to it, never crossing the parent. */
export function movingBodies(m: ModelState, d: JointDraft): string[] {
  const j = draftPreviewJoint(d);
  if (!j) return [];
  const others = m.joints.filter((x) => x.id !== j.id);
  let links = subtreeLinks(directJoints(m.links, [...others, j], m.rootLinkId).joints, j.id);
  if (!links.length) {
    links = [j.child_link_id];
    const seen = new Set([j.parent_link_id, j.child_link_id]);
    for (let i = 0; i < links.length; i++) {
      for (const o of others) {
        const next = o.parent_link_id === links[i] ? o.child_link_id : o.child_link_id === links[i] ? o.parent_link_id : null;
        if (next && !seen.has(next)) { seen.add(next); links.push(next); }
      }
    }
  }
  return m.links.filter((l) => links.includes(l.id)).flatMap((l) => l.body_ids);
}

export interface SweepRequest {
  joint: { type: "revolute" | "prismatic"; origin_xyz: Vec3; axis: Vec3 };
  moving: Body[];
  obstacles: { link_id: string; bodies: Body[] }[];
}

export function sweepRequest(m: ModelState, d: JointDraft): SweepRequest | null {
  const j = draftPreviewJoint(d);
  if (!j || !j.axis || (d.motion !== "revolute" && d.motion !== "slider")) return null;
  const moving = new Set(movingBodies(m, d));
  const movingLinks = new Set(m.links.filter((l) => l.body_ids.some((b) => moving.has(b))).map((l) => l.id));
  // obstacles: the links joined to the fixed side. Links not joined yet have no settled position
  // (they usually ride along with the moving part once connected), so they are not walls.
  const fixedSide = new Set([j.parent_link_id]);
  const queue = [j.parent_link_id];
  while (queue.length) {
    const cur = queue.shift()!;
    for (const x of m.joints) {
      if (x.id === j.id) continue;
      const next = x.parent_link_id === cur ? x.child_link_id : x.child_link_id === cur ? x.parent_link_id : null;
      if (next && !fixedSide.has(next) && !movingLinks.has(next)) { fixedSide.add(next); queue.push(next); }
    }
  }
  return {
    joint: { type: d.motion === "slider" ? "prismatic" : "revolute", origin_xyz: j.origin_xyz, axis: j.axis },
    moving: [...moving].map((id) => m.bodies[id]),
    obstacles: m.links.filter((l) => fixedSide.has(l.id) && !movingLinks.has(l.id))
      .map((l) => ({ link_id: l.id, bodies: l.body_ids.map((id) => m.bodies[id]) })),
  };
}

/** Slider/number range for driving: a full turn, or ± the moving link's size, widened to include the limits. */
export function driveRange(m: ModelState, d: JointDraft): [number, number] {
  const lim = d.noLimits ? [0, 0] : [d.lower ?? 0, d.upper ?? 0];
  if (d.motion === "slider") {
    const l = m.links.find((x) => x.id === d.moving);
    const b = l ? linkBox(l, m.bodies) : null;
    const size = b ? Math.hypot(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]) : 0.1;
    return [Math.min(-size, lim[0]), Math.max(size, lim[1])];
  }
  return [Math.min(-Math.PI, lim[0]), Math.max(Math.PI, lim[1])];
}

export function beyondSide(d: JointDraft, value: number): "lower" | "upper" | null {
  if (d.noLimits || (d.motion !== "revolute" && d.motion !== "slider")) return null;
  if (d.lower !== null && value < d.lower - 1e-9) return "lower";
  if (d.upper !== null && value > d.upper + 1e-9) return "upper";
  return null;
}

export const fmtJointValue = (v: number, slider: boolean) =>
  `${v > 0 ? "+" : ""}${slider ? `${(v * 1000).toFixed(0)} mm` : `${deg(v).toFixed(0)}°`}`;

export function sweepSummary(r: SweepResult, slider: boolean, name: (id: string) => string): string {
  if (r.at_rest_overlap.length) {
    return `These parts overlap where they sit, so limits can't be found: ${r.at_rest_overlap.map(name).join(", ")}.`;
  }
  if (r.free) return "Spins freely: no collision in a full turn.";
  const side = (v: number | null, by: string | null) =>
    `${v === null ? "?" : fmtJointValue(v, slider)} (${by ? `hits ${name(by)}` : "no contact"})`;
  return `Stops at ${side(r.lower, r.blocked_by.lower)} and ${side(r.upper, r.blocked_by.upper)}.`;
}
