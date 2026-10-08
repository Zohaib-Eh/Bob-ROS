import type { Body, Joint, JointType, Vec3 } from "../api/types";
import { linkBox } from "./geometry";
import type { NextPair } from "./nextJoint";
import type { ModelState, Suggestion } from "./types";

export type Motion = "rigid" | "revolute" | "slider" | "planar";

export interface JointDraft {
  id: string;
  fixed: string | null;
  moving: string | null;
  origin: Vec3 | null;
  axis: Vec3 | null;
  motion: Motion | null;
  noLimits: boolean;
  lower: number | null;   // radians (revolute) or metres (slider)
  upper: number | null;
  suggestionKey: string | null;
  detected: JointType | null;
  ambiguous: boolean;
  source: Joint["source"];
}

export const DEFAULT_LIMITS: { revolute: [number, number]; slider: [number, number] } = {
  revolute: [-Math.PI, Math.PI], slider: [0, 0.1],
};

export function motionOf(t: JointType): Motion | null {
  switch (t) {
    case "fixed": return "rigid";
    case "revolute": case "continuous": case "cylindrical": return "revolute";
    case "prismatic": return "slider";
    case "planar": return "planar";
    default: return null;
  }
}

export function jointType(d: Pick<JointDraft, "motion" | "noLimits">): JointType | null {
  switch (d.motion) {
    case "rigid": return "fixed";
    case "revolute": return d.noLimits ? "continuous" : "revolute";
    case "slider": return "prismatic";
    case "planar": return "planar";
    default: return null;
  }
}

export function emptyDraft(fixed: string | null = null, moving: string | null = null): JointDraft {
  return { id: crypto.randomUUID(), fixed, moving, origin: null, axis: null, motion: null, noLimits: false,
    lower: null, upper: null, suggestionKey: null, detected: null, ambiguous: false, source: "manual" };
}

export function setMotion(d: JointDraft, motion: Motion): JointDraft {
  if (d.motion === motion) return d;
  const lim = motion === "revolute" || motion === "slider" ? DEFAULT_LIMITS[motion] : null;
  return { ...d, motion, lower: lim?.[0] ?? null, upper: lim?.[1] ?? null };
}

export function draftFromJoint(j: Joint, suggestionKey: string | null = null): JointDraft {
  const d: JointDraft = { id: j.id, fixed: j.parent_link_id, moving: j.child_link_id, origin: j.origin_xyz,
    axis: j.axis, motion: motionOf(j.type), noLimits: j.type === "continuous", lower: j.limit_lower,
    upper: j.limit_upper, suggestionKey, detected: j.source === "suggested" ? j.type : null,
    ambiguous: !!j.ambiguous, source: j.source };
  const limited = (d.motion === "revolute" && !d.noLimits) || d.motion === "slider";
  if (limited && (d.lower === null || d.upper === null)) {
    const [lo, hi] = DEFAULT_LIMITS[d.motion as "revolute" | "slider"];
    return { ...d, lower: lo, upper: hi };
  }
  return d;
}

/** The connected link (`fixed`) is the parent whichever way detection ordered the pair. */
export function draftFromSuggestion(g: Suggestion, fixed: string): JointDraft {
  const j = g.result.joint!;
  const moving = j.parent_link_id === fixed ? j.child_link_id : j.parent_link_id;
  return draftFromJoint({ ...j, parent_link_id: fixed, child_link_id: moving }, g.key);
}

export function contactCenter(m: ModelState, a: string, b: string): Vec3 | null {
  const la = m.links.find((l) => l.id === a);
  const lb = m.links.find((l) => l.id === b);
  const A = la ? linkBox(la, m.bodies) : null;
  const B = lb ? linkBox(lb, m.bodies) : null;
  if (!A || !B) return null;
  return [0, 1, 2].map((i) => {
    const lo = Math.max(A.min[i], B.min[i]);
    const hi = Math.min(A.max[i], B.max[i]);
    return lo <= hi ? (lo + hi) / 2 : (A.min[i] + A.max[i] + B.min[i] + B.max[i]) / 4;
  }) as Vec3;
}

export function draftForPair(m: ModelState, n: NextPair): JointDraft {
  if (n.suggestion) return draftFromSuggestion(n.suggestion, n.fixed);
  return { ...emptyDraft(n.fixed, n.moving), origin: contactCenter(m, n.fixed, n.moving) };
}

export function draftProblem(d: JointDraft): string | null {
  if (!d.fixed) return "Pick the fixed part.";
  if (!d.moving) return "Pick the moving part.";
  if (d.fixed === d.moving) return "Pick two different links.";
  if (!d.motion) return "Choose how the joint moves.";
  if (!d.origin) return "Click a feature to place the joint origin.";
  if (d.motion !== "rigid" && !d.axis) return "Click a round or flat face to set the axis.";
  if ((d.motion === "revolute" && !d.noLimits) || d.motion === "slider") {
    if (d.lower === null || d.upper === null) return "Set both limits.";
    if (d.lower > d.upper) return "The lower limit must not be above the upper limit.";
  }
  return null;
}

/** Only call when draftProblem(d) is null. */
export function draftToJoint(d: JointDraft): Joint {
  const type = jointType(d)!;
  const limited = type === "revolute" || type === "prismatic";
  return { id: d.id, parent_link_id: d.fixed!, child_link_id: d.moving!, type, origin_xyz: d.origin!,
    origin_rpy: [0, 0, 0], axis: d.axis, limit_lower: limited ? d.lower : null,
    limit_upper: limited ? d.upper : null, source: d.source === "suggested" ? "confirmed" : d.source };
}

/** A joint to draw while the draft is incomplete; null until both parts and an origin exist. */
export function draftPreviewJoint(d: JointDraft): Joint | null {
  if (!d.fixed || !d.moving || !d.origin) return null;
  return { id: d.id, parent_link_id: d.fixed, child_link_id: d.moving, type: jointType(d) ?? "fixed",
    origin_xyz: d.origin, origin_rpy: [0, 0, 0], axis: d.axis, limit_lower: d.lower, limit_upper: d.upper,
    source: d.source };
}

/** Snap point under a clicked triangle: a round face gives its axis line, a flat face its normal. */
export function snapAt(body: Body, triangle: number): { origin: Vec3; axis: Vec3; kind: "cylinder" | "plane" } | null {
  const f = body.faces.find((x) => x.triangle_indices.includes(triangle));
  if (!f) return null;
  if (f.kind === "cylinder" && f.axis) return { kind: "cylinder", origin: f.center, axis: f.axis };
  if (f.kind === "plane" && f.normal) {
    const t = body.triangles[triangle];
    if (!t) return null;
    const [a, b, c] = t.map((i) => body.vertices[i]);
    // plane centres can be any point on the plane, so use where the user clicked
    return { kind: "plane", origin: [0, 1, 2].map((k) => (a[k] + b[k] + c[k]) / 3) as Vec3, axis: f.normal };
  }
  return null;
}
