import type { Joint, Vec3 } from "../api/types";
import type { GripperSel, ModelState } from "./types";

export const ONE_PART = "The fingers are one part; the gripper needs separate finger parts. Pick a different model or split it in your CAD tool.";
export const NOT_PARALLEL = "The fingers don't slide along the same line.";
export const NO_LIMITS = "Set the fingers' travel (limits) first.";
export const TWO_PARENTS = "Both fingers must be joined to the same gripper base.";
export const DEFAULT_GRIP_FORCE = 40;   // N per finger when the finger joint has no Max torque
const TIP_BAND = 0.015;    // m back from the fingertips: where objects are gripped (backend export/gripper.TIP_BAND)
const FACE_BAND = 0.04;    // m: the gripping faces behind the tips, for spotting hooks

const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const unit = (a: Vec3): Vec3 => { const n = Math.hypot(...a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
const mean = (vs: Vec3[]): Vec3 => {
  const s = vs.reduce((acc, v) => [acc[0] + v[0], acc[1] + v[1], acc[2] + v[2]] as Vec3, [0, 0, 0] as Vec3);
  return [s[0] / vs.length, s[1] / vs.length, s[2] / vs.length];
};

const dist = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/** A link's vertices plus points every 2 mm along its triangle edges, so big flat faces count, not just corners. */
function points(m: ModelState, linkId: string | undefined): Vec3[] {
  const out: Vec3[] = [];
  for (const id of m.links.find((l) => l.id === linkId)?.body_ids ?? []) {
    const b = m.bodies[id];
    if (!b) continue;
    out.push(...b.vertices);
    for (const t of b.triangles) for (const [p, q] of [[t[0], t[1]], [t[1], t[2]], [t[2], t[0]]]) {
      const u = b.vertices[p], w = b.vertices[q], n = Math.ceil(dist(u, w) / 0.002);
      for (let s = 1; s < n; s++) out.push([u[0] + (w[0] - u[0]) * s / n, u[1] + (w[1] - u[1]) * s / n, u[2] + (w[2] - u[2]) * s / n]);
    }
  }
  return out;
}

function parts(m: ModelState, g: GripperSel) {
  const leader = m.joints.find((j) => j.id === g.leader), follower = m.joints.find((j) => j.id === g.follower);
  const verts = (linkId: string | undefined) => (m.links.find((l) => l.id === linkId)?.body_ids ?? [])
    .flatMap((id) => m.bodies[id]?.vertices ?? []);
  return { leader, follower, verts };
}

/** Plain-language reasons this pair of joints can't be the gripper; [] when it can. */
export function gripperIssues(m: ModelState, g: GripperSel): string[] {
  const { leader, follower } = parts(m, g);
  if (!leader || !follower) return [];
  const out: string[] = [];
  const bodiesOf = (j: Joint) => m.links.find((l) => l.id === j.child_link_id)?.body_ids ?? [];
  const shared = bodiesOf(leader).some((id) => bodiesOf(follower).includes(id));
  if (leader.child_link_id === follower.child_link_id || shared) out.push(ONE_PART);
  if (!leader.axis || !follower.axis || Math.abs(dot(unit(leader.axis), unit(follower.axis))) < 0.999) out.push(NOT_PARALLEL);
  if ([leader, follower].some((j) => j.limit_lower == null || j.limit_upper == null)) out.push(NO_LIMITS);
  if (leader.parent_link_id !== follower.parent_link_id) out.push(TWO_PARENTS);
  return out;
}

/** Approach axis (base centroid → fingers' midpoint) and the fingertips' far end along it. */
function approach(m: ModelState, g: GripperSel): { ax: Vec3; tip: Vec3 } | null {
  const { leader, follower, verts } = parts(m, g);
  if (!leader || !follower) return null;
  const fl = verts(leader.child_link_id), ff = verts(follower.child_link_id), base = verts(leader.parent_link_id);
  if (!fl.length || !ff.length) return null;
  const mid = mean([mean(fl), mean(ff)]);
  const ax = unit(base.length ? [mid[0] - mean(base)[0], mid[1] - mean(base)[1], mid[2] - mean(base)[2]] : [0, 0, -1]);
  const reach = Math.max(...[...fl, ...ff].map((v) => dot([v[0] - mid[0], v[1] - mid[1], v[2] - mid[2]], ax)));
  return { ax, tip: [mid[0] + ax[0] * reach, mid[1] + ax[1] * reach, mid[2] + ax[2] * reach] };
}

/** Gap between the jaws along the leader's closing direction, as modelled (m), over the points `from`..`to`
 *  m back from the fingertips (default: the whole fingers). With the travel's closing sense. */
function gapIn(m: ModelState, g: GripperSel, from = 0, to = Infinity): { gap: number; leaderCloses: boolean } | null {
  const { leader, follower, verts } = parts(m, g);
  const ap = approach(m, g);
  if (!leader?.axis || !follower || !ap) return null;
  const a = unit(leader.axis);
  const leaderFirst = dot(mean(verts(leader.child_link_id)), a) < dot(mean(verts(follower.child_link_id)), a);
  const c: Vec3 = leaderFirst ? a : [-a[0], -a[1], -a[2]];   // leader → follower
  const back = (v: Vec3) => -dot([v[0] - ap.tip[0], v[1] - ap.tip[1], v[2] - ap.tip[2]], ap.ax);
  const sel = (linkId: string) => points(m, linkId).filter((v) => back(v) >= from - 1e-9 && back(v) < to).map((v) => dot(v, c));
  const pl = sel(leader.child_link_id), pf = sel(follower.child_link_id);
  if (!pl.length || !pf.length) return null;
  return { gap: Math.min(...pf) - Math.max(...pl), leaderCloses: leaderFirst };
}

/** Gap between the jaw tips (the last 15 mm), as modelled (m). Mirrors backend export/gripper.jaw_gap. */
export function jawGap(m: ModelState, g: GripperSel): number {
  return gapIn(m, g, 0, TIP_BAND)?.gap ?? 0;
}

/** Jaw tips apart fully closed and fully open (m), from the gap as modelled and the leader's travel. */
function gripRange(m: ModelState, g: GripperSel) {
  const leader = m.joints.find((j) => j.id === g.leader);
  const tips = gapIn(m, g, 0, TIP_BAND);
  if (!leader || !tips || leader.limit_lower == null || leader.limit_upper == null) return null;
  const [lo, hi] = [leader.limit_lower, leader.limit_upper];
  const close = tips.leaderCloses ? hi : -lo, open = tips.leaderCloses ? -lo : hi;   // travel each way per finger
  return { closed: tips.gap - 2 * close, open: tips.gap + 2 * open, close };
}

const mm = (v: number) => `${Math.round(v * 1000)} mm`;

/** "Grips objects 30 to 60 mm across the jaws · grip force 40 N per finger". */
export function gripSummary(m: ModelState, g: GripperSel): string {
  const leader = m.joints.find((j) => j.id === g.leader);
  const r = gripRange(m, g);
  const force = `grip force ${leader?.max_effort ?? DEFAULT_GRIP_FORCE} N per finger`;
  if (!r) return force;
  return r.closed < 0.001 ? `Opens to ${mm(r.open)} and closes fully · ${force}`
    : `Grips objects ${Math.round(r.closed * 1000)} to ${mm(r.open)} across the jaws · ${force}`;
}

/** Designs that work but cause trouble in use, each with the reason (shown in the Gripper card). */
export function gripperWarnings(m: ModelState, g: GripperSel): string[] {
  const out: string[] = [];
  const r = gripRange(m, g), all = gapIn(m, g), tips = gapIn(m, g, 0, TIP_BAND), faces = gapIn(m, g, TIP_BAND, FACE_BAND);
  if (r && all && all.gap < 0.0005 && r.close > 0.0005)
    out.push("The jaws already touch as modelled, so they can't close any further, yet their travel closes them. "
      + "Change the limits so the jaws open from here: for jaws modelled closed that is usually minus the stroke to 0 (e.g. -15 to 0 mm).");
  else if (r && r.closed > 0.002)
    out.push(`Fully closed, the jaw tips are still ${mm(r.closed)} apart, so anything narrower than ${mm(r.closed)} can't be gripped: `
      + "the jaws close on nothing. Jaws like these are often made for fingers you bolt on; add them in your CAD to grip smaller parts.");
  if (tips && faces && tips.gap < faces.gap - 0.002)
    out.push(`The fingertips reach in further than the gripping faces (tips ${mm(tips.gap)} apart, faces ${mm(faces.gap)}). `
      + "An object has to pass between the tips, and once it shifts in the grip they catch under it, so lifting off or backing "
      + "away can hit it. Fingers with flat inner faces avoid this.");
  return out;
}

/** Tool point: midway between the fingers, pushed out to their far end along the gripper axis
 *  (base centroid → fingers' midpoint), plus tcp_offset. Mirrors backend export/gripper.tcp_point. */
export function tcpPoint(m: ModelState, g: GripperSel): Vec3 {
  const ap = approach(m, g);
  if (!ap) return [0, 0, 0];
  const { ax, tip } = ap;
  return [tip[0] + ax[0] * g.tcp_offset, tip[1] + ax[1] * g.tcp_offset, tip[2] + ax[2] * g.tcp_offset];
}
