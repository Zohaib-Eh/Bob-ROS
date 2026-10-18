import type { Joint, Vec3 } from "../api/types";
import type { MobileSel, ModelState } from "./types";

/** A wheeled base, as the Mobile base card sees it (backend export/mobile.py does the exact fit at export):
 *  wheels are turning joints from the base (root) link to a round part with a level axle. Forward is +x,
 *  axles along y (ROS REP-103); left is +y. */

const ROUND = 0.97;          // the rim stays within 3% of its radius (backend export/mobile.ROUND)
const BINS = 12;             // directions around the axle that must reach the rim: a 24-sided CAD wheel fills them, a box (4 corners) can't
const LEVEL = 0.1;           // |axle z| below this: a level axle
const CASTER_BAND = 0.005;   // m: base points this close to the floor carry it (backend export/mobile.CASTER_BAND)

export interface Wheel { jointId: string; radius: number; center: Vec3; axis: Vec3 }

const unit = (a: Vec3): Vec3 => { const n = Math.hypot(...a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
const verts = (m: ModelState, linkId: string): Vec3[] =>
  (m.links.find((l) => l.id === linkId)?.body_ids ?? []).flatMap((id) => m.bodies[id]?.vertices ?? []);

/** The cylinder a part makes about joint j's axis; null when it isn't round (in each of 12 directions around
 *  the axis it should reach within 3% of its radius). */
export function fitWheel(m: ModelState, j: Joint): Wheel | null {
  if (!j.axis) return null;
  const a = unit(j.axis), o = j.origin_xyz;
  const vs = verts(m, j.child_link_id);
  if (!vs.length) return null;
  const ref: Vec3 = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const u = unit([a[1] * ref[2] - a[2] * ref[1], a[2] * ref[0] - a[0] * ref[2], a[0] * ref[1] - a[1] * ref[0]]);
  const w: Vec3 = [a[1] * u[2] - a[2] * u[1], a[2] * u[0] - a[0] * u[2], a[0] * u[1] - a[1] * u[0]];
  const reach = new Array(BINS).fill(0);
  let r = 0, lo = Infinity, hi = -Infinity;
  for (const v of vs) {
    const d: Vec3 = [v[0] - o[0], v[1] - o[1], v[2] - o[2]];
    const along = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
    const rad: Vec3 = [d[0] - along * a[0], d[1] - along * a[1], d[2] - along * a[2]];
    const dist = Math.hypot(...rad);
    const bin = Math.floor(((Math.atan2(rad[0] * w[0] + rad[1] * w[1] + rad[2] * w[2], rad[0] * u[0] + rad[1] * u[1] + rad[2] * u[2]) + Math.PI) / (2 * Math.PI)) * BINS) % BINS;
    reach[bin] = Math.max(reach[bin], dist);
    r = Math.max(r, dist);
    lo = Math.min(lo, along);
    hi = Math.max(hi, along);
  }
  if (Math.min(...reach) < ROUND * r) return null;
  const mid = (lo + hi) / 2;
  return { jointId: j.id, radius: r, axis: a, center: [o[0] + mid * a[0], o[1] + mid * a[1], o[2] + mid * a[2]] };
}

/** Joints that look like wheels: from the base link, a level axle, a round part. */
export function wheelCandidates(m: ModelState): Wheel[] {
  return m.joints
    .filter((j) => j.parent_link_id === m.rootLinkId && (j.type === "continuous" || j.type === "revolute")
      && j.axis && Math.abs(unit(j.axis)[2]) < LEVEL)
    .map((j) => fitWheel(m, j))
    .filter((w): w is Wheel => w !== null);
}

/** Ticked wheels split into sides: left is +y of the wheels' middle. */
export function sides(wheels: Wheel[]): MobileSel {
  const midY = wheels.reduce((s, w) => s + w.center[1], 0) / (wheels.length || 1);
  return { left: wheels.filter((w) => w.center[1] > midY).map((w) => w.jointId),
           right: wheels.filter((w) => w.center[1] <= midY).map((w) => w.jointId) };
}

/** Radians to turn the robot about z so the axles lie along y; 0 when they already do. */
export function turnToDrive(wheels: Wheel[]): number {
  if (!wheels.length) return 0;
  const [ax, ay] = wheels[0].axis;
  if (Math.abs(ay) >= 0.99 * Math.hypot(ax, ay)) return 0;
  let t = Math.PI / 2 - Math.atan2(ay, ax);                  // the axle onto +y...
  t = ((t + Math.PI / 2) % Math.PI + Math.PI) % Math.PI - Math.PI / 2;   // ...or -y, whichever is the smaller turn
  return t;
}

const picked = (m: ModelState, sel: MobileSel) =>
  [...sel.left, ...sel.right].map((id) => m.joints.find((j) => j.id === id)).filter((j): j is Joint => !!j)
    .map((j) => fitWheel(m, j)).filter((w): w is Wheel => w !== null);

/** "Differential drive · 160 mm wheels, 0.50 m apart". */
export function mobileSummary(m: ModelState, sel: MobileSel): string {
  const ws = picked(m, sel);
  if (!ws.length) return "";
  const r = ws.reduce((s, w) => s + w.radius, 0) / ws.length;
  const y = (ids: string[]) => ws.filter((w) => ids.includes(w.jointId)).reduce((s, w, _, a) => s + w.center[1] / a.length, 0);
  const kind = Math.max(sel.left.length, sel.right.length) > 1 ? "Skid steer" : "Differential drive";
  return `${kind} · ${Math.round(2 * r * 1000)} mm wheels, ${(y(sel.left) - y(sel.right)).toFixed(2)} m apart`;
}

/** Plain-words problems with the ticked wheels (export refuses the first two; the rest are warnings). */
export function mobileWarnings(m: ModelState, sel: MobileSel): string[] {
  const out: string[] = [];
  const ws = picked(m, sel);
  if (!sel.left.length || !sel.right.length) out.push("Tick wheels on both sides of the base.");
  if (turnToDrive(ws) !== 0) out.push("The axles must lie along y (forward is +x): turn the robot.");
  if (ws.length < 2) return out;
  const radii = ws.map((w) => w.radius);
  if (Math.max(...radii) > 1.02 * Math.min(...radii)) out.push("The wheels aren't the same size: the base will veer.");
  if (Math.max(sel.left.length, sel.right.length) === 1) {
    const ground = Math.min(...ws.map((w) => w.center[2] - w.radius));
    const wheelLinks = new Set(ws.map((w) => m.joints.find((j) => j.id === w.jointId)!.child_link_id));
    const base = m.links.filter((l) => !wheelLinks.has(l.id) && (l.id === m.rootLinkId
      || m.joints.some((j) => j.type === "fixed" && j.parent_link_id === m.rootLinkId && j.child_link_id === l.id)));
    const low = base.some((l) => verts(m, l.id).some((v) => v[2] <= ground + CASTER_BAND));
    if (!low) out.push("Two wheels and nothing else touching the floor: the base will tip. Add a caster.");
  }
  return out;
}
