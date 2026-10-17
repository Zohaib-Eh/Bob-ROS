import type { Joint, Vec3 } from "../api/types";
import type { GripperSel, ModelState } from "./types";

export const ONE_PART = "The fingers are one part; the gripper needs separate finger parts. Pick a different model or split it in your CAD tool.";
export const NOT_PARALLEL = "The fingers don't slide along the same line.";
export const NO_LIMITS = "Set the fingers' travel (limits) first.";
export const TWO_PARENTS = "Both fingers must be joined to the same gripper base.";
export const DEFAULT_GRIP_FORCE = 40;   // N per finger when the finger joint has no Max torque

const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const unit = (a: Vec3): Vec3 => { const n = Math.hypot(...a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
const mean = (vs: Vec3[]): Vec3 => {
  const s = vs.reduce((acc, v) => [acc[0] + v[0], acc[1] + v[1], acc[2] + v[2]] as Vec3, [0, 0, 0] as Vec3);
  return [s[0] / vs.length, s[1] / vs.length, s[2] / vs.length];
};

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

/** Gap between the fingers' facing sides along the leader's axis, as modelled (m). */
export function jawGap(m: ModelState, g: GripperSel): number {
  const { leader, follower, verts } = parts(m, g);
  if (!leader?.axis || !follower) return 0;
  const a = unit(leader.axis);
  const pl = verts(leader.child_link_id).map((v) => dot(v, a)), pf = verts(follower.child_link_id).map((v) => dot(v, a));
  if (!pl.length || !pf.length) return 0;
  const leaderFirst = dot(mean(verts(leader.child_link_id)), a) < dot(mean(verts(follower.child_link_id)), a);
  return leaderFirst ? Math.min(...pf) - Math.max(...pl) : Math.min(...pl) - Math.max(...pf);
}

/** Tool point: midway between the fingers, pushed out to their far end along the gripper axis
 *  (base centroid → fingers' midpoint), plus tcp_offset. Mirrors backend export/gripper.tcp_point. */
export function tcpPoint(m: ModelState, g: GripperSel): Vec3 {
  const { leader, follower, verts } = parts(m, g);
  if (!leader || !follower) return [0, 0, 0];
  const fl = verts(leader.child_link_id), ff = verts(follower.child_link_id), base = verts(leader.parent_link_id);
  const mid = mean([mean(fl), mean(ff)]);
  const ax = unit(base.length ? [mid[0] - mean(base)[0], mid[1] - mean(base)[1], mid[2] - mean(base)[2]] : [0, 0, -1]);
  const reach = Math.max(...[...fl, ...ff].map((v) => dot([v[0] - mid[0], v[1] - mid[1], v[2] - mid[2]], ax))) + g.tcp_offset;
  return [mid[0] + ax[0] * reach, mid[1] + ax[1] * reach, mid[2] + ax[2] * reach];
}
