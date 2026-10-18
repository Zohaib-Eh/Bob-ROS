import type { WorkcellObject } from "../api/program";

export type NewKind = "box" | "cylinder" | "table";
export type Reach = [number, number][] | null;   // see api Workcell.reach

const height = (o: WorkcellObject) => (o.shape === "box" ? o.size[2] : o.size[1]);
/** Footprint half-widths on the floor (ignoring turns: a box turned on the floor may overhang a little). */
const half = (o: WorkcellObject): [number, number] => (o.shape === "box" ? [o.size[0] / 2, o.size[1] / 2] : [o.size[0], o.size[0]]);

/** How far out (from the base's axis) the arm gets at height z; 0 above or below anything it reaches; null if unknown. */
export function reachAt(reach: Reach, z: number): number | null {
  if (!reach) return null;
  return reach.find(([zc]) => Math.abs(zc - z) <= 0.05 + 1e-9)?.[1] ?? 0;
}

export const top = (o: WorkcellObject) => o.xyz[2] + height(o) / 2;

/** A new object to place: a unique name, well inside the reach at its top, standing on the floor. */
export function newObject(kind: NewKind, existing: WorkcellObject[], reach: Reach): WorkcellObject {
  const taken = new Set(existing.map((o) => o.name));
  let n = 1;
  while (taken.has(`${kind}_${n}`)) n++;
  const x = Math.max(0.2, 0.6 * (reachAt(reach, kind === "table" ? 0.3 : 0.05) || 1));
  const base = { name: `${kind}_${n}`, rpy: [0, 0, 0] as [number, number, number], friction: null };
  if (kind === "table") return { ...base, shape: "box", size: [0.6, 0.4, 0.3], xyz: [x, 0, 0.15], fixed: true, mass: null };
  if (kind === "cylinder") return { ...base, shape: "cylinder", size: [0.02, 0.06], xyz: [x, 0.1 * n, 0.03], fixed: false, mass: 0.1 };
  return { ...base, shape: "box", size: [0.04, 0.04, 0.04], xyz: [x, 0.1 * n, 0.02], fixed: false, mass: 0.2 };
}

/** The object moved to rest on whatever is under its middle: the floor (z = 0) or the highest top below it. */
export function settle(o: WorkcellObject, all: WorkcellObject[]): WorkcellObject {
  const [x, y] = o.xyz;
  const [hx, hy] = half(o);
  let floor = 0;
  for (const p of all) {
    if (p.name === o.name) continue;
    const [px, py] = half(p);
    const overlaps = Math.abs(p.xyz[0] - x) < px + hx && Math.abs(p.xyz[1] - y) < py + hy;
    if (overlaps) floor = Math.max(floor, p.xyz[2] + height(p) / 2);
  }
  return { ...o, xyz: [x, y, floor + height(o) / 2] };
}

/** True when the arm can't get out to the object at its top (where a gripper comes in from above). */
export function outOfReach(o: WorkcellObject, reach: Reach): boolean {
  const r = reachAt(reach, top(o));
  return r !== null && Math.hypot(o.xyz[0], o.xyz[1]) > r;
}
