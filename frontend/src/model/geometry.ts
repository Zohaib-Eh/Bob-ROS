import type { Body, Link, Vec3 } from "../api/types";

export type Box = { min: Vec3; max: Vec3 };

const cache = new WeakMap<Body, Box | null>();

export function bodyBox(b: Body): Box | null {
  if (cache.has(b)) return cache.get(b)!;
  let box: Box | null = null;
  if (b.vertices.length) {
    const min: Vec3 = [Infinity, Infinity, Infinity];
    const max: Vec3 = [-Infinity, -Infinity, -Infinity];
    for (const v of b.vertices) {
      for (let i = 0; i < 3; i++) {
        if (v[i] < min[i]) min[i] = v[i];
        if (v[i] > max[i]) max[i] = v[i];
      }
    }
    box = { min, max };
  }
  cache.set(b, box);
  return box;
}

export function unionBox(boxes: (Box | null)[]): Box | null {
  const real = boxes.filter((b): b is Box => b !== null);
  if (!real.length) return null;
  return {
    min: [0, 1, 2].map((i) => Math.min(...real.map((b) => b.min[i]))) as Vec3,
    max: [0, 1, 2].map((i) => Math.max(...real.map((b) => b.max[i]))) as Vec3,
  };
}

export const boxesOverlap = (a: Box, b: Box, tol: number) =>
  [0, 1, 2].every((i) => a.min[i] <= b.max[i] + tol && b.min[i] <= a.max[i] + tol);

export const boxVolume = (b: Box) =>
  (b.max[0] - b.min[0]) * (b.max[1] - b.min[1]) * (b.max[2] - b.min[2]);

export const linkBox = (link: Link, bodies: Record<string, Body>) =>
  unionBox(link.body_ids.map((id) => (bodies[id] ? bodyBox(bodies[id]) : null)));

/** The link that most likely sits on the ground: lowest box bottom, ties broken by larger volume. */
export function baseGuess(links: Link[], bodies: Record<string, Body>): string | null {
  let best: { id: string; z: number; v: number } | null = null;
  for (const l of links) {
    const b = linkBox(l, bodies);
    if (!b) continue;
    const z = b.min[2];
    const v = boxVolume(b);
    if (!best || z < best.z - 1e-9 || (Math.abs(z - best.z) <= 1e-9 && v > best.v)) best = { id: l.id, z, v };
  }
  return best?.id ?? links[0]?.id ?? null;
}
