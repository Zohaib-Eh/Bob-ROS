import { pairKey, touchingPairs } from "./pairs";
import type { ModelState, Suggestion } from "./types";

export interface NextPair { fixed: string; moving: string; suggestion: Suggestion | null }

/** Link id → number of joints between it and the base, following joints in either direction. */
export function chainDepth(m: ModelState): Map<string, number> {
  const depth = new Map<string, number>();
  if (!m.rootLinkId) return depth;
  depth.set(m.rootLinkId, 0);
  const queue = [m.rootLinkId];
  while (queue.length) {
    const cur = queue.shift()!;
    for (const j of m.joints) {
      const other = j.parent_link_id === cur ? j.child_link_id : j.child_link_id === cur ? j.parent_link_id : null;
      if (other !== null && !depth.has(other)) {
        depth.set(other, depth.get(cur)! + 1);
        queue.push(other);
      }
    }
  }
  return depth;
}

/** The next joint to set up, walking out from the base: a touching pair with exactly one side
 *  connected, pairs with a detected joint first, then nearest the base, then detection order. */
export function nextJoint(m: ModelState, onlyLink?: string): NextPair | null {
  const depth = chainDepth(m);
  const options: (NextPair & { d: number; i: number })[] = [];
  touchingPairs(m.links, m.bodies).forEach(([a, b], i) => {
    if (onlyLink && a.id !== onlyLink && b.id !== onlyLink) return;
    const inA = depth.has(a.id);
    if (inA === depth.has(b.id)) return;
    const g = m.suggestions[pairKey(a.id, b.id)];
    if (g?.status === "rejected") return;
    const [fixed, moving] = inA ? [a.id, b.id] : [b.id, a.id];
    options.push({ fixed, moving, suggestion: g?.status === "review" && g.result.joint ? g : null, d: depth.get(fixed)!, i });
  });
  options.sort((x, y) => Number(!x.suggestion) - Number(!y.suggestion) || x.d - y.d || x.i - y.i);
  const best = options[0];
  return best ? { fixed: best.fixed, moving: best.moving, suggestion: best.suggestion } : null;
}
