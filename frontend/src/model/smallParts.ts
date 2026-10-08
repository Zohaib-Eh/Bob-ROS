import type { Body, Link } from "../api/types";
import { boxVolume, linkBox } from "./geometry";
import { touchingPairs } from "./pairs";

const SMALL_FRACTION = 0.01;

export function smallLinks(links: Link[], bodies: Record<string, Body>): { linkId: string; neighbourId: string }[] {
  const vol = new Map(links.map((l) => { const b = linkBox(l, bodies); return [l.id, b ? boxVolume(b) : 0]; }));
  const largest = Math.max(0, ...vol.values());
  if (largest <= 0) return [];
  const neighbours = new Map<string, string[]>(links.map((l) => [l.id, []]));
  for (const [a, b] of touchingPairs(links, bodies)) {
    neighbours.get(a.id)!.push(b.id);
    neighbours.get(b.id)!.push(a.id);
  }
  return links
    .filter((l) => vol.get(l.id)! < SMALL_FRACTION * largest && neighbours.get(l.id)!.length === 1)
    .map((l) => ({ linkId: l.id, neighbourId: neighbours.get(l.id)![0] }));
}
