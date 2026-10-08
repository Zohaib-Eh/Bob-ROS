import type { Body, Link } from "../api/types";
import { bodyBox, boxesOverlap, type Box } from "./geometry";

export const CONTACT_TOL = 1e-4;

export const pairKey = (a: string, b: string) => [a, b].sort().join("|");

export function membership(a: Link, b: Link): string {
  const [x, y] = a.id < b.id ? [a, b] : [b, a];
  return `${[...x.body_ids].sort().join(",")}|${[...y.body_ids].sort().join(",")}`;
}

const boxes = (l: Link, bodies: Record<string, Body>): Box[] =>
  l.body_ids.map((id) => (bodies[id] ? bodyBox(bodies[id]) : null)).filter((b): b is Box => b !== null);

export function touchingPairs(links: Link[], bodies: Record<string, Body>, tol = CONTACT_TOL): [Link, Link][] {
  const out: [Link, Link][] = [];
  const bx = links.map((l) => boxes(l, bodies));
  for (let i = 0; i < links.length; i++) {
    for (let j = i + 1; j < links.length; j++) {
      if (bx[i].some((a) => bx[j].some((b) => boxesOverlap(a, b, tol)))) out.push([links[i], links[j]]);
    }
  }
  return out;
}
