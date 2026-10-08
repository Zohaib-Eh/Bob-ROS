import type { Body, Face, Status } from "../api/types";

export function bodyArrays(b: Body) {
  const positions = new Float32Array(b.vertices.length * 3);
  b.vertices.forEach((v, i) => positions.set(v, i * 3));
  const indices = new Uint32Array(b.triangles.length * 3);
  b.triangles.forEach((t, i) => indices.set(t, i * 3));
  return { positions, indices };
}

const COLORS: Record<Status, number> = {
  ok: 0x9ea7b3, repaired: 0x8fbf8f, approximate: 0xe0a040, failed: 0xd05050,
};
export const statusColor = (s: Status) => COLORS[s];

export const isClick = (down: { x: number; y: number }, up: { x: number; y: number }, px = 4) =>
  Math.hypot(up.x - down.x, up.y - down.y) < px;

export const faceAtTriangle = (b: Body, tri: number): Face | null =>
  b.faces.find((f) => f.triangle_indices.includes(tri)) ?? null;
