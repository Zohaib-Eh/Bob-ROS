import type { Body, Vec3 } from "../api/types";

/** Axis-aligned box body (12 triangles) from min to max corner. */
export function boxBody(id: string, min: Vec3, max: Vec3, extra: Partial<Body> = {}): Body {
  const [x0, y0, z0] = min, [x1, y1, z1] = max;
  const vertices: Vec3[] = [
    [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
    [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
  ];
  const triangles: [number, number, number][] = [
    [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
    [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
  ];
  return { id, name: id, vertices, triangles, faces: [], status: "ok", warnings: [], ...extra };
}

/** A wheel: a 36-sided cylinder of radius r and width w about the y axis, centred at c. */
export function wheelBody(id: string, c: Vec3, r = 0.08, w = 0.04): Body {
  const vertices: Vec3[] = [];
  for (const y of [-w / 2, w / 2]) for (let k = 0; k < 36; k++) {
    const t = (2 * Math.PI * k) / 36;
    vertices.push([c[0] + r * Math.cos(t), c[1] + y, c[2] + r * Math.sin(t)]);
  }
  const triangles: [number, number, number][] = [];
  for (let k = 0; k < 36; k++) {
    const n = (k + 1) % 36;
    triangles.push([k, n, 36 + n], [k, 36 + n, 36 + k]);
    if (k > 0 && k < 35) triangles.push([0, n, k], [36, 36 + k, 36 + n]);
  }
  return { id, name: id, vertices, triangles, faces: [], status: "ok", warnings: [] };
}
