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
