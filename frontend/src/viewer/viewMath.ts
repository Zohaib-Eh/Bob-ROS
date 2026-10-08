import type { Vec3 } from "../api/types";

export type ViewName = "front" | "back" | "left" | "right" | "top" | "iso";

/** Unit vector from the target towards the camera (z is up). */
export function viewDirection(v: ViewName): Vec3 {
  switch (v) {
    case "front": return [0, -1, 0];
    case "back": return [0, 1, 0];
    case "left": return [-1, 0, 0];
    case "right": return [1, 0, 0];
    case "top": { const t = 1e-3; const l = Math.hypot(t, 1); return [0, -t / l, 1 / l]; }
    case "iso": { const s = 1 / Math.sqrt(3); return [s, -s, s]; }
  }
}

/** Camera distance that fits a sphere of this radius in the view, with a margin. */
export function fitDistance(radius: number, fovDeg: number): number {
  return ((radius || 0.5) / Math.sin(((fovDeg / 2) * Math.PI) / 180)) * 1.15;
}
