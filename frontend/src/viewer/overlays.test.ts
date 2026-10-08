import type { Mesh, Object3D } from "three";
import { describe, expect, it } from "vitest";
import type { Joint, Vec3 } from "../api/types";
import {
  angleAroundAxis, arcPoints, buildGlyph, closestParamOnAxis, perpendicular, rayPlaneHit,
} from "./overlays";

const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const J = (extra: Partial<Joint>): Joint => ({ id: "j", parent_link_id: "a", child_link_id: "b", type: "revolute",
  origin_xyz: [0, 0, 0], origin_rpy: [0, 0, 0], axis: [0, 0, 1], limit_lower: -1, limit_upper: 1, source: "manual", ...extra });
const handles = (o: Object3D) => { const out: string[] = []; o.traverse((c) => { if (c.userData.handle) out.push(c.userData.handle); }); return out.sort(); };

describe("vector helpers", () => {
  it("perpendicular is a unit vector at right angles to the axis", () => {
    for (const a of [[0, 0, 1], [1, 0, 0], [0.3, -0.5, 0.8]] as Vec3[]) {
      const p = perpendicular(a);
      expect(Math.hypot(...p)).toBeCloseTo(1);
      expect(dot(p, a)).toBeCloseTo(0);
    }
  });
  it("arc points sit on the circle and angleAroundAxis reads their angle back", () => {
    const o: Vec3 = [1, 2, 3], a: Vec3 = [0, 0, 1];
    const pts = arcPoints(o, a, 0.5, -1, 1, 8);
    expect(pts).toHaveLength(9);
    for (const p of pts) expect(Math.hypot(p[0] - 1, p[1] - 2, p[2] - 3)).toBeCloseTo(0.5);
    expect(angleAroundAxis(pts[0], o, a)).toBeCloseTo(-1);
    expect(angleAroundAxis(pts[8], o, a)).toBeCloseTo(1);
  });
  it("ray–plane hit and closest point on an axis", () => {
    expect(rayPlaneHit([0, 0, 5], [0, 0, -1], [0, 0, 1], [0, 0, 1])).toEqual([0, 0, 1]);
    expect(rayPlaneHit([0, 0, 5], [1, 0, 0], [0, 0, 1], [0, 0, 1])).toBeNull();
    // a ray along y through x = 0.3 passes closest to the x axis at t = 0.3
    expect(closestParamOnAxis([0.3, -5, 0], [0, 1, 0], [0, 0, 0], [1, 0, 0])).toBeCloseTo(0.3);
  });
});

describe("buildGlyph", () => {
  it("an active revolute joint has lower and upper handles", () => {
    expect(handles(buildGlyph({ joint: J({}), state: "active" }, 1))).toEqual(["lower", "upper"]);
  });
  it("done and suggested joints have no handles; suggested ones are faded", () => {
    expect(handles(buildGlyph({ joint: J({}), state: "done" }, 1))).toEqual([]);
    const g = buildGlyph({ joint: J({}), state: "suggested" }, 1);
    const faded: number[] = [];
    g.traverse((c) => { const m = (c as Mesh).material as { opacity?: number } | undefined; if (m?.opacity !== undefined) faded.push(m.opacity); });
    expect(faded.every((o) => o <= 0.6)).toBe(true);
  });
  it("an active prismatic joint has handles; a fixed joint is a single marker", () => {
    expect(handles(buildGlyph({ joint: J({ type: "prismatic", limit_lower: 0, limit_upper: 0.1 }), state: "active" }, 1)))
      .toEqual(["lower", "upper"]);
    expect(buildGlyph({ joint: J({ type: "fixed", axis: null }), state: "done" }, 1).children).toHaveLength(2);
  });
});
