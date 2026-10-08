import { describe, expect, it } from "vitest";
import { fitDistance, viewDirection, type ViewName } from "./viewMath";

describe("viewMath", () => {
  it("view directions are unit vectors pointing from the target to the camera", () => {
    const views: ViewName[] = ["front", "back", "left", "right", "top", "iso"];
    for (const v of views) expect(Math.hypot(...viewDirection(v))).toBeCloseTo(1);
    expect(viewDirection("front")).toEqual([0, -1, 0]);
    expect(viewDirection("right")).toEqual([1, 0, 0]);
    expect(viewDirection("top")[2]).toBeGreaterThan(0.99);
    expect(viewDirection("top")[1]).toBeLessThan(0);   // never exactly vertical: OrbitControls keeps z up
  });
  it("fit distance grows with size and shrinks with a wider lens", () => {
    expect(fitDistance(2, 45)).toBeGreaterThan(fitDistance(1, 45));
    expect(fitDistance(1, 90)).toBeLessThan(fitDistance(1, 45));
    expect(fitDistance(0, 45)).toBeGreaterThan(0);
  });
});
