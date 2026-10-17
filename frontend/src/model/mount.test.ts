import { describe, expect, it } from "vitest";
import type { Face, Vec3 } from "../api/types";
import { invert, mountMatrix, transformBody, type Mat4 } from "./mount";
import { boxBody } from "./testBodies";

const plane = (center: Vec3, normal: Vec3): Face => ({
  kind: "plane", origin: "brep", normal, axis: null, center, radius: null, residual: 0, triangle_indices: [],
});
const apply = (M: Mat4, p: Vec3): Vec3 =>
  [0, 1, 2].map((r) => M[4 * r] * p[0] + M[4 * r + 1] * p[1] + M[4 * r + 2] * p[2] + M[4 * r + 3]) as Vec3;
const close = (a: number[], b: number[]) => a.forEach((v, i) => expect(v).toBeCloseTo(b[i], 9));

// cube A: [0,1]^3, its +z face; cube B: [5,6]x[0,1]x[0,1], its +x face
const aTop = plane([0.5, 0.5, 1], [0, 0, 1]);
const bSide = plane([6, 0.5, 0.5], [1, 0, 0]);

describe("mountMatrix", () => {
  it("face_meets_face", () => {
    const M = mountMatrix(aTop, bSide, 0);
    close(apply(M, aTop.center), bSide.center);
    const moved = transformBody({ ...boxBody("a", [0, 0, 0], [1, 1, 1]), faces: [aTop] }, M);
    close(moved.faces[0].normal!, [-1, 0, 0]);   // points back at B
    close(moved.faces[0].center, bSide.center);
    moved.vertices.forEach((v) => expect(v[0]).toBeGreaterThanOrEqual(6 - 1e-9));  // A sits outside B, flush
  });

  it("turn_rotates_about_the_target_normal", () => {
    const p0 = apply(mountMatrix(aTop, bSide, 0), [1, 0.5, 1]);   // a point 0.5 off the face centre
    const p90 = apply(mountMatrix(aTop, bSide, 90), [1, 0.5, 1]);
    const c = bSide.center;
    const v0 = [p0[1] - c[1], p0[2] - c[2]], v90 = [p90[1] - c[1], p90[2] - c[2]];
    expect(p90[0]).toBeCloseTo(p0[0], 9);                         // stays in the face plane
    expect(v0[0] * v90[0] + v0[1] * v90[1]).toBeCloseTo(0, 9);   // turned 90°
    expect(Math.hypot(v90[0], v90[1])).toBeCloseTo(Math.hypot(v0[0], v0[1]), 9);
  });

  it("non_plane_face_is_refused", () => {
    const cyl: Face = { ...aTop, kind: "cylinder", normal: null, axis: [0, 0, 1], radius: 0.1 };
    const msg = "Pick a flat face (the gripper's mounting plate and the flange).";
    expect(() => mountMatrix(cyl, bSide, 0)).toThrow(msg);
    expect(() => mountMatrix(aTop, cyl, 0)).toThrow(msg);
  });

  it("antiparallel_normals_still_mount", () => {   // already facing each other: no 0/0 in the rotation
    const M = mountMatrix(plane([0, 0, 0], [1, 0, 0]), bSide, 0);
    close(apply(M, [0, 0, 0]), bSide.center);
  });

  it("invert_round_trips_within_1e-9", () => {
    const M = mountMatrix(aTop, bSide, 37);
    const b = boxBody("a", [0, 0, 0], [1, 1, 1]);
    const back = transformBody(transformBody(b, M), invert(M));
    back.vertices.forEach((v, i) => close(v, b.vertices[i]));
  });
});
