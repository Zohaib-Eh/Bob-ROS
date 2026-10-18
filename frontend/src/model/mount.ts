import type { Body, Face, Vec3 } from "../api/types";
import type { ModelState } from "./types";

/** 4x4 rigid transform, 16 numbers, row-major. */
export type Mat4 = number[];

const FLAT_ONLY = "Pick a flat face (the gripper's mounting plate and the flange).";

const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = (a: Vec3): Vec3 => { const n = Math.hypot(...a); return [a[0] / n, a[1] / n, a[2] / n]; };

/** Rotation (3x3, row-major) by `angle` about unit `axis` (Rodrigues). */
function rotation(axis: Vec3, angle: number): number[] {
  const [x, y, z] = axis, c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [t * x * x + c, t * x * y - s * z, t * x * z + s * y,
          t * x * y + s * z, t * y * y + c, t * y * z - s * x,
          t * x * z - s * y, t * y * z + s * x, t * z * z + c];
}

const mul3 = (A: number[], B: number[]) =>
  [0, 1, 2].flatMap((r) => [0, 1, 2].map((c) => A[3 * r] * B[c] + A[3 * r + 1] * B[3 + c] + A[3 * r + 2] * B[6 + c]));
const rot3 = (R: number[], v: Vec3): Vec3 =>
  [0, 1, 2].map((r) => R[3 * r] * v[0] + R[3 * r + 1] * v[1] + R[3 * r + 2] * v[2]) as Vec3;

/** Rotation taking unit vector a onto unit vector b. */
function align(a: Vec3, b: Vec3): number[] {
  const c = dot(a, b), axis = cross(a, b);
  if (Math.hypot(...axis) < 1e-12) {
    if (c > 0) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
    const helper: Vec3 = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];   // 180°: any perpendicular axis
    return rotation(unit(cross(a, helper)), Math.PI);
  }
  return rotation(unit(axis), Math.acos(Math.max(-1, Math.min(1, c))));
}

/** Rigid transform putting flat face `moving` flush on flat face `target`: centres coincide, normals
 *  opposite, then turned `turnDeg` about the target normal. */
export function mountMatrix(moving: Face, target: Face, turnDeg: number): Mat4 {
  if (moving.kind !== "plane" || target.kind !== "plane" || !moving.normal || !target.normal) throw new Error(FLAT_ONLY);
  const nt = unit(target.normal);
  const R = mul3(rotation(nt, (turnDeg * Math.PI) / 180), align(unit(moving.normal), [-nt[0], -nt[1], -nt[2]]));
  const t = sub(target.center, rot3(R, moving.center));
  return [R[0], R[1], R[2], t[0], R[3], R[4], R[5], t[1], R[6], R[7], R[8], t[2], 0, 0, 0, 1];
}

/** A·B: B first, then A. */
export const compose = (A: Mat4, B: Mat4): Mat4 =>
  [0, 1, 2, 3].flatMap((r) => [0, 1, 2, 3].map((c) => [0, 1, 2, 3].reduce((x, k) => x + A[4 * r + k] * B[4 * k + c], 0)));

/** The file a body came from; bodies from before sources existed share "". */
export const sourceOf = (b: Body) => b.source ?? "";

export function invert(M: Mat4): Mat4 {
  const Rt = [M[0], M[4], M[8], M[1], M[5], M[9], M[2], M[6], M[10]];
  const t = rot3(Rt, [M[3], M[7], M[11]]);
  return [Rt[0], Rt[1], Rt[2], -t[0], Rt[3], Rt[4], Rt[5], -t[1], Rt[6], Rt[7], Rt[8], -t[2], 0, 0, 0, 1];
}

const point = (M: Mat4, p: Vec3): Vec3 =>
  [0, 1, 2].map((r) => M[4 * r] * p[0] + M[4 * r + 1] * p[1] + M[4 * r + 2] * p[2] + M[4 * r + 3]) as Vec3;
const direction = (M: Mat4, v: Vec3): Vec3 =>
  [0, 1, 2].map((r) => M[4 * r] * v[0] + M[4 * r + 1] * v[1] + M[4 * r + 2] * v[2]) as Vec3;

/** The body moved by M: vertices and face centres as points, normals and axes as directions. */
export function transformBody(b: Body, M: Mat4): Body {
  return {
    ...b,
    vertices: b.vertices.map((v) => point(M, v)),
    faces: b.faces.map((f) => ({
      ...f, center: point(M, f.center),
      normal: f.normal && direction(M, f.normal), axis: f.axis && direction(M, f.axis),
    })),
  };
}

/** True when the Mount card bolted these two links together (one holds the mounted face, the other the
 *  face it was put on): their joint is fixed, whatever the contact faces suggest (two flat faces: planar). */
export function isMountedPair(m: ModelState, linkA: string, linkB: string): boolean {
  const bodies = (id: string) => m.links.find((l) => l.id === id)?.body_ids ?? [];
  const [a, b] = [bodies(linkA), bodies(linkB)];
  return Object.values(m.mounts ?? {}).some((mt) => !!mt.fromBody && !!mt.toBody
    && ((a.includes(mt.fromBody) && b.includes(mt.toBody)) || (b.includes(mt.fromBody) && a.includes(mt.toBody))));
}
