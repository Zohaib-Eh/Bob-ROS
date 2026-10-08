import {
  ArrowHelper, BufferGeometry, Group, Line, LineBasicMaterial, LineDashedMaterial, Material, Mesh, MeshBasicMaterial,
  Object3D, OctahedronGeometry, SphereGeometry, Vector3,
} from "three";
import type { Joint, Vec3 } from "../api/types";

export interface JointGlyph { joint: Joint; state: "done" | "suggested" | "active" }
export const GLYPH_COLORS = { done: 0x3f4349, suggested: 0x6b7280, active: 0x2563eb } as const;
const HANDLE = { lower: 0x2563eb, upper: 0xf59e0b } as const;

const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a: Vec3): Vec3 => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const add = (a: Vec3, b: Vec3, s = 1): Vec3 => [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s];
const v3 = (p: Vec3) => new Vector3(...p);

/** Unit vector at right angles to the axis: the 0° direction for arcs and angles. */
export function perpendicular(axis: Vec3): Vec3 {
  const a = norm(axis);
  const helper: Vec3 = Math.abs(a[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
  return norm(cross(helper, a));
}

export function arcPoints(origin: Vec3, axis: Vec3, radius: number, from: number, to: number, segments = 48): Vec3[] {
  const a = norm(axis), u = perpendicular(a), v = cross(a, u);
  return Array.from({ length: segments + 1 }, (_, i) => {
    const t = from + ((to - from) * i) / segments;
    return add(add(origin, u, radius * Math.cos(t)), v, radius * Math.sin(t));
  });
}

/** Signed angle of p around the axis, measured from perpendicular(axis), in (−π, π]. */
export function angleAroundAxis(p: Vec3, origin: Vec3, axis: Vec3): number {
  const a = norm(axis), u = perpendicular(a), v = cross(a, u), d = sub(p, origin);
  return Math.atan2(dot(d, v), dot(d, u));
}

export function rayPlaneHit(ro: Vec3, rd: Vec3, origin: Vec3, normal: Vec3): Vec3 | null {
  const denom = dot(rd, normal);
  if (Math.abs(denom) < 1e-9) return null;
  const t = dot(sub(origin, ro), normal) / denom;
  return t < 0 ? null : add(ro, rd, t);
}

/** Position along the axis line of the point closest to the ray. */
export function closestParamOnAxis(ro: Vec3, rd: Vec3, origin: Vec3, axis: Vec3): number {
  const a = norm(axis), d = norm(rd), w = sub(origin, ro);
  const b = dot(a, d), denom = 1 - b * b;
  if (denom < 1e-9) return 0;
  return (b * dot(w, d) - dot(w, a)) / denom;
}

function line(points: Vec3[], color: number): Line {
  return new Line(new BufferGeometry().setFromPoints(points.map(v3)), new LineBasicMaterial({ color }));
}

function ball(at: Vec3, r: number, color: number, handle?: "lower" | "upper"): Mesh {
  const m = new Mesh(new SphereGeometry(r, 16, 12), new MeshBasicMaterial({ color }));
  m.position.set(...at);
  if (handle) m.userData.handle = handle;
  return m;
}

export function buildGlyph(g: JointGlyph, size: number): Group {
  const { joint: j, state } = g;
  const color = GLYPH_COLORS[state];
  const grp = new Group();
  const o = j.origin_xyz;
  grp.add(ball(o, size * 0.035, color));
  if (j.type === "fixed" || !j.axis) {
    const m = new Mesh(new OctahedronGeometry(size * 0.08), new MeshBasicMaterial({ color }));
    m.position.set(...o);
    grp.add(m);
  } else {
    const a = norm(j.axis);
    const active = state === "active";
    grp.add(new ArrowHelper(v3(a), v3(add(o, a, -size * 0.5)), size, color, size * 0.15, size * 0.08));
    if (j.type === "revolute" || j.type === "continuous" || j.type === "cylindrical") {
      const limited = j.type === "revolute" && j.limit_lower !== null && j.limit_upper !== null;
      const [lo, hi] = limited ? [j.limit_lower!, j.limit_upper!] : [-Math.PI, Math.PI];
      const r = size * 0.45;
      grp.add(line(arcPoints(o, a, r, lo, hi), color));
      if (active && limited) {
        grp.add(ball(arcPoints(o, a, r, lo, lo, 1)[0], size * 0.06, HANDLE.lower, "lower"));
        grp.add(ball(arcPoints(o, a, r, hi, hi, 1)[0], size * 0.06, HANDLE.upper, "upper"));
      }
    } else if (j.type === "prismatic") {
      const lo = j.limit_lower ?? 0, hi = j.limit_upper ?? 0;
      grp.add(line([add(o, a, lo), add(o, a, hi)], color));
      if (active) {
        grp.add(ball(add(o, a, lo), size * 0.06, HANDLE.lower, "lower"));
        grp.add(ball(add(o, a, hi), size * 0.06, HANDLE.upper, "upper"));
      }
    } else if (j.type === "planar") {
      grp.add(line(arcPoints(o, a, size * 0.4, Math.PI / 4, Math.PI / 4 + 2 * Math.PI, 4), color));
    }
  }
  if (state === "suggested") {
    grp.traverse((c) => {
      const mat = (c as Mesh).material as Material | undefined;
      if (mat) { mat.transparent = true; mat.opacity = 0.55; }
    });
  }
  return grp;
}

export function buildSnap(s: { origin: Vec3; axis: Vec3 }, size: number): Group {
  const g = new Group();
  const c = GLYPH_COLORS.active;
  const a = norm(s.axis);
  g.add(line(arcPoints(s.origin, a, size * 0.2, 0, 2 * Math.PI), c));
  g.add(line([add(s.origin, a, -size * 0.6), add(s.origin, a, size * 0.6)], c));
  g.add(ball(s.origin, size * 0.035, c));
  return g;
}

export function buildLooseLine(a: Vec3, b: Vec3, size: number): Line {
  const l = new Line(new BufferGeometry().setFromPoints([v3(a), v3(b)]),
    new LineDashedMaterial({ color: 0xdc2626, dashSize: size * 0.08, gapSize: size * 0.05 }));
  l.computeLineDistances();
  return l;
}

export function disposeTree(o: Object3D) {
  o.traverse((c) => {
    const m = c as Mesh;
    m.geometry?.dispose();
    const mat = m.material as Material | Material[] | undefined;
    if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
    else mat?.dispose();
  });
}
