import { Matrix4, Vector3 } from "three";
import type { Joint } from "../api/types";

export function jointMotion(j: Joint, value: number): Matrix4 {
  if (!j.axis) return new Matrix4();
  const axis = new Vector3(...j.axis).normalize();
  const o = new Vector3(...j.origin_xyz);
  if (j.type === "revolute" || j.type === "continuous") {
    return new Matrix4().makeTranslation(o.x, o.y, o.z)
      .multiply(new Matrix4().makeRotationAxis(axis, value))
      .multiply(new Matrix4().makeTranslation(-o.x, -o.y, -o.z));
  }
  if (j.type === "prismatic") {
    const d = axis.multiplyScalar(value);
    return new Matrix4().makeTranslation(d.x, d.y, d.z);
  }
  return new Matrix4();
}

export function previewRange(j: Joint): [number, number] {
  if (j.type === "continuous") return [-Math.PI, Math.PI];
  if ((j.type === "revolute" || j.type === "prismatic") && j.limit_lower !== null && j.limit_upper !== null) {
    return [j.limit_lower, j.limit_upper];
  }
  return [0, 0];
}

export function subtreeLinks(directed: Joint[], jointId: string): string[] {
  const start = directed.find((j) => j.id === jointId);
  if (!start) return [];
  const out = [start.child_link_id];
  for (let i = 0; i < out.length; i++) {
    for (const j of directed) if (j.parent_link_id === out[i]) out.push(j.child_link_id);
  }
  return out;
}
