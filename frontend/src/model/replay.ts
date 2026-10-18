import type { TfTransform } from "../ros/liveRobot";

/** One recorded transform: [child, parent, x, y, z, qx, qy, qz, qw] (see templates/recorder.py). */
export type Row = (string | number)[];
/** A run's recording, <robot>_tasks/runs/<program>-<UTC>.json. poses: MuJoCo's loose objects, [x y z qx qy qz qw]. */
export interface Trace {
  version: number; robot: string; program: string; base: string; result: string;
  static: Row[];
  frames: { t: number; tf: Row[]; poses: Record<string, number[]> }[];
  scenes: { t: number; scene: unknown }[];
}

export const duration = (tr: Trace) => tr.frames[tr.frames.length - 1]?.t ?? 0;

/** Index of the latest sample at or before t (the first one before it starts). */
export function at(samples: { t: number }[], t: number): number {
  let lo = 0, hi = samples.length - 1;
  if (hi < 0 || t <= samples[0].t) return 0;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (samples[mid].t <= t) lo = mid; else hi = mid - 1;
  }
  return lo;
}

export const toTf = (rows: Row[]): (TfTransform & { transform: { translation: { x: number; y: number; z: number }; rotation: { x: number; y: number; z: number; w: number } } })[] =>
  rows.map(([child, parent, x, y, z, qx, qy, qz, qw]) => ({
    child_frame_id: String(child), header: { frame_id: String(parent) },
    transform: { translation: { x: Number(x), y: Number(y), z: Number(z) }, rotation: { x: Number(qx), y: Number(qy), z: Number(qz), w: Number(qw) } },
  }));

/* eslint-disable @typescript-eslint/no-explicit-any */
/** The recorded MoveIt scene with each loose object at MuJoCo's pose (the truth under physics: a held object
 *  that slips, a box that falls); held copies become world objects there. Without poses, the scene as is. */
export function sceneWithPoses(scene: unknown, poses: Record<string, number[]>, base: string): unknown {
  const names = Object.keys(poses);
  if (!names.length) return scene;
  const s = scene as any;
  const place = (o: any) => {
    const p = poses[o.id];
    if (!p) return o;
    return { ...o, header: { ...(o.header ?? {}), frame_id: base },
      pose: { position: { x: p[0], y: p[1], z: p[2] }, orientation: { x: p[3], y: p[4], z: p[5], w: p[6] } } };
  };
  const attached: any[] = s?.robot_state?.attached_collision_objects ?? [];
  return {
    ...s,
    world: { ...(s?.world ?? {}), collision_objects: [...(s?.world?.collision_objects ?? []).map(place),
      ...attached.filter((a) => poses[a.object?.id]).map((a) => place(a.object))] },
    robot_state: { ...(s?.robot_state ?? {}), attached_collision_objects: attached.filter((a) => !poses[a.object?.id]) },
  };
}
