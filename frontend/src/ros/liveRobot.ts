import { Euler, Matrix4, Quaternion, Vector3 } from "three";
import type { Joint, Link } from "../api/types";

export interface TfTransform {
  header: { frame_id: string };
  child_frame_id: string;
  transform: { translation: { x: number; y: number; z: number }; rotation: { x: number; y: number; z: number; w: number } };
}
export interface SceneObject { id: string; shape: "box" | "cylinder"; dims: number[]; matrix: Matrix4; attached: boolean }
/** GetPlanningScene components: ROBOT_STATE_ATTACHED_OBJECTS (4) | WORLD_OBJECT_GEOMETRY (16). */
export const SCENE_COMPONENTS = 20;

const clean = (f: string) => f.replace(/^\//, "");
const poseMatrix = (p: { position: { x: number; y: number; z: number }; orientation: { x: number; y: number; z: number; w: number } }) =>
  new Matrix4().compose(new Vector3(p.position.x, p.position.y, p.position.z),
    new Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), new Vector3(1, 1, 1));

/** Parent links and transforms from /tf and /tf_static. */
export class FrameTree {
  private edges = new Map<string, { parent: string; m: Matrix4 }>();

  update(ts: TfTransform[]) {
    for (const t of ts) {
      const { translation: p, rotation: q } = t.transform;
      this.edges.set(clean(t.child_frame_id), { parent: clean(t.header.frame_id),
        m: new Matrix4().compose(new Vector3(p.x, p.y, p.z), new Quaternion(q.x, q.y, q.z, q.w), new Vector3(1, 1, 1)) });
    }
  }

  world(frame: string, root: string): Matrix4 | null {
    const m = new Matrix4();
    let f = clean(frame);
    for (let guard = 0; f !== root; guard++) {
      const e = this.edges.get(f);
      if (!e || guard > 100) return null;
      m.premultiply(e.m);
      f = e.parent;
    }
    return m;
  }
}

/** Each link's frame at the CAD pose (the exported URDF's zero configuration), by link name. */
export function homeFrames(links: Link[], joints: Joint[], rootId: string): Map<string, Matrix4> {
  const byChild = new Map(joints.map((j) => [j.child_link_id, j]));
  const out = new Map<string, Matrix4>();
  for (const l of links) {
    const j = byChild.get(l.id);
    if (l.id === rootId || !j) { out.set(l.name, new Matrix4()); continue; }
    const [r, p, y] = j.origin_rpy;
    out.set(l.name, new Matrix4().compose(new Vector3(...j.origin_xyz),
      new Quaternion().setFromEuler(new Euler(r, p, y, "ZYX")), new Vector3(1, 1, 1)));
  }
  return out;
}

/** Body meshes are in the CAD world frame: move each by its link's motion since the CAD pose. */
export function bodyPoses(links: Link[], tree: FrameTree, home: Map<string, Matrix4>, base: string): Record<string, Matrix4> {
  const out: Record<string, Matrix4> = {};
  for (const l of links) {
    const now = l.name === base ? new Matrix4() : tree.world(l.name, base);
    const h = home.get(l.name);
    if (!now || !h) continue;
    const delta = now.clone().multiply(h.clone().invert());
    for (const id of l.body_ids) out[id] = delta;
  }
  return out;
}

function objectsOf(o: any, frameM: Matrix4, attached: boolean): SceneObject[] {
  const base = frameM.clone().multiply(o.pose ? poseMatrix(o.pose) : new Matrix4());
  return (o.primitives ?? []).flatMap((p: any, i: number) => {
    const shape = p.type === 1 ? "box" : p.type === 3 ? "cylinder" : null;
    if (!shape) return [];
    const local = o.primitive_poses?.[i] ? poseMatrix(o.primitive_poses[i]) : new Matrix4();
    return [{ id: o.id, shape, dims: [...p.dimensions], matrix: base.clone().multiply(local), attached }];
  });
}

export function sceneObjects(scene: any, tree: FrameTree, base: string): SceneObject[] {
  const frame = (f: string) => (clean(f || base) === base ? new Matrix4() : tree.world(f, base));
  const out: SceneObject[] = [];
  for (const o of scene?.world?.collision_objects ?? []) {
    const m = frame(o.header?.frame_id);
    if (m) out.push(...objectsOf(o, m, false));
  }
  for (const a of scene?.robot_state?.attached_collision_objects ?? []) {
    const m = frame(a.object?.header?.frame_id || a.link_name);
    if (m) out.push(...objectsOf(a.object, m, true));
  }
  return out;
}
