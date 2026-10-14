import { Matrix4, Vector3 } from "three";
import { describe, expect, it } from "vitest";
import type { Joint, Link } from "../api/types";
import { FrameTree, bodyPoses, homeFrames, sceneObjects } from "./liveRobot";

const tf = (parent: string, child: string, t: [number, number, number], q: [number, number, number, number] = [0, 0, 0, 1]) =>
  ({ header: { frame_id: parent }, child_frame_id: child,
     transform: { translation: { x: t[0], y: t[1], z: t[2] }, rotation: { x: q[0], y: q[1], z: q[2], w: q[3] } } });
const link = (id: string, bodies: string[]): Link => ({ id, name: id, body_ids: bodies } as unknown as Link);
const joint = (child: string, xyz: [number, number, number]): Joint =>
  ({ id: child, parent_link_id: "base", child_link_id: child, type: "revolute", origin_xyz: xyz, origin_rpy: [0, 0, 0],
     axis: [0, 0, 1], limit_lower: -1, limit_upper: 1, source: "manual" } as Joint);
const at = (m: Matrix4, p: [number, number, number]) => new Vector3(...p).applyMatrix4(m).toArray();

describe("liveRobot", () => {
  it("composes TF frames up to the base", () => {
    const tree = new FrameTree();
    tree.update([tf("base", "arm", [0, 0, 1]), tf("arm", "hand", [1, 0, 0])]);
    expect(at(tree.world("hand", "base")!, [0, 0, 0])).toEqual([1, 0, 1]);
    expect(tree.world("ghost", "base")).toBeNull();
  });

  it("moves each link's bodies by its motion since home", () => {
    const links = [link("base", ["b0"]), link("arm", ["b1"])];
    const home = homeFrames(links, [joint("arm", [0, 0, 1])], "base");
    const tree = new FrameTree();
    tree.update([tf("base", "arm", [0, 0, 1])]);
    const still = bodyPoses(links, tree, home, "base");
    expect(at(still.b1, [2, 0, 1])).toEqual([2, 0, 1]);
    const s = Math.SQRT1_2;
    tree.update([tf("base", "arm", [0, 0, 1], [0, 0, s, s])]);   // arm turned 90° about z
    const p = at(bodyPoses(links, tree, home, "base").b1, [2, 0, 1]);
    expect(p[0]).toBeCloseTo(0); expect(p[1]).toBeCloseTo(2); expect(p[2]).toBeCloseTo(1);
  });

  it("joint origins are world poses: a downstream link stays put at zero and turns about its own frame", () => {
    const links = [link("base", ["b0"]), link("arm", ["b1"]), link("hand", ["b2"])];
    const home = homeFrames(links, [joint("arm", [0, 0, 1]), joint("hand", [1, 0, 1])], "base");
    const tree = new FrameTree();
    tree.update([tf("base", "arm", [0, 0, 1]), tf("arm", "hand", [1, 0, 0])]);
    expect(at(bodyPoses(links, tree, home, "base").b2, [2, 0, 1])).toEqual([2, 0, 1]);
    const s = Math.SQRT1_2;
    tree.update([tf("arm", "hand", [1, 0, 0], [0, 0, s, s])]);
    const p = at(bodyPoses(links, tree, home, "base").b2, [2, 0, 1]);
    expect(p[0]).toBeCloseTo(1); expect(p[1]).toBeCloseTo(1); expect(p[2]).toBeCloseTo(1);
  });

  it("places world boxes and attached cylinders", () => {
    const tree = new FrameTree();
    tree.update([tf("base", "hand", [1, 0, 0])]);
    const pose = (x: number) => ({ position: { x, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } });
    const scene = {
      world: { collision_objects: [{ id: "box", header: { frame_id: "base" }, pose: pose(0),
        primitives: [{ type: 1, dimensions: [0.1, 0.2, 0.3] }], primitive_poses: [pose(2)] }] },
      robot_state: { attached_collision_objects: [{ link_name: "hand", object: { id: "can", header: { frame_id: "hand" },
        pose: pose(0), primitives: [{ type: 3, dimensions: [0.4, 0.05] }], primitive_poses: [pose(0.5)] } }] },
    };
    const objs = sceneObjects(scene, tree, "base");
    expect(objs.map((o) => [o.id, o.shape, o.attached])).toEqual([["box", "box", false], ["can", "cylinder", true]]);
    expect(at(objs[0].matrix, [0, 0, 0])).toEqual([2, 0, 0]);
    expect(at(objs[1].matrix, [0, 0, 0])).toEqual([1.5, 0, 0]);
    expect(objs[1].dims).toEqual([0.4, 0.05]);
  });
});
