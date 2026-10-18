import { describe, expect, it } from "vitest";
import { at, duration, sceneWithPoses, toTf, type Trace } from "./replay";

const row = (child: string, x: number) => [child, "base", x, 0, 0, 0, 0, 0, 1] as const;
const trace: Trace = {
  version: 1, robot: "bot", program: "programs/task_001.py", base: "base", result: "ok",
  static: [[...row("world", 0)]],
  frames: [{ t: 0.0, tf: [[...row("hand", 0.1)]], poses: {} }, { t: 0.05, tf: [[...row("hand", 0.2)]], poses: { box: [0.5, 0, 0.3, 0, 0, 0, 1] } },
    { t: 0.1, tf: [[...row("hand", 0.3)]], poses: { box: [0.5, 0, 0.4, 0, 0, 0, 1] } }],
  scenes: [{ t: 0.0, scene: { world: { collision_objects: [{ id: "box", header: { frame_id: "base" }, pose: { position: { x: 0.5, y: 0, z: 0.2 } } }] } } },
    { t: 0.06, scene: { world: { collision_objects: [] }, robot_state: { attached_collision_objects: [
      { link_name: "hand", object: { id: "box", header: { frame_id: "hand" }, pose: { position: { x: 0, y: 0, z: 0 } } } }] } } }],
};

describe("replay", () => {
  it("finds the latest sample at or before a time", () => {
    expect(at(trace.frames, -1)).toBe(0);      // before the first: the first
    expect(at(trace.frames, 0.07)).toBe(1);
    expect(at(trace.frames, 9)).toBe(2);       // after the last: the last
    expect(at(trace.scenes, 0.05)).toBe(0);
    expect(duration(trace)).toBe(0.1);
  });

  it("turns stored rows back into transforms for the frame tree", () => {
    const [t] = toTf([[...row("hand", 0.2)]]);
    expect(t.child_frame_id).toBe("hand");
    expect(t.header.frame_id).toBe("base");
    expect(t.transform.translation.x).toBe(0.2);
    expect(t.transform.rotation.w).toBe(1);
  });

  it("draws loose objects where MuJoCo had them, even while held (so a slip shows)", () => {
    const s = sceneWithPoses(trace.scenes[1].scene, trace.frames[2].poses, "base") as any;
    expect(s.robot_state.attached_collision_objects).toEqual([]);
    const [box] = s.world.collision_objects;
    expect(box.id).toBe("box");
    expect(box.header.frame_id).toBe("base");
    expect(box.pose.position).toEqual({ x: 0.5, y: 0, z: 0.4 });
    expect(sceneWithPoses(trace.scenes[0].scene, {}, "base")).toBe(trace.scenes[0].scene);   // no physics: as recorded
  });
});
