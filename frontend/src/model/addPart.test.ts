import { describe, expect, it } from "vitest";
import type { Body } from "../api/types";
import { bodyBox } from "./geometry";
import { historyReducer, initialHistory } from "./history";
import { modelReducer } from "./reducer";
import { boxBody } from "./testBodies";
import { emptyModel } from "./types";

// a robot whose base sits on the world origin, like the irb1300 on its pedestal
const robot = () => modelReducer(emptyModel(), { type: "import", source: "robot.step", warnings: [],
  bodies: [boxBody("pedestal", [-0.3, -0.3, 0], [0.3, 0.3, 0.8]), boxBody("arm", [0, -0.1, 0.8], [1.2, 0.1, 1.0])] });
// a gripper whose own file origin is inside that pedestal (AirTAC HFK40 from GrabCAD)
const gripper = () => [boxBody("gbody", [-0.139, -0.066, -0.024], [0, 0.066, 0.024])];
const empty: Body = { id: "dummy", name: "HFK40(CL)_dummy", vertices: [], triangles: [], faces: [], status: "failed",
  warnings: [{ code: "empty", message: "HFK40(CL)_dummy: no geometry was transferred", fixes: ["replace", "ignore"] }] };

describe("Add part from CAD", () => {
  it("a part that would sit inside the robot is set down beside it, on the same floor", () => {
    const m = modelReducer(robot(), { type: "import", source: "hfk40.stp", warnings: [], bodies: gripper() });
    const g = bodyBox(m.bodies.gbody)!;
    expect(g.min[0]).toBeGreaterThan(1.2);                   // clear of the robot (arm reaches x 1.2)
    expect(g.min[2]).toBeCloseTo(0, 9);                      // on the robot's floor
    expect(g.max[0] - g.min[0]).toBeCloseTo(0.139, 9);       // moved, not resized
    expect(m.bodies.pedestal.vertices).toEqual(robot().bodies.pedestal.vertices);
  });

  it("one undo takes the added part away again", () => {
    let h = historyReducer(initialHistory(), { type: "import", source: "robot.step", warnings: [], bodies: Object.values(robot().bodies) });
    h = historyReducer(h, { type: "import", source: "hfk40.stp", warnings: [], bodies: gripper() });
    h = historyReducer(h, { type: "undo" });
    expect(h.present.bodies.gbody).toBeUndefined();
  });

  it("the first import into an empty project stays where the file puts it", () => {
    const m = modelReducer(emptyModel(), { type: "import", source: "hfk40.stp", warnings: [], bodies: gripper() });
    expect(m.bodies.gbody.vertices).toEqual(gripper()[0].vertices);
  });

  it("an empty placeholder in the file is left out (listed as ignored), not shown as a failed part", () => {
    const m = modelReducer(robot(), { type: "import", source: "hfk40.stp", warnings: [], bodies: [...gripper(), empty] });
    expect(m.ignored).toContain("dummy");
    expect(m.links.some((l) => l.body_ids.includes("dummy"))).toBe(false);
  });
});
