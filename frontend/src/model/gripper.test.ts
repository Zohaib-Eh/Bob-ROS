import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { buildExportRequest } from "./exportRequest";
import { gripSummary, gripperIssues, gripperWarnings, jawGap, jawTwinAxis, tcpPoint } from "./gripper";
import { modelReducer } from "./reducer";
import { boxBody } from "./testBodies";
import { emptyModel, type ModelState } from "./types";
import type { Body } from "../api/types";

const slider = (id: string, child: string, axis: [number, number, number], over: Partial<Joint> = {}): Joint => ({
  id, parent_link_id: "L_base", child_link_id: child, type: "prismatic", origin_xyz: [0, 0, 0], origin_rpy: [0, 0, 0],
  axis, limit_lower: 0, limit_upper: 0.025, source: "manual", max_effort: null, ...over,
});

/** Base plate on top, two box fingers 50 mm apart hanging below (like the backend fixture, without hooks). */
function model(): ModelState {
  let m = modelReducer(emptyModel(), { type: "import", source: "gripper.step", warnings: [], bodies: [
    boxBody("base", [-0.04, -0.02, 0], [0.04, 0.02, 0.01]),
    boxBody("fl", [-0.035, -0.01, -0.06], [-0.025, 0.01, 0]),
    boxBody("fr", [0.025, -0.01, -0.06], [0.035, 0.01, 0]),
  ] });
  m = { ...m, links: m.links.map((l) => ({ ...l, id: `L_${l.body_ids[0]}` })) };
  m = modelReducer(m, { type: "addJoint", joint: slider("jl", "L_fl", [1, 0, 0]) });
  return modelReducer(m, { type: "addJoint", joint: slider("jr", "L_fr", [-1, 0, 0]) });
}
const G = { leader: "jl", follower: "jr", tcp_offset: 0 };

describe("gripperIssues", () => {
  it("a valid finger pair has no issues", () => expect(gripperIssues(model(), G)).toEqual([]));

  it("fingers that are one part", () => {
    const m = model();
    const one = { ...m, links: m.links.map((l) => (l.id === "L_fr" ? { ...l, body_ids: ["fl"] } : l)) };
    expect(gripperIssues(one, G)).toContain(
      "The fingers are one part; the gripper needs separate finger parts. Pick a different model or split it in your CAD tool.");
  });

  it("finger axes that are not parallel", () => {
    const m = modelReducer(model(), { type: "updateJoint", joint: slider("jr", "L_fr", [0, 1, 0]) });
    expect(gripperIssues(m, G)).toContain("The fingers don't slide along the same line.");
  });

  it("missing limits", () => {
    const m = modelReducer(model(), { type: "updateJoint", joint: slider("jr", "L_fr", [-1, 0, 0], { limit_upper: null }) });
    expect(gripperIssues(m, G)).toContain("Set the fingers' travel (limits) first.");
  });

  it("fingers on different parents", () => {
    const m = modelReducer(model(), { type: "updateJoint", joint: slider("jr", "L_fr", [-1, 0, 0], { parent_link_id: "L_fl" }) });
    expect(gripperIssues(m, G)).toContain("Both fingers must be joined to the same gripper base.");
  });
});

describe("gripper in the model", () => {
  it("setGripper stores it and the export request carries it", () => {
    const m = modelReducer(model(), { type: "setGripper", gripper: G });
    expect(m.gripper).toEqual(G);
    expect(buildExportRequest(m).gripper).toEqual(G);
    expect(buildExportRequest(model()).gripper).toBeNull();
  });

  it("removing_a_finger_joint_clears_the_gripper", () => {   // Review Focus 4
    let m = modelReducer(model(), { type: "setGripper", gripper: G });
    m = modelReducer(m, { type: "removeJoint", jointId: "jr" });
    expect(m.gripper).toBeNull();
  });

  it("deleting a finger's part clears the gripper", () => {   // Review Focus 4
    let m = modelReducer(model(), { type: "setGripper", gripper: G });
    m = modelReducer(m, { type: "ignoreBody", bodyId: "fl" });
    expect(m.gripper).toBeNull();
  });

  it("jaw gap and tool point between the fingertips", () => {
    expect(jawGap(model(), G)).toBeCloseTo(0.05, 9);
    const [x, y, z] = tcpPoint(model(), G);
    expect(x).toBeCloseTo(0, 9); expect(y).toBeCloseTo(0, 9);
    expect(z).toBeCloseTo(-0.06, 9);   // fingertips' far end, away from the base plate
    expect(tcpPoint(model(), { ...G, tcp_offset: 0.01 })[2]).toBeCloseTo(-0.07, 9);
  });
});

/** Fingers made of several boxes each (merged into one link per finger), sliding along x. */
function fingers(fl: Body[], fr: Body[], [lo, hi]: [number, number]): ModelState {
  let m = modelReducer(emptyModel(), { type: "import", source: "g.step", warnings: [],
    bodies: [boxBody("base", [-0.04, -0.02, 0], [0.04, 0.02, 0.01]), ...fl, ...fr] });
  m = { ...m, links: m.links.map((l) => ({ ...l, id: `L_${l.body_ids[0]}` })) };
  for (const group of [fl, fr]) if (group.length > 1) m = modelReducer(m, { type: "merge", linkIds: group.map((b) => `L_${b.id}`) });
  m = modelReducer(m, { type: "addJoint", joint: slider("jl", `L_${fl[0].id}`, [1, 0, 0], { limit_lower: lo, limit_upper: hi }) });
  return modelReducer(m, { type: "addJoint", joint: slider("jr", `L_${fr[0].id}`, [-1, 0, 0], { limit_lower: lo, limit_upper: hi }) });
}
const mirror = (id: string, [x0, y0, z0]: [number, number, number], [x1, y1, z1]: [number, number, number]) =>
  [boxBody(`${id}l`, [-x1, y0, z0], [-x0, y1, z1]), boxBody(`${id}r`, [x0, y0, z0], [x1, y1, z1])];
/** AirTAC HFK40-like bare jaws: slides touch inside the plate and poke 4 mm out below it, between the jaws;
 *  the tips that stick out are 30 mm apart. */
const [slideL, slideR] = mirror("slide", [0, -0.01, -0.004], [0.035, 0.01, 0.008]);
const [tipL, tipR] = mirror("tip", [0.015, -0.01, -0.06], [0.025, 0.01, 0]);

describe("what the gripper can hold, and warnings that say why", () => {
  it("fingers that close fully: no warnings", () => {
    expect(gripSummary(model(), G)).toBe("Opens to 50 mm and closes fully · jaws 60 mm deep · grip force 40 N per finger");
    expect(gripperWarnings(model(), G)).toEqual([]);
  });

  it("bare jaws whose tips stay apart: the size range and why smaller parts aren't gripped", () => {
    const m = fingers([slideL, tipL], [slideR, tipR], [-0.015, 0]);
    expect(gripSummary(m, G)).toBe("Grips objects 30 to 60 mm across the jaws · jaws 56 mm deep · grip force 40 N per finger");
    const [w] = gripperWarnings(m, G);
    expect(w).toContain("Fully closed, the jaw tips are still 30 mm apart");
    expect(w).toContain("fingers");
  });

  it("jaws modelled closed with travel that closes them further", () => {
    const m = fingers([slideL, tipL], [slideR, tipR], [0, 0.015]);
    expect(gripperWarnings(m, G).join(" ")).toContain("already touch");
  });

  it("hooks at the fingertips", () => {
    const [upL, upR] = mirror("up", [0.025, -0.01, -0.06], [0.035, 0.01, 0]);
    const [hookL, hookR] = mirror("hook", [0.021, -0.01, -0.06], [0.025, 0.01, -0.052]);
    const w = gripperWarnings(fingers([upL, hookL], [upR, hookR], [0, 0.025]), G).join(" ");
    expect(w).toContain("The fingertips reach in further than the gripping faces (tips 42 mm apart, faces 50 mm)");
  });
});

describe("jawTwinAxis", () => {
  it("a jaw with a same-size twin slides toward it", () => {
    const m = model();
    const t = jawTwinAxis(m, "L_fl", "L_base")!;
    expect(t.twin).toBe("L_fr");
    t.axis.forEach((v, i) => expect(v).toBeCloseTo([1, 0, 0][i], 9));   // fl at -x, fr at +x
  });

  it("a part without a twin gets nothing", () => {
    expect(jawTwinAxis(model(), "L_base", "L_fl")).toBeNull();
  });
});
