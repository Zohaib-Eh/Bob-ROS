import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { buildExportRequest } from "./exportRequest";
import { gripperIssues, jawGap, tcpPoint } from "./gripper";
import { modelReducer } from "./reducer";
import { boxBody } from "./testBodies";
import { emptyModel, type ModelState } from "./types";

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
