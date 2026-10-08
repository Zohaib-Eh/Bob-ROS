import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { modelReducer, sanitizeName } from "./reducer";
import { emptyModel, type ModelState } from "./types";
import { boxBody } from "./testBodies";

const big = boxBody("big", [0, 0, 0], [1, 1, 1]);
const arm = boxBody("arm", [1, 0, 0], [1.5, 0.2, 0.2]);
const screw = boxBody("screw", [0.9, 0.4, 0.4], [1.0, 0.45, 0.45]);

function imported(): ModelState {
  return modelReducer(emptyModel(), { type: "import", bodies: [big, arm, screw], warnings: [] });
}
const linkOf = (s: ModelState, bodyId: string) => s.links.find((l) => l.body_ids.includes(bodyId))!;
const joint = (id: string, a: string, b: string): Joint => ({
  id, parent_link_id: a, child_link_id: b, type: "fixed", origin_xyz: [0, 0, 0],
  origin_rpy: [0, 0, 0], axis: null, limit_lower: null, limit_upper: null, source: "manual" });

describe("modelReducer", () => {
  it("import makes one link per part and picks the largest as base", () => {
    const s = imported();
    expect(s.links).toHaveLength(3);
    expect(s.links.map((l) => l.name)).toEqual(["big", "arm", "screw"]);
    expect(s.rootLinkId).toBe(linkOf(s, "big").id);
    expect(linkOf(s, "arm").density_kg_m3).toBe(1240);
  });

  it("sanitizes and de-duplicates link names", () => {
    const a = boxBody("x1", [0, 0, 0], [1, 1, 1], { name: "IRB1300 IP40 (1)" });
    const b = boxBody("x2", [2, 0, 0], [3, 1, 1], { name: "IRB1300 IP40 (1)" });
    const c = boxBody("x3", [4, 0, 0], [5, 1, 1], { name: "-lead" });
    const s = modelReducer(emptyModel(), { type: "import", bodies: [a, b, c], warnings: [] });
    expect(s.links.map((l) => l.name)).toEqual(["IRB1300_IP40__1_", "IRB1300_IP40__1__2", "p-lead"]);
    expect(sanitizeName("")).toBe("part");
  });

  it("merge keeps the first link and remaps joints, dropping self-joints", () => {
    let s = imported();
    const [b, a, sc] = ["big", "arm", "screw"].map((id) => linkOf(s, id).id);
    s = modelReducer(s, { type: "addJoint", joint: joint("j1", b, a) });
    s = modelReducer(s, { type: "addJoint", joint: joint("j2", sc, b) });
    s = modelReducer(s, { type: "merge", linkIds: [b, sc] });
    expect(s.links).toHaveLength(2);
    expect(linkOf(s, "screw").id).toBe(b);
    expect(s.joints.map((j) => j.id)).toEqual(["j1"]);
  });

  it("moveBody removes an emptied link and its joints", () => {
    let s = imported();
    const [b, sc] = [linkOf(s, "big").id, linkOf(s, "screw").id];
    s = modelReducer(s, { type: "addJoint", joint: joint("j", b, sc) });
    s = modelReducer(s, { type: "moveBody", bodyId: "screw", toLinkId: b });
    expect(s.links).toHaveLength(2);
    expect(linkOf(s, "screw").id).toBe(b);
    expect(s.joints).toHaveLength(0);
  });

  it("ignore and unignore a part", () => {
    let s = modelReducer(imported(), { type: "ignoreBody", bodyId: "screw" });
    expect(s.ignored).toEqual(["screw"]);
    expect(s.links.some((l) => l.body_ids.includes("screw"))).toBe(false);
    s = modelReducer(s, { type: "unignoreBody", bodyId: "screw" });
    expect(s.ignored).toEqual([]);
    expect(linkOf(s, "screw").body_ids).toEqual(["screw"]);
  });

  it("base moves to the largest remaining link when the base link disappears", () => {
    let s = imported();
    s = modelReducer(s, { type: "setRoot", linkId: linkOf(s, "screw").id });
    s = modelReducer(s, { type: "ignoreBody", bodyId: "screw" });
    expect(s.rootLinkId).toBe(linkOf(s, "big").id);
  });

  it("accepting a suggestion stores a confirmed joint; removing it returns the suggestion to review", () => {
    let s = imported();
    const [b, a] = [linkOf(s, "big").id, linkOf(s, "arm").id];
    const sug = { key: "k", linkA: b, linkB: a, membership: "m", status: "review" as const,
      result: { joint: { ...joint("j", b, a), source: "suggested" as const }, reason: null, evidence: null } };
    s = modelReducer(s, { type: "setSuggestion", suggestion: sug });
    s = modelReducer(s, { type: "acceptSuggestion", key: "k", joint: { ...sug.result.joint, type: "continuous" } });
    expect(s.joints[0]).toMatchObject({ id: "j", type: "continuous", source: "confirmed" });
    expect(s.suggestions.k.status).toBe("accepted");
    s = modelReducer(s, { type: "removeJoint", jointId: "j" });
    expect(s.joints).toHaveLength(0);
    expect(s.suggestions.k.status).toBe("review");
  });

  it("replaceBody swaps geometry but keeps the link", () => {
    let s = imported();
    const lid = linkOf(s, "arm").id;
    s = modelReducer(s, { type: "replaceBody", body: { ...arm, status: "ok", name: "arm2" } });
    expect(s.bodies.arm.name).toBe("arm2");
    expect(linkOf(s, "arm").id).toBe(lid);
  });

  it("unchanged state is returned by identity for unknown links", () => {
    const s = imported();
    expect(modelReducer(s, { type: "renameLink", linkId: "nope", name: "x" })).toBe(s);
  });
});
