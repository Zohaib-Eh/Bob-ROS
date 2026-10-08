import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { modelReducer, sanitizeName } from "./reducer";
import { membership } from "./pairs";
import { emptyModel, type ModelState } from "./types";
import { boxBody } from "./testBodies";

const big = boxBody("big", [0, 0, 0], [1, 1, 1]);
const arm = boxBody("arm", [1, 0, 0], [1.5, 0.2, 0.2]);
const screw = boxBody("screw", [0.9, 0.4, 0.4], [1.0, 0.45, 0.45]);

function imported(): ModelState {
  return modelReducer(emptyModel(), { type: "import", bodies: [big, arm, screw], warnings: [] });
}
const linkOf = (s: ModelState, bodyId: string) => s.links.find((l) => l.body_ids.includes(bodyId))!;
/** Every non-ignored body sits in exactly one link; ignored bodies sit in none. */
function expectEachBodyPlacedOnce(s: ModelState) {
  const placed = s.links.flatMap((l) => l.body_ids).sort();
  const expected = Object.keys(s.bodies).filter((id) => !s.ignored.includes(id)).sort();
  expect(placed).toEqual(expected);
}
const joint = (id: string, a: string, b: string): Joint => ({
  id, parent_link_id: a, child_link_id: b, type: "fixed", origin_xyz: [0, 0, 0],
  origin_rpy: [0, 0, 0], axis: null, limit_lower: null, limit_upper: null, source: "manual" });

describe("modelReducer", () => {
  it("import makes one link per part and picks the lowest (then largest) as base", () => {
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

  it("merge ignores duplicate ids and keeps the target holding both bodies", () => {
    let s = imported();
    const [a, b] = [linkOf(s, "big").id, linkOf(s, "arm").id];
    s = modelReducer(s, { type: "merge", linkIds: [a, b, a] });
    expect(s.links).toHaveLength(2);
    expect(linkOf(s, "big").id).toBe(a);
    expect([...linkOf(s, "big").body_ids].sort()).toEqual(["arm", "big"]);
    expectEachBodyPlacedOnce(s);
  });

  it("mergeSmall keeps every body when a neighbour was already merged away", () => {
    const s1 = boxBody("s1", [2, 0, 0], [2.1, 0.1, 0.1]);
    const s2 = boxBody("s2", [3, 0, 0], [3.1, 0.1, 0.1]);
    const s3 = boxBody("s3", [4, 0, 0], [4.1, 0.1, 0.1]);
    let s = modelReducer(emptyModel(), { type: "import", bodies: [big, s1, s2, s3], warnings: [] });
    const [l1, l2, l3] = ["s1", "s2", "s3"].map((id) => linkOf(s, id).id);
    // s1 merges into s2 first; then s3's neighbour s1 is already gone and must resolve to s2.
    s = modelReducer(s, { type: "mergeSmall", pairs: [
      { linkId: l1, neighbourId: l2 }, { linkId: l3, neighbourId: l1 }] });
    expectEachBodyPlacedOnce(s);
    expect(linkOf(s, "s3").id).toBe(l2);
  });

  it("mergeSmall keeps every body for a chain of small links into the base", () => {
    const s1 = boxBody("s1", [2, 0, 0], [2.1, 0.1, 0.1]);
    const s2 = boxBody("s2", [3, 0, 0], [3.1, 0.1, 0.1]);
    let s = modelReducer(emptyModel(), { type: "import", bodies: [big, s1, s2], warnings: [] });
    const [l1, l2, lb] = ["s1", "s2", "big"].map((id) => linkOf(s, id).id);
    s = modelReducer(s, { type: "mergeSmall", pairs: [
      { linkId: l1, neighbourId: l2 }, { linkId: l2, neighbourId: lb }] });
    expect(s.links).toHaveLength(1);
    expectEachBodyPlacedOnce(s);
  });

  it("moveBody does nothing for an ignored body", () => {
    const s = modelReducer(imported(), { type: "ignoreBody", bodyId: "screw" });
    expect(modelReducer(s, { type: "moveBody", bodyId: "screw", toLinkId: linkOf(s, "big").id })).toBe(s);
    expect(modelReducer(s, { type: "moveBody", bodyId: "nope", toLinkId: linkOf(s, "big").id })).toBe(s);
  });

  it("every body is in exactly one link or ignored after a sequence of merge, move and ignore", () => {
    let s = imported();
    const lid = (bodyId: string) => linkOf(s, bodyId).id;
    s = modelReducer(s, { type: "ignoreBody", bodyId: "screw" });
    expectEachBodyPlacedOnce(s);
    s = modelReducer(s, { type: "unignoreBody", bodyId: "screw" });
    expectEachBodyPlacedOnce(s);
    s = modelReducer(s, { type: "merge", linkIds: [lid("big"), lid("arm"), lid("big")] });
    expectEachBodyPlacedOnce(s);
    s = modelReducer(s, { type: "moveBody", bodyId: "screw", toLinkId: lid("big") });
    expectEachBodyPlacedOnce(s);
    s = modelReducer(s, { type: "ignoreBody", bodyId: "arm" });
    expectEachBodyPlacedOnce(s);
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
    const sug = { key: "k", linkA: b, linkB: a, membership: membership(linkOf(s, "big"), linkOf(s, "arm")), status: "review" as const,
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

  it("ignores a suggestion whose membership is stale; stores a current one", () => {
    const s = imported();
    const [a, b] = [linkOf(s, "big"), linkOf(s, "arm")];
    const sug = { key: "k", linkA: a.id, linkB: b.id, membership: membership(a, b), status: "review" as const,
      result: { joint: null, reason: "no_contact_faces" as const, evidence: null } };
    expect(modelReducer(s, { type: "setSuggestion", suggestion: { ...sug, membership: "stale" } })).toBe(s);
    expect(modelReducer(s, { type: "setSuggestion", suggestion: { ...sug, linkB: "gone" } })).toBe(s);
    expect(modelReducer(s, { type: "setSuggestion", suggestion: sug }).suggestions.k).toEqual(sug);
  });

  it("replaceBody drops suggestions involving its link", () => {
    let s = imported();
    const [a, b, c] = [linkOf(s, "big"), linkOf(s, "arm"), linkOf(s, "screw")];
    const mk = (key: string, x: typeof a, y: typeof a) => ({ key, linkA: x.id, linkB: y.id, membership: membership(x, y),
      status: "review" as const, result: { joint: null, reason: "no_contact_faces" as const, evidence: null } });
    s = modelReducer(s, { type: "setSuggestion", suggestion: mk("ab", a, b) });
    s = modelReducer(s, { type: "setSuggestion", suggestion: mk("bc", b, c) });
    s = modelReducer(s, { type: "replaceBody", body: { ...arm, name: "arm2" } });
    expect(Object.keys(s.suggestions)).toEqual([]);
    s = modelReducer(s, { type: "setSuggestion", suggestion: mk("ab", a, b) });
    s = modelReducer(s, { type: "replaceBody", body: { ...screw, name: "s2" } });
    expect(Object.keys(s.suggestions)).toEqual(["ab"]);
  });

  it("unchanged state is returned by identity for unknown links", () => {
    const s = imported();
    expect(modelReducer(s, { type: "renameLink", linkId: "nope", name: "x" })).toBe(s);
  });
});

describe("part and link editing", () => {
  it("renameBody renames the part and its untouched single-part link", () => {
    let s = imported();
    s = modelReducer(s, { type: "renameBody", bodyId: "arm", name: "upper arm" });
    expect(s.bodies.arm.name).toBe("upper arm");
    expect(linkOf(s, "arm").name).toBe("upper_arm");
  });

  it("renameBody also follows de-duplicated default names like arm_2", () => {
    let s = modelReducer(imported(), { type: "import", warnings: [], bodies: [boxBody("arm2", [5, 5, 5], [6, 6, 6], { name: "arm" })] });
    expect(linkOf(s, "arm2").name).toBe("arm_2");
    s = modelReducer(s, { type: "renameBody", bodyId: "arm2", name: "wrist" });
    expect(linkOf(s, "arm2").name).toBe("wrist");
  });

  it("renameBody keeps a link name the user chose, and links of several parts", () => {
    let s = imported();
    s = modelReducer(s, { type: "renameLink", linkId: linkOf(s, "arm").id, name: "shoulder" });
    s = modelReducer(s, { type: "renameBody", bodyId: "arm", name: "Arm v2" });
    expect(linkOf(s, "arm").name).toBe("shoulder");
    s = modelReducer(s, { type: "merge", linkIds: [linkOf(s, "big").id, linkOf(s, "screw").id] });
    s = modelReducer(s, { type: "renameBody", bodyId: "big", name: "Base plate" });
    expect(linkOf(s, "big").name).toBe("big");
  });

  it("renameBody keeps the geometry arrays, so the viewer need not rebuild the mesh", () => {
    const s = imported();
    const t = modelReducer(s, { type: "renameBody", bodyId: "arm", name: "x" });
    expect(t.bodies.arm.vertices).toBe(s.bodies.arm.vertices);
    expect(modelReducer(s, { type: "renameBody", bodyId: "nope", name: "x" })).toBe(s);
    expect(modelReducer(s, { type: "renameBody", bodyId: "arm", name: "  " })).toBe(s);
  });

  it("ungroupLink splits a link into one link per part and drops its joints", () => {
    let s = imported();
    s = modelReducer(s, { type: "merge", linkIds: [linkOf(s, "big").id, linkOf(s, "screw").id] });
    const group = linkOf(s, "big").id;
    s = modelReducer(s, { type: "addJoint", joint: joint("j", group, linkOf(s, "arm").id) });
    s = modelReducer(s, { type: "ungroupLink", linkId: group });
    expect(s.links).toHaveLength(3);
    expect(linkOf(s, "big").id).toBe(group);
    expect(linkOf(s, "screw").body_ids).toEqual(["screw"]);
    expect(s.joints).toEqual([]);
    expectEachBodyPlacedOnce(s);
    expect(modelReducer(s, { type: "ungroupLink", linkId: linkOf(s, "arm").id })).toBe(s);
  });

  it("ignoreBodies removes several parts from the robot at once", () => {
    let s = imported();
    s = modelReducer(s, { type: "ignoreBodies", bodyIds: ["arm", "screw", "arm", "nope"] });
    expect(s.ignored).toEqual(["arm", "screw"]);
    expectEachBodyPlacedOnce(s);
  });
});
