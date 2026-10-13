import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { exportChecklist } from "./checklist";
import { baseGuess } from "./geometry";
import { chainDepth, nextJoint } from "./nextJoint";
import { membership, pairKey } from "./pairs";
import { partProblems } from "./problems";
import { modelReducer } from "./reducer";
import { boxBody } from "./testBodies";
import { emptyModel, type ModelState } from "./types";

// pedestal (low, small) under a big arm segment, a second segment, and a far-away loose part
const bodies = () => [
  boxBody("arm", [0, 0, 1], [2, 2, 3]),
  boxBody("pedestal", [0.5, 0.5, 0], [1.5, 1.5, 1]),
  boxBody("fore", [2, 0, 2], [3, 1, 3]),
  boxBody("far", [9, 9, 9], [10, 10, 10]),
];
const imported = (b = bodies()): ModelState => modelReducer(emptyModel(), { type: "import", bodies: b, warnings: [] });
const id = (m: ModelState, body: string) => m.links.find((l) => l.body_ids.includes(body))!.id;
const J = (jid: string, a: string, b: string, extra: Partial<Joint> = {}): Joint => ({ id: jid, parent_link_id: a,
  child_link_id: b, type: "fixed", origin_xyz: [0, 0, 0], origin_rpy: [0, 0, 0], axis: null, limit_lower: null,
  limit_upper: null, source: "manual", ...extra });
function suggest(m: ModelState, a: string, b: string, joint: Joint | null): ModelState {
  const la = m.links.find((l) => l.id === a)!, lb = m.links.find((l) => l.id === b)!;
  return modelReducer(m, { type: "setSuggestion", suggestion: { key: pairKey(a, b), linkA: a, linkB: b,
    membership: membership(la, lb), status: "review",
    result: { joint, reason: joint ? null : "no_contact_faces", evidence: null } } });
}

describe("baseGuess", () => {
  it("picks the lowest link, not the biggest", () => {
    const m = imported();
    expect(baseGuess(m.links, m.bodies)).toBe(id(m, "pedestal"));
    expect(m.rootLinkId).toBe(id(m, "pedestal"));
  });
  it("breaks ties on the lowest z by volume", () => {
    const m = imported([boxBody("small", [0, 0, 0], [1, 1, 1]), boxBody("big", [2, 0, 0], [5, 3, 3])]);
    expect(m.rootLinkId).toBe(id(m, "big"));
  });
  it("a second import keeps the chosen base", () => {
    let m = imported();
    m = modelReducer(m, { type: "setRoot", linkId: id(m, "fore") });
    m = modelReducer(m, { type: "import", bodies: [boxBody("floor", [0, 0, -5], [9, 9, -4])], warnings: [] });
    expect(m.rootLinkId).toBe(id(m, "fore"));
  });
});

describe("nextJoint", () => {
  it("offers a touching pair with exactly one side connected, fixed side first", () => {
    const m = imported();
    expect(nextJoint(m)).toEqual({ fixed: id(m, "pedestal"), moving: id(m, "arm"), suggestion: null });
  });
  it("prefers a pair with a detected joint and walks outward", () => {
    let m = imported();
    m = modelReducer(m, { type: "addJoint", joint: J("j1", id(m, "pedestal"), id(m, "arm")) });
    expect(chainDepth(m).get(id(m, "arm"))).toBe(1);
    m = suggest(m, id(m, "arm"), id(m, "fore"), J("s", id(m, "arm"), id(m, "fore"), { type: "revolute", source: "suggested" }));
    const n = nextJoint(m)!;
    expect([n.fixed, n.moving]).toEqual([id(m, "arm"), id(m, "fore")]);
    expect(n.suggestion?.result.joint?.type).toBe("revolute");
  });
  it("skips rejected pairs and returns null when nothing touches the chain", () => {
    let m = imported();
    m = suggest(m, id(m, "pedestal"), id(m, "arm"), null);
    m = modelReducer(m, { type: "rejectSuggestion", key: pairKey(id(m, "pedestal"), id(m, "arm")) });
    expect(nextJoint(m)).toBeNull();
  });
  it("can be limited to pairs involving one link", () => {
    let m = imported();
    m = modelReducer(m, { type: "addJoint", joint: J("j1", id(m, "pedestal"), id(m, "arm")) });
    expect(nextJoint(m, id(m, "fore"))).toEqual({ fixed: id(m, "arm"), moving: id(m, "fore"), suggestion: null });
    expect(nextJoint(m, id(m, "far"))).toBeNull();
  });
});

describe("partProblems", () => {
  it("lists failed parts, unresolved approximate parts and other warnings; ignored parts drop out", () => {
    let m = imported([
      boxBody("ok", [0, 0, 0], [1, 1, 1]),
      boxBody("gap", [1, 0, 0], [2, 1, 1], { status: "approximate" }),
      boxBody("bad", [0, 0, 0], [1, 1, 1], { status: "failed", vertices: [], triangles: [] }),
      boxBody("odd", [0, 1, 0], [1, 2, 1], { warnings: [{ code: "x", message: "odd faces", fixes: ["reexport"] }] }),
    ]);
    expect(partProblems(m).map((p) => `${p.kind}:${p.bodyId}`)).toEqual(["approximate:gap", "failed:bad", "warning:odd"]);
    m = modelReducer(m, { type: "setLinkProps", linkId: id(m, "gap"), props: { accept_approximate_mass: true } });
    m = modelReducer(m, { type: "ignoreBody", bodyId: "bad" });
    expect(partProblems(m).map((p) => p.bodyId)).toEqual(["odd"]);
  });
});

describe("exportChecklist", () => {
  it("reports unconnected links, mass decisions, names and joint settings", () => {
    let m = imported([boxBody("a", [0, 0, 0], [1, 1, 1]), boxBody("b", [1, 0, 0], [2, 1, 1], { status: "approximate" })]);
    let c = exportChecklist(m, "robot");
    expect(c.map((i) => [i.id, i.ok])).toEqual([["connected", false], ["mass", false], ["names", true], ["joints", true], ["limits", true]]);
    expect(c[0].linkIds).toEqual([id(m, "b")]);
    expect(c[0].step).toBe("connect");
    expect(c[1].detail).toMatch(/b/);
    m = modelReducer(m, { type: "addJoint", joint: J("j", id(m, "a"), id(m, "b"), { type: "revolute", axis: [0, 0, 1] }) });
    m = modelReducer(m, { type: "setLinkProps", linkId: id(m, "b"), props: { mass_override_kg: 1 } });
    m = modelReducer(m, { type: "renameLink", linkId: id(m, "b"), name: "a" });
    c = exportChecklist(m, "bad name");
    expect(c.map((i) => [i.id, i.ok])).toEqual([["connected", true], ["mass", true], ["names", false], ["joints", false], ["limits", true]]);
    expect(c[3].jointIds).toEqual(["j"]);
  });
  it("an empty project asks for an import", () => {
    expect(exportChecklist(emptyModel(), "robot")[0]).toMatchObject({ ok: false, detail: "Import a model first." });
  });
});
