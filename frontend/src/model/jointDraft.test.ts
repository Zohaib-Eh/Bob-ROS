import { describe, expect, it } from "vitest";
import type { Body, Joint } from "../api/types";
import { contactCenter, DEFAULT_LIMITS, draftForPair, draftFromJoint, draftFromSuggestion, draftPreviewJoint, draftProblem, draftToJoint, emptyDraft, jointType, motionOf, setMotion, snapAt, parseEffort } from "./jointDraft";
import { modelReducer } from "./reducer";
import { boxBody } from "./testBodies";
import { emptyModel, type Suggestion } from "./types";

const joint = (extra: Partial<Joint> = {}): Joint => ({ id: "j1", parent_link_id: "A", child_link_id: "B",
  type: "cylindrical", origin_xyz: [1, 2, 3], origin_rpy: [0, 0, 0], axis: [0, 0, 1], limit_lower: null,
  limit_upper: null, source: "suggested", ambiguous: true, ...extra });
const sugg = (j: Joint): Suggestion => ({ key: "A|B", linkA: "A", linkB: "B", membership: "m", status: "review",
  result: { joint: j, reason: null, evidence: null } });

describe("motion mapping", () => {
  it("maps Fusion motions to URDF types and back", () => {
    expect(jointType({ motion: "rigid", noLimits: false })).toBe("fixed");
    expect(jointType({ motion: "revolute", noLimits: false })).toBe("revolute");
    expect(jointType({ motion: "revolute", noLimits: true })).toBe("continuous");
    expect(jointType({ motion: "slider", noLimits: false })).toBe("prismatic");
    expect(jointType({ motion: "planar", noLimits: false })).toBe("planar");
    expect(jointType({ motion: null, noLimits: false })).toBeNull();
    expect(motionOf("cylindrical")).toBe("revolute");
    expect(motionOf("continuous")).toBe("revolute");
    expect(motionOf("spherical")).toBeNull();
  });
  it("choosing a motion pre-fills its default limits; re-choosing keeps edits", () => {
    let d = setMotion(emptyDraft("A", "B"), "revolute");
    expect([d.lower, d.upper]).toEqual(DEFAULT_LIMITS.revolute);
    d = setMotion({ ...d, lower: -1 }, "revolute");
    expect(d.lower).toBe(-1);
    d = setMotion(d, "slider");
    expect([d.lower, d.upper]).toEqual(DEFAULT_LIMITS.slider);
    expect(setMotion(d, "rigid").lower).toBeNull();
  });
});

describe("drafts from detection", () => {
  it("a cylindrical suggestion becomes Revolute with default limits", () => {
    const d = draftFromSuggestion(sugg(joint()), "A");
    expect(d).toMatchObject({ fixed: "A", moving: "B", motion: "revolute", noLimits: false, detected: "cylindrical",
      ambiguous: true, suggestionKey: "A|B", origin: [1, 2, 3], axis: [0, 0, 1] });
    expect([d.lower, d.upper]).toEqual(DEFAULT_LIMITS.revolute);
    expect(draftProblem(d)).toBeNull();
    expect(draftToJoint(d)).toMatchObject({ type: "revolute", parent_link_id: "A", child_link_id: "B",
      limit_lower: -Math.PI, limit_upper: Math.PI, source: "confirmed" });
  });
  it("puts the connected link on the fixed side even when detection had it as child", () => {
    const d = draftFromSuggestion(sugg(joint()), "B");
    expect([d.fixed, d.moving]).toEqual(["B", "A"]);
  });
  it("a planar suggestion stays Planar so the preview can show it is wrong", () => {
    expect(draftFromSuggestion(sugg(joint({ type: "planar" })), "A").motion).toBe("planar");
  });
  it("an existing joint round-trips", () => {
    const j = joint({ type: "continuous", source: "manual", ambiguous: undefined });
    const d = draftFromJoint(j);
    expect(d.noLimits).toBe(true);
    expect(draftToJoint(d)).toMatchObject({ id: "j1", type: "continuous", limit_lower: null, source: "manual" });
  });
});

describe("draftProblem", () => {
  it("names the first missing piece", () => {
    let d = emptyDraft();
    expect(draftProblem(d)).toMatch(/fixed part/);
    d = { ...d, fixed: "A" };
    expect(draftProblem(d)).toMatch(/moving part/);
    d = { ...d, moving: "A" };
    expect(draftProblem(d)).toMatch(/two different/);
    d = { ...d, moving: "B" };
    expect(draftProblem(d)).toMatch(/how the joint moves/);
    d = setMotion(d, "revolute");
    expect(draftProblem(d)).toMatch(/origin/);
    d = { ...d, origin: [0, 0, 0] };
    expect(draftProblem(d)).toMatch(/axis/);
    d = { ...d, axis: [0, 0, 1], lower: 1, upper: 0 };
    expect(draftProblem(d)).toMatch(/lower limit/);
    d = { ...d, lower: null };
    expect(draftProblem(d)).toMatch(/both limits/);
    expect(draftProblem({ ...d, noLimits: true })).toBeNull();
    expect(draftProblem({ ...setMotion(d, "rigid"), axis: null })).toBeNull();
  });
  it("preview joint needs parts and an origin", () => {
    expect(draftPreviewJoint(emptyDraft("A", "B"))).toBeNull();
    expect(draftPreviewJoint({ ...emptyDraft("A", "B"), origin: [0, 0, 0] })?.type).toBe("fixed");
  });
});

describe("snapAt", () => {
  const body: Body = { ...boxBody("b", [0, 0, 0], [1, 1, 1]), faces: [
    { kind: "cylinder", origin: "brep", normal: null, axis: [0, 1, 0], center: [0.5, 0.5, 0.5], radius: 0.1,
      residual: 0, triangle_indices: [0, 1] },
    { kind: "plane", origin: "brep", normal: [0, 0, 1], axis: null, center: [9, 9, 9], residual: 0, radius: null,
      triangle_indices: [2, 3] },
  ] };
  it("a round face gives its axis through its centre", () => {
    expect(snapAt(body, 1)).toEqual({ kind: "cylinder", origin: [0.5, 0.5, 0.5], axis: [0, 1, 0] });
  });
  it("a flat face gives its normal at the clicked triangle's centre", () => {
    const s = snapAt(body, 2)!;
    expect(s.kind).toBe("plane");
    expect(s.axis).toEqual([0, 0, 1]);
    const [a, b, c] = body.triangles[2].map((i) => body.vertices[i]);
    expect(s.origin).toEqual([0, 1, 2].map((k) => (a[k] + b[k] + c[k]) / 3));
  });
  it("other triangles give nothing", () => {
    expect(snapAt(body, 7)).toBeNull();
  });
});

describe("contactCenter and draftForPair", () => {
  const m = modelReducer(emptyModel(), { type: "import", warnings: [], bodies: [
    boxBody("a", [0, 0, 0], [1, 1, 1]), boxBody("b", [1, 0, 0], [2, 1, 1])] });
  const [la, lb] = m.links.map((l) => l.id);
  it("is the middle of the boxes' overlap", () => {
    expect(contactCenter(m, la, lb)).toEqual([1, 0.5, 0.5]);
  });
  it("a pair without a suggestion gets the contact centre as origin", () => {
    const d = draftForPair(m, { fixed: la, moving: lb, suggestion: null });
    expect(d).toMatchObject({ fixed: la, moving: lb, origin: [1, 0.5, 0.5], motion: null, source: "manual" });
  });
});

describe("max effort", () => {
  const j = { id: "j", parent_link_id: "a", child_link_id: "b", type: "revolute" as const, origin_xyz: [0, 0, 0] as [number, number, number],
    origin_rpy: [0, 0, 0] as [number, number, number], axis: [0, 0, 1] as [number, number, number], limit_lower: -1, limit_upper: 1,
    source: "manual" as const };
  it("carries max_effort through the draft; missing means auto", () => {
    expect(draftFromJoint(j).maxEffort).toBeNull();
    expect(draftToJoint({ ...draftFromJoint(j), maxEffort: 12 }).max_effort).toBe(12);
    expect(draftToJoint(draftFromJoint(j)).max_effort).toBeNull();
  });
  it("parseEffort keeps only finite positive numbers", () => {
    expect(parseEffort("140")).toBe(140);
    for (const bad of ["", "abc", "-5", "0", "NaN", "Infinity"]) expect(parseEffort(bad)).toBeNull();
  });
});
