import { describe, expect, it } from "vitest";
import type { SweepResult } from "../api/types";
import { historyReducer } from "../model/history";
import { emptyDraft, setMotion, type JointDraft } from "../model/jointDraft";
import { boxBody } from "../model/testBodies";
import { unwrapStep } from "../viewer/overlays";
import { beyondSide, driveRange, fmtJointValue, sweepRequest, sweepSummary } from "./drive";
import { imported } from "./testing";

const model = () => {
  const h = imported([boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1]), boxBody("tip", [2, 0, 0], [3, 1, 1])]);
  const [base, arm, tip] = h.present.links.map((l) => l.id);
  const m = historyReducer(h, { type: "addJoint", joint: { id: "j2", parent_link_id: arm, child_link_id: tip, type: "fixed",
    origin_xyz: [2, 0.5, 0.5], origin_rpy: [0, 0, 0], axis: null, limit_lower: null, limit_upper: null, source: "manual" } }).present;
  return { m, base, arm, tip };
};
const draft = (fixed: string, moving: string, extra: Partial<JointDraft> = {}): JointDraft =>
  ({ ...setMotion({ ...emptyDraft(fixed, moving), origin: [1, 0.5, 0.5], axis: [0, 0, 1] }, "revolute"), ...extra });

describe("sweepRequest", () => {
  it("moves the child and everything below it; every other link is an obstacle", () => {
    const { m, base, arm } = model();
    const r = sweepRequest(m, draft(base, arm))!;
    expect(r.joint).toEqual({ type: "revolute", origin_xyz: [1, 0.5, 0.5], axis: [0, 0, 1] });
    expect(r.moving.map((b) => b.id).sort()).toEqual(["arm", "tip"]);
    expect(r.obstacles.map((o) => o.link_id)).toEqual([base]);
  });
  it("only links already joined to the fixed side are obstacles; loose links ride along later", () => {
    const h = imported([boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1]),
      boxBody("tip", [2, 0, 0], [3, 1, 1]), boxBody("stand", [0, -1, 0], [1, 0, 1])]);
    const [base, arm, , stand] = h.present.links.map((l) => l.id);
    const m = historyReducer(h, { type: "addJoint", joint: { id: "js", parent_link_id: base, child_link_id: stand, type: "fixed",
      origin_xyz: [0.5, 0, 0.5], origin_rpy: [0, 0, 0], axis: null, limit_lower: null, limit_upper: null, source: "manual" } }).present;
    const r = sweepRequest(m, draft(base, arm))!;
    expect(r.obstacles.map((o) => o.link_id).sort()).toEqual([base, stand].sort());   // tip isn't joined yet: not an obstacle
  });

  it("sweeps a no-limits revolute as revolute, a slider as prismatic, and nothing for rigid or no axis", () => {
    const { m, base, arm } = model();
    expect(sweepRequest(m, draft(base, arm, { noLimits: true }))!.joint.type).toBe("revolute");
    expect(sweepRequest(m, setMotion(draft(base, arm), "slider"))!.joint.type).toBe("prismatic");
    expect(sweepRequest(m, setMotion(draft(base, arm), "rigid"))).toBeNull();
    expect(sweepRequest(m, draft(base, arm, { axis: null }))).toBeNull();
  });
});

describe("drive helpers", () => {
  it("drive range: a full turn, widened by limits; a slider spans the moving link's size", () => {
    const { m, base, arm } = model();
    expect(driveRange(m, draft(base, arm))).toEqual([-Math.PI, Math.PI]);
    expect(driveRange(m, draft(base, arm, { lower: -1, upper: 5 }))).toEqual([-Math.PI, 5]);
    const [lo, hi] = driveRange(m, setMotion(draft(base, arm), "slider"));
    expect(hi).toBeCloseTo(Math.sqrt(3));
    expect(lo).toBeCloseTo(-Math.sqrt(3));
  });
  it("beyondSide compares the pose with the draft's limits; never for no limits or rigid", () => {
    const { base, arm } = model();
    const d = draft(base, arm, { lower: -1, upper: 1 });
    expect(beyondSide(d, 0.5)).toBeNull();
    expect(beyondSide(d, 1.2)).toBe("upper");
    expect(beyondSide(d, -1.2)).toBe("lower");
    expect(beyondSide({ ...d, noLimits: true }, 3)).toBeNull();
    expect(beyondSide(setMotion(d, "rigid"), 3)).toBeNull();
  });
  it("formats poses and sweep results", () => {
    expect(fmtJointValue(1, false)).toBe("+57°");
    expect(fmtJointValue(-0.0183, true)).toBe("-18 mm");
    const name = (id: string) => ({ L1: "upper_arm", L2: "base" } as Record<string, string>)[id] ?? id;
    const r: SweepResult = { lower: -1.62, upper: 2.25, free: false, blocked_by: { lower: "L1", upper: null }, at_rest_overlap: [] };
    expect(sweepSummary(r, false, name)).toBe("Stops at -93° (hits upper_arm) and +129° (no contact).");
    expect(sweepSummary({ ...r, free: true }, false, name)).toMatch(/^Spins freely/);
    expect(sweepSummary({ ...r, at_rest_overlap: ["L1", "L2"] }, false, name)).toMatch(/overlap where they sit.*upper_arm, base/);
  });
  it("unwrapStep takes the short way round", () => {
    expect(unwrapStep(3.1, -3.1)).toBeCloseTo(2 * Math.PI - 6.2);
    expect(unwrapStep(0.1, 0.3)).toBeCloseTo(0.2);
  });
});
