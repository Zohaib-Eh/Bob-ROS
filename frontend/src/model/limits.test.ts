import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { exportChecklist } from "./checklist";
import { isDefaultLimits, jointAtDefaultLimits } from "./jointDraft";
import { modelReducer } from "./reducer";
import { boxBody } from "./testBodies";
import { emptyModel, type ModelState } from "./types";

const joint = (id: string, parent: string, child: string, over: Partial<Joint> = {}): Joint => ({
  id, parent_link_id: parent, child_link_id: child, type: "revolute", origin_xyz: [1, 0, 0], origin_rpy: [0, 0, 0],
  axis: [0, 0, 1], limit_lower: -Math.PI, limit_upper: Math.PI, source: "manual", ...over });

function connected(): ModelState {
  let m = modelReducer(emptyModel(), { type: "import", warnings: [],
    bodies: [boxBody("a", [0, 0, 0], [1, 1, 1]), boxBody("b", [1, 0, 0], [2, 1, 1]), boxBody("c", [2, 0, 0], [3, 1, 1])] });
  const [a, b, c] = m.links.map((l) => l.id);
  m = modelReducer(m, { type: "addJoint", joint: joint("j1", a, b) });
  return modelReducer(m, { type: "addJoint", joint: joint("j2", b, c, { type: "prismatic", limit_lower: 0, limit_upper: 0.1 }) });
}

describe("default joint limits", () => {
  it("recognises the untouched defaults only", () => {
    expect(isDefaultLimits("revolute", -Math.PI, Math.PI)).toBe(true);
    expect(isDefaultLimits("slider", 0, 0.1)).toBe(true);
    expect(isDefaultLimits("revolute", -1, Math.PI)).toBe(false);
    expect(isDefaultLimits("rigid", null, null)).toBe(false);
    const m = connected();
    expect(m.joints.map(jointAtDefaultLimits)).toEqual([true, true]);
    expect(jointAtDefaultLimits({ ...m.joints[0], type: "continuous", limit_lower: null, limit_upper: null })).toBe(false);
  });

  it("setLimits changes several joints in one undoable step", () => {
    const m = connected();
    const next = modelReducer(m, { type: "setLimits", changes: [
      { jointId: "j1", lower: -1, upper: 2, continuous: false },
      { jointId: "j2", lower: 0, upper: 0.05, continuous: false }] });
    expect(next.joints.map((j) => [j.limit_lower, j.limit_upper])).toEqual([[-1, 2], [0, 0.05]]);
    const free = modelReducer(m, { type: "setLimits", changes: [{ jointId: "j1", lower: null, upper: null, continuous: true }] });
    expect(free.joints[0]).toMatchObject({ type: "continuous", limit_lower: null, limit_upper: null });
  });

  it("the export checklist warns about defaults without blocking", () => {
    const m = connected();
    const item = exportChecklist(m, "robot").find((i) => i.id === "limits")!;
    expect(item).toMatchObject({ ok: false, warning: true, step: "connect", jointIds: ["j1", "j2"] });
    expect(item.detail).toBe("2 joint(s) still use the default limits; find them from geometry in the Connect step.");
    const set = modelReducer(m, { type: "setLimits", changes: [
      { jointId: "j1", lower: -1, upper: 1, continuous: false }, { jointId: "j2", lower: 0, upper: 0.05, continuous: false }] });
    expect(exportChecklist(set, "robot").find((i) => i.id === "limits")!.ok).toBe(true);
  });
});
