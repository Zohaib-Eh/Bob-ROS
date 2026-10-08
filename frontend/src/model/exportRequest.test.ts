import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { buildExportRequest } from "./exportRequest";
import { modelReducer } from "./reducer";
import { boxBody } from "./testBodies";
import { emptyModel } from "./types";

const model = () => modelReducer(emptyModel(), { type: "import", warnings: [], bodies: [
  boxBody("base", [0, 0, 0], [2, 2, 2]), boxBody("arm", [2, 0, 0], [3, 1, 1]), boxBody("tool", [3, 0, 0], [3.5, 1, 1])] });
const J = (id: string, a: string, b: string): Joint => ({ id, parent_link_id: a, child_link_id: b, type: "fixed",
  origin_xyz: [0, 0, 0], origin_rpy: [0, 0, 0], axis: null, limit_lower: null, limit_upper: null, source: "manual" });
const idOf = (m: ReturnType<typeof model>, body: string) => m.links.find((l) => l.body_ids.includes(body))!.id;

describe("buildExportRequest", () => {
  it("directs joints from the base and groups bodies by link", () => {
    let m = model();
    m = modelReducer(m, { type: "addJoint", joint: J("j1", idOf(m, "arm"), idOf(m, "base")) });
    m = modelReducer(m, { type: "addJoint", joint: J("j2", idOf(m, "tool"), idOf(m, "arm")) });
    const r = buildExportRequest(m);
    expect(r.errors).toEqual([]);
    expect(r.joints.map((j) => j.parent_link_id)).toEqual([idOf(m, "base"), idOf(m, "arm")]);
    expect(r.bodiesByLink[idOf(m, "arm")].map((b) => b.id)).toEqual(["arm"]);
  });

  it("returns errors for a disconnected tree", () => {
    let m = model();
    m = modelReducer(m, { type: "addJoint", joint: J("j1", idOf(m, "arm"), idOf(m, "base")) });
    expect(buildExportRequest(m).errors[0].linkIds).toEqual([idOf(m, "tool")]);
  });
});
