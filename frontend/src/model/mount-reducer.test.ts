import { describe, expect, it } from "vitest";
import { isMountedPair, mountMatrix, invert, transformBody } from "./mount";
import { loadProject, saveProject, PROJECT_FORMAT, PROJECT_VERSION } from "./project";
import { modelReducer } from "./reducer";
import { boxBody } from "./testBodies";
import { emptyModel } from "./types";
import { pairsToDetect } from "./detection";
import type { Face, Vec3 } from "../api/types";

const plane = (center: Vec3, normal: Vec3): Face => ({
  kind: "plane", origin: "brep", normal, axis: null, center, radius: null, residual: 0, triangle_indices: [],
});
const robot = () => modelReducer(emptyModel(), { type: "import", source: "robot.step", warnings: [],
  bodies: [boxBody("flange", [5, 0, 0], [6, 1, 1])] });
const withGripper = () => modelReducer(robot(), { type: "import", source: "gripper.step", warnings: [],
  bodies: [boxBody("plate", [0, 0, 0], [1, 1, 1], { faces: [plane([0.5, 0.5, 1], [0, 0, 1])] })] });

describe("sources and transformSource", () => {
  it("import_stamps_a_unique_source_per_file", () => {   // Review Focus 1
    let m = withGripper();
    m = modelReducer(m, { type: "import", source: "gripper.step", warnings: [], bodies: [boxBody("plate2", [0, 0, 0], [1, 1, 1])] });
    expect(m.bodies.flange.source).toBe("robot.step");
    expect(m.bodies.plate.source).toBe("gripper.step");
    expect(m.bodies.plate2.source).toBe("gripper.step (2)");
  });

  it("transformSource_moves_only_that_source", () => {
    const M = mountMatrix(plane([0.5, 0.5, 1], [0, 0, 1]), plane([6, 0.5, 0.5], [1, 0, 0]), 0);
    const m = modelReducer(withGripper(), { type: "transformSource", source: "gripper.step", matrix: M });
    expect(m.bodies.flange.vertices).toEqual(robot().bodies.flange.vertices);
    expect(m.bodies.plate.vertices).toEqual(transformBody(withGripper().bodies.plate, M).vertices);
  });

  it("undo_after_rename_restores_geometry", () => {   // Review Focus 3
    const before = withGripper();
    const M = mountMatrix(plane([0.5, 0.5, 1], [0, 0, 1]), plane([6, 0.5, 0.5], [1, 0, 0]), 30);
    let m = modelReducer(before, { type: "transformSource", source: "gripper.step", matrix: M });
    m = modelReducer(m, { type: "renameLink", linkId: m.links[0].id, name: "flange_link" });
    m = modelReducer(m, { type: "transformSource", source: "gripper.step", matrix: invert(M) });
    m.bodies.plate.vertices.forEach((v, i) => v.forEach((x, k) =>
      expect(x).toBeCloseTo(before.bodies.plate.vertices[i][k], 9)));
    expect(m.links[0].name).toBe("flange_link");
  });

  it("transformSource_drops_suggestions_of_moved_links_so_they_are_detected_again", () => {
    let m = modelReducer(robot(), { type: "import", source: "gripper.step", warnings: [],
      bodies: [boxBody("plate", [0, 0, 0], [1, 1, 1]), boxBody("finger", [1, 0, 0], [2, 1, 1])] });
    const id = (b: string) => m.links.find((l) => l.body_ids.includes(b))!.id;
    const [pending] = pairsToDetect(m);
    m = modelReducer(m, { type: "setSuggestion", suggestion: { ...pending, status: "review",
      result: { joint: null, reason: "no_contact_faces", evidence: null } } });
    expect(pairsToDetect(m)).toHaveLength(0);
    const M = mountMatrix(plane([0.5, 0.5, 1], [0, 0, 1]), plane([6, 0.5, 0.5], [1, 0, 0]), 0);
    m = modelReducer(m, { type: "transformSource", source: m.bodies.finger.source!, matrix: M });
    expect(m.suggestions[pending.key]).toBeUndefined();
    expect(pairsToDetect(m).map((p) => p.key)).toContain(pending.key);
    expect(id("finger")).toBeTruthy();
  });

  it("a mounted part's joint to the robot is suggested fixed, not planar", () => {   // found driving the app
    const M = mountMatrix(plane([0.5, 0.5, 1], [0, 0, 1]), plane([6, 0.5, 0.5], [1, 0, 0]), 0);
    let m = modelReducer(withGripper(), { type: "transformSource", source: "gripper.step", matrix: M,
      mount: { M, from: plane([0.5, 0.5, 1], [0, 0, 1]), to: plane([6, 0.5, 0.5], [1, 0, 0]), turn: 0, fromBody: "plate", toBody: "flange" } });
    const id = (b: string) => m.links.find((l) => l.body_ids.includes(b))!.id;
    const [p] = pairsToDetect(m);
    const planar = { id: "s", parent_link_id: id("flange"), child_link_id: id("plate"), type: "planar" as const,
      origin_xyz: [6, 0.5, 0.5] as [number, number, number], origin_rpy: [0, 0, 0] as [number, number, number],
      axis: [1, 0, 0] as [number, number, number], limit_lower: null, limit_upper: null, source: "suggested" as const };
    m = modelReducer(m, { type: "setSuggestion", suggestion: { ...p, status: "review", result: { joint: planar, reason: null, evidence: null } } });
    const j = m.suggestions[p.key].result.joint!;
    expect(j.type).toBe("fixed");
    expect(j.origin_xyz).toEqual([6, 0.5, 0.5]);
    expect(isMountedPair(m, id("flange"), id("plate"))).toBe(true);
  });

  it("old_project_without_source_loads_unchanged", () => {   // Review Focus 2
    const old = { ...robot(), bodies: { flange: { ...boxBody("flange", [5, 0, 0], [6, 1, 1]) } } };
    delete (old as Record<string, unknown>).gripper;
    const text = JSON.stringify({ format: PROJECT_FORMAT, version: PROJECT_VERSION, model: old });
    const m = loadProject(text);
    expect(m.bodies.flange.source).toBeUndefined();
    expect(loadProject(saveProject(m)).bodies.flange.vertices).toEqual(old.bodies.flange.vertices);
  });
});
