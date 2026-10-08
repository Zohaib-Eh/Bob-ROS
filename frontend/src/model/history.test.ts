import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { HISTORY_CAP, historyReducer, initialHistory } from "./history";
import { loadProject, saveProject } from "./project";
import { boxBody } from "./testBodies";

const imported = () => historyReducer(initialHistory(), {
  type: "import", warnings: [],
  bodies: [boxBody("a", [0, 0, 0], [1, 1, 1]), boxBody("b", [1, 0, 0], [2, 1, 1])] });

describe("history", () => {
  it("undo and redo walk through edits", () => {
    let h = imported();
    const [a, b] = h.present.links.map((l) => l.id);
    h = historyReducer(h, { type: "merge", linkIds: [a, b] });
    expect(h.present.links).toHaveLength(1);
    h = historyReducer(h, { type: "undo" });
    expect(h.present.links).toHaveLength(2);
    h = historyReducer(h, { type: "redo" });
    expect(h.present.links).toHaveLength(1);
  });

  it("undo restores joints dropped by a merge", () => {
    let h = imported();
    const [a, b] = h.present.links.map((l) => l.id);
    const j: Joint = { id: "j", parent_link_id: a, child_link_id: b, type: "fixed", origin_xyz: [0, 0, 0],
      origin_rpy: [0, 0, 0], axis: null, limit_lower: null, limit_upper: null, source: "manual" };
    h = historyReducer(h, { type: "addJoint", joint: j });
    h = historyReducer(h, { type: "merge", linkIds: [a, b] });
    expect(h.present.joints).toHaveLength(0);
    h = historyReducer(h, { type: "undo" });
    expect(h.present.joints.map((x) => x.id)).toEqual(["j"]);
  });

  it("detection results are not undo steps; a new edit clears redo", () => {
    let h = imported();
    const pastLen = h.past.length;
    h = historyReducer(h, { type: "setSuggestion", suggestion: { key: "k", linkA: "x", linkB: "y",
      membership: "m", status: "review", result: { joint: null, reason: "no_contact_faces", evidence: null } } });
    expect(h.past.length).toBe(pastLen);
    h = historyReducer(h, { type: "undo" });
    h = historyReducer(h, { type: "redo" });
    expect(h.present.suggestions.k).toBeDefined();
    expect(h.future.length).toBe(0);
    h = historyReducer(h, { type: "undo" });
    expect(h.future.length).toBe(1);
    h = historyReducer(h, { type: "import", warnings: [], bodies: [boxBody("c", [5, 5, 5], [6, 6, 6])] });
    expect(h.future).toEqual([]);
  });

  it("caps history and ignores no-op actions", () => {
    let h = imported();
    const id = h.present.links[0].id;
    for (let i = 0; i < HISTORY_CAP + 20; i++) {
      h = historyReducer(h, { type: "renameLink", linkId: id, name: `n${i}` });
    }
    expect(h.past.length).toBe(HISTORY_CAP);
    const same = historyReducer(h, { type: "renameLink", linkId: "missing", name: "x" });
    expect(same).toBe(h);
  });
});

describe("project files", () => {
  it("round-trips the model", () => {
    const m = imported().present;
    expect(loadProject(saveProject(m))).toEqual(m);
  });
  it("rejects non-JSON and foreign files", () => {
    expect(() => loadProject("not json")).toThrow(/not a project file/i);
    expect(() => loadProject(JSON.stringify({ format: "other", version: 1 }))).toThrow(/not a project file/i);
    expect(() => loadProject(JSON.stringify({ format: "cad-to-urdf-project", version: 9, model: {} })))
      .toThrow(/version 9/);
  });
  it("load clears history", () => {
    let h = imported();
    h = historyReducer(h, { type: "renameLink", linkId: h.present.links[0].id, name: "x" });
    h = historyReducer(h, { type: "load", model: loadProject(saveProject(h.present)) });
    expect(h.past).toEqual([]);
    expect(h.future).toEqual([]);
  });
});
