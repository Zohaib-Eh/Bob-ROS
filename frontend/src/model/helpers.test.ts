import { describe, expect, it } from "vitest";
import type { Joint, Link } from "../api/types";
import { splitExportError } from "./errors";
import { membership, pairKey, touchingPairs } from "./pairs";
import { smallLinks } from "./smallParts";
import { boxBody } from "./testBodies";
import { directJoints } from "./tree";

const L = (id: string, bodyIds: string[]): Link => ({
  id, name: id, body_ids: bodyIds, density_kg_m3: 1000, mass_override_kg: null, accept_approximate_mass: false });
const J = (id: string, a: string, b: string): Joint => ({
  id, parent_link_id: a, child_link_id: b, type: "revolute", origin_xyz: [0, 0, 0],
  origin_rpy: [0, 0, 0], axis: [0, 0, 1], limit_lower: -1, limit_upper: 1, source: "confirmed" });

const bodies = {
  base: boxBody("base", [0, 0, 0], [1, 1, 1]),
  arm: boxBody("arm", [1, 0, 0], [2, 1, 1]),          // touches base at x = 1
  far: boxBody("far", [5, 5, 5], [6, 6, 6]),
  screw: boxBody("screw", [0.5, 0.5, 1], [0.6, 0.6, 1.1]), // sits on base only
};

describe("pairs", () => {
  it("finds link pairs whose part boxes touch within tolerance", () => {
    const links = [L("a", ["base"]), L("b", ["arm"]), L("c", ["far"])];
    expect(touchingPairs(links, bodies).map(([x, y]) => [x.id, y.id])).toEqual([["a", "b"]]);
  });
  it("pair keys and membership are order-independent", () => {
    const a = L("a", ["base", "screw"]), b = L("b", ["arm"]);
    expect(pairKey("b", "a")).toBe(pairKey("a", "b"));
    expect(membership(b, a)).toBe(membership(a, b));
    expect(membership(a, b)).not.toBe(membership(L("a", ["base"]), b));
  });
});

describe("smallLinks", () => {
  it("lists tiny links touching exactly one neighbour", () => {
    const links = [L("a", ["base"]), L("b", ["arm"]), L("s", ["screw"])];
    expect(smallLinks(links, bodies)).toEqual([{ linkId: "s", neighbourId: "a" }]);
  });
});

describe("directJoints", () => {
  it("orients joints away from the base", () => {
    const links = [L("a", []), L("b", []), L("c", [])];
    const out = directJoints(links, [J("j1", "b", "a"), J("j2", "c", "b")], "a");
    expect(out.errors).toEqual([]);
    expect(out.joints.map((j) => [j.id, j.parent_link_id, j.child_link_id]))
      .toEqual([["j1", "a", "b"], ["j2", "b", "c"]]);
  });
  it("flags links not connected to the base", () => {
    const out = directJoints([L("a", []), L("b", []), L("c", [])], [J("j1", "a", "b")], "a");
    expect(out.errors).toEqual([{ linkIds: ["c"], message: "Not connected to the base link by any joint." }]);
  });
  it("flags cycles and duplicate pairs", () => {
    const links = [L("a", []), L("b", []), L("c", [])];
    const cyc = directJoints(links, [J("j1", "a", "b"), J("j2", "b", "c"), J("j3", "c", "a")], "a");
    expect(cyc.errors.some((e) => (e.jointId === "j2" || e.jointId === "j3") && /loop/.test(e.message))).toBe(true);
    const dup = directJoints(links.slice(0, 2), [J("j1", "a", "b"), J("j2", "b", "a")], "a");
    expect(dup.errors.some((e) => e.jointId === "j2" && /more than one joint/.test(e.message))).toBe(true);
  });
  it("reports a missing base", () => {
    expect(directJoints([L("a", [])], [], null).errors[0].message).toMatch(/base/);
  });
});

describe("splitExportError", () => {
  it("pins pieces to links and joints", () => {
    const msg = "Cannot export URDF -- structural validation failed: Link 'a' contains approximate part 'p'; set mass_override_kg or accept_approximate_mass.; Joint j1 (revolute) requires an axis.; Something general";
    const out = splitExportError(msg);
    expect(Object.keys(out.byLink)).toEqual(["a"]);
    expect(Object.keys(out.byJoint)).toEqual(["j1"]);
    expect(out.general).toEqual(["Something general"]);
  });
});
