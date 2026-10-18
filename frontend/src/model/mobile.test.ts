import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { mobileSummary, mobileWarnings, sides, turnToDrive, wheelCandidates } from "./mobile";
import { modelReducer } from "./reducer";
import { boxBody, wheelBody } from "./testBodies";
import { emptyModel, type ModelState } from "./types";

const link = (id: string) => ({ id, name: id, body_ids: [`${id}_b`], density_kg_m3: 1000, mass_override_kg: null, accept_approximate_mass: false });
const wheel = (id: string, y: number, axis: [number, number, number] = [0, 1, 0]): Joint => ({
  id, parent_link_id: "chassis", child_link_id: id, type: "continuous", origin_xyz: [0.1, y, 0.08], origin_rpy: [0, 0, 0],
  axis, limit_lower: null, limit_upper: null, source: "confirmed" });

/** The backend fixture's base (tests/ros_helpers.mobile_arm): chassis z 0.06..0.18, wheels r 0.08 at y ±0.25. */
function base(caster = true): ModelState {
  const chassis = boxBody("chassis_b", [-0.25, -0.2, caster ? 0 : 0.06], [0.25, 0.2, 0.18]);
  return { ...emptyModel(), rootLinkId: "chassis", links: [link("chassis"), link("wl"), link("wr"), link("lid")],
    bodies: { chassis_b: chassis, wl_b: wheelBody("wl_b", [0.1, 0.25, 0.08]), wr_b: wheelBody("wr_b", [0.1, -0.25, 0.08]),
      lid_b: boxBody("lid_b", [-0.1, -0.1, 0.18], [0.1, 0.1, 0.2]) },
    joints: [wheel("wl", 0.25), wheel("wr", -0.25),
      { ...wheel("lid", 0), child_link_id: "lid", origin_xyz: [0, 0, 0.18], axis: [0, 0, 1] }] };   // a turning lid isn't a wheel
}

describe("mobile base", () => {
  it("suggests round parts on level axles from the base, and sorts them into sides", () => {
    const m = base();
    const ws = wheelCandidates(m);
    expect(ws.map((w) => w.jointId)).toEqual(["wl", "wr"]);
    expect(ws[0].radius).toBeCloseTo(0.08, 3);
    expect(sides(ws)).toEqual({ left: ["wl"], right: ["wr"] });
    expect(mobileSummary(m, { left: ["wl"], right: ["wr"] })).toBe("Differential drive · 160 mm wheels, 0.50 m apart");
    expect(mobileWarnings(m, { left: ["wl"], right: ["wr"] })).toEqual([]);
  });

  it("warns about a base that would tip, and one side with no wheel", () => {
    expect(mobileWarnings(base(false), { left: ["wl"], right: ["wr"] })).toEqual([
      "Two wheels and nothing else touching the floor: the base will tip. Add a caster."]);
    expect(mobileWarnings(base(), { left: ["wl"], right: [] })[0]).toMatch(/both sides/);
  });

  it("turns a robot whose axles lie along x so they lie along y, and keeps the selection while the wheels exist", () => {
    let m = base();
    m = { ...m, joints: m.joints.map((j) => (j.id === "lid" ? j : { ...j, axis: [1, 0, 0] as [number, number, number],
      origin_xyz: [j.origin_xyz[1], 0.1, 0.08] as [number, number, number] })),
      bodies: { ...m.bodies, wl_b: { ...m.bodies.wl_b, vertices: m.bodies.wl_b.vertices.map(([x, y, z]) => [y, x, z] as [number, number, number]) },
        wr_b: { ...m.bodies.wr_b, vertices: m.bodies.wr_b.vertices.map(([x, y, z]) => [y, x, z] as [number, number, number]) } } };
    const yaw = turnToDrive(wheelCandidates(m));
    expect(Math.abs(yaw)).toBeCloseTo(Math.PI / 2);
    m = modelReducer(m, { type: "turnRobot", yaw });
    const ws = wheelCandidates(m);
    expect(turnToDrive(ws)).toBe(0);
    expect(Math.abs(ws[0].axis[1])).toBeCloseTo(1);
    m = modelReducer(m, { type: "setMobile", mobile: sides(ws) });
    expect(m.mobile!.left).toHaveLength(1);
    m = modelReducer(m, { type: "updateJoint", joint: { ...m.joints[0], type: "fixed" } });
    expect(m.mobile).toBeNull();            // a wheel made fixed is no wheel
  });
});
