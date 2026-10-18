import { describe, expect, it } from "vitest";
import type { WorkcellObject } from "../api/program";
import { newObject, outOfReach, reachAt, settle, type Reach } from "./workcell";

// an arm on a pedestal: 0.9 m out at table height, 1.4 m higher up
const reach: Reach = [[0.05, 0.6], [0.15, 0.8], [0.25, 0.9], [0.35, 0.9], [0.95, 1.4]];
const table: WorkcellObject = { name: "table", shape: "box", size: [0.6, 0.4, 0.3], xyz: [0.5, 0, 0.15], rpy: [0, 0, 0], fixed: true, mass: null, friction: null };

describe("workcell objects", () => {
  it("new objects get unique names and sit inside the reach, on the floor", () => {
    const a = newObject("box", [], reach);
    expect(a.name).toBe("box_1");
    expect(a.xyz[2]).toBeCloseTo(a.size[2] / 2, 9);
    expect(outOfReach(a, reach)).toBe(false);
    expect(newObject("box", [a], reach).name).toBe("box_2");
    const t = newObject("table", [], reach);
    expect(t.fixed).toBe(true);
    expect(outOfReach(t, reach)).toBe(false);
    expect(newObject("cylinder", [], null).shape).toBe("cylinder");
  });

  it("a dragged object settles onto what is under it: the floor, or a table's top", () => {
    const box: WorkcellObject = { ...newObject("box", [], reach), xyz: [2, 2, 0.9] };
    expect(settle(box, [table]).xyz[2]).toBeCloseTo(box.size[2] / 2, 9);           // nothing under it: floor
    const onTable = settle({ ...box, xyz: [0.55, 0.05, 0.9] }, [table, box]);
    expect(onTable.xyz[2]).toBeCloseTo(0.3 + box.size[2] / 2, 9);                    // on the table top
    expect(onTable.xyz.slice(0, 2)).toEqual([0.55, 0.05]);
  });

  it("an imported CAD table is used by its bounding box: a box settles on its top", () => {
    const cad: WorkcellObject = { ...table, shape: "mesh", file: "table.stl", pieces: 4 };
    const box: WorkcellObject = { ...newObject("box", [], reach), xyz: [0.5, 0, 0.9] };
    expect(settle(box, [cad]).xyz[2]).toBeCloseTo(0.3 + box.size[2] / 2, 9);
  });

  it("flags objects the arm can't reach at their top, where a gripper comes in", () => {
    expect(reachAt(reach, 0.3)).toBe(0.9);
    expect(reachAt(reach, 2.0)).toBe(0);                                 // above anything the arm gets to
    expect(outOfReach({ ...table, xyz: [0.85, 0, 0.15] }, reach)).toBe(false);   // top 0.3: reaches 0.9
    expect(outOfReach({ ...table, xyz: [1.2, 0, 0.15] }, reach)).toBe(true);     // 1.4 only applies higher up
    expect(outOfReach(table, null)).toBe(false);   // reach unknown until ROS has set the robot up once
  });
});
