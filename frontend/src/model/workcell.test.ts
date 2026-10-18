import { describe, expect, it } from "vitest";
import type { WorkcellObject } from "../api/program";
import { newObject, outOfReach, settle } from "./workcell";

const table: WorkcellObject = { name: "table", shape: "box", size: [0.6, 0.4, 0.3], xyz: [0.5, 0, 0.15], rpy: [0, 0, 0], fixed: true, mass: null, friction: null };

describe("workcell objects", () => {
  it("new objects get unique names and sit inside the reach, on the floor", () => {
    const a = newObject("box", [], 1.0);
    expect(a.name).toBe("box_1");
    expect(a.xyz[2]).toBeCloseTo(a.size[2] / 2, 9);
    expect(Math.hypot(a.xyz[0], a.xyz[1])).toBeLessThan(1.0);
    expect(newObject("box", [a], 1.0).name).toBe("box_2");
    const t = newObject("table", [], 1.0);
    expect(t.fixed).toBe(true);
    expect(newObject("cylinder", [], 1.0).shape).toBe("cylinder");
  });

  it("a dragged object settles onto what is under it: the floor, or a table's top", () => {
    const box: WorkcellObject = { ...newObject("box", [], 1.0), xyz: [2, 2, 0.9] };
    expect(settle(box, [table]).xyz[2]).toBeCloseTo(box.size[2] / 2, 9);           // nothing under it: floor
    const onTable = settle({ ...box, xyz: [0.55, 0.05, 0.9] }, [table, box]);
    expect(onTable.xyz[2]).toBeCloseTo(0.3 + box.size[2] / 2, 9);                    // on the table top
    expect(onTable.xyz.slice(0, 2)).toEqual([0.55, 0.05]);
  });

  it("flags objects the arm can't reach", () => {
    expect(outOfReach({ ...table, xyz: [0.5, 0, 0.15] }, 0.9)).toBe(false);
    expect(outOfReach({ ...table, xyz: [1.5, 0, 0.15] }, 0.9)).toBe(true);
    expect(outOfReach(table, null)).toBe(false);   // reach unknown until ROS has set the robot up once
  });
});
