import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { buildExportRequest } from "../model/exportRequest";
import { historyReducer, initialHistory, type History } from "../model/history";
import { boxBody, wheelBody } from "../model/testBodies";
import type { Ctx } from "./ctx";
import MobileCard from "./MobileCard";
import { Harness } from "./testing";

const wheel = (id: string, child: string, xyz: [number, number, number], axis: [number, number, number]): Joint => ({
  id, parent_link_id: "L_chassis", child_link_id: child, type: "revolute", origin_xyz: xyz, origin_rpy: [0, 0, 0],
  axis, limit_lower: -3, limit_upper: 3, source: "suggested" });

/** A chassis with a caster bump and two wheels; alongX: its axles lie along x (CAD drawn sideways). */
function start(alongX = false): History {
  const swap = (b: ReturnType<typeof wheelBody>) => (alongX ? { ...b, vertices: b.vertices.map(([x, y, z]) => [y, x, z] as [number, number, number]) } : b);
  let h = historyReducer(initialHistory(), { type: "import", source: "base.step", warnings: [], bodies: [
    boxBody("chassis", [-0.25, -0.2, 0.0], [0.25, 0.2, 0.18]),
    swap(wheelBody("wl", [0.1, 0.25, 0.08])), swap(wheelBody("wr", [0.1, -0.25, 0.08])),
  ] });
  const present = { ...h.present, rootLinkId: "L_chassis",
    links: h.present.links.map((l) => ({ ...l, id: `L_${l.body_ids[0]}`, name: l.body_ids[0] })) };
  const at = (x: number, y: number): [number, number, number] => (alongX ? [y, x, 0.08] : [x, y, 0.08]);
  const ax: [number, number, number] = alongX ? [1, 0, 0] : [0, 1, 0];
  h = historyReducer({ ...h, present }, { type: "addJoint", joint: wheel("jl", "L_wl", at(0.1, 0.25), ax) });
  return historyReducer(h, { type: "addJoint", joint: wheel("jr", "L_wr", at(0.1, -0.25), ax) });
}
let ctx: Ctx;
const ui = (h = start()) => render(<Harness start={h} step="connect" ui={(c) => { ctx = c; return <MobileCard ctx={c} />; }} />);

describe("MobileCard", () => {
  it("ticking the wheels makes a drivable base: sides, continuous joints, a summary, and the export carries it", () => {
    ui();
    fireEvent.click(screen.getByRole("checkbox", { name: /wl/ }));
    fireEvent.click(screen.getByRole("checkbox", { name: /wr/ }));
    expect(ctx.model.mobile).toEqual({ left: ["jl"], right: ["jr"] });
    expect(ctx.model.joints.every((j) => j.type === "continuous")).toBe(true);
    expect(screen.getByTestId("mobile-summary").textContent).toBe("Differential drive · 160 mm wheels, 0.50 m apart");
    expect(screen.queryByRole("alert")).toBeNull();
    expect(buildExportRequest(ctx.model).mobile).toEqual({ left: ["jl"], right: ["jr"] });
    fireEvent.click(screen.getByRole("checkbox", { name: /wr/ }));
    expect(screen.getByRole("alert").textContent).toMatch(/both sides/);
  });

  it("offers to turn a robot drawn sideways, and then its wheels roll along x", () => {
    ui(start(true));
    fireEvent.click(screen.getByRole("button", { name: /Turn the robot/ }));
    expect(screen.queryByRole("button", { name: /Turn the robot/ })).toBeNull();
    const axis = ctx.model.joints.find((j) => j.id === "jl")!.axis!;
    expect(Math.abs(axis[1])).toBeCloseTo(1);
    fireEvent.click(screen.getByRole("checkbox", { name: /wl/ }));
    fireEvent.click(screen.getByRole("checkbox", { name: /wr/ }));
    expect(ctx.model.mobile!.left).toHaveLength(1);
    expect(screen.queryByRole("alert")).toBeNull();
  });

  it("stays out of the way on a robot without wheels", () => {
    const h = historyReducer(initialHistory(), { type: "import", source: "arm.step", warnings: [], bodies: [boxBody("a", [0, 0, 0], [1, 1, 1])] });
    render(<Harness start={h} step="connect" ui={(c) => <MobileCard ctx={c} />} />);
    expect(screen.queryByTestId("mobile")).toBeNull();
  });
});
