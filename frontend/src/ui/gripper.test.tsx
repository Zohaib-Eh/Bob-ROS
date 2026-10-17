import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { historyReducer, initialHistory, type History } from "../model/history";
import { boxBody } from "../model/testBodies";
import type { Ctx } from "./ctx";
import GripperCard from "./GripperCard";
import { markersFor } from "./markers";
import { Harness } from "./testing";

const slider = (id: string, child: string, axis: [number, number, number], over: Partial<Joint> = {}): Joint => ({
  id, parent_link_id: "L_base", child_link_id: child, type: "prismatic", origin_xyz: [0, 0, 0], origin_rpy: [0, 0, 0],
  axis, limit_lower: 0, limit_upper: 0.025, source: "manual", max_effort: null, ...over,
});
function start(rightAxis: [number, number, number] = [-1, 0, 0]): History {
  let h = historyReducer(initialHistory(), { type: "import", source: "gripper.step", warnings: [], bodies: [
    boxBody("base", [-0.04, -0.02, 0], [0.04, 0.02, 0.01]),
    boxBody("fl", [-0.035, -0.01, -0.06], [-0.025, 0.01, 0]),
    boxBody("fr", [0.025, -0.01, -0.06], [0.035, 0.01, 0]),
  ] });
  const present = { ...h.present, links: h.present.links.map((l) => ({ ...l, id: `L_${l.body_ids[0]}`, name: l.body_ids[0] })) };
  h = historyReducer({ ...h, present }, { type: "addJoint", joint: slider("jl", "L_fl", [1, 0, 0]) });
  return historyReducer(h, { type: "addJoint", joint: slider("jr", "L_fr", rightAxis, { max_effort: 25 }) });
}
let ctx: Ctx;
const ui = (h = start()) => render(<Harness start={h} step="connect" ui={(c) => { ctx = c; return <GripperCard ctx={c} />; }} />);

describe("GripperCard", () => {
  it("ticking the two sliding joints makes them the gripper and describes it", () => {
    ui();
    fireEvent.click(screen.getByRole("checkbox", { name: /fl/ }));
    fireEvent.click(screen.getByRole("checkbox", { name: /fr/ }));
    expect(ctx.model.gripper).toEqual({ leader: "jl", follower: "jr", tcp_offset: 0 });
    expect(screen.getByTestId("gripper-summary").textContent)
      .toBe("Jaws 50 mm apart as modelled · travel 25 mm per finger · grip force 40 N per finger");
  });

  it("the tool point offset field moves the tool point and shows a marker there", () => {
    ui();
    fireEvent.click(screen.getByRole("checkbox", { name: /fl/ }));
    fireEvent.click(screen.getByRole("checkbox", { name: /fr/ }));
    fireEvent.change(screen.getByLabelText("Tool point offset (mm)"), { target: { value: "10" } });
    expect(ctx.model.gripper!.tcp_offset).toBeCloseTo(0.01, 9);
    const tcp = markersFor("connect", ctx.model).find((m) => m.kind === "tcp")!;
    expect(tcp.at[2]).toBeCloseTo(-0.07, 9);
  });

  it("problems with the pair show as warnings", () => {
    ui(start([0, 1, 0]));
    fireEvent.click(screen.getByRole("checkbox", { name: /fl/ }));
    fireEvent.click(screen.getByRole("checkbox", { name: /fr/ }));
    expect(screen.getByRole("alert").textContent).toContain("The fingers don't slide along the same line.");
  });

  it("unticking a finger clears the gripper", () => {
    ui();
    fireEvent.click(screen.getByRole("checkbox", { name: /fl/ }));
    fireEvent.click(screen.getByRole("checkbox", { name: /fr/ }));
    fireEvent.click(screen.getByRole("checkbox", { name: /fr/ }));
    expect(ctx.model.gripper).toBeNull();
  });
});
