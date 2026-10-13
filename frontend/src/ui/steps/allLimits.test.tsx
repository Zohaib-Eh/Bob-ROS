import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import type { Joint, SweepResult } from "../../api/types";
import { historyReducer, type History } from "../../model/history";
import { boxBody } from "../../model/testBodies";
import AllLimits from "../AllLimits";
import { Harness, imported, state } from "../testing";

afterEach(() => vi.restoreAllMocks());

const joint = (id: string, parent: string, child: string, over: Partial<Joint> = {}): Joint => ({
  id, parent_link_id: parent, child_link_id: child, type: "revolute", origin_xyz: [1, 0.5, 0.5], origin_rpy: [0, 0, 0],
  axis: [0, 0, 1], limit_lower: -Math.PI, limit_upper: Math.PI, source: "confirmed", ...over });
const result = (r: Partial<SweepResult>): SweepResult =>
  ({ lower: -1, upper: 1.5, free: false, blocked_by: { lower: null, upper: null }, at_rest_overlap: [], ...r });

/** base → arm (default limits) → hand (default limits) → tip (custom limits) */
function robot(): History {
  let h = imported([boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1]),
    boxBody("hand", [2, 0, 0], [3, 1, 1]), boxBody("tip", [3, 0, 0], [4, 1, 1])]);
  const [b, a, hd, t] = h.present.links.map((l) => l.id);
  h = historyReducer(h, { type: "addJoint", joint: joint("j1", b, a) });
  h = historyReducer(h, { type: "addJoint", joint: joint("j2", a, hd, { origin_xyz: [2, 0.5, 0.5] }) });
  return historyReducer(h, { type: "addJoint", joint: joint("j3", hd, t, { origin_xyz: [3, 0.5, 0.5], limit_lower: -1, limit_upper: 1 }) });
}
const ui = (h: History) => render(<Harness start={h} step="connect" ui={(ctx) => <AllLimits ctx={ctx} />} />);
const joints = () => state().joints as Joint[];

describe("Find limits for all joints", () => {
  it("sweeps only the joints at the default limits and applies what it found in one undoable step", async () => {
    const h = robot();
    const spy = vi.spyOn(client, "sweepLimits")
      .mockResolvedValueOnce(result({ lower: -1, upper: 1.5 }))
      .mockResolvedValueOnce(result({ free: true, lower: -Math.PI, upper: Math.PI }));
    ui(h);
    expect(screen.getByTestId("find-all-limits").textContent).toMatch(/2 joints/);
    const before = state().past;
    await act(async () => { fireEvent.click(screen.getByTestId("find-all-limits")); });
    expect(spy).toHaveBeenCalledTimes(2);
    expect(joints().map((j) => [j.id, j.limit_lower, j.limit_upper])).toEqual(
      [["j1", -1, 1.5], ["j2", -Math.PI, Math.PI], ["j3", -1, 1]]);
    expect(state().past).toBe(before + 1);
    expect(screen.getByTestId("limit-row-j1").textContent).toMatch(/-57° … \+86°/);
    expect(screen.getByTestId("limit-row-j2").textContent).toMatch(/turns freely/i);
  });

  it("a joint that turns freely can become No limits", async () => {
    vi.spyOn(client, "sweepLimits").mockResolvedValue(result({ free: true, lower: -Math.PI, upper: Math.PI }));
    ui(robot());
    await act(async () => { fireEvent.click(screen.getByTestId("find-all-limits")); });
    fireEvent.click(screen.getByTestId("no-limits-j2"));
    expect(joints().find((j) => j.id === "j2")).toMatchObject({ type: "continuous", limit_lower: null, limit_upper: null });
  });

  it("overlapping parts and errors leave the limits alone and say why", async () => {
    vi.spyOn(client, "sweepLimits")
      .mockResolvedValueOnce(result({ lower: null, upper: null, at_rest_overlap: ["x"] }))
      .mockRejectedValueOnce(new Error("Cannot reach the backend"));
    ui(robot());
    await act(async () => { fireEvent.click(screen.getByTestId("find-all-limits")); });
    expect(joints().map((j) => j.limit_lower)).toEqual([-Math.PI, -Math.PI, -1]);
    expect(screen.getByTestId("limit-row-j1").textContent).toMatch(/overlap/i);
    expect(screen.getByTestId("limit-row-j2").textContent).toMatch(/Cannot reach the backend/);
  });

  it("is hidden when no joint uses the defaults", () => {
    let h = robot();
    h = historyReducer(h, { type: "setLimits", changes: [
      { jointId: "j1", lower: -1, upper: 1, continuous: false }, { jointId: "j2", lower: -1, upper: 1, continuous: false }] });
    ui(h);
    expect(screen.queryByTestId("find-all-limits")).toBeNull();
  });
});
