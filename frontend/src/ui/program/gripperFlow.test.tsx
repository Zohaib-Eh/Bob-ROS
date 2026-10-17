import { act, render, renderHook, screen, waitFor } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import * as api from "../../api/program";
import type { Joint } from "../../api/types";
import { historyReducer, initialHistory } from "../../model/history";
import { boxBody } from "../../model/testBodies";
import { Harness } from "../testing";
import { useEffortSuggestions } from "../useEffortSuggestions";
import PhysicsCard from "./PhysicsCard";
import { useRosSession } from "./useRosSession";

afterEach(() => vi.restoreAllMocks());

const slider = (id: string, child: string): Joint => ({
  id, parent_link_id: "L_base", child_link_id: child, type: "prismatic", origin_xyz: [0, 0, 0], origin_rpy: [0, 0, 0],
  axis: [1, 0, 0], limit_lower: 0, limit_upper: 0.025, source: "manual", max_effort: null,
});
const G = { leader: "jl", follower: "jr", tcp_offset: 0 };
function start() {
  let h = historyReducer(initialHistory(), { type: "import", source: "g.step", warnings: [], bodies: [
    boxBody("base", [-0.04, -0.02, 0], [0.04, 0.02, 0.01]),
    boxBody("fl", [-0.035, -0.01, -0.06], [-0.025, 0.01, 0]),
    boxBody("fr", [0.025, -0.01, -0.06], [0.035, 0.01, 0]),
  ] });
  h = { ...h, present: { ...h.present, links: h.present.links.map((l) => ({ ...l, id: `L_${l.body_ids[0]}` })), rootLinkId: "L_base" } };
  h = historyReducer(h, { type: "addJoint", joint: slider("jl", "L_fl") });
  h = historyReducer(h, { type: "addJoint", joint: slider("jr", "L_fr") });
  return historyReducer(h, { type: "setGripper", gripper: G });
}

describe("the gripper reaches the backend from Simulate and Program", () => {
  it("the Physics card's check sends the gripper", async () => {
    const spy = vi.spyOn(client, "physicsCheck").mockResolvedValue({ ok: true, kind: "ok", message: "Holds pose." });
    render(<Harness start={start()} step="program" ui={(ctx) => <PhysicsCard ctx={ctx} />} />);
    await screen.findByText("Holds pose.");
    expect(spy.mock.calls[0][4]).toEqual(G);
  });

  it("starting the ROS session sends the gripper", async () => {
    const spy = vi.spyOn(api, "startSession").mockResolvedValue({ state: "starting", robot: "robot" });
    vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "stopped" });
    let ros: ReturnType<typeof useRosSession> | null = null;
    render(<Harness start={start()} step="program" ui={(ctx) => { ros = useRosSession(ctx); return null; }} />);
    await act(async () => { await ros!.start(); });
    expect(spy.mock.calls[0][4]).toEqual(G);
  });

  it("finger joints are suggested the default grip force, not an arm torque", async () => {
    vi.spyOn(client, "suggestEfforts").mockResolvedValue({ jl: 0.3, jr: 0.3 });
    const model = start().present;
    const { result } = renderHook(() => useEffortSuggestions({ model, robotName: "robot" }));
    await waitFor(() => expect(result.current).toEqual({ jl: 40, jr: 40 }), { timeout: 2000 });
  });
});
