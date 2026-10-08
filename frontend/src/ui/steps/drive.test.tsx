import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import type { Joint, SweepResult } from "../../api/types";
import { historyReducer } from "../../model/history";
import { membership, pairKey } from "../../model/pairs";
import { boxBody } from "../../model/testBodies";
import type { Viewer } from "../../viewer/Viewer";
import JointTool from "../JointTool";
import { Harness, imported, state } from "../testing";
import ConnectStep from "./ConnectStep";

afterEach(() => vi.restoreAllMocks());

function start() {
  const h = imported([boxBody("block", [0, 0, 0], [1, 1, 1]), boxBody("peg", [1, 0, 0], [2, 1, 1])]);
  const [la, lb] = h.present.links;
  const joint: Joint = { id: "s1", parent_link_id: la.id, child_link_id: lb.id, type: "cylindrical", origin_xyz: [1, 0.5, 0.5],
    origin_rpy: [0, 0, 0], axis: [1, 0, 0], limit_lower: null, limit_upper: null, source: "suggested" };
  return historyReducer(h, { type: "setSuggestion", suggestion: { key: pairKey(la.id, lb.id), linkA: la.id, linkB: lb.id,
    membership: membership(la, lb), status: "review", result: { joint, reason: null, evidence: null } } });
}
const fakeViewer = () => ({ setPreview: vi.fn(), onDrive: vi.fn(), setWarning: vi.fn(), focus: vi.fn(),
  screenPoint: () => ({ x: 10, y: 10 }), onCameraChange: () => () => {} }) as unknown as Viewer;
function open(viewer: Viewer | null = fakeViewer()) {
  const h = start();
  render(<Harness start={h} step="connect" viewer={viewer} ui={(ctx) => (
    <>
      <ConnectStep ctx={ctx} />
      {ctx.jointDraft && <JointTool ctx={ctx} />}
    </>
  )} />);
  fireEvent.click(screen.getByTestId("next-joint"));
  return { viewer, block: h.present.links[0].id };
}
const result = (r: Partial<SweepResult>): SweepResult =>
  ({ lower: -1, upper: 1.5, free: false, blocked_by: { lower: null, upper: null }, at_rest_overlap: [], ...r });

describe("Find limits from geometry", () => {
  it("fills the limits from the sweep and names what blocks each side", async () => {
    const { block } = open();
    const spy = vi.spyOn(client, "sweepLimits").mockResolvedValue(result({ blocked_by: { lower: block, upper: null } }));
    await act(async () => { fireEvent.click(screen.getByTestId("find-limits")); });
    expect(spy).toHaveBeenCalledWith({ type: "revolute", origin_xyz: [1, 0.5, 0.5], axis: [1, 0, 0] },
      [expect.objectContaining({ id: "peg" })], [{ link_id: block, bodies: [expect.objectContaining({ id: "block" })] }]);
    expect(state().jointDraft).toMatchObject({ lower: -1, upper: 1.5 });
    expect(screen.getByTestId("sweep-status").textContent).toBe("Stops at -57° (hits block) and +86° (no contact).");
  });

  it("a free spin offers No limits", async () => {
    open();
    vi.spyOn(client, "sweepLimits").mockResolvedValue(result({ free: true, lower: -Math.PI, upper: Math.PI }));
    await act(async () => { fireEvent.click(screen.getByTestId("find-limits")); });
    fireEvent.click(screen.getByTestId("use-no-limits"));
    expect(state().jointDraft.noLimits).toBe(true);
  });

  it("parts overlapping at rest leave the limits alone", async () => {
    const { block } = open();
    vi.spyOn(client, "sweepLimits").mockResolvedValue(result({ lower: null, upper: null, at_rest_overlap: [block] }));
    await act(async () => { fireEvent.click(screen.getByTestId("find-limits")); });
    expect(screen.getByTestId("sweep-status").textContent).toMatch(/overlap where they sit.*block/);
    expect(state().jointDraft).toMatchObject({ lower: -Math.PI, upper: Math.PI });
  });

  it("a result for an outdated draft is ignored", async () => {
    open();
    let resolve!: (r: SweepResult) => void;
    vi.spyOn(client, "sweepLimits").mockReturnValue(new Promise((r) => { resolve = r; }));
    fireEvent.click(screen.getByTestId("find-limits"));
    expect(screen.getByTestId("find-limits").textContent).toMatch(/Sweeping/);
    fireEvent.click(screen.getByTestId("flip"));
    await act(async () => { resolve(result({ lower: -0.2, upper: 0.2 })); });
    expect(state().jointDraft).toMatchObject({ lower: -Math.PI, upper: Math.PI });
    expect(screen.queryByTestId("sweep-status")).toBeNull();
  });

  it("is not offered for Rigid", () => {
    open();
    fireEvent.click(screen.getByTestId("motion-rigid"));
    expect(screen.queryByTestId("find-limits")).toBeNull();
    expect(screen.queryByTestId("drive-slider")).toBeNull();
  });
});

describe("Drive", () => {
  it("the slider poses the moving part, the label shows it, and the viewer can drive it", () => {
    const { viewer } = open();
    expect(viewer!.onDrive).toHaveBeenLastCalledWith(expect.objectContaining({ bodyIds: ["peg"], value: 0 }));
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    expect(viewer!.setPreview).toHaveBeenLastCalledWith(["peg"], expect.anything());
    expect(screen.getByTestId("drive-label").textContent).toBe("+57°");
    expect((screen.getByTestId("drive-value") as HTMLInputElement).value).toBe("57.3");
  });

  it("past a limit the part turns red and the label names the blocker", async () => {
    const { viewer, block } = open();
    vi.spyOn(client, "sweepLimits").mockResolvedValue(result({ lower: -0.5, upper: 0.5, blocked_by: { lower: null, upper: block } }));
    await act(async () => { fireEvent.click(screen.getByTestId("find-limits")); });
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    expect(screen.getByTestId("drive-label").textContent).toBe("+57° · beyond upper limit (hits block)");
    expect(viewer!.setWarning).toHaveBeenLastCalledWith(["peg"]);
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "0.2" } });
    expect(viewer!.setWarning).toHaveBeenLastCalledWith([]);
  });

  it("Set lower/upper here copy the pose", () => {
    open();
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    fireEvent.click(screen.getByTestId("set-upper-here"));
    expect(state().jointDraft.upper).toBe(1);
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "-0.5" } });
    fireEvent.click(screen.getByTestId("set-lower-here"));
    expect(state().jointDraft.lower).toBe(-0.5);
  });

  it("set-here never makes lower exceed upper", () => {
    open();
    fireEvent.change(screen.getByTestId("limit-upper"), { target: { value: "10" } });
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    expect((screen.getByTestId("set-lower-here") as HTMLButtonElement).disabled).toBe(true);
    expect((screen.getByTestId("set-upper-here") as HTMLButtonElement).disabled).toBe(false);
  });

  it("closing the tool clears the pose, tint and drive", async () => {
    const { viewer } = open();
    fireEvent.change(screen.getByTestId("drive-slider"), { target: { value: "1" } });
    fireEvent.click(screen.getByTestId("cancel"));
    await waitFor(() => expect(viewer!.onDrive).toHaveBeenLastCalledWith(null));
    expect(viewer!.setPreview).toHaveBeenLastCalledWith([], null);
    expect(viewer!.setWarning).toHaveBeenLastCalledWith([]);
  });
});
