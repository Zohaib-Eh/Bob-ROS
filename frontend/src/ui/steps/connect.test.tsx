import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import type { DetectResponse, Joint } from "../../api/types";
import { historyReducer, type History } from "../../model/history";
import { draftFromJoint, emptyDraft } from "../../model/jointDraft";
import { membership, pairKey } from "../../model/pairs";
import { boxBody } from "../../model/testBodies";
import type { Viewer } from "../../viewer/Viewer";
import JointTool from "../JointTool";
import { Harness, imported, state } from "../testing";
import ConnectStep from "./ConnectStep";

const base = () => imported([boxBody("block", [0, 0, 0], [1, 1, 1]), boxBody("peg", [1, 0, 0], [2, 1, 1])]);
const ids = (h: History) => ({ block: h.present.links[0].id, peg: h.present.links[1].id });
const J = (extra: Partial<Joint>): Joint => ({ id: "s1", parent_link_id: "", child_link_id: "", type: "cylindrical",
  origin_xyz: [1, 0.5, 0.5], origin_rpy: [0, 0, 0], axis: [1, 0, 0], limit_lower: null, limit_upper: null,
  source: "suggested", ...extra });
function withSuggestion(h: History, joint: Joint | null): History {
  const { block, peg } = ids(h);
  const [la, lb] = h.present.links;
  const result: DetectResponse = { joint: joint && { ...joint, parent_link_id: block, child_link_id: peg },
    reason: joint ? null : "no_contact_faces", evidence: null };
  return historyReducer(h, { type: "setSuggestion", suggestion: { key: pairKey(block, peg), linkA: block, linkB: peg,
    membership: membership(la, lb), status: "review", result } });
}
const both = (start: History, viewer: Viewer | null = null, detecting = false) => render(
  <Harness start={start} step="connect" viewer={viewer} detecting={detecting} ui={(ctx) => (
    <>
      <button data-testid="h-undo" onClick={() => ctx.dispatch({ type: "undo" })} />
      <button data-testid="h-open" onClick={() => ctx.setJointDraft(ctx.model.joints[0] ? draftFromJoint(ctx.model.joints[0]) : emptyDraft())} />
      <ConnectStep ctx={ctx} />
      {ctx.jointDraft && <JointTool ctx={ctx} />}
    </>
  )} />);

describe("ConnectStep", () => {
  it("shows the base and lets the user pick another", () => {
    both(base());
    expect(screen.getByTestId("base-name").textContent).toBe("block");
    fireEvent.click(screen.getByTestId("pick-base"));
    expect(state().pick).toBe("base");
  });

  it("offers the next joint with its suggestion and opens the tool pre-filled", () => {
    both(withSuggestion(base(), J({})));
    expect(screen.getByTestId("next-card").textContent).toMatch(/Suggested/);
    fireEvent.click(screen.getByTestId("next-joint"));
    expect(state().jointDraft).toMatchObject({ motion: "revolute", detected: "cylindrical" });
    expect(screen.getByTestId("suggested").textContent).toMatch(/cylindrical/);
  });

  it("Not connected rejects the pair", () => {
    const h = withSuggestion(base(), null);
    both(h);
    fireEvent.click(screen.getByTestId("not-connected"));
    expect(state().suggestions[pairKey(ids(h).block, ids(h).peg)].status).toBe("rejected");
    expect(screen.getByTestId("stuck")).toBeTruthy();
  });

  it("while detection runs with no candidate, says it is checking", () => {
    const h0 = withSuggestion(base(), null);
    const h = historyReducer(h0, { type: "rejectSuggestion", key: pairKey(ids(h0).block, ids(h0).peg) });
    both(h, null, true);
    expect(screen.getByTestId("checking")).toBeTruthy();
    expect(screen.queryByTestId("not-connected")).toBeNull();
  });

  it("while detection runs, an unchecked pair says it is being checked", () => {
    both(base(), null, true);
    expect(screen.getByTestId("next-card").textContent).toMatch(/Checking this pair/);
    expect(screen.getByTestId("next-card").textContent).not.toMatch(/No joint detected/);
  });

  it("says when everything is connected", () => {
    const h = base();
    const { block, peg } = ids(h);
    both(historyReducer(h, { type: "addJoint", joint: J({ id: "j", type: "fixed", parent_link_id: block, child_link_id: peg, source: "manual" }) }));
    expect(screen.getByTestId("all-connected").textContent).toMatch(/All 2 links/);
  });
});

describe("JointTool", () => {
  it("accepting a suggestion stores a confirmed revolute joint with the default limits", () => {
    both(withSuggestion(base(), J({})));
    fireEvent.click(screen.getByTestId("next-joint"));
    expect((screen.getByTestId("ok") as HTMLButtonElement).disabled).toBe(false);
    fireEvent.click(screen.getByTestId("ok"));
    const s = state();
    expect(s.joints[0]).toMatchObject({ id: "s1", type: "revolute", source: "confirmed", limit_lower: -Math.PI, limit_upper: Math.PI });
    expect(Object.values(s.suggestions)[0]).toMatchObject({ status: "accepted" });
    expect(s.jointDraft).toBeNull();
  });

  it("limits are typed in degrees and stored in radians; No limits makes it continuous", () => {
    both(withSuggestion(base(), J({})));
    fireEvent.click(screen.getByTestId("next-joint"));
    fireEvent.change(screen.getByTestId("limit-lower"), { target: { value: "-90" } });
    expect(state().jointDraft.lower).toBeCloseTo(-Math.PI / 2);
    fireEvent.click(screen.getByTestId("no-limits"));
    fireEvent.click(screen.getByTestId("ok"));
    expect(state().joints[0].type).toBe("continuous");
  });

  it("Max torque shows the suggested strength and stores what you type", async () => {
    vi.spyOn(client, "suggestEfforts").mockResolvedValue({ s1: 140 });
    both(withSuggestion(base(), J({})));
    fireEvent.click(screen.getByTestId("next-joint"));
    fireEvent.click(screen.getByTestId("ok"));
    fireEvent.click(screen.getByTestId("h-open"));
    await waitFor(() => expect(screen.getByTestId("max-effort").getAttribute("placeholder")).toBe("auto · 140 N·m"), { timeout: 2000 });
    fireEvent.change(screen.getByTestId("max-effort"), { target: { value: "50" } });
    expect(state().jointDraft.maxEffort).toBe(50);
    fireEvent.change(screen.getByTestId("max-effort"), { target: { value: "-3" } });
    expect(state().jointDraft.maxEffort).toBeNull();
  });

  it("Max torque accepts values below 1, typed digit by digit", () => {  // review I-4
    both(withSuggestion(base(), J({})));
    fireEvent.click(screen.getByTestId("next-joint"));
    const field = () => screen.getByTestId("max-effort") as HTMLInputElement;
    for (const text of ["0", "0.1", "0.18"]) {
      fireEvent.change(field(), { target: { value: text } });
      expect(field().value).toBe(text);  // a half-typed number is never wiped
    }
    expect(state().jointDraft.maxEffort).toBe(0.18);
  });

  it("a manual rigid joint between picked parts", () => {
    both(base());
    fireEvent.click(screen.getByTestId("next-joint"));    // no suggestion: parts and origin pre-filled
    expect(screen.getByTestId("problem").textContent).toMatch(/how the joint moves/);
    fireEvent.click(screen.getByTestId("motion-rigid"));
    fireEvent.click(screen.getByTestId("ok"));
    expect(state().joints[0]).toMatchObject({ type: "fixed", source: "manual", origin_xyz: [1, 0.5, 0.5] });
  });

  it("Flip reverses the axis and pick buttons switch the pick mode", () => {
    both(withSuggestion(base(), J({})));
    fireEvent.click(screen.getByTestId("next-joint"));
    fireEvent.click(screen.getByTestId("flip"));
    expect(state().jointDraft.axis).toEqual([-1, 0, 0]);   // JSON drops the sign of -0
    fireEvent.click(screen.getByTestId("pick-origin"));
    expect(state().pick).toBe("origin");
  });

  it("motion buttons hold no nested buttons, so a click never lands on the ⓘ", () => {
    both(base());
    fireEvent.click(screen.getByTestId("next-joint"));
    for (const m of ["rigid", "revolute", "slider", "planar"]) {
      expect(screen.getByTestId(`motion-${m}`).querySelector("button")).toBeNull();
      expect(screen.getByRole("button", { name: new RegExp(`What is ${m}`, "i") })).toBeTruthy();
    }
  });

  it("New joint while editing a stored joint opens an empty tool", () => {
    const h = base();
    const { block, peg } = ids(h);
    both(historyReducer(h, { type: "addJoint", joint: J({ id: "j", type: "fixed", parent_link_id: block, child_link_id: peg, source: "manual" }) }));
    fireEvent.click(screen.getByTestId("h-open"));
    fireEvent.click(screen.getByTestId("new-joint"));
    expect(screen.getByTestId("joint-tool")).toBeTruthy();
    expect(state().jointDraft).toMatchObject({ fixed: null, moving: null });
    expect(state().pick).toBe("fixed");
  });

  it("removes an existing joint", () => {
    const h = base();
    const { block, peg } = ids(h);
    both(historyReducer(h, { type: "addJoint", joint: J({ id: "j", type: "fixed", parent_link_id: block, child_link_id: peg, source: "manual" }) }));
    fireEvent.click(screen.getByTestId("h-open"));
    fireEvent.click(screen.getByTestId("remove-joint"));
    expect(state().joints).toEqual([]);
  });

  it("undo while editing closes the tool when the joint disappears", () => {
    const h = base();
    const { block, peg } = ids(h);
    both(historyReducer(h, { type: "addJoint", joint: J({ id: "j", type: "fixed", parent_link_id: block, child_link_id: peg, source: "manual" }) }));
    fireEvent.click(screen.getByTestId("h-open"));
    fireEvent.click(screen.getByTestId("motion-slider"));
    fireEvent.click(screen.getByTestId("h-undo"));
    expect(state().joints).toEqual([]);
    expect(screen.queryByTestId("joint-tool")).toBeNull();
  });

  it("Animate moves the moving link's parts through the viewer", async () => {
    const viewer = { setPreview: vi.fn(), focus: vi.fn(), onDrive: vi.fn(), setWarning: vi.fn(),
      screenPoint: () => null, onCameraChange: () => () => {} } as unknown as Viewer;
    both(withSuggestion(base(), J({})), viewer);
    fireEvent.click(screen.getByTestId("next-joint"));
    fireEvent.click(screen.getByTestId("animate"));
    await waitFor(() => expect(viewer.setPreview).toHaveBeenCalledWith(["peg"], expect.anything()));
    fireEvent.click(screen.getByTestId("cancel"));
    expect(viewer.setPreview).toHaveBeenLastCalledWith([], null);
  });
});
