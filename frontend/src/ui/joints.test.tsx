import { fireEvent, render, screen } from "@testing-library/react";
import { useReducer, useState } from "react";
import { describe, expect, it, vi } from "vitest";
import type { DetectResponse } from "../api/types";
import { historyReducer, initialHistory, type History } from "../model/history";
import { membership, pairKey } from "../model/pairs";
import { boxBody } from "../model/testBodies";
import DetailsPanel from "./DetailsPanel";
import type { Selection } from "./selection";

function setup(result: DetectResponse) {
  let h: History = historyReducer(initialHistory(), { type: "import", warnings: [],
    bodies: [boxBody("block", [0, 0, 0], [1, 1, 1]), boxBody("peg", [1, 0, 0], [2, 1, 1])] });
  const [a, b] = h.present.links.map((l) => l.id);
  const key = pairKey(a, b);
  const joint = result.joint ? { ...result.joint, parent_link_id: a, child_link_id: b } : null;
  h = historyReducer(h, { type: "setSuggestion",
    suggestion: { key, linkA: a, linkB: b, membership: membership(h.present.links[0], h.present.links[1]), status: "review", result: { ...result, joint } } });
  return { h, key };
}

const cylindrical: DetectResponse = {
  joint: { id: "j1", parent_link_id: "", child_link_id: "", type: "cylindrical", origin_xyz: [1, 0.5, 0.5],
    origin_rpy: [0, 0, 0], axis: [1, 0, 0], limit_lower: null, limit_upper: null, source: "suggested", ambiguous: true },
  reason: null,
  evidence: { pairs: [{ kind: "cylinder", radius: 0.1 }], candidate_count: 2, tolerance: 1e-4 },
};

function Harness({ start, initial }: { start: History; initial: Selection }) {
  const [h, dispatch] = useReducer(historyReducer, start);
  const [selection, setSelection] = useState<Selection>(initial);
  const setPreview = vi.fn();
  (globalThis as { __setPreview?: unknown }).__setPreview = setPreview;
  return (
    <>
      <DetailsPanel model={h.present} dispatch={dispatch} selection={selection} setSelection={setSelection}
        setBanner={vi.fn()} viewer={null} setPreview={setPreview} lastPick={null} />
      <div data-testid="state">{JSON.stringify({ joints: h.present.joints, suggestions: h.present.suggestions })}</div>
    </>
  );
}
const state = () => JSON.parse(screen.getByTestId("state").textContent!);

describe("Overview", () => {
  it("lists suggestions needing review and no-suggestion pairs with reasons", () => {
    const { h, key } = setup(cylindrical);
    render(<Harness start={h} initial={null} />);
    expect(screen.getByTestId(`suggestion-${key}`)).toBeTruthy();
    const none = setup({ joint: null, reason: "no_contact_faces", evidence: null });
    render(<Harness start={none.h} initial={null} />);
    expect(screen.getByText(/no contact faces/i)).toBeTruthy();
  });
});

describe("JointDetails", () => {
  it("cylindrical suggestions need a URDF type before accept; then the joint is confirmed", () => {
    const { h, key } = setup(cylindrical);
    render(<Harness start={h} initial={{ kind: "suggestion", key }} />);
    expect(screen.getByText(/⚠ ambiguous/)).toBeTruthy();
    expect((screen.getByTestId("accept") as HTMLButtonElement).disabled).toBe(true);
    fireEvent.change(screen.getByTestId("joint-type"), { target: { value: "continuous" } });
    fireEvent.click(screen.getByTestId("accept"));
    expect(state().joints[0]).toMatchObject({ id: "j1", type: "continuous", source: "confirmed" });
    expect(state().suggestions[key].status).toBe("accepted");
  });

  it("revolute needs limits before accept", () => {
    const { h, key } = setup(cylindrical);
    render(<Harness start={h} initial={{ kind: "suggestion", key }} />);
    fireEvent.change(screen.getByTestId("joint-type"), { target: { value: "revolute" } });
    expect((screen.getByTestId("accept") as HTMLButtonElement).disabled).toBe(true);
    fireEvent.change(screen.getByTestId("limit-lower"), { target: { value: "-1" } });
    fireEvent.change(screen.getByTestId("limit-upper"), { target: { value: "1" } });
    expect((screen.getByTestId("accept") as HTMLButtonElement).disabled).toBe(false);
  });

  it("reject marks the suggestion rejected", () => {
    const { h, key } = setup(cylindrical);
    render(<Harness start={h} initial={{ kind: "suggestion", key }} />);
    fireEvent.click(screen.getByTestId("reject"));
    expect(state().suggestions[key].status).toBe("rejected");
  });

  it("the preview slider previews the child subtree", () => {
    const { h, key } = setup(cylindrical);
    render(<Harness start={h} initial={{ kind: "suggestion", key }} />);
    fireEvent.change(screen.getByTestId("joint-type"), { target: { value: "continuous" } });
    fireEvent.change(screen.getByTestId("preview-slider"), { target: { value: "1" } });
    const setPreview = (globalThis as { __setPreview?: ReturnType<typeof vi.fn> }).__setPreview!;
    expect(setPreview).toHaveBeenLastCalledWith(expect.objectContaining({ bodyIds: ["peg"] }));
    fireEvent.mouseUp(screen.getByTestId("preview-slider"));
    expect(setPreview).toHaveBeenLastCalledWith(null);
  });
});

describe("add manual joint", () => {
  it("works on meshes too large to spread into Math.min", () => {
    const big = boxBody("big", [0, 0, 0], [1, 1, 1]);
    big.vertices = [...big.vertices, ...Array.from({ length: 300_000 }, () => [0.5, 0.5, 0.5] as [number, number, number])];
    const h = historyReducer(initialHistory(), { type: "import", warnings: [],
      bodies: [big, boxBody("peg", [1, 0, 0], [2, 1, 1])] });
    render(<Harness start={h} initial={{ kind: "bodies", ids: ["big", "peg"] }} />);
    fireEvent.click(screen.getByTestId("add-manual-joint"));
    expect(state().joints[0]).toMatchObject({ type: "fixed", source: "manual", origin_xyz: [1, 0.5, 0.5] });
  });
});
