import { fireEvent, render, screen } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import * as client from "../api/client";
import type { Joint } from "../api/types";
import { historyReducer } from "../model/history";
import { boxBody } from "../model/testBodies";
import InfoCard from "./InfoCard";
import type { Selection } from "./selection";
import { Harness, imported, state } from "./testing";

beforeEach(() => {
  vi.restoreAllMocks();
  vi.spyOn(client, "linkMass").mockResolvedValue({ mass_kg: 2, center_of_mass: [0, 0, 0], inertia: [] });
});

function setup(sel: (ids: { a: string; b: string }) => Selection) {
  let h = imported([boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1])]);
  const [a, b] = h.present.links.map((l) => l.id);
  const j: Joint = { id: "j", parent_link_id: a, child_link_id: b, type: "revolute", origin_xyz: [1, 0.5, 0.5],
    origin_rpy: [0, 0, 0], axis: [0, 0, 1], limit_lower: -1, limit_upper: 1, source: "manual" };
  h = historyReducer(h, { type: "addJoint", joint: j });
  render(<Harness start={h} ui={(ctx) => (
    <>
      <button data-testid="sel" onClick={() => ctx.setSelection(sel({ a, b }))} />
      <InfoCard ctx={ctx} />
    </>
  )} />);
  fireEvent.click(screen.getByTestId("sel"));
  return { a, b };
}

describe("InfoCard", () => {
  it("a part card moves the part to another link", () => {
    const { a } = setup(() => ({ kind: "bodies", ids: ["arm"] }));
    expect(screen.getByTestId("infocard").textContent).toMatch(/arm/);
    fireEvent.change(screen.getByTestId("move-arm"), { target: { value: a } });
    expect(state().links.find((l: { body_ids: string[] }) => l.body_ids.includes("arm")).id).toBe(a);
  });
  it("a link card makes the link the base", () => {
    const { b } = setup(({ b }) => ({ kind: "link", id: b }));
    fireEvent.click(screen.getByTestId("make-base"));
    expect(state().rootLinkId).toBe(b);
  });
  it("a joint card opens the joint in the Joint tool", () => {
    setup(() => ({ kind: "joint", id: "j" }));
    expect(screen.getByTestId("infocard").textContent).toMatch(/Revolute/);
    fireEvent.click(screen.getByTestId("info-edit"));
    expect(state().step).toBe("connect");
    expect(state().jointDraft).toMatchObject({ id: "j", motion: "revolute" });
  });
  it("closes on ×", () => {
    setup(() => ({ kind: "bodies", ids: ["arm"] }));
    fireEvent.click(screen.getByRole("button", { name: "Close" }));
    expect(screen.queryByTestId("infocard")).toBeNull();
  });
});
