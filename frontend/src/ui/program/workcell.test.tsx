import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as api from "../../api/program";
import type { WorkcellObject } from "../../api/program";
import WorkcellCard from "./WorkcellCard";

const table: WorkcellObject = { name: "table", shape: "box", size: [0.6, 0.4, 0.3], xyz: [0.5, 0, 0.15], rpy: [0, 0, 0], fixed: true, mass: null, friction: null };
afterEach(() => { vi.restoreAllMocks(); vi.useRealTimers(); });

async function ui(objects: WorkcellObject[] = [table], reach: number | null = 1.0) {
  vi.useFakeTimers();
  vi.spyOn(api, "getWorkcell").mockResolvedValue({ objects, error: null, reach_m: reach });
  const put = vi.spyOn(api, "putWorkcell").mockImplementation(async (_r, objs) => objs);
  render(<WorkcellCard robot="bot" viewer={null} />);
  await act(async () => { await vi.advanceTimersByTimeAsync(10); });
  return put;
}
const saved = async (put: Awaited<ReturnType<typeof ui>>) => {
  await act(async () => { await vi.advanceTimersByTimeAsync(1500); });
  return put.mock.calls.at(-1)?.[1] as WorkcellObject[];
};

describe("WorkcellCard", () => {
  it("shows what is in the cell, and adding a box saves it", async () => {
    const put = await ui();
    expect(screen.getByText(/1 object/)).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: "Edit workcell" }));
    fireEvent.click(screen.getByRole("button", { name: "Box" }));
    const objs = await saved(put);
    expect(objs.map((o) => o.name)).toEqual(["table", "box_1"]);
    expect(put).toHaveBeenCalledWith("bot", objs);
  });

  it("the selected object's fields edit it: size in mm, bolted down", async () => {
    const put = await ui();
    fireEvent.click(screen.getByRole("button", { name: "Edit workcell" }));
    fireEvent.click(screen.getByRole("button", { name: /^table/ }));
    fireEvent.change(screen.getByLabelText("Size x (mm)"), { target: { value: "800" } });
    fireEvent.click(screen.getByLabelText("Bolted down"));
    const [t] = await saved(put);
    expect(t.size[0]).toBeCloseTo(0.8, 9);
    expect(t.fixed).toBe(false);
    expect(t.mass).toBeGreaterThan(0);   // a loose object needs a mass
  });

  it("a refused save says why", async () => {
    await ui();
    vi.spyOn(api, "putWorkcell").mockRejectedValue(new Error("workcell: 'table': size must be 3 numbers > 0"));
    fireEvent.click(screen.getByRole("button", { name: "Edit workcell" }));
    fireEvent.click(screen.getByRole("button", { name: /^table/ }));
    fireEvent.change(screen.getByLabelText("Size x (mm)"), { target: { value: "0" } });
    await act(async () => { await vi.advanceTimersByTimeAsync(1500); });
    expect(screen.getByText(/size must be 3 numbers > 0/)).toBeTruthy();
  });

  it("warns about objects the arm can't reach", async () => {
    await ui([{ ...table, xyz: [1.6, 0, 0.15] }], 0.9);
    expect(screen.getByText(/table is out of the arm's reach \(0\.90 m\)/)).toBeTruthy();
  });
});
