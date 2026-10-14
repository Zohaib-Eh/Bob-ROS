import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import { boxBody } from "../../model/testBodies";
import { Harness, imported } from "../testing";
import PhysicsCard from "./PhysicsCard";

afterEach(() => vi.restoreAllMocks());
const ui = () => render(<Harness start={imported([boxBody("base", [0, 0, 0], [1, 1, 1])])} step="program"
  ui={(ctx) => <PhysicsCard ctx={ctx} />} />);

describe("PhysicsCard", () => {
  it("runs on open and shows a pass", async () => {
    const spy = vi.spyOn(client, "physicsCheck").mockResolvedValue({ ok: true, kind: "ok", message: "Holds pose." });
    ui();
    expect(await screen.findByText("Holds pose.")).toBeTruthy();
    expect(spy).toHaveBeenCalledTimes(1);
  });

  it("shows a sag with its message and can run again", async () => {
    const spy = vi.spyOn(client, "physicsCheck").mockResolvedValue(
      { ok: false, kind: "sag", link: "base", joint: "j", needed: 140, has: 100, message: "base sags 18°: needs ~140 N·m, has 100." });
    ui();
    expect(await screen.findByText(/sags 18°/)).toBeTruthy();
    expect(screen.getByTestId("physics-card").className).toMatch(/bad/);
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /run check/i })); });
    expect(spy).toHaveBeenCalledTimes(2);
  });

  it("a failed request shows in the card, not as a banner", async () => {
    vi.spyOn(client, "physicsCheck").mockRejectedValue(new Error("Cannot reach the backend"));
    ui();
    expect(await screen.findByText(/Cannot reach the backend/)).toBeTruthy();
    expect(document.querySelector(".banner")).toBeNull();
  });

  it("shows Checking physics… with a timer while it runs", async () => {
    vi.spyOn(client, "physicsCheck").mockImplementation(() => new Promise(() => {}));
    ui();
    expect(await screen.findByText(/Checking physics…/)).toBeTruthy();
  });
});
