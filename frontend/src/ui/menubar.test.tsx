import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../api/client";
import { boxBody } from "../model/testBodies";
import MenuBar from "./MenuBar";
import { Harness, imported, state } from "./testing";

afterEach(() => { vi.restoreAllMocks(); localStorage.clear(); });
const bar = (start = imported([])) => render(<Harness start={start} ui={(ctx) => <MenuBar ctx={ctx} />} />);
const open = (menu: string) => fireEvent.click(screen.getByRole("button", { name: menu }));

describe("MenuBar", () => {
  it("File ▸ Import CAD imports with progress, like the old button", async () => {
    const spy = vi.spyOn(client, "importFile").mockResolvedValue({ bodies: [boxBody("x", [0, 0, 0], [1, 1, 1])], applied_scale: 0.001, warnings: [] });
    bar();
    open("File");
    const file = new File(["x"], "a.stl");
    await act(async () => { fireEvent.change(screen.getByTestId("import-input"), { target: { files: [file] } }); });
    expect(spy).toHaveBeenCalledWith(file, 0.001, expect.any(Function));
    expect(state().links).toHaveLength(1);
  });

  it("Edit ▸ Undo undoes, and is disabled with nothing to undo", () => {
    bar(imported([boxBody("x", [0, 0, 0], [1, 1, 1])]));
    const before = state().links.length;
    open("Edit");
    fireEvent.click(screen.getByRole("menuitem", { name: /undo/i }));
    expect(state().links.length).not.toBe(before);
    open("Edit");
    expect(screen.getByRole("menuitem", { name: /undo/i })).toHaveProperty("disabled", true);
  });

  it("View toggles the grid and remembers it", () => {
    bar();
    open("View");
    fireEvent.click(screen.getByRole("menuitemcheckbox", { name: /grid/i }));
    expect(JSON.parse(localStorage.getItem("cad-to-urdf.view")!)).toMatchObject({ grid: false });
  });

  it("Simulate is disabled until the Review checklist passes, and says why", () => {
    bar();
    const sim = screen.getByRole("button", { name: "Simulate" });
    expect(sim).toHaveProperty("disabled", true);
    expect(sim.getAttribute("title")).toBe("Fix the Review checklist first.");
  });

  it("Escape closes an open menu", () => {
    bar();
    open("File");
    expect(screen.getByRole("menu")).toBeTruthy();
    fireEvent.keyDown(document, { key: "Escape" });
    expect(screen.queryByRole("menu")).toBeNull();
  });
});
