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

  it("File ▸ Add part from CAD adds a second file's parts and records which file they came from", async () => {
    bar();
    open("File");
    expect(screen.getByRole("menuitem", { name: "Add part from CAD…" })).toHaveProperty("disabled", true);   // nothing to add to yet
    vi.spyOn(client, "importFile")
      .mockResolvedValueOnce({ bodies: [boxBody("arm", [0, 0, 0], [1, 1, 1])], applied_scale: 0.001, warnings: [] })
      .mockResolvedValueOnce({ bodies: [boxBody("plate", [2, 0, 0], [3, 1, 1])], applied_scale: 0.001, warnings: [] });
    await act(async () => { fireEvent.change(screen.getByTestId("import-input"), { target: { files: [new File(["x"], "robot.step")] } }); });
    expect(screen.getByRole("menuitem", { name: "Add part from CAD…" })).toHaveProperty("disabled", false);   // menu still open
    await act(async () => { fireEvent.change(screen.getByTestId("import-input"), { target: { files: [new File(["x"], "gripper.step")] } }); });
    expect(state().links).toHaveLength(2);
    expect(state().sources).toEqual(["robot.step", "gripper.step"]);
  });

  it("File ▸ Import URDF replaces the robot with the file's links, joints, gripper and name", async () => {
    const link = (id: string) => ({ id, name: id, body_ids: [`${id}_part`], density_kg_m3: 1240, mass_override_kg: 2, accept_approximate_mass: true });
    const joint = { id: "j1", parent_link_id: "base", child_link_id: "arm", type: "revolute" as const, origin_xyz: [0, 0, 1] as [number, number, number],
      origin_rpy: [0, 0, 0] as [number, number, number], axis: [0, 0, 1] as [number, number, number], limit_lower: -1, limit_upper: 1, source: "confirmed" as const };
    const spy = vi.spyOn(client, "importUrdf").mockResolvedValue({
      robot_name: "ur5", bodies: [boxBody("base_part", [0, 0, 0], [1, 1, 1]), boxBody("arm_part", [0, 0, 1], [1, 1, 2])],
      links: [link("base"), link("arm")], joints: [joint], root_link_id: "base", gripper: null,
      warnings: [{ code: "no_faces", message: "no CAD faces", fixes: [] }] });
    let notify: ReturnType<typeof vi.fn> | null = null;
    render(<Harness start={imported([boxBody("old", [0, 0, 0], [1, 1, 1])])}
      ui={(ctx) => { notify = ctx.notify as ReturnType<typeof vi.fn>; return <MenuBar ctx={ctx} />; }} />);
    vi.spyOn(window, "confirm").mockReturnValue(true);              // it replaces the robot being built
    open("File");
    const file = new File(["x"], "ur5.zip");
    await act(async () => { fireEvent.change(screen.getByTestId("urdf-input"), { target: { files: [file] } }); });
    expect(spy).toHaveBeenCalledWith(file);
    const s = state();
    expect(s.links.map((l: { id: string }) => l.id)).toEqual(["base", "arm"]);
    expect(s.joints).toHaveLength(1);
    expect([s.rootLinkId, s.robotName, s.step, s.sources]).toEqual(["base", "ur5", "connect", ["ur5.zip"]]);
    expect(notify!).toHaveBeenCalledWith("Imported ur5.zip. no CAD faces");   // what came in, and what didn't
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
