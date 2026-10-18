import { act, fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it } from "vitest";
import type { Body, Face, Vec3 } from "../api/types";
import { historyReducer, initialHistory } from "../model/history";
import { boxBody } from "../model/testBodies";
import type { Ctx } from "./ctx";
import MountCard from "./MountCard";
import { routePick } from "./pick";
import { Harness } from "./testing";

// one triangle per face so a hit on triangle k picks face k
const plane = (center: Vec3, normal: Vec3, tri: number): Face => ({
  kind: "plane", origin: "brep", normal, axis: null, center, radius: null, residual: 0, triangle_indices: [tri],
});
const flange = boxBody("flange", [5, 0, 0], [6, 1, 1], { faces: [plane([6, 0.5, 0.5], [1, 0, 0], 0)] });
const plate = (): Body => boxBody("plate", [0, 0, 0], [1, 1, 1], {
  faces: [plane([0.5, 0.5, 1], [0, 0, 1], 0),
    { kind: "cylinder", origin: "brep", normal: null, axis: [0, 0, 1], center: [0, 0, 0], radius: 0.1, residual: 0, triangle_indices: [1] }],
});
const start = () => {
  let h = historyReducer(initialHistory(), { type: "import", source: "robot.step", warnings: [], bodies: [flange] });
  h = historyReducer(h, { type: "import", source: "gripper.step", warnings: [], bodies: [plate()] });
  return h;
};

let ctx: Ctx;
const ui = () => render(<Harness start={start()} ui={(c) => { ctx = c; return <MountCard ctx={c} />; }} />);
const pickFace = (mode: "mount-from" | "mount-to", bodyId: string, triangle: number) => act(() => {
  const r = routePick(mode, { bodyId, triangle }, false, ctx.model, null, null);
  if (r.face) ctx.setPickedFace(r.face);
  if (r.mode) ctx.setPick(r.mode);
});

describe("MountCard", () => {
  it("routePick returns the clicked face in mount modes", () => {
    const m = historyReducer(start(), { type: "undo" }).present;   // any model with the flange
    expect(routePick("mount-to", { bodyId: "flange", triangle: 0 }, false, m, null, null))
      .toEqual({ face: { mode: "mount-to", bodyId: "flange", faceIndex: 0 }, mode: "select" });
    expect(routePick("mount-to", { bodyId: "flange", triangle: 9 }, false, m, null, null)).toEqual({});
  });

  it("mounts the newest file flush on the robot face, and Undo puts it back", () => {
    ui();
    pickFace("mount-from", "plate", 0);
    pickFace("mount-to", "flange", 0);
    fireEvent.click(screen.getByRole("button", { name: "Mount" }));
    const xs = ctx.model.bodies.plate.vertices.map((v) => v[0]);
    expect(Math.min(...xs)).toBeCloseTo(6, 9);        // flush on the flange face, outside it
    expect(ctx.model.bodies.flange.vertices).toEqual(flange.vertices);
    fireEvent.click(screen.getByRole("button", { name: /Undo/ }));
    ctx.model.bodies.plate.vertices.forEach((v, i) => v.forEach((x, k) => expect(x).toBeCloseTo(plate().vertices[i][k], 9)));
  });

  it("turn steps rotate the mounted part about the face axis", () => {
    ui();
    pickFace("mount-from", "plate", 0);
    pickFace("mount-to", "flange", 0);
    fireEvent.click(screen.getByRole("button", { name: "Mount" }));
    const before = ctx.model.bodies.plate.vertices[1];
    fireEvent.click(screen.getByRole("button", { name: "Turn +15°" }));
    const after = ctx.model.bodies.plate.vertices[1];
    expect(after[0]).toBeCloseTo(before[0], 9);
    expect(after).not.toEqual(before);
  });

  it("a file added while the card is showing becomes the part to mount", () => {   // Add part from CAD in Build
    render(<Harness start={historyReducer(initialHistory(), { type: "import", source: "robot.step", warnings: [], bodies: [flange] })}
      ui={(c) => { ctx = c; return <MountCard ctx={c} />; }} />);
    act(() => ctx.dispatch({ type: "import", source: "gripper.step", warnings: [], bodies: [plate()] }));
    expect((screen.getByRole("combobox") as HTMLSelectElement).value).toBe("gripper.step");
    pickFace("mount-from", "plate", 0);
    pickFace("mount-to", "flange", 0);
    fireEvent.click(screen.getByRole("button", { name: "Mount" }));
    expect(screen.queryByRole("alert")).toBeNull();
    expect(Math.min(...ctx.model.bodies.plate.vertices.map((v) => v[0]))).toBeCloseTo(6, 9);
  });

  it("a project saved before sources existed can still have a part mounted on it", () => {
    const old = historyReducer(initialHistory(), { type: "import", warnings: [], bodies: [flange] });   // no source
    render(<Harness start={old} ui={(c) => { ctx = c; return <MountCard ctx={c} />; }} />);
    act(() => ctx.dispatch({ type: "import", source: "gripper.step", warnings: [], bodies: [plate()] }));
    pickFace("mount-from", "plate", 0);
    pickFace("mount-to", "flange", 0);
    fireEvent.click(screen.getByRole("button", { name: "Mount" }));
    expect(screen.queryByRole("alert")).toBeNull();
    expect(Math.min(...ctx.model.bodies.plate.vertices.map((v) => v[0]))).toBeCloseTo(6, 9);
    expect(ctx.model.bodies.flange.vertices).toEqual(flange.vertices);
  });

  it("Edit > Undo and the card stay in step: one undo per Turn, and Undo after Ctrl+Z does nothing odd", () => {
    ui();
    pickFace("mount-from", "plate", 0);
    pickFace("mount-to", "flange", 0);
    fireEvent.click(screen.getByRole("button", { name: "Mount" }));
    const mounted = ctx.model.bodies.plate.vertices;
    fireEvent.click(screen.getByRole("button", { name: "Turn +15°" }));
    act(() => ctx.dispatch({ type: "undo" }));          // one Ctrl+Z undoes the turn, not half of it
    ctx.model.bodies.plate.vertices.forEach((v, i) => v.forEach((x, k) => expect(x).toBeCloseTo(mounted[i][k], 9)));
    expect((screen.getByLabelText("Turn angle (degrees)") as HTMLInputElement).value).toBe("0");
    act(() => ctx.dispatch({ type: "undo" }));          // undoes the mount
    ctx.model.bodies.plate.vertices.forEach((v, i) => v.forEach((x, k) => expect(x).toBeCloseTo(plate().vertices[i][k], 9)));
    expect((screen.getByRole("button", { name: /Undo/ }) as HTMLButtonElement).disabled).toBe(true);
  });

  it("each pick says what it got, and a curved pick is refused by name", () => {   // found driving the app
    ui();
    pickFace("mount-from", "plate", 1);
    pickFace("mount-to", "flange", 0);
    expect(screen.getByRole("button", { name: /plate · curved face/ })).toBeTruthy();
    expect(screen.getByRole("button", { name: /flange · flat face/ })).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: "Mount" }));
    expect(screen.getByRole("alert").textContent)
      .toBe("The face picked on plate is curved. Pick a flat face: the gripper's mounting plate and the flange.");
  });
});
