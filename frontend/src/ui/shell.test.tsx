import { act, fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import * as client from "../api/client";
import type { Body, Joint } from "../api/types";
import { historyReducer } from "../model/history";
import { emptyDraft } from "../model/jointDraft";
import { boxBody } from "../model/testBodies";
import { emptyModel } from "../model/types";
import Browser from "./Browser";
import { markersFor } from "./markers";
import { routePick } from "./pick";
import { shortcutFor } from "./shortcuts";
import StepPanel from "./steps/StepPanel";
import Stepper from "./Stepper";
import { Harness, imported, state } from "./testing";

const two = () => imported([boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1], { status: "approximate" })]);
const idle = { done: 0, total: 0, running: false, paused: false };

describe("shortcutFor", () => {
  const ev = (key: string, extra: object = {}) => ({ key, ctrlKey: false, metaKey: false, shiftKey: false, target: document.body, ...extra });
  it("maps keys to actions", () => {
    expect(shortcutFor(ev("z", { ctrlKey: true }))).toBe("undo");
    expect(shortcutFor(ev("Z", { ctrlKey: true, shiftKey: true }))).toBe("redo");
    expect(shortcutFor(ev("y", { metaKey: true }))).toBe("redo");
    expect(shortcutFor(ev("j"))).toBe("joint");
    expect(shortcutFor(ev("G"))).toBe("group");
    expect(shortcutFor(ev(" "))).toBe("animate");
    expect(shortcutFor(ev("f"))).toBe("focus");
    expect(shortcutFor(ev("Escape"))).toBe("back");
    expect(shortcutFor(ev("q"))).toBeNull();
  });
  it("ignores typing in fields except Escape", () => {
    const input = document.createElement("input");
    expect(shortcutFor(ev("j", { target: input }))).toBeNull();
    expect(shortcutFor(ev("z", { ctrlKey: true, target: input }))).toBeNull();
    expect(shortcutFor(ev("Escape", { target: input }))).toBe("back");
  });
});

describe("routePick", () => {
  const m = two().present;
  const lid = (b: string) => m.links.find((l) => l.body_ids.includes(b))!.id;
  it("select mode selects, shift adds, empty space clears", () => {
    expect(routePick("select", { bodyId: "arm", triangle: 0 }, false, m, null, null)).toEqual({ selection: { kind: "bodies", ids: ["arm"] } });
    expect(routePick("select", { bodyId: "base", triangle: 0 }, true, m, { kind: "bodies", ids: ["arm"] }, null).selection)
      .toEqual({ kind: "bodies", ids: ["arm", "base"] });
    expect(routePick("select", null, false, m, null, null)).toEqual({ selection: null });
  });
  it("base mode sets the base link and returns to select", () => {
    expect(routePick("base", { bodyId: "arm", triangle: 0 }, false, m, null, null))
      .toEqual({ action: { type: "setRoot", linkId: lid("arm") }, mode: "select" });
  });
  it("fixed then moving fills the draft and places the origin between the parts", () => {
    const r1 = routePick("fixed", { bodyId: "base", triangle: 0 }, false, m, null, emptyDraft());
    expect(r1.draft?.fixed).toBe(lid("base"));
    expect(r1.mode).toBe("moving");
    const r2 = routePick("moving", { bodyId: "arm", triangle: 0 }, false, m, null, r1.draft!);
    expect(r2.draft).toMatchObject({ moving: lid("arm"), origin: [1, 0.5, 0.5] });
    expect(r2.mode).toBe("select");
  });
  it("origin mode snaps to a round face", () => {
    const withHole: Body = { ...m.bodies.arm, faces: [{ kind: "cylinder", origin: "brep", normal: null, axis: [1, 0, 0],
      center: [1.5, 0.5, 0.5], radius: 0.1, residual: 0, triangle_indices: [3] }] };
    const mm = { ...m, bodies: { ...m.bodies, arm: withHole } };
    const r = routePick("origin", { bodyId: "arm", triangle: 3 }, false, mm, null, emptyDraft("a", "b"));
    expect(r.draft).toMatchObject({ origin: [1.5, 0.5, 0.5], axis: [1, 0, 0] });
    expect(r.mode).toBe("select");
  });
  it("empty space keeps the pick mode", () => {
    expect(routePick("base", null, false, m, null, null)).toEqual({});
    expect(routePick("fixed", null, false, m, null, emptyDraft())).toEqual({});
    expect(routePick("origin", { bodyId: "arm", triangle: 99 }, false, m, null, emptyDraft())).toEqual({});
  });
});

describe("markersFor", () => {
  it("pins problems on Parts and Export only; loose links only on Connect", () => {
    const m = two().present;
    expect(markersFor("parts", m).map((k) => k.kind)).toEqual(["problem"]);
    expect(markersFor("export", m).map((k) => k.kind)).toEqual(["problem"]);
    expect(markersFor("connect", m).map((k) => k.kind)).toEqual(["loose"]);
  });
  it("marks only loose links that touch the connected chain", () => {
    const m = imported([boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1]),
      boxBody("tip", [2, 0, 0], [3, 1, 1])]).present;
    expect(markersFor("connect", m).map((k) => k.id)).toEqual([`loose-${m.links[1].id}`]);
  });
  it("shows joints on Connect and Export", () => {
    let h = two();
    const [a, b] = h.present.links.map((l) => l.id);
    const j: Joint = { id: "j", parent_link_id: a, child_link_id: b, type: "fixed", origin_xyz: [1, 0.5, 0.5],
      origin_rpy: [0, 0, 0], axis: null, limit_lower: null, limit_upper: null, source: "manual" };
    h = historyReducer(h, { type: "addJoint", joint: j });
    expect(markersFor("export", h.present).find((k) => k.kind === "joint")).toMatchObject({ at: [1, 0.5, 0.5], target: { kind: "joint", id: "j" } });
  });
});

describe("Stepper", () => {
  it("shows badges and switches steps", () => {
    render(<Harness start={two()} ui={(ctx) => <Stepper ctx={ctx} progress={idle} onCancel={vi.fn()} onResume={vi.fn()} />} />);
    expect(screen.getByTestId("step-parts").querySelector(".badge")?.textContent).toBe("1");   // one approximate part
    fireEvent.click(screen.getByTestId("step-connect"));
    expect(state().step).toBe("connect");
  });
  it("imports a file with the current scale", async () => {
    const spy = vi.spyOn(client, "importFile").mockResolvedValue({ bodies: [boxBody("x", [0, 0, 0], [1, 1, 1])], applied_scale: 0.001, warnings: [] });
    render(<Harness start={imported([])} ui={(ctx) => <Stepper ctx={ctx} progress={idle} onCancel={vi.fn()} onResume={vi.fn()} />} />);
    const file = new File(["x"], "a.stl");
    await act(async () => { fireEvent.change(screen.getByTestId("import-input"), { target: { files: [file] } }); });
    expect(spy).toHaveBeenCalledWith(file, 0.001);
    expect(state().links).toHaveLength(1);
    spy.mockRestore();
  });
});

describe("Browser and chain", () => {
  it("click selects a part, shift-click adds", () => {
    render(<Harness start={two()} ui={(ctx) => <Browser ctx={ctx} />} />);
    fireEvent.click(screen.getByTestId("part-base"));
    fireEvent.click(screen.getByTestId("part-arm"), { shiftKey: true });
    expect(state().selection).toEqual({ kind: "bodies", ids: ["base", "arm"] });
  });
  it("marks the base and lists loose links in the chain", () => {
    render(<Harness start={two()} ui={(ctx) => <Browser ctx={ctx} />} />);
    const base = state().rootLinkId;
    expect(screen.getByTestId(`link-${base}`).querySelector("[data-base]")).toBeTruthy();
    expect(screen.getByTestId("chain").textContent).toMatch(/Not connected yet/);
  });
});

describe("StepPanel", () => {
  it("asks for an import on an empty project", () => {
    render(<Harness start={{ past: [], present: emptyModel(), future: [] }} ui={(ctx) => <StepPanel ctx={ctx} />} />);
    expect(screen.getByTestId("empty").textContent).toMatch(/Import/);
  });
});
