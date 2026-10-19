import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import type { Joint } from "../../api/types";
import { historyReducer, type History } from "../../model/history";
import { boxBody } from "../../model/testBodies";
import * as download from "../download";
import { Harness, imported, state } from "../testing";
import ExportStep from "./ExportStep";

afterEach(() => vi.restoreAllMocks());
const loose = () => imported([boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1])]);
function ready(): History {
  const h = loose();
  const [a, b] = h.present.links.map((l) => l.id);
  const j: Joint = { id: "j", parent_link_id: a, child_link_id: b, type: "fixed", origin_xyz: [1, 0.5, 0.5],
    origin_rpy: [0, 0, 0], axis: null, limit_lower: null, limit_upper: null, source: "manual" };
  return historyReducer(h, { type: "addJoint", joint: j });
}
const ui = (h: History) => render(<Harness start={h} step="export" ui={(ctx) => <ExportStep ctx={ctx} />} />);

describe("ExportStep", () => {
  it("lists what is missing and jumps to it", () => {
    const h = loose();
    ui(h);
    expect(screen.getByTestId("check-connected").className).toMatch(/bad/);
    expect((screen.getByTestId("export") as HTMLButtonElement).disabled).toBe(true);
    fireEvent.click(screen.getByTestId("check-connected"));
    expect(state().step).toBe("connect");
    expect(state().selection).toEqual({ kind: "link", id: h.present.links[1].id });
  });

  it("an invalid robot name blocks export", () => {
    ui(ready());
    expect((screen.getByTestId("export") as HTMLButtonElement).disabled).toBe(false);
    fireEvent.change(screen.getByTestId("robot-name"), { target: { value: "my robot" } });
    expect((screen.getByTestId("export") as HTMLButtonElement).disabled).toBe(true);
    expect(screen.getByTestId("check-names").className).toMatch(/bad/);
  });

  it("Continue to Simulate validates on the server and opens Simulate without downloading", async () => {
    const spy = vi.spyOn(client, "exportRobot").mockResolvedValue({ blob: new Blob(["z"]), warnings: [] });
    const dl = vi.spyOn(download, "downloadBlob").mockImplementation(() => {});
    ui(ready());
    expect(screen.getByTestId("summary").textContent).toMatch(/2 links · 1 joints · 2 meshes/);
    expect(screen.queryByText(/export urdf \(\.zip\)/i)).toBeNull();
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /continue to simulate/i })); });
    expect(spy).toHaveBeenCalledWith("robot", expect.any(Array), [expect.objectContaining({ id: "j" })], expect.any(Object), null, null);   // gripper, mobile base: validates the robot as exported
    expect(dl).not.toHaveBeenCalled();
    expect(state().step).toBe("program");
  });

  it("shows server errors with link names", async () => {
    const h = ready();
    const id = h.present.links[0].id;
    vi.spyOn(client, "exportRobot").mockRejectedValue(new Error(`Cannot export URDF -- Link '${id}' has no mesh geometry.`));
    ui(h);
    await act(async () => { fireEvent.click(screen.getByTestId("export")); });
    expect(screen.getByTestId("server-error").textContent).toMatch(/Link "base" has no mesh geometry/);
  });
});
