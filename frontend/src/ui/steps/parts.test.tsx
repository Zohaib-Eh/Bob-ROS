import { act, fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import { boxBody } from "../../model/testBodies";
import { Harness, imported, state } from "../testing";
import PartsStep from "./PartsStep";

const start = () => imported([
  boxBody("base", [0, 0, 0], [1, 1, 2]),
  boxBody("cover", [1, 0, 0], [1.2, 1, 1], { status: "approximate" }),
  boxBody("bolt", [0, 0, 2], [0.1, 0.1, 2.1], { status: "failed", vertices: [], triangles: [] }),
  boxBody("plate", [0, 1, 0], [1, 1.1, 1], { warnings: [{ code: "unaccounted_faces", message: "2 faces could not be classified", fixes: ["reexport"] }] }),
]);
const ui = () => render(<Harness start={start()} ui={(ctx) => <PartsStep ctx={ctx} />} />);
const linkOf = (body: string) => state().links.find((l: { body_ids: string[] }) => l.body_ids.includes(body));

describe("PartsStep", () => {
  it("shows the model size and edits the scale", () => {
    ui();
    expect(screen.getByTestId("size").textContent).toMatch(/2\.00 m tall/);
    expect(screen.getByTestId("size").textContent).toMatch(/1\.20 m/);
    fireEvent.change(screen.getByTestId("scale-input"), { target: { value: "1" } });
    expect(state().scale).toBe(1);
  });

  it("lists one card per problem and counts the fine parts", () => {
    ui();
    for (const id of ["cover", "bolt", "plate"]) expect(screen.getByTestId(`problem-${id}`)).toBeTruthy();
    expect(screen.queryByTestId("problem-base")).toBeNull();
    expect(screen.getByTestId("fine").textContent).toMatch(/1 part looks fine/);
  });

  it("Use the estimate accepts the approximate mass and clears the card", () => {
    ui();
    fireEvent.click(screen.getByTestId("estimate-cover"));
    expect(linkOf("cover").accept_approximate_mass).toBe(true);
    expect(screen.queryByTestId("problem-cover")).toBeNull();
  });

  it("a real mass sets the link's mass override", () => {
    ui();
    fireEvent.change(screen.getByTestId("mass-cover"), { target: { value: "0.25" } });
    fireEvent.click(screen.getByTestId("set-mass-cover"));
    expect(linkOf("cover").mass_override_kg).toBe(0.25);
  });

  it("ignore moves a failed part to the ignored list; Use again brings it back", () => {
    ui();
    fireEvent.click(screen.getByTestId("ignore-bolt"));
    expect(state().ignored).toEqual(["bolt"]);
    fireEvent.click(screen.getByTestId("unignore-bolt"));
    expect(state().ignored).toEqual([]);
  });

  it("replace sends the part's id, name, box and the import scale", async () => {
    const spy = vi.spyOn(client, "replacePart").mockResolvedValue(boxBody("bolt", [0, 0, 2], [0.1, 0.1, 2.1]));
    ui();
    const file = new File(["x"], "bolt.step");
    await act(async () => { fireEvent.change(screen.getByTestId("replace-bolt"), { target: { files: [file] } }); });
    expect(spy).toHaveBeenCalledWith(file, "bolt", "bolt", null, 0.001);
    expect(screen.queryByTestId("problem-bolt")).toBeNull();
    spy.mockRestore();
  });

  it("warning cards show the message and the re-export advice", () => {
    ui();
    expect(screen.getByText("2 faces could not be classified")).toBeTruthy();
    fireEvent.click(screen.getByTestId("advice-plate"));
    expect(screen.getByText(/Re-export this part/)).toBeTruthy();
  });

  it("clicking a card selects its part", () => {
    ui();
    fireEvent.click(screen.getByTestId("problem-cover"));
    expect(state().selection).toEqual({ kind: "bodies", ids: ["cover"] });
  });
});
