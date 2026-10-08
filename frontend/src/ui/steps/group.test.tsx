import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import * as client from "../../api/client";
import { boxBody } from "../../model/testBodies";
import { Harness, imported, state } from "../testing";
import GroupStep from "./GroupStep";

const start = () => imported([
  boxBody("base", [0, 0, 0], [1, 1, 1]),
  boxBody("arm", [1, 0, 0], [2, 1, 1]),
  boxBody("screw", [0.5, 0.5, 1], [0.55, 0.55, 1.05]),
]);
const linkOf = (body: string) => state().links.find((l: { body_ids: string[] }) => l.body_ids.includes(body));
const ui = () => render(<Harness start={start()} ui={(ctx) => (
  <>
    <button data-testid="sel" onClick={() => ctx.setSelection({ kind: "bodies", ids: ["base", "arm"] })} />
    <GroupStep ctx={ctx} />
  </>
)} />);

beforeEach(() => {
  vi.restoreAllMocks();
  vi.spyOn(client, "linkMass").mockResolvedValue({ mass_kg: 1.234, center_of_mass: [0, 0, 0], inertia: [] });
});

describe("GroupStep", () => {
  it("groups small parts once on first visit and can undo it", () => {
    ui();
    expect(screen.getByTestId("auto-merged").textContent).toMatch(/1 small part/);
    expect(state().links).toHaveLength(2);
    fireEvent.click(screen.getByTestId("undo-auto"));
    expect(state().links).toHaveLength(3);
    expect(screen.queryByTestId("auto-merged")).toBeNull();
    expect(screen.getByTestId("merge-small")).toBeTruthy();
  });

  it("renames a link on blur", () => {
    ui();
    const id = linkOf("arm").id;
    fireEvent.change(screen.getByTestId(`name-${id}`), { target: { value: "shoulder" } });
    fireEvent.blur(screen.getByTestId(`name-${id}`));
    expect(linkOf("arm").name).toBe("shoulder");
  });

  it("invalid and duplicate names are not stored", () => {
    ui();
    const id = linkOf("arm").id;
    const input = screen.getByTestId(`name-${id}`) as HTMLInputElement;
    fireEvent.change(input, { target: { value: "bad name" } });
    expect(screen.getByText(/letters, digits/)).toBeTruthy();
    fireEvent.blur(input);
    expect(linkOf("arm").name).toBe("arm");
    expect(input.value).toBe("arm");
    fireEvent.change(input, { target: { value: "base" } });
    expect(screen.getByText(/already has this name/)).toBeTruthy();
    fireEvent.blur(input);
    expect(linkOf("arm").name).toBe("arm");
  });

  it("groups the selected parts' links", () => {
    ui();
    fireEvent.click(screen.getByTestId("sel"));
    fireEvent.click(screen.getByTestId("group-selected"));
    expect(state().links).toHaveLength(1);
  });

  it("dropping a part on a link card moves it there", () => {
    ui();
    const target = linkOf("base").id;
    fireEvent.drop(screen.getByTestId(`linkcard-${target}`), { dataTransfer: { getData: () => "arm" } });
    expect(linkOf("arm").id).toBe(target);
  });

  it("shows the link mass, and errors with link names", async () => {
    ui();
    const id = linkOf("arm").id;
    await waitFor(() => expect(screen.getAllByText(/1\.234 kg/)).toHaveLength(2), { timeout: 2000 });
    vi.spyOn(client, "linkMass").mockRejectedValue(new Error(`Link '${id}': volume of approximate part(s) can't be trusted`));
    fireEvent.change(screen.getByTestId(`density-${id}`), { target: { value: "Steel" } });
    expect(await screen.findByText(/Link "arm": volume/, undefined, { timeout: 2000 })).toBeTruthy();
  });
});
