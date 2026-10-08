import { act, fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import { boxBody } from "../model/testBodies";
import type { Viewer } from "../viewer/Viewer";
import Browser from "./Browser";
import { Harness, imported, state } from "./testing";

const start = () => imported([boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1]),
  boxBody("screw", [0.5, 0.5, 1], [0.55, 0.55, 1.05])]);
const linkOf = (body: string) => state().links.find((l: { body_ids: string[] }) => l.body_ids.includes(body));
const ui = (viewer: Viewer | null = null) => render(<Harness start={start()} viewer={viewer} ui={(ctx) => (
  <>
    <button data-testid="merge" onClick={() => ctx.dispatch({ type: "merge", linkIds: ctx.model.links.slice(0, 2).map((l) => l.id) })} />
    <button data-testid="sel-arm" onClick={() => ctx.setSelection({ kind: "bodies", ids: ["arm"] })} />
    <Browser ctx={ctx} />
  </>
)} />);

describe("Browser editing", () => {
  it("double-click renames a part in place; its default link follows", () => {
    ui();
    fireEvent.doubleClick(screen.getByTestId("part-arm"));
    const input = screen.getByTestId("rename-input");
    fireEvent.change(input, { target: { value: "Upper arm" } });
    fireEvent.keyDown(input, { key: "Enter" });
    expect(screen.getByTestId("part-arm").textContent).toMatch(/Upper arm/);
    expect(linkOf("arm").name).toBe("Upper_arm");
  });

  it("Escape cancels a rename", () => {
    ui();
    fireEvent.doubleClick(screen.getByTestId("part-arm"));
    fireEvent.change(screen.getByTestId("rename-input"), { target: { value: "nope" } });
    fireEvent.keyDown(screen.getByTestId("rename-input"), { key: "Escape" });
    expect(screen.queryByTestId("rename-input")).toBeNull();
    expect(state().past).toBe(1);   // only the import
    expect(screen.getByTestId("part-arm").textContent).toMatch(/^arm/);
  });

  it("an invalid link name shows why and is not stored", () => {
    ui();
    const id = linkOf("arm").id;
    fireEvent.doubleClick(screen.getByTestId(`link-${id}`));
    fireEvent.change(screen.getByTestId("rename-input"), { target: { value: "upper arm" } });
    expect(screen.getByTestId("rename-error").textContent).toMatch(/letters, digits/);
    fireEvent.keyDown(screen.getByTestId("rename-input"), { key: "Enter" });
    expect(linkOf("arm").name).toBe("arm");
  });

  it("F2 renames the selected part", () => {
    ui();
    fireEvent.click(screen.getByTestId("sel-arm"));
    act(() => { window.dispatchEvent(new Event("cad:rename")); });
    expect((screen.getByTestId("rename-input") as HTMLInputElement).value).toBe("arm");
  });

  it("deletes a part and brings it back", () => {
    ui();
    fireEvent.click(screen.getByTestId("delete-part-screw"));
    expect(state().ignored).toEqual(["screw"]);
    fireEvent.click(screen.getByTestId("restore-screw"));
    expect(state().ignored).toEqual([]);
  });

  it("ungroups and deletes links", () => {
    ui();
    fireEvent.click(screen.getByTestId("merge"));
    const group = linkOf("base").id;
    fireEvent.click(screen.getByTestId(`ungroup-${group}`));
    expect(state().links).toHaveLength(3);
    fireEvent.click(screen.getByTestId(`delete-link-${linkOf("arm").id}`));
    expect(state().ignored).toEqual(["arm"]);
  });

  it("hovering a row lights the part up in 3D", () => {
    const viewer = { setHovered: vi.fn(), focus: vi.fn() } as unknown as Viewer;
    ui(viewer);
    fireEvent.mouseEnter(screen.getByTestId("part-arm"));
    expect(viewer.setHovered).toHaveBeenLastCalledWith(["arm"]);
    fireEvent.mouseLeave(screen.getByTestId("part-arm"));
    expect(viewer.setHovered).toHaveBeenLastCalledWith([]);
    fireEvent.mouseEnter(screen.getByTestId(`link-${linkOf("base").id}`));
    expect(viewer.setHovered).toHaveBeenLastCalledWith(["base"]);
  });

  it("a part hovered in 3D is highlighted in the list", () => {
    render(<Harness start={start()} hovered="arm" ui={(ctx) => <Browser ctx={ctx} />} />);
    expect(screen.getByTestId("part-arm").className).toMatch(/hovered/);
  });
});
