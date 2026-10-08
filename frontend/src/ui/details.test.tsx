import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { useReducer, useState } from "react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as client from "../api/client";
import { historyReducer, initialHistory } from "../model/history";
import { boxBody } from "../model/testBodies";
import DetailsPanel from "./DetailsPanel";
import type { Selection } from "./selection";

const warned = boxBody("p", [0, 0, 0], [1, 1, 1], { status: "approximate",
  warnings: [{ code: "open_geometry", message: "p: 4 open edges remain", fixes: ["set_mass", "replace", "ignore"] }] });

function Harness({ initial }: { initial: (linkId: string) => Selection }) {
  const [h, dispatch] = useReducer(historyReducer, undefined, () =>
    historyReducer(initialHistory(), { type: "import", bodies: [warned], warnings: [] }));
  const [selection, setSelection] = useState<Selection>(() => initial(h.present.links[0].id));
  return (
    <>
      <DetailsPanel model={h.present} dispatch={dispatch} selection={selection} setSelection={setSelection}
        setBanner={vi.fn()} viewer={null} setPreview={vi.fn()} lastPick={null} />
      <div data-testid="state">{JSON.stringify({ links: h.present.links, ignored: h.present.ignored, sel: selection })}</div>
    </>
  );
}
const state = () => JSON.parse(screen.getByTestId("state").textContent!);

afterEach(() => vi.restoreAllMocks());

describe("PartDetails", () => {
  it("offers exactly the warning's fixes", () => {
    render(<Harness initial={() => ({ kind: "bodies", ids: ["p"] })} />);
    expect(screen.getByText("p: 4 open edges remain")).toBeTruthy();
    expect(screen.getByTestId("fix-set_mass")).toBeTruthy();
    expect(screen.getByTestId("fix-ignore")).toBeTruthy();
    expect(screen.queryByTestId("fix-reexport")).toBeNull();
  });

  it("ignore removes the part from its link", () => {
    render(<Harness initial={() => ({ kind: "bodies", ids: ["p"] })} />);
    fireEvent.click(screen.getByTestId("fix-ignore"));
    expect(state().ignored).toEqual(["p"]);
  });

  it("set mass jumps to the link", () => {
    render(<Harness initial={() => ({ kind: "bodies", ids: ["p"] })} />);
    fireEvent.click(screen.getByTestId("fix-set_mass"));
    expect(state().sel.kind).toBe("link");
  });

  it("replace sends the part's id, name and bounding box", async () => {
    const spy = vi.spyOn(client, "replacePart").mockResolvedValue({ ...warned, status: "ok", warnings: [] });
    render(<Harness initial={() => ({ kind: "bodies", ids: ["p"] })} />);
    const file = new File(["x"], "fixed.stl");
    await act(async () => { fireEvent.change(screen.getByTestId("replace-input"), { target: { files: [file] } }); });
    expect(spy).toHaveBeenCalledWith(file, "p", "p", { min: [0, 0, 0], max: [1, 1, 1] }, 0.001);
  });
});

describe("LinkDetails", () => {
  it("edits density, override and acceptance, and shows link mass", async () => {
    vi.spyOn(client, "linkMass").mockResolvedValue({ mass_kg: 2.5, center_of_mass: [0, 0, 0], inertia: [0, 0, 0, 0, 0, 0] });
    render(<Harness initial={(id) => ({ kind: "link", id })} />);
    fireEvent.change(screen.getByTestId("density-preset"), { target: { value: "Aluminium" } });
    fireEvent.change(screen.getByTestId("mass-override"), { target: { value: "2.5" } });
    fireEvent.click(screen.getByTestId("accept-approx"));
    const link = state().links[0];
    expect(link.density_kg_m3).toBe(2700);
    expect(link.mass_override_kg).toBe(2.5);
    expect(link.accept_approximate_mass).toBe(true);
    await waitFor(() => expect(screen.getByTestId("link-mass").textContent).toMatch(/2\.5/));
  });

  it("shows the link-mass error text", async () => {
    vi.spyOn(client, "linkMass").mockRejectedValue(new client.ApiError(422, "Link 'x': contains approximate part 'p'"));
    render(<Harness initial={(id) => ({ kind: "link", id })} />);
    await waitFor(() => expect(screen.getByTestId("link-mass").textContent).toMatch(/approximate part/));
  });
});
