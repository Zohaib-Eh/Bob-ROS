import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import { memoryStore } from "../model/autosave";
import { initialHistory } from "../model/history";
import { saveProject } from "../model/project";
import { modelReducer } from "../model/reducer";
import { boxBody } from "../model/testBodies";
import { emptyModel } from "../model/types";
import { Harness } from "./testing";
import RestoreBanner from "./RestoreBanner";
import { useAutosave } from "./useAutosave";
import type { Ctx } from "./ctx";

const robot = () => modelReducer(emptyModel(), { type: "import", source: "robot.step", warnings: [], bodies: [boxBody("arm", [0, 0, 0], [1, 1, 1])] });
let ctx: Ctx;
function Probe({ store }: { store: ReturnType<typeof memoryStore> }) {
  const a = useAutosave(ctx.model, ctx.dispatch, store);
  return <RestoreBanner offer={a.offer} onRestore={a.restore} onDiscard={a.discard} />;
}
const ui = (store: ReturnType<typeof memoryStore>) =>
  render(<Harness start={initialHistory()} ui={(c) => { ctx = c; return <Probe store={store} />; }} />);
afterEach(() => vi.useRealTimers());

describe("autosave (found driving the app: a page reload lost a whole gripper setup)", () => {
  it("after a reload, offers the last unsaved work and restores it", async () => {
    const store = memoryStore({ text: saveProject(robot()), at: Date.now() });
    ui(store);
    expect(await screen.findByText(/Restore your unsaved work from/)).toBeTruthy();
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: "Restore" })); });
    expect(Object.keys(ctx.model.bodies)).toEqual(["arm"]);
    expect(screen.queryByText(/Restore your unsaved work/)).toBeNull();
  });

  it("Discard forgets it", async () => {
    const store = memoryStore({ text: saveProject(robot()), at: Date.now() });
    ui(store);
    await screen.findByText(/Restore your unsaved work from/);
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: "Discard" })); });
    expect(store.saved).toBeNull();
    expect(Object.keys(ctx.model.bodies)).toEqual([]);
  });

  it("keeps saving as you work, a moment after each change", async () => {
    vi.useFakeTimers();
    const store = memoryStore();
    ui(store);
    await act(async () => { await vi.advanceTimersByTimeAsync(10); });
    act(() => ctx.dispatch({ type: "import", source: "robot.step", warnings: [], bodies: [boxBody("arm", [0, 0, 0], [1, 1, 1])] }));
    expect(store.saved).toBeNull();
    await act(async () => { await vi.advanceTimersByTimeAsync(2000); });
    expect(store.saved && JSON.parse(store.saved.text).model.bodies.arm).toBeTruthy();
  });
});
