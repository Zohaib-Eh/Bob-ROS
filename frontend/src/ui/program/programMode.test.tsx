import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import * as api from "../../api/program";
import { boxBody } from "../../model/testBodies";
import { Harness, imported, state } from "../testing";
import ProgramMode from "./ProgramMode";
import { POLL } from "./useRosSession";

vi.mock("../../ros/rosbridge", () => ({
  RosBridge: class { ready = Promise.resolve(); subscribe() {} call() { return Promise.resolve({ scene: {} }); } close() {} },
}));
beforeEach(() => { POLL.starting = 10; POLL.ready = 10_000; });
afterEach(() => vi.restoreAllMocks());

const conv = (id: string, title: string) => ({ id, title, created: 1, updated: 1, claude_session_id: null, edited: [], messages: [] });
function ui() {
  return render(<Harness start={imported([boxBody("base", [0, 0, 0], [1, 1, 1])])} step="program"
    ui={(ctx) => <ProgramMode ctx={ctx} preview={<div data-testid="viewer" />} />} />);
}
async function ready() {
  vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "ready", robot: "bot" });
  vi.spyOn(api, "startSession").mockResolvedValue({ state: "starting", robot: "bot" });
  ui();
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /start ros/i })); });
  await screen.findByText(/ROS ready/);
}

describe("ProgramMode", () => {
  it("fills the window, shows the preview and goes back to the robot", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([]);
    await ready();
    expect(screen.getByTestId("viewer")).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: /back to robot/i }));
    expect(state().step).toBe("export");
  });

  it("lists conversations, creates, switches and renames", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "Square" }]);
    vi.spyOn(api, "getConversation").mockImplementation(async (id) => conv(id, id === "aaaaaaaaaaaa" ? "Square" : "New conversation"));
    const create = vi.spyOn(api, "createConversation").mockResolvedValue(conv("bbbbbbbbbbbb", "New conversation"));
    const rename = vi.spyOn(api, "renameConversation").mockResolvedValue(conv("aaaaaaaaaaaa", "Square path"));
    await ready();
    await screen.findByText("Square");
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /new/i })); });
    expect(create).toHaveBeenCalled();
    expect(screen.getByTestId("conv-bbbbbbbbbbbb").className).toMatch(/active/);
    await act(async () => { fireEvent.doubleClick(screen.getByText("Square")); });
    fireEvent.change(screen.getByDisplayValue("Square"), { target: { value: "Square path" } });
    await act(async () => { fireEvent.keyDown(screen.getByDisplayValue("Square path"), { key: "Enter" }); });
    expect(rename).toHaveBeenCalledWith("aaaaaaaaaaaa", "Square path");
  });

  it("shows broken conversations without crashing", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "cccccccccccc", title: "Unreadable conversation", broken: true }]);
    await ready();
    expect((await screen.findByText("Unreadable conversation")).closest("li")!.className).toMatch(/broken/);
  });
});

describe("ProgramMode sidebar fixes", () => {
  const two = [{ id: "aaaaaaaaaaaa", title: "Alpha" }, { id: "bbbbbbbbbbbb", title: "Beta" }];
  const activeIds = () => two.filter((c) => screen.queryByTestId(`conv-${c.id}`)?.className.includes("active")).map((c) => c.id);

  it("deleting the active conversation selects the next one", async () => {
    vi.spyOn(window, "confirm").mockReturnValue(true);
    vi.spyOn(api, "listConversations").mockResolvedValueOnce(two).mockResolvedValue([two[1]]);
    const get = vi.spyOn(api, "getConversation").mockImplementation(async (id) => conv(id, id));
    vi.spyOn(api, "deleteConversation").mockResolvedValue(undefined as never);
    await ready();
    await vi.waitFor(() => expect(activeIds()).toEqual(["aaaaaaaaaaaa"]));
    await act(async () => { fireEvent.click(screen.getByLabelText("Delete Alpha")); });
    await vi.waitFor(() => expect(activeIds()).toEqual(["bbbbbbbbbbbb"]));
    expect(get).not.toHaveBeenLastCalledWith("aaaaaaaaaaaa");
    expect(document.querySelector(".error")).toBeNull();
  });

  it("a slow earlier switch cannot override a later one", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue(two);
    const slow: Record<string, (c: ReturnType<typeof conv>) => void> = {};
    vi.spyOn(api, "getConversation").mockImplementationOnce(async (id) => conv(id, id))
      .mockImplementation((id) => new Promise((r) => { slow[id] = r; }));
    await ready();
    await vi.waitFor(() => expect(activeIds()).toEqual(["aaaaaaaaaaaa"]));
    await act(async () => { fireEvent.click(screen.getByText("Alpha")); });
    await act(async () => { fireEvent.click(screen.getByText("Beta")); });
    await act(async () => { slow.bbbbbbbbbbbb(conv("bbbbbbbbbbbb", "Beta")); });
    await act(async () => { slow.aaaaaaaaaaaa(conv("aaaaaaaaaaaa", "Alpha")); });
    expect(activeIds()).toEqual(["bbbbbbbbbbbb"]);
  });

  it("shows an error when creating fails", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([]);
    vi.spyOn(api, "createConversation").mockRejectedValue(new Error("boom"));
    await ready();
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /new/i })); });
    expect(await screen.findByText("boom")).toBeTruthy();
  });

  it("renaming another conversation keeps the active one", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue(two);
    const get = vi.spyOn(api, "getConversation").mockImplementation(async (id) => conv(id, id));
    vi.spyOn(api, "renameConversation").mockResolvedValue(conv("bbbbbbbbbbbb", "Gamma"));
    await ready();
    await vi.waitFor(() => expect(activeIds()).toEqual(["aaaaaaaaaaaa"]));
    get.mockClear();
    await act(async () => { fireEvent.doubleClick(screen.getByText("Beta")); });
    fireEvent.change(screen.getByDisplayValue("Beta"), { target: { value: "Gamma" } });
    await act(async () => { fireEvent.keyDown(screen.getByDisplayValue("Gamma"), { key: "Enter" }); });
    expect(await screen.findByText("Gamma")).toBeTruthy();
    expect(activeIds()).toEqual(["aaaaaaaaaaaa"]);
    expect(get).not.toHaveBeenCalled();
  });
});

it("a stream keeps writing into the conversation it started in", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }, { id: "bbbbbbbbbbbb", title: "B" }]);
  vi.spyOn(api, "getConversation").mockImplementation(async (id) => conv(id, id === "aaaaaaaaaaaa" ? "A" : "B"));
  let finish!: () => void;
  const run = vi.spyOn(api, "runConversation").mockImplementation(async (_id, _t, on) => {
    on({ kind: "text", text: "streaming for A" });
    await new Promise<void>((r) => { finish = r; });
  });
  await ready();
  await screen.findByText("A");
  fireEvent.change(screen.getByLabelText(/describe a task/i), { target: { value: "go" } });
  await act(async () => { fireEvent.keyDown(screen.getByLabelText(/describe a task/i), { key: "Enter" }); });
  expect(run.mock.calls[0][0]).toBe("aaaaaaaaaaaa");
  expect(screen.getByText("streaming for A")).toBeTruthy();
  expect((screen.getByText("B") as HTMLButtonElement).disabled).toBe(true);   // switching is locked while running
  await act(async () => { finish(); });
});
