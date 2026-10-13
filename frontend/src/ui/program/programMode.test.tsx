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
