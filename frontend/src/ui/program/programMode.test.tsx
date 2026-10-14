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
vi.mock("./Editor", () => ({ default: ({ value }: { value: string }) => <textarea data-testid="editor" value={value} readOnly /> }));
beforeEach(() => { POLL.starting = 10; POLL.ready = 10_000; });
afterEach(() => vi.restoreAllMocks());

const conv = (id: string, title: string) => ({ id, title, created: 1, updated: 1, claude_session_id: null, edited: [], messages: [] });
function ui() {
  return render(<Harness start={imported([boxBody("base", [0, 0, 0], [1, 1, 1])])} step="program"
    ui={(ctx) => <ProgramMode ctx={ctx} preview={<div data-testid="viewer" />} />} />);
}
async function ready() {
  vi.spyOn(api, "sessionStatus").mockResolvedValueOnce({ state: "stopped" }).mockResolvedValue({ state: "ready", robot: "bot" });  // first call: the mount refresh
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
    vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, id === "aaaaaaaaaaaa" ? "Square" : "New conversation"));
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
    expect(rename).toHaveBeenCalledWith("bot", "aaaaaaaaaaaa", "Square path");
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
    let deleted = false;
    vi.spyOn(api, "listConversations").mockImplementation(async () => (deleted ? [two[1]] : two));
    const get = vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, id));
    vi.spyOn(api, "deleteConversation").mockImplementation(async () => { deleted = true; });
    await ready();
    await vi.waitFor(() => expect(activeIds()).toEqual(["aaaaaaaaaaaa"]));
    await act(async () => { fireEvent.click(screen.getByLabelText("Delete Alpha")); });
    await vi.waitFor(() => expect(activeIds()).toEqual(["bbbbbbbbbbbb"]));
    expect(get).not.toHaveBeenLastCalledWith("bot", "aaaaaaaaaaaa");
    expect(document.querySelector(".error")).toBeNull();
  });

  it("a slow earlier switch cannot override a later one", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue(two);
    const slow: Record<string, (c: ReturnType<typeof conv>) => void> = {};
    vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, id));
    await ready();
    await vi.waitFor(() => expect(activeIds()).toEqual(["aaaaaaaaaaaa"]));
    vi.spyOn(api, "getConversation").mockImplementation((_r, id) => new Promise((r) => { slow[id] = r; }));
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
    const get = vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, id));
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
  vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, id === "aaaaaaaaaaaa" ? "A" : "B"));
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
  expect((screen.getByText("B") as HTMLButtonElement).disabled).toBe(false);  // other chats stay open to browse
  await act(async () => { finish(); });
});

it("shows ROS ready and the conversations on mount when a session is already running", async () => {
  vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "ready", robot: "bot" });
  const start = vi.spyOn(api, "startSession");
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "Existing" }]);
  vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, "Existing"));
  ui();
  expect(await screen.findByText(/ROS ready/)).toBeTruthy();
  expect(await screen.findByText("Existing")).toBeTruthy();
  expect(start).not.toHaveBeenCalled();
});

it("disables Back to robot while a task runs", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
  vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, "A"));
  let finish!: () => void;
  vi.spyOn(api, "runConversation").mockImplementation(async () => { await new Promise<void>((r) => { finish = r; }); });
  await ready();
  await screen.findByText("A");
  fireEvent.change(screen.getByLabelText(/describe a task/i), { target: { value: "go" } });
  await act(async () => { fireEvent.keyDown(screen.getByLabelText(/describe a task/i), { key: "Enter" }); });
  expect((screen.getByRole("button", { name: /back to robot/i }) as HTMLButtonElement).disabled).toBe(true);
  await act(async () => { finish(); });
});

it("clears the top error when a new task is sent", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
  vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, "A"));
  vi.spyOn(api, "createConversation").mockRejectedValue(new Error("old failure"));
  vi.spyOn(api, "runConversation").mockImplementation(async () => { await new Promise<void>(() => {}); });
  await ready();
  await screen.findByText("A");
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /new/i })); });
  expect(await screen.findByText("old failure")).toBeTruthy();
  fireEvent.change(screen.getByLabelText(/describe a task/i), { target: { value: "go" } });
  await act(async () => { fireEvent.keyDown(screen.getByLabelText(/describe a task/i), { key: "Enter" }); });
  expect(screen.queryByText("old failure")).toBeNull();
});

it("locks the chat and Run while a manual run is pending", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
  vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, "A"));
  vi.spyOn(api, "listFiles").mockResolvedValue([{ path: "bot_tasks/programs/task_001.py", editable: true }]);
  vi.spyOn(api, "readFile").mockImplementation(async (p) => ({ path: p, content: "x", editable: true }));
  let done!: (v: { ok: boolean; output: string }) => void;
  vi.spyOn(api, "runFile").mockImplementation(() => new Promise((r) => { done = r; }));
  await ready();
  await screen.findByText("A");
  await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
  await act(async () => { fireEvent.click(await screen.findByText("task_001.py")); });
  const box = () => screen.getByLabelText(/describe a task/i) as HTMLTextAreaElement;
  const runBtn = () => screen.getByRole("button", { name: /^run$/i }) as HTMLButtonElement;
  expect(box().disabled).toBe(false);
  await act(async () => { fireEvent.click(runBtn()); });
  expect(box().disabled).toBe(true);
  expect(runBtn().disabled).toBe(true);
  await act(async () => { done({ ok: true, output: "" }); });
  expect(box().disabled).toBe(false);
  expect(runBtn().disabled).toBe(false);
});

it("Run again runs that program live in Preview and locks the UI while it runs", async () => {
  const withProgram = { ...conv("aaaaaaaaaaaa", "A"), messages: [{ role: "claude", items: [], summary: "done", result: { ok: true, runs: 1 },
    error: null, program: "programs/task_001.py" }] };
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
  vi.spyOn(api, "getConversation").mockResolvedValue(withProgram as never);
  vi.spyOn(api, "listFiles").mockResolvedValue([]);
  let finish!: () => void;
  const run = vi.spyOn(api, "runFile").mockImplementation(() => new Promise((r) => { finish = () => r({ ok: true, output: "RESULT: ok" }); }));
  await ready();
  await screen.findByText("A");
  fireEvent.click(screen.getByRole("tab", { name: "Code" }));
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /run again/i })); });
  expect(run).toHaveBeenCalledWith("bot_tasks/programs/task_001.py", "aaaaaaaaaaaa");
  expect(screen.getByRole("tab", { name: "Preview" }).getAttribute("aria-selected")).toBe("true");
  expect((screen.getByLabelText(/describe a task/i) as HTMLTextAreaElement).disabled).toBe(true);
  await act(async () => { finish(); });
  expect((screen.getByLabelText(/describe a task/i) as HTMLTextAreaElement).disabled).toBe(false);
});


it("the Preview tab has a Run bar for the conversation's latest program", async () => {
  const withProgram = { ...conv("aaaaaaaaaaaa", "A"), messages: [
    { role: "claude", items: [], summary: "", result: { ok: true, runs: 1 }, error: null, program: "programs/task_001.py" },
    { role: "claude", items: [], summary: "", result: { ok: true, runs: 1 }, error: null, program: "programs/task_002.py" }] };
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
  vi.spyOn(api, "getConversation").mockResolvedValue(withProgram as never);
  const run = vi.spyOn(api, "runFile").mockResolvedValue({ ok: true, output: "RESULT: ok" });
  await ready();
  await screen.findByText("A");
  const bar = screen.getByTestId("preview-run");
  expect(bar.textContent).toMatch(/task_002\.py/);
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /^run$/i })); });
  expect(run).toHaveBeenCalledWith("bot_tasks/programs/task_002.py", "aaaaaaaaaaaa");
});

it("no Run bar when the conversation has no program yet", async () => {
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
  vi.spyOn(api, "getConversation").mockResolvedValue(conv("aaaaaaaaaaaa", "A") as never);
  await ready();
  await screen.findByText("A");
  expect(screen.queryByTestId("preview-run")).toBeNull();
});

it("the Preview Run bar has a dropdown of all programs, defaulting to the latest", async () => {
  const withProgram = { ...conv("aaaaaaaaaaaa", "A"), messages: [
    { role: "claude", items: [], summary: "", result: { ok: true, runs: 1 }, error: null, program: "programs/task_002.py" }] };
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
  vi.spyOn(api, "getConversation").mockResolvedValue(withProgram as never);
  vi.spyOn(api, "listFiles").mockResolvedValue([
    { path: "bot_program/robot.py", editable: false },
    { path: "bot_tasks/programs/task_001.py", editable: true },
    { path: "bot_tasks/programs/task_002.py", editable: true }]);
  const run = vi.spyOn(api, "runFile").mockResolvedValue({ ok: true, output: "RESULT: ok" });
  await ready();
  await screen.findByText("A");
  const pick = await screen.findByRole("combobox", { name: /program to run/i }) as HTMLSelectElement;
  expect(pick.value).toBe("bot_tasks/programs/task_002.py");
  fireEvent.change(pick, { target: { value: "__all__" } });
  expect([...pick.options].map((o) => o.textContent)).toEqual(["task_001.py", "task_002.py"]);
  fireEvent.change(pick, { target: { value: "bot_tasks/programs/task_001.py" } });
  await act(async () => { fireEvent.click(screen.getByRole("button", { name: /^run$/i })); });
  expect(run).toHaveBeenCalledWith("bot_tasks/programs/task_001.py", "aaaaaaaaaaaa");
});

it("the dropdown lists this conversation's programs, with All programs to see the rest", async () => {
  const withProgram = { ...conv("aaaaaaaaaaaa", "A"), messages: [
    { role: "claude", items: [{ kind: "file", path: "programs/task_002_stack.py" }], summary: "", result: { ok: true, runs: 1 },
      error: null, program: "programs/task_002_stack.py" }] };
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
  vi.spyOn(api, "getConversation").mockResolvedValue(withProgram as never);
  vi.spyOn(api, "listFiles").mockResolvedValue([
    { path: "bot_tasks/programs/task_001_old.py", editable: true },
    { path: "bot_tasks/programs/task_002_stack.py", editable: true }]);
  await ready();
  await screen.findByText("A");
  const pick = await screen.findByRole("combobox", { name: /program to run/i }) as HTMLSelectElement;
  expect([...pick.options].map((o) => o.textContent)).toEqual(["task_002_stack.py", "All programs…"]);
  fireEvent.change(pick, { target: { value: "__all__" } });
  const all = screen.getByRole("combobox", { name: /program to run/i }) as HTMLSelectElement;
  expect([...all.options].map((o) => o.textContent)).toEqual(["task_001_old.py", "task_002_stack.py"]);
  expect(all.value).toBe("bot_tasks/programs/task_002_stack.py");
});

it("a deleted latest program is not offered; the dropdown falls back to one that exists", async () => {
  const c = { ...conv("aaaaaaaaaaaa", "A"), messages: [
    { role: "claude", items: [], summary: "", result: { ok: true, runs: 1 }, error: null, program: "programs/task_001_a.py" },
    { role: "claude", items: [], summary: "", result: { ok: true, runs: 1 }, error: null, program: "programs/task_002_b.py" }] };
  vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
  vi.spyOn(api, "getConversation").mockResolvedValue(c as never);
  vi.spyOn(api, "listFiles").mockResolvedValue([{ path: "bot_tasks/programs/task_001_a.py", editable: true }]);
  await ready();
  await screen.findByText("A");
  const pick = await screen.findByRole("combobox", { name: /program to run/i }) as HTMLSelectElement;
  await act(async () => {});
  expect([...pick.options].map((o) => o.textContent)).toEqual(["task_001_a.py", "All programs…"]);
  expect(pick.value).toBe("bot_tasks/programs/task_001_a.py");
});

describe("ROS startup progress", () => {
  it("shows the startup stage in the header and the checklist on click", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([]);
    vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "starting", robot: "bot", stage: "collisions", done: 4200, total: 10000 });
    ui();
    const pill = await screen.findByRole("button", { name: /Checking collisions 4,200 \/ 10,000/ });
    fireEvent.click(pill);
    const panel = screen.getByTestId("startup-panel");
    expect(panel.querySelector('[data-stage="export"]')!.className).toMatch(/done/);
    expect(panel.querySelector('[data-stage="collisions"]')!.className).toMatch(/current/);
    expect(panel.querySelector('[data-stage="build"]')!.className).toMatch(/todo/);
  });

  it("opens the checklist by itself on failure, marking the stage that failed", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([]);
    vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "failed", robot: "bot", stage: "image", error: "docker build failed" });
    ui();
    const panel = await screen.findByTestId("startup-panel");
    expect(panel.querySelector('[data-stage="image"]')!.className).toMatch(/failed/);
    expect(panel.textContent).toMatch(/docker build failed/);
  });

  it("never has two status requests in flight", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([]);
    let inFlight = 0, most = 0, calls = 0;
    vi.spyOn(api, "sessionStatus").mockImplementation(async () => {
      calls++; inFlight++; most = Math.max(most, inFlight);
      await new Promise((r) => setTimeout(r, 40));  // slower than POLL.starting (10 ms)
      inFlight--;
      return { state: "starting", robot: "bot", stage: "build" };
    });
    ui();
    await vi.waitFor(() => expect(calls).toBeGreaterThanOrEqual(4), { timeout: 3000 });
    expect(most).toBe(1);
  });
});

describe("Program mode unlocked", () => {
  it("lists and creates conversations while ROS is stopped", async () => {
    vi.spyOn(api, "sessionStatus").mockResolvedValue({ state: "stopped" });
    const list = vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "Square" }]);
    vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, "Square"));
    const create = vi.spyOn(api, "createConversation").mockResolvedValue(conv("bbbbbbbbbbbb", "New conversation"));
    ui();
    await screen.findByText("Square");
    expect(list).toHaveBeenCalledWith("robot");
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /new/i })); });
    expect(create).toHaveBeenCalledWith("robot");
    expect(screen.getByRole("textbox", { name: /describe a task/i })).toHaveProperty("disabled", true);
    expect(screen.getByText(/Start ROS to send a task/)).toBeTruthy();
  });

  it("while a task runs: other chats can be opened, the running one keeps its live stream", async () => {
    const two = [{ id: "aaaaaaaaaaaa", title: "Alpha" }, { id: "bbbbbbbbbbbb", title: "Beta" }];
    vi.spyOn(api, "listConversations").mockResolvedValue(two);
    vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, id === "aaaaaaaaaaaa" ? "Alpha" : "Beta"));
    let push!: (e: api.ProgramEvent) => void;
    let end!: () => void;
    vi.spyOn(api, "runConversation").mockImplementation((_id, _t, onEvent) => { push = onEvent; return new Promise((r) => { end = () => r(); }); });
    await ready();
    await screen.findByText("Alpha");
    fireEvent.change(screen.getByRole("textbox", { name: /describe a task/i }), { target: { value: "go" } });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /send/i })); });
    await act(async () => { push({ kind: "text", text: "Working on Alpha" }); });
    await act(async () => { fireEvent.click(screen.getByText("Beta")); });
    expect(screen.getByTestId("conv-bbbbbbbbbbbb").className).toMatch(/active/);
    expect(screen.queryByText("Working on Alpha")).toBeNull();
    expect(screen.getByText(/Waiting for task in .Alpha./)).toBeTruthy();
    expect(screen.getByTestId("conv-aaaaaaaaaaaa").querySelector(".running-dot")).toBeTruthy();
    expect(screen.queryByLabelText("Delete Alpha")).toBeNull();
    await act(async () => { fireEvent.click(screen.getByText("Alpha")); });
    expect(screen.getByText("Working on Alpha")).toBeTruthy();
    await act(async () => { fireEvent.click(screen.getByText("Beta")); });
    await act(async () => { end(); });
    expect(screen.getByTestId("conv-bbbbbbbbbbbb").className).toMatch(/active/);
  });

  it("Back to robot says why it is disabled during a run", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([]);
    vi.spyOn(api, "createConversation").mockResolvedValue(conv("aaaaaaaaaaaa", "New conversation"));
    vi.spyOn(api, "runConversation").mockImplementation(() => new Promise(() => {}));
    await ready();
    fireEvent.change(screen.getByRole("textbox", { name: /describe a task/i }), { target: { value: "go" } });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /send/i })); });
    const back = screen.getByRole("button", { name: /back to robot/i });
    expect(back).toHaveProperty("disabled", true);
    expect(back.getAttribute("title")).toMatch(/task is running/i);
  });
});

describe("review fixes", () => {
  const withProgram = (id: string, title: string) => ({ ...conv(id, title), messages: [{ role: "claude", items: [], summary: "done",
    result: { ok: true, runs: 1 }, error: null, program: "programs/task_001.py" }] });

  it("a Run again that ends while another chat is open does not jump back", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }, { id: "bbbbbbbbbbbb", title: "B" }]);
    vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => withProgram(id, id === "aaaaaaaaaaaa" ? "A" : "B") as never);
    vi.spyOn(api, "listFiles").mockResolvedValue([{ path: "bot_tasks/programs/task_001.py", editable: true }]);
    let finish!: () => void;
    vi.spyOn(api, "runFile").mockImplementation(() => new Promise((r) => { finish = () => r({ ok: true, output: "RESULT: ok" }); }));
    await ready();
    await screen.findByText("A");
    await act(async () => { fireEvent.click(screen.getAllByRole("button", { name: /run again/i })[0]); });
    expect(screen.getByLabelText("Program to run").getAttribute("title")).toMatch(/task is running/i);  // #12
    await act(async () => { fireEvent.click(screen.getByText("B")); });
    await act(async () => { finish(); });
    expect(screen.getByTestId("conv-bbbbbbbbbbbb").className).toMatch(/active/);
  });

  it("the streamed answer stays on screen until the saved one has loaded", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([{ id: "aaaaaaaaaaaa", title: "A" }]);
    const get = vi.spyOn(api, "getConversation").mockImplementation(async (_r, id) => conv(id, "A"));
    let end!: () => void;
    vi.spyOn(api, "runConversation").mockImplementation(async (_id, _t, on) => {
      on({ kind: "text", text: "the streamed answer" });
      await new Promise<void>((r) => { end = r; });
    });
    await ready();
    await screen.findByText("A");
    fireEvent.change(screen.getByLabelText(/describe a task/i), { target: { value: "go" } });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /send/i })); });
    get.mockImplementation(() => new Promise(() => {}));  // the reload after the run never returns
    await act(async () => { end(); });
    expect(screen.getByText("the streamed answer")).toBeTruthy();
  });

  it("a Code-tab run with no chat open still locks Back to robot", async () => {
    vi.spyOn(api, "listConversations").mockResolvedValue([]);
    vi.spyOn(api, "listFiles").mockResolvedValue([{ path: "bot_tasks/programs/task_001.py", editable: true }]);
    vi.spyOn(api, "readFile").mockImplementation(async (p) => ({ path: p, content: "x", editable: true }));
    vi.spyOn(api, "runFile").mockImplementation(() => new Promise(() => {}));
    await ready();
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
    await act(async () => { fireEvent.click(await screen.findByText("task_001.py")); });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /^run$/i })); });
    expect(screen.getByRole("button", { name: /back to robot/i })).toHaveProperty("disabled", true);
  });
});
