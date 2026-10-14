import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as api from "../../api/program";
import Workspace from "./Workspace";

vi.mock("./Editor", () => ({
  default: ({ value, readOnly, onChange }: { value: string; readOnly: boolean; onChange(v: string): void }) =>
    <textarea data-testid="editor" value={value} readOnly={readOnly} onChange={(e) => onChange(e.target.value)} />,
}));
afterEach(() => vi.restoreAllMocks());

const files = [{ path: "bot_program/robot.py", editable: false }, { path: "bot_tasks/programs/task_001.py", editable: true }];
function ui(running = false, onRan = vi.fn(), onRunning = vi.fn()) {
  vi.spyOn(api, "listFiles").mockResolvedValue(files);
  vi.spyOn(api, "readFile").mockImplementation(async (p) => ({ path: p, content: `# ${p}`, editable: p.includes("programs") }));
  return render(<Workspace preview={<div data-testid="viewer" />} conversation="aaaaaaaaaaaa" running={running}
    changed={new Set(["bot_tasks/programs/task_001.py"])} onRan={onRan} onRunning={onRunning} />);
}

describe("Workspace", () => {
  it("switches between Preview and Code and keeps the viewer mounted", async () => {
    ui();
    expect(screen.getByTestId("viewer")).toBeTruthy();
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
    expect(screen.getByTestId("viewer")).toBeTruthy();          // hidden, not unmounted
    expect(await screen.findByText("task_001.py")).toBeTruthy();
    expect(screen.getByText("task_001.py").closest("li")!.className).toMatch(/changed/);
  });

  it("opens a program, saves it with its conversation, then runs it", async () => {
    const save = vi.spyOn(api, "saveFile").mockResolvedValue();
    const run = vi.spyOn(api, "runFile").mockResolvedValue({ ok: true, output: "RESULT: ok" });
    const onRan = vi.fn();
    ui(false, onRan);
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
    await act(async () => { fireEvent.click(await screen.findByText("task_001.py")); });
    fireEvent.change(screen.getByTestId("editor"), { target: { value: "print(2)" } });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /save/i })); });
    expect(save).toHaveBeenCalledWith("bot_tasks/programs/task_001.py", "print(2)", "aaaaaaaaaaaa");
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /^run$/i })); });
    expect(run).toHaveBeenCalledWith("bot_tasks/programs/task_001.py", "aaaaaaaaaaaa");
    expect(onRan).toHaveBeenCalledWith("aaaaaaaaaaaa");
  });

  it("library files are read-only and nothing can be saved while a task runs", async () => {
    ui(true);
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
    await act(async () => { fireEvent.click(await screen.findByText("robot.py")); });
    expect((screen.getByTestId("editor") as HTMLTextAreaElement).readOnly).toBe(true);
    expect(screen.queryByRole("button", { name: /save/i })).toBeNull();
    await act(async () => { fireEvent.click(screen.getByText("task_001.py")); });
    expect((screen.getByRole("button", { name: /save/i }) as HTMLButtonElement).disabled).toBe(true);
  });

  async function openCode() {
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
  }

  it("asks before discarding unsaved edits when switching files", async () => {
    vi.spyOn(api, "listFiles").mockResolvedValue([...files, { path: "bot_tasks/programs/task_002.py", editable: true }]);
    vi.spyOn(api, "readFile").mockImplementation(async (p) => ({ path: p, content: `# ${p}`, editable: true }));
    render(<Workspace preview={<div />} conversation="aaaaaaaaaaaa" running={false} changed={new Set()} onRan={vi.fn()} onRunning={vi.fn()} />);
    await openCode();
    await act(async () => { fireEvent.click(await screen.findByText("task_001.py")); });
    fireEvent.change(screen.getByTestId("editor"), { target: { value: "edited" } });
    const confirm = vi.spyOn(window, "confirm").mockReturnValue(false);
    await act(async () => { fireEvent.click(screen.getByText("task_002.py")); });
    expect(confirm).toHaveBeenCalledWith("Discard unsaved changes to task_001.py?");
    expect((screen.getByTestId("editor") as HTMLTextAreaElement).value).toBe("edited");
    confirm.mockReturnValue(true);
    await act(async () => { fireEvent.click(screen.getByText("task_002.py")); });
    expect((screen.getByTestId("editor") as HTMLTextAreaElement).value).toBe("# bot_tasks/programs/task_002.py");
  });

  it("a slow earlier open cannot replace a later one", async () => {
    vi.spyOn(api, "listFiles").mockResolvedValue(files);
    let resolveA!: (v: { path: string; content: string; editable: boolean }) => void;
    vi.spyOn(api, "readFile").mockImplementation((p) => p.includes("robot.py")
      ? new Promise((r) => { resolveA = r; })
      : Promise.resolve({ path: p, content: `# ${p}`, editable: true }));
    render(<Workspace preview={<div />} conversation="aaaaaaaaaaaa" running={false} changed={new Set()} onRan={vi.fn()} onRunning={vi.fn()} />);
    await openCode();
    await act(async () => { fireEvent.click(await screen.findByText("robot.py")); });
    await act(async () => { fireEvent.click(screen.getByText("task_001.py")); });
    await act(async () => { resolveA({ path: "bot_program/robot.py", content: "# A", editable: false }); });
    expect((screen.getByTestId("editor") as HTMLTextAreaElement).value).toBe("# bot_tasks/programs/task_001.py");
  });

  it("clears 'Saved' once the buffer changes again", async () => {
    vi.spyOn(api, "saveFile").mockResolvedValue();
    ui();
    await openCode();
    await act(async () => { fireEvent.click(await screen.findByText("task_001.py")); });
    fireEvent.change(screen.getByTestId("editor"), { target: { value: "x" } });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /save/i })); });
    expect(screen.getByText("Saved")).toBeTruthy();
    fireEvent.change(screen.getByTestId("editor"), { target: { value: "xy" } });
    expect(screen.queryByText("Saved")).toBeNull();
  });

  it("shows the error when a run is refused", async () => {
    vi.spyOn(api, "runFile").mockRejectedValue(new Error("A task is already running."));
    ui();
    await openCode();
    await act(async () => { fireEvent.click(await screen.findByText("task_001.py")); });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /^run$/i })); });
    expect(screen.getByText("A task is already running.")).toBeTruthy();
  });

  it("reports running while a manual run is pending and reloads the conversation it started in", async () => {
    let done!: (v: { ok: boolean; output: string }) => void;
    vi.spyOn(api, "runFile").mockImplementation(() => new Promise((r) => { done = r; }));
    const onRan = vi.fn(), onRunning = vi.fn();
    ui(false, onRan, onRunning);
    await openCode();
    await act(async () => { fireEvent.click(await screen.findByText("task_001.py")); });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /^run$/i })); });
    expect(onRunning).toHaveBeenLastCalledWith(true);
    await act(async () => { done({ ok: true, output: "" }); });
    expect(onRunning).toHaveBeenLastCalledWith(false);
    expect(onRan).toHaveBeenCalledWith("aaaaaaaaaaaa");
  });

  it("re-lists files when the conversation changes", async () => {
    const list = vi.spyOn(api, "listFiles").mockRejectedValueOnce(new Error("not ready")).mockResolvedValue(files);
    const props = { preview: <div />, running: false, changed: new Set<string>(), onRan: vi.fn(), onRunning: vi.fn() };
    const { rerender } = render(<Workspace {...props} conversation={null} />);
    await openCode();
    await vi.waitFor(() => expect(list).toHaveBeenCalledTimes(1));
    rerender(<Workspace {...props} conversation="aaaaaaaaaaaa" />);
    expect(await screen.findByText("task_001.py")).toBeTruthy();
    expect(list).toHaveBeenCalledTimes(2);
  });
});

describe("deleting programs", () => {
  it("deletes the open program after a confirm, then closes it and tells the parent", async () => {
    const del = vi.spyOn(api, "deleteFile").mockResolvedValue();
    const onRan = vi.fn();
    ui(false, onRan);
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
    await act(async () => { fireEvent.click(await screen.findByText("task_001.py")); });
    vi.spyOn(window, "confirm").mockReturnValueOnce(false);
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /delete/i })); });
    expect(del).not.toHaveBeenCalled();
    vi.spyOn(window, "confirm").mockReturnValueOnce(true);
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /delete/i })); });
    expect(del).toHaveBeenCalledWith("bot_tasks/programs/task_001.py");
    expect(screen.queryByTestId("editor")).toBeNull();
    expect(onRan).toHaveBeenCalled();
  });
  it("library files have no delete button", async () => {
    ui();
    await act(async () => { fireEvent.click(screen.getByRole("tab", { name: "Code" })); });
    await act(async () => { fireEvent.click(await screen.findByText("robot.py")); });
    expect(screen.queryByRole("button", { name: /delete/i })).toBeNull();
  });
});
