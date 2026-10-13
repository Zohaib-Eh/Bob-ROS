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
function ui(running = false, onRan = vi.fn()) {
  vi.spyOn(api, "listFiles").mockResolvedValue(files);
  vi.spyOn(api, "readFile").mockImplementation(async (p) => ({ path: p, content: `# ${p}`, editable: p.includes("programs") }));
  return render(<Workspace preview={<div data-testid="viewer" />} conversation="aaaaaaaaaaaa" running={running}
    changed={new Set(["bot_tasks/programs/task_001.py"])} onRan={onRan} />);
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
    expect(onRan).toHaveBeenCalled();
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
});
