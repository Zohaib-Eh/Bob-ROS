import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import type { Message } from "../../model/chat";
import Chat from "./Chat";

const msgs: Message[] = [
  { role: "user", text: "Move the tool" },
  { role: "claude", items: [{ kind: "text", text: "Planning **carefully**" }, { kind: "file", path: "programs/task_001.py" },
    { kind: "run", n: 1, ok: false, message: "failed: Unreachable: too far", output: "trace...\nRESULT: failed" },
    { kind: "run", n: 2, ok: true, message: "ok", output: "RESULT: ok" }],
    summary: "**Moved** the tool.\n- run 2 ok", result: { ok: true, runs: 2 }, error: null, program: "programs/task_001.py" },
  { role: "run", path: "bot_tasks/programs/task_001.py", ok: true, output: "RESULT: ok" },
];

describe("Chat", () => {
  it("renders bubbles, markdown, steps and one result line", () => {
    render(<Chat messages={msgs} live={null} running={false} disabled={false} onSend={() => {}} />);
    expect(screen.getByText("Move the tool").closest(".bubble")).toBeTruthy();
    expect(screen.getByText("carefully").tagName).toBe("STRONG");
    expect(screen.getByText("Moved").tagName).toBe("STRONG");
    expect(screen.getAllByText(/Moved/)).toHaveLength(1);
    expect(screen.getByText(/Wrote programs\/task_001\.py/)).toBeTruthy();
    expect(screen.getByText(/Run 1 ✗ failed: Unreachable: too far/)).toBeTruthy();
    expect(screen.getByText(/Worked on run 2\/3/)).toBeTruthy();
    expect(screen.getByText(/You ran bot_tasks\/programs\/task_001\.py ✓/)).toBeTruthy();
  });

  it("expands a run to its full output", () => {
    render(<Chat messages={msgs} live={null} running={false} disabled={false} onSend={() => {}} />);
    expect(screen.queryByText(/trace\.\.\./)).toBeNull();
    fireEvent.click(screen.getByText(/Run 1 ✗/));
    expect(screen.getByText(/trace\.\.\./)).toBeTruthy();
  });

  it("expands a manual run to its output", () => {
    render(<Chat messages={msgs} live={null} running={false} disabled={false} onSend={() => {}} />);
    expect(screen.queryByText("RESULT: ok", { selector: "pre" })).toBeNull();
    fireEvent.click(screen.getByText(/You ran/));
    expect(screen.getByText("RESULT: ok", { selector: "pre" })).toBeTruthy();
  });

  it("sends with Enter, keeps Shift+Enter for new lines, and is disabled while running", () => {
    const onSend = vi.fn();
    const { rerender } = render(<Chat messages={[]} live={null} running={false} disabled={false} onSend={onSend} />);
    const box = screen.getByLabelText(/describe a task/i);
    fireEvent.change(box, { target: { value: "go" } });
    fireEvent.keyDown(box, { key: "Enter", shiftKey: true });
    expect(onSend).not.toHaveBeenCalled();
    fireEvent.keyDown(box, { key: "Enter" });
    expect(onSend).toHaveBeenCalledWith("go");
    rerender(<Chat messages={[]} live={{ role: "claude", items: [], summary: "", result: null, error: null, program: null }}
      running={true} disabled={false} onSend={onSend} />);
    expect((screen.getByLabelText(/describe a task/i) as HTMLTextAreaElement).disabled).toBe(true);
    expect(screen.getByText(/Working/)).toBeTruthy();
  });
});
