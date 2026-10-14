import { describe, expect, it } from "vitest";
import type { ProgramEvent } from "../api/program";
import { applyEvent, emptyClaude } from "./chat";

const fold = (es: ProgramEvent[]) => es.reduce(applyEvent, emptyClaude());

describe("chat fold", () => {
  it("builds items, attaches run results and shows the final answer once", () => {
    const m = fold([
      { kind: "text", text: "Planning" }, { kind: "file", path: "programs/task_001.py" },
      { kind: "file", path: "programs/task_001.py" },
      { kind: "run", n: 1 }, { kind: "run_result", ok: false, message: "failed: X", output: "log1" },
      { kind: "text", text: "Done." },
      { kind: "done", summary: "Done.", runs: 1, ok: false, session_id: "s", program: "programs/task_001.py", code: null },
    ]);
    expect(m.items).toEqual([{ kind: "text", text: "Planning" }, { kind: "file", path: "programs/task_001.py" },
      { kind: "run", n: 1, ok: false, message: "failed: X", output: "log1" }]);
    expect(m).toMatchObject({ summary: "Done.", result: { ok: false, runs: 1 }, program: "programs/task_001.py", error: null });
  });
  it("records errors and does not mutate its input", () => {
    const start = emptyClaude();
    const m = applyEvent(start, { kind: "error", message: "boom" });
    expect(m.error).toBe("boom");
    expect(start.error).toBeNull();
  });
});
