import type { ProgramEvent } from "../api/program";

export type Item =
  | { kind: "text"; text: string }
  | { kind: "file"; path: string }
  | { kind: "run"; n: number; ok?: boolean; message?: string; output?: string };
export interface ClaudeMessage {
  role: "claude"; items: Item[]; summary: string; result: { ok: boolean; runs: number } | null; error: string | null; program: string | null;
}
export type Message = { role: "user"; text: string } | ClaudeMessage | { role: "run"; path: string; ok: boolean; output: string };

export const emptyClaude = (): ClaudeMessage => ({ role: "claude", items: [], summary: "", result: null, error: null, program: null });

/** One agent event folded into the Claude message (mirror of backend conversations.fold). */
export function applyEvent(m: ClaudeMessage, e: ProgramEvent): ClaudeMessage {
  const items = [...m.items];
  switch (e.kind) {
    case "text": return { ...m, items: [...items, { kind: "text", text: e.text }] };
    case "file": {
      const last = items[items.length - 1];
      return last?.kind === "file" && last.path === e.path ? m : { ...m, items: [...items, { kind: "file", path: e.path }] };
    }
    case "run": return { ...m, items: [...items, { kind: "run", n: e.n }] };
    case "run_result": {
      for (let i = items.length - 1; i >= 0; i--) {
        const it = items[i];
        if (it.kind === "run") { items[i] = { ...it, ok: e.ok, message: e.message, output: e.output ?? "" }; break; }
      }
      return { ...m, items };
    }
    case "done": {
      while (items.length) {
        const last = items[items.length - 1];
        if (last.kind !== "text" || last.text.trim() !== e.summary.trim()) break;
        items.pop();
      }
      return { ...m, items, summary: e.summary, result: { ok: e.ok, runs: e.runs }, program: e.program };
    }
    case "error": return { ...m, error: e.message };
  }
}
