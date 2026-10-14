import { ChevronDown, ChevronRight, FileCode, Loader2, Play, Send } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import Markdown from "react-markdown";
import type { ClaudeMessage, Item, Message } from "../../model/chat";

const MAX_RUNS = 3;

function RunOutput({ head, output, cls }: { head: string; output?: string; cls: string }) {
  const [open, setOpen] = useState(false);
  return (
    <div className={`step run ${cls}`}>
      <button className="step-head" onClick={() => setOpen((o) => !o)}>{open ? <ChevronDown size={12} /> : <ChevronRight size={12} />}{head}</button>
      {open && output && <pre className="step-output">{output}</pre>}
    </div>
  );
}

function Step({ item }: { item: Item }) {
  if (item.kind === "text") return <div className="narration"><Markdown>{item.text}</Markdown></div>;
  if (item.kind === "file") return <div className="step"><FileCode size={12} />Wrote {item.path}</div>;
  const mark = item.ok === undefined ? "…" : item.ok ? "✓" : "✗";
  return <RunOutput head={`Run ${item.n} ${mark} ${item.message ?? "running"}`} output={item.output} cls={item.ok === false ? "bad" : item.ok ? "ok" : ""} />;
}

/** Re-runs a program live in the 3D view. */
function RunAgain({ path, onRunAgain, disabled }: { path: string; onRunAgain?: (path: string) => void; disabled: boolean }) {
  if (!onRunAgain) return null;
  return <button className="btn run-again" disabled={disabled} onClick={() => onRunAgain(path)}><Play size={12} />Run again</button>;
}

function Claude({ m, live, onRunAgain, busy }: { m: ClaudeMessage; live: boolean; onRunAgain?: (path: string) => void; busy: boolean }) {
  return (
    <div className="claude-msg">
      {m.items.map((it, i) => <Step key={i} item={it} />)}
      {m.summary && <div className="answer"><Markdown>{m.summary}</Markdown></div>}
      {m.result && <div className={`result ${m.result.ok ? "ok" : "bad"}`}>
        {m.result.ok ? `✓ Worked on run ${m.result.runs}/${MAX_RUNS}` : `✗ Failed after ${m.result.runs} run${m.result.runs === 1 ? "" : "s"}`}
      </div>}
      {m.error && <div className="error">{m.error}</div>}
      {!live && m.result && m.program && <RunAgain path={m.program} onRunAgain={onRunAgain} disabled={busy} />}
      {live && !m.result && !m.error && <div className="muted"><Loader2 size={12} className="spin" /> Working…</div>}
    </div>
  );
}

interface Props {
  messages: Message[]; live: ClaudeMessage | null; running: boolean; disabled: boolean; onSend(task: string): void;
  onRunAgain?(path: string): void; waiting?: string;
}

export default function Chat({ messages, live, running, disabled, onSend, onRunAgain, waiting }: Props) {
  const [task, setTask] = useState("");
  const end = useRef<HTMLDivElement>(null);
  useEffect(() => { end.current?.scrollIntoView?.({ block: "end" }); }, [messages, live]);
  const submit = () => { const t = task.trim(); if (t && !running && !disabled) { onSend(t); setTask(""); } };
  return (
    <>
      <div className="chat-scroll">
        {messages.map((m, i) => m.role === "user" ? <div key={i} className="bubble">{m.text}</div>
          : m.role === "run" ? <div key={i}><RunOutput head={`You ran ${m.path} ${m.ok ? "✓" : "✗"}`} output={m.output} cls={m.ok ? "ok" : "bad"} />
              <RunAgain path={m.path} onRunAgain={onRunAgain} disabled={running || disabled} /></div>
          : <Claude key={i} m={m} live={false} onRunAgain={onRunAgain} busy={running || disabled} />)}
        {live && <Claude m={live} live={true} busy={true} />}
        <div ref={end} />
      </div>
      {waiting && <p className="muted chat-waiting">{waiting}</p>}
      <div className="prompt">
        <textarea aria-label="Describe a task" value={task} disabled={running || disabled} rows={3}
          placeholder="Describe a task, e.g. pick the box at (0.5, 0.2) and place it at (0.4, -0.3)"
          onChange={(e) => setTask(e.target.value)}
          onKeyDown={(e) => { if (e.key === "Enter" && !e.shiftKey) { e.preventDefault(); submit(); } }} />
        <button className="btn btn-primary" onClick={submit} disabled={running || disabled || !task.trim()}><Send size={14} />Send</button>
      </div>
    </>
  );
}
