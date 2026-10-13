import { ChevronDown, ChevronRight, FileCode, Loader2, Send } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import Markdown from "react-markdown";
import type { ClaudeMessage, Item, Message } from "../../model/chat";

const MAX_RUNS = 3;

function Step({ item }: { item: Item }) {
  const [open, setOpen] = useState(false);
  if (item.kind === "text") return <div className="narration"><Markdown>{item.text}</Markdown></div>;
  if (item.kind === "file") return <div className="step"><FileCode size={12} />Wrote {item.path}</div>;
  const mark = item.ok === undefined ? "…" : item.ok ? "✓" : "✗";
  return (
    <div className={`step run ${item.ok === false ? "bad" : item.ok ? "ok" : ""}`}>
      <button className="step-head" onClick={() => setOpen((o) => !o)}>
        {open ? <ChevronDown size={12} /> : <ChevronRight size={12} />}Run {item.n} {mark} {item.message ?? "running"}
      </button>
      {open && item.output && <pre className="step-output">{item.output}</pre>}
    </div>
  );
}

function Claude({ m, live }: { m: ClaudeMessage; live: boolean }) {
  return (
    <div className="claude-msg">
      {m.items.map((it, i) => <Step key={i} item={it} />)}
      {m.summary && <div className="answer"><Markdown>{m.summary}</Markdown></div>}
      {m.result && <div className={`result ${m.result.ok ? "ok" : "bad"}`}>
        {m.result.ok ? `✓ Worked on run ${m.result.runs}/${MAX_RUNS}` : `✗ Failed after ${m.result.runs} run${m.result.runs === 1 ? "" : "s"}`}
      </div>}
      {m.error && <div className="error">{m.error}</div>}
      {live && !m.result && !m.error && <div className="muted"><Loader2 size={12} className="spin" /> Working…</div>}
    </div>
  );
}

interface Props { messages: Message[]; live: ClaudeMessage | null; running: boolean; disabled: boolean; onSend(task: string): void }

export default function Chat({ messages, live, running, disabled, onSend }: Props) {
  const [task, setTask] = useState("");
  const end = useRef<HTMLDivElement>(null);
  useEffect(() => { end.current?.scrollIntoView?.({ block: "end" }); }, [messages, live]);
  const submit = () => { const t = task.trim(); if (t && !running && !disabled) { onSend(t); setTask(""); } };
  return (
    <>
      <div className="chat-scroll">
        {messages.map((m, i) => m.role === "user" ? <div key={i} className="bubble">{m.text}</div>
          : m.role === "run" ? <div key={i} className={`step ${m.ok ? "ok" : "bad"}`}>You ran {m.path} {m.ok ? "✓" : "✗"}</div>
          : <Claude key={i} m={m} live={false} />)}
        {live && <Claude m={live} live={true} />}
        <div ref={end} />
      </div>
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
