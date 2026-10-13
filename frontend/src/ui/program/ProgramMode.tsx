import { ArrowLeft, Play, RotateCcw, Square } from "lucide-react";
import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import * as api from "../../api/program";
import type { Conversation, ConversationInfo } from "../../api/program";
import type { Ctx } from "../ctx";
import { applyEvent, emptyClaude, type ClaudeMessage } from "../../model/chat";
import Chat from "./Chat";
import ConversationList from "./ConversationList";
import Workspace from "./Workspace";
import { useRosSession } from "./useRosSession";

export default function ProgramMode({ ctx, preview }: { ctx: Ctx; preview: ReactNode }) {
  const ros = useRosSession(ctx);
  const { status } = ros;
  const [convs, setConvs] = useState<ConversationInfo[]>([]);
  const [active, setActive] = useState<Conversation | null>(null);
  const [running, setRunning] = useState(false);
  const [live, setLive] = useState<ClaudeMessage | null>(null);
  const abort = useRef<AbortController | null>(null);
  const known = status.state === "ready" || status.state === "starting";

  const activeId = useRef<string | null>(null);
  const req = useRef(0);
  const mounted = useRef(true);
  useEffect(() => { activeId.current = active?.id ?? null; }, [active]);
  useEffect(() => { mounted.current = true; return () => { mounted.current = false; abort.current?.abort(); }; }, []);
  const fail = (e: unknown) => ros.setError((e as Error).message);

  // select: id = switch to it, null = first non-broken of the fresh list, undefined = keep current
  async function reload(select?: string | null) {
    ros.setError(null);
    const token = ++req.current;
    try {
      const list = await api.listConversations();
      if (token !== req.current || !mounted.current) return;
      setConvs(list);
      const id = select === null ? list.find((c) => !c.broken)?.id : select ?? activeId.current ?? list.find((c) => !c.broken)?.id;
      if (!id) { if (select === null) setActive(null); return; }
      const c = await api.getConversation(id);
      if (token === req.current && mounted.current) setActive(c);
    } catch (e) { if (token === req.current && mounted.current) fail(e); }
  }
  useEffect(() => { if (known) void reload(); }, [known]);  // eslint-disable-line react-hooks/exhaustive-deps

  async function newConversation() {
    try {
      const c = await api.createConversation();
      req.current++;
      setActive(c);
      setConvs((cs) => [{ id: c.id, title: c.title, created: c.created, updated: c.updated }, ...cs]);
    } catch (e) { fail(e); }
  }

  async function send(task: string) {
    let c = active;
    if (!c) {
      try {
        c = await api.createConversation();
        if (!mounted.current) return;
        req.current++;
        setActive(c);
        const n = c;
        setConvs((cs) => [{ id: n.id, title: n.title, created: n.created, updated: n.updated }, ...cs]);
      } catch (e) { fail(e); return; }
    }
    ros.setError(null);
    const id = c.id;  // the stream writes only into this conversation; the sidebar is locked while running
    setActive((a) => (a && a.id === id ? { ...a, messages: [...a.messages, { role: "user", text: task }] } : a));
    setRunning(true);
    let msg = emptyClaude();
    setLive(msg);
    const ctl = new AbortController();
    abort.current = ctl;
    let failed = false;
    try {
      await api.runConversation(id, task, (e) => { msg = applyEvent(msg, e); if (mounted.current) setLive(msg); }, ctl.signal);
    } catch (e) {
      if ((e as Error).name === "AbortError") return;
      failed = true;
      if (mounted.current) fail(e);
    }
    if (!mounted.current) return;
    setRunning(false);
    await reload(id);
    if (mounted.current) setLive(null);  // keep the streamed answer on screen until the saved one is loaded
    if (failed || msg.error) await ros.refresh();
  }

  async function rename(id: string, title: string) {
    try {
      await api.renameConversation(id, title);
      setConvs((cs) => cs.map((c) => (c.id === id ? { ...c, title } : c)));
      setActive((a) => (a?.id === id ? { ...a, title } : a));
    } catch (e) { fail(e); }
  }

  async function remove(id: string) {
    try {
      await api.deleteConversation(id);
      await reload(activeId.current === id ? null : undefined);
    } catch (e) { fail(e); }
  }

  const changed = useMemo(() => {
    const out = new Set<string>();
    const lastClaude = [...(active?.messages ?? [])].reverse().find((m) => m.role === "claude");
    if (lastClaude && lastClaude.role === "claude" && status.robot) {
      for (const it of lastClaude.items) if (it.kind === "file") out.add(`${status.robot}_tasks/${it.path}`);
    }
    return out;
  }, [active, status.robot]);

  return (
    <div className="program-mode">
      <header className="program-bar">
        <button className="btn" onClick={() => ctx.goTo("export")} disabled={running}><ArrowLeft size={14} />Back to robot</button>
        <strong>{ctx.robotName}</strong>
        <span className={`dot ${status.state}`} />
        <span className="muted">
          {status.state === "starting" && "Starting ROS… (first start builds the workspace)"}
          {status.state === "ready" && "ROS ready"}
          {status.state === "failed" && "ROS session stopped"}
          {status.state === "stopped" && "ROS not running"}
        </span>
        <span className="spacer" />
        {status.state === "stopped" || status.state === "failed"
          ? <button className="btn" onClick={() => void ros.start()}><Play size={14} />Start ROS</button>
          : <button className="btn" onClick={() => void ros.stop()} disabled={ros.busy || running}><Square size={14} />Stop ROS</button>}
        {(status.state === "ready" || status.state === "failed") &&
          <button className="btn" onClick={() => void ros.restart()} disabled={ros.busy || running}><RotateCcw size={14} />Restart session</button>}
      </header>
      {ros.error && <p className="error program-error">{ros.error}</p>}
      {status.state === "failed" && status.log && <pre className="log program-error">{status.log}</pre>}
      <div className="program-columns">
        <ConversationList items={convs} active={active?.id ?? null} disabled={running || !known}
          onSelect={(id) => void reload(id)} onNew={() => void newConversation()}
          onRename={(id, title) => void rename(id, title)} onDelete={(id) => void remove(id)} />
        <section className="chat-column" data-testid="chat-slot">
          <Chat messages={active?.messages ?? []} live={live} running={running} disabled={status.state !== "ready"} onSend={(t) => void send(t)} />
        </section>
        <section className="workspace-column" data-testid="workspace-slot">
          <Workspace preview={preview} conversation={active?.id ?? null} running={running} changed={changed}
            onRan={(id) => void reload(id)} onRunning={(r) => { if (mounted.current) setRunning(r); }} />
        </section>
      </div>
    </div>
  );
}
