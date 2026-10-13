import { ArrowLeft, Play, RotateCcw, Square } from "lucide-react";
import { useEffect, useState, type ReactNode } from "react";
import * as api from "../../api/program";
import type { Conversation, ConversationInfo } from "../../api/program";
import type { Ctx } from "../ctx";
import ConversationList from "./ConversationList";
import { useRosSession } from "./useRosSession";

export default function ProgramMode({ ctx, preview }: { ctx: Ctx; preview: ReactNode }) {
  const ros = useRosSession(ctx);
  const { status } = ros;
  const [convs, setConvs] = useState<ConversationInfo[]>([]);
  const [active, setActive] = useState<Conversation | null>(null);
  const [running] = useState(false);  // wired by the chat task
  const known = status.state === "ready" || status.state === "starting";

  async function reload(select?: string) {
    try {
      const list = await api.listConversations();
      setConvs(list);
      const id = select ?? active?.id ?? list.find((c) => !c.broken)?.id;
      if (id) setActive(await api.getConversation(id));
    } catch (e) { ros.setError((e as Error).message); }
  }
  useEffect(() => { if (known) void reload(); }, [known]);  // eslint-disable-line react-hooks/exhaustive-deps

  async function newConversation() {
    const c = await api.createConversation();
    setActive(c);
    setConvs((cs) => [{ id: c.id, title: c.title, created: c.created, updated: c.updated }, ...cs]);
  }

  return (
    <div className="program-mode">
      <header className="program-bar">
        <button className="btn" onClick={() => ctx.goTo("export")}><ArrowLeft size={14} />Back to robot</button>
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
          onRename={async (id, title) => { await api.renameConversation(id, title); await reload(id); }}
          onDelete={async (id) => { await api.deleteConversation(id); if (active?.id === id) setActive(null); await reload(); }} />
        <section className="chat-column" data-testid="chat-slot" />
        <section className="workspace-column" data-testid="workspace-slot">{preview}</section>
      </div>
    </div>
  );
}
