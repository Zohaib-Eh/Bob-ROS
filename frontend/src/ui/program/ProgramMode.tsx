import { Play, RotateCcw, Square } from "lucide-react";
import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import * as api from "../../api/program";
import type { Conversation, ConversationInfo } from "../../api/program";
import type { Ctx } from "../ctx";
import { buildExportRequest } from "../../model/exportRequest";
import { applyEvent, emptyClaude, type ClaudeMessage } from "../../model/chat";
import MenuBar from "../MenuBar";
import Chat from "./Chat";
import PhysicsCard from "./PhysicsCard";
import ConversationList from "./ConversationList";
import Workspace from "./Workspace";
import WorkcellCard from "./WorkcellCard";
import StartupPanel, { stageText } from "./StartupPanel";
import { useRosSession } from "./useRosSession";
import Replay from "./Replay";
import type { Trace } from "../../model/replay";
import { fmtElapsed, useElapsed } from "../useElapsed";

const NO_CHAT = "";  // runningId for a Code-tab run with no chat open: still locks, matches no chat

export default function ProgramMode({ ctx, preview }: { ctx: Ctx; preview: ReactNode }) {
  const [replay, setReplay] = useState<Trace | null>(null);   // a finished run shown again in the 3D view
  const ros = useRosSession(ctx, !!replay);
  const { status } = ros;
  const [convs, setConvs] = useState<ConversationInfo[]>([]);
  const [active, setActive] = useState<Conversation | null>(null);
  const [runningId, setRunningId] = useState<string | null>(null);  // the conversation a task runs in
  const running = runningId !== null;
  const [live, setLive] = useState<{ id: string; msg: ClaudeMessage } | null>(null);  // the streamed answer and its chat
  const abort = useRef<AbortController | null>(null);
  const robot = status.robot ?? ctx.robotName;  // conversations live in this robot's workspace
  const [panel, setPanel] = useState(false);
  const elapsed = useElapsed(status.state === "starting");
  const showPanel = panel || status.state === "failed";

  const activeId = useRef<string | null>(null);
  const req = useRef(0);
  const mounted = useRef(true);
  useEffect(() => { activeId.current = active?.id ?? null; }, [active]);
  useEffect(() => { mounted.current = true; return () => { mounted.current = false; abort.current?.abort(); }; }, []);
  const fail = (e: unknown) => ros.setError((e as Error).message);

  async function startReplay(trace: string) {
    try {
      const t = await api.getTrace(robot, trace);
      if (t.robot !== robot) throw new Error("it was recorded for another robot");
      setReplay(t);
      setPreviewTick((n) => n + 1);   // show the 3D view
    } catch (e) { ros.setError(`Replay not available: ${(e as Error).message}`); }
  }

  // select: id = switch to it, null = first non-broken of the fresh list, undefined = keep current
  async function reload(select?: string | null) {
    ros.setError(null);
    const token = ++req.current;
    try {
      const list = await api.listConversations(robot);
      if (token !== req.current || !mounted.current) return;
      setConvs(list);
      const id = select === null ? list.find((c) => !c.broken)?.id : select ?? activeId.current ?? list.find((c) => !c.broken)?.id;
      if (!id) { if (select === null) setActive(null); return; }
      const c = await api.getConversation(robot, id);
      if (token === req.current && mounted.current) setActive(c);
    } catch (e) { if (token === req.current && mounted.current) fail(e); }
  }
  useEffect(() => { void reload(); }, [robot]);  // eslint-disable-line react-hooks/exhaustive-deps

  async function newConversation() {
    try {
      const c = await api.createConversation(robot);
      req.current++;
      setActive(c);
      setConvs((cs) => [{ id: c.id, title: c.title, created: c.created, updated: c.updated }, ...cs]);
    } catch (e) { fail(e); }
  }

  async function send(task: string) {
    let c = active;
    if (!c) {
      try {
        c = await api.createConversation(robot);
        if (!mounted.current) return;
        req.current++;
        setActive(c);
        const n = c;
        setConvs((cs) => [{ id: n.id, title: n.title, created: n.created, updated: n.updated }, ...cs]);
      } catch (e) { fail(e); return; }
    }
    ros.setError(null);
    const id = c.id;  // the stream writes only into this conversation, whichever one is open
    setActive((a) => (a && a.id === id ? { ...a, messages: [...a.messages, { role: "user", text: task }] } : a));
    setRunningId(id);
    let msg = emptyClaude();
    setLive({ id, msg });
    const ctl = new AbortController();
    abort.current = ctl;
    let failed = false;
    try {
      await api.runConversation(id, task, (e) => { msg = applyEvent(msg, e); if (mounted.current) setLive({ id, msg }); }, ctl.signal);
    } catch (e) {
      if ((e as Error).name === "AbortError") return;
      failed = true;
      if (mounted.current) fail(e);
    }
    if (!mounted.current) return;
    setRunningId(null);
    await reload();  // keeps whichever chat is open; the finished one reloads with its saved answer
    if (mounted.current) setLive(null);  // keep the streamed answer on screen until the saved one is loaded
    if (failed || msg.error) await ros.refresh();
  }

  async function rename(id: string, title: string) {
    try {
      await api.renameConversation(robot, id, title);
      setConvs((cs) => cs.map((c) => (c.id === id ? { ...c, title } : c)));
      setActive((a) => (a?.id === id ? { ...a, title } : a));
    } catch (e) { fail(e); }
  }

  async function remove(id: string) {
    try {
      await api.deleteConversation(robot, id);
      await reload(activeId.current === id ? null : undefined);
    } catch (e) { fail(e); }
  }

  const [previewTick, setPreviewTick] = useState(0);
  // the conversation's latest program: the last answer's program or the last file you ran
  const latest = useMemo(() => {
    for (const m of [...(active?.messages ?? [])].reverse()) {
      if (m.role === "claude" && m.program) return m.program;
      if (m.role === "run") return m.path;
    }
    return null;
  }, [active]);

  // every program in the workspace, for the Preview Run dropdown; refreshed when the conversation changes or a run ends
  const [programs, setPrograms] = useState<string[]>([]);
  const [chosen, setChosen] = useState<string | null>(null);
  useEffect(() => {
    if (status.state !== "ready" || running) return;
    api.listFiles().then((fs) => { if (mounted.current) setPrograms(fs.filter((f) => f.editable).map((f) => f.path)); })
      .catch(() => {});  // the dropdown just stays as it was
  }, [status.state, running, active]);
  const full = (p: string) => (p.includes("_tasks/") ? p : `${status.robot}_tasks/${p}`);
  useEffect(() => { setChosen(latest && status.robot ? full(latest) : null); }, [latest, status.robot]);  // eslint-disable-line react-hooks/exhaustive-deps
  // the dropdown shows this conversation's programs; "All programs…" widens it to every program in the workspace
  const [showAll, setShowAll] = useState(false);
  useEffect(() => setShowAll(false), [active?.id]);
  const mine = useMemo(() => {
    const out = new Set<string>();
    for (const m of active?.messages ?? []) {
      if (m.role === "run") out.add(full(m.path));
      if (m.role === "claude") {
        if (m.program) out.add(full(m.program));
        for (const it of m.items) if (it.kind === "file" && it.path.startsWith("programs/")) out.add(full(it.path));
      }
    }
    return out;
  }, [active, status.robot]);  // eslint-disable-line react-hooks/exhaustive-deps
  const shown = showAll ? programs : programs.filter((p) => mine.has(p));
  // once the list has loaded, a chosen program that no longer exists (deleted) falls back to the newest shown one
  const pickable = programs.length ? shown : chosen ? [chosen] : [];
  const pick = chosen && pickable.includes(chosen) ? chosen : pickable[pickable.length - 1] ?? null;

  async function runAgain(path: string) {
    if (!active || !status.robot) return;
    const id = active.id;
    ros.setError(null);
    setPreviewTick((n) => n + 1);
    setRunningId(id);
    try { await api.runFile(full(path), id); }
    catch (e) { fail(e); }
    finally { if (mounted.current) setRunningId(null); }
    if (mounted.current) await reload();  // keeps whichever chat is open
  }

  const busyWhy = running ? "A task is running. Wait for it to finish first." : undefined;
  const runningTitle = convs.find((c) => c.id === runningId)?.title ?? "another chat";
  const waiting = runningId !== null && runningId !== active?.id ? `Waiting for task in “${runningTitle}”…`
    : status.state === "starting" ? `Waiting for ROS… (${stageText(status)})`
    : status.state !== "ready" ? "Start ROS to send a task." : undefined;

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
      <MenuBar ctx={ctx} lock={busyWhy ?? null} />
      <header className="program-bar">
        <strong>{ctx.robotName}</strong>
        <span className={`dot ${status.state}`} />
        {status.state === "starting" || status.state === "failed" ? (
          <span className="startup-anchor">
            <button className="btn btn-ghost startup-pill" aria-expanded={showPanel} onClick={() => setPanel((p) => !p)}>
              {status.state === "failed" ? "ROS session stopped" : `Starting ROS · ${stageText(status)} · ${fmtElapsed(elapsed)}`}
            </button>
            {showPanel && <StartupPanel status={status} />}
          </span>
        ) : (
          <span className="muted">
            {status.state === "ready" ? "ROS ready" : "ROS not running"}
            {status.state === "ready" && status.physics
              && ` · ${status.physics === "mujoco" ? "Physics: MuJoCo" : `Physics ${status.physics}`}`}
          </span>
        )}
        <span className="spacer" />
        {status.state === "stopped" || status.state === "failed"
          ? <button className="btn" onClick={() => void ros.start()}><Play size={14} />Start ROS</button>
          : <button className="btn" onClick={() => void ros.stop()} disabled={ros.busy || running} title={busyWhy}><Square size={14} />Stop ROS</button>}
        {(status.state === "ready" || status.state === "failed") &&
          <button className="btn" onClick={() => void ros.restart()} disabled={ros.busy || running} title={busyWhy}><RotateCcw size={14} />Restart session</button>}
      </header>
      {ros.error && <p className="error program-error">{ros.error}</p>}
      <div className="program-columns">
        <ConversationList items={convs} active={active?.id ?? null} runningId={runningId}
          onSelect={(id) => void reload(id)} onNew={() => void newConversation()}
          onRename={(id, title) => void rename(id, title)} onDelete={(id) => void remove(id)} />
        <section className="chat-column" data-testid="chat-slot">
          <PhysicsCard ctx={ctx} />
          <WorkcellCard robot={robot} viewer={ctx.viewer} />
          <Chat messages={active?.messages ?? []} live={live && active?.id === live.id ? live.msg : null} running={active?.id === runningId}
            disabled={!!waiting} waiting={waiting} onSend={(t) => void send(t)} onRunAgain={(p) => void runAgain(p)}
            onReplay={(t) => void startReplay(t)} />
        </section>
        <section className="workspace-column" data-testid="workspace-slot">
          <Workspace preview={preview} conversation={active?.id ?? null} running={running} changed={changed}
            sessionReady={status.state === "ready"} onReplay={(t) => void startReplay(t)}
            onRan={() => void reload()}
            onRunning={(r) => { if (mounted.current) setRunningId(r ? activeId.current ?? NO_CHAT : null); }} showPreview={previewTick}
            overlay={replay ? <Replay trace={replay} viewer={ctx.viewer} links={buildExportRequest(ctx.model).links}
                joints={buildExportRequest(ctx.model).joints} rootLinkId={ctx.model.rootLinkId} onStop={() => setReplay(null)} />
              : pick && (
              <div className="preview-run" data-testid="preview-run">
                <select aria-label="Program to run" value={pick} disabled={running} title={busyWhy}
                  onChange={(e) => { if (e.target.value === "__all__") setShowAll(true); else setChosen(e.target.value); }}>
                  {pickable.map((p) => <option key={p} value={p}>{p.split("/").pop()}</option>)}
                  {!showAll && <option value="__all__">All programs…</option>}
                </select>
                <button className="btn btn-primary" disabled={running || status.state !== "ready"}
                  title={busyWhy ?? (status.state !== "ready" ? "Waiting for ROS…" : undefined)}
                  onClick={() => void runAgain(pick)}><Play size={14} />Run</button>
              </div>
            )} />
        </section>
      </div>
    </div>
  );
}
