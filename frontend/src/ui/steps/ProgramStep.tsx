import { Play, RotateCcw, Send, Square } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import * as api from "../../api/program";
import type { ProgramEvent, SessionStatus } from "../../api/program";
import { buildExportRequest } from "../../model/exportRequest";
import { FrameTree, SCENE_COMPONENTS, bodyPoses, homeFrames, sceneObjects } from "../../ros/liveRobot";
import { RosBridge } from "../../ros/rosbridge";
import type { Ctx } from "../ctx";
import { withNames } from "../useLinkMass";

const MAX_RUNS = 3;
interface TaskCard { task: string; lines: string[]; runs: number; done?: Extract<ProgramEvent, { kind: "done" }>; error?: string }

export default function ProgramStep({ ctx }: { ctx: Ctx }) {
  const { model, robotName, viewer } = ctx;
  const [status, setStatus] = useState<SessionStatus>({ state: "stopped" });
  const [error, setError] = useState<string | null>(null);
  const [cards, setCards] = useState<TaskCard[]>([]);
  const [task, setTask] = useState("");
  const [busy, setBusy] = useState(false);
  const [resume, setResume] = useState<string | null>(null);
  const [rerunOut, setRerunOut] = useState<string | null>(null);
  const ros = useRef<RosBridge | null>(null);
  const last = [...cards].reverse().find((c) => c.done?.code);

  async function start() {
    setError(null);
    try {
      const req = buildExportRequest(model);
      setStatus(await api.startSession(robotName, req.links, req.joints, req.bodiesByLink));
    } catch (e) { setError(withNames((e as Error).message, model.links)); }
  }

  useEffect(() => {  // poll until ready or failed
    if (status.state !== "starting") return;
    const t = setInterval(async () => {
      try { setStatus(await api.sessionStatus()); } catch (e) { setError((e as Error).message); }
    }, 1000);
    return () => clearInterval(t);
  }, [status.state]);

  useEffect(() => {  // live robot while ready
    if (status.state !== "ready" || !viewer) return;
    const req = buildExportRequest(model);
    const root = model.rootLinkId ?? req.links[0]?.id;
    const base = req.links.find((l) => l.id === root)?.name ?? "";
    const home = homeFrames(req.links, req.joints, root ?? "");
    const tree = new FrameTree();
    const bridge = new RosBridge(`ws://${location.hostname || "localhost"}:9090`);
    ros.current = bridge;
    let frame = 0;
    const redraw = () => { frame = 0; viewer.setBodyPoses(bodyPoses(req.links, tree, home, base)); };
    const onTf = (m: { transforms: never[] }) => { tree.update(m.transforms); if (!frame) frame = requestAnimationFrame(redraw); };
    const refreshScene = () => bridge.call<{ scene: unknown }>("/get_planning_scene", "moveit_msgs/srv/GetPlanningScene",
      { components: { components: SCENE_COMPONENTS } }).then((r) => viewer.setSceneObjects(sceneObjects(r.scene, tree, base))).catch(() => {});
    bridge.ready.then(() => {
      bridge.subscribe("/tf", "tf2_msgs/msg/TFMessage", onTf);
      bridge.subscribe("/tf_static", "tf2_msgs/msg/TFMessage", onTf);
      bridge.subscribe("/monitored_planning_scene", "moveit_msgs/msg/PlanningScene", refreshScene, 300);
      refreshScene();
    }).catch((e) => setError(e.message));
    return () => { cancelAnimationFrame(frame); bridge.close(); ros.current = null; viewer.setBodyPoses({}); viewer.setSceneObjects([]); };
  }, [status.state, viewer, model]);

  async function send() {
    const text = task.trim();
    if (!text || busy) return;
    setTask(""); setBusy(true); setRerunOut(null);
    setCards((cs) => [...cs, { task: text, lines: [], runs: 0 }]);
    const update = (f: (c: TaskCard) => TaskCard) => setCards((cs) => cs.map((c, i) => (i === cs.length - 1 ? f(c) : c)));
    try {
      await api.runTask(text, resume, (e) => {
        if (e.kind === "text") update((c) => ({ ...c, lines: [...c.lines, e.text] }));
        else if (e.kind === "run") update((c) => ({ ...c, runs: e.n }));
        else if (e.kind === "run_result") update((c) => ({ ...c, lines: [...c.lines, `Run ${c.runs}: ${e.message}`] }));
        else if (e.kind === "done") { update((c) => ({ ...c, done: e })); if (e.session_id) setResume(e.session_id); }
        else if (e.kind === "error") update((c) => ({ ...c, error: e.message }));
      });
    } catch (e) { update((c) => ({ ...c, error: (e as Error).message })); }
    setBusy(false);
  }

  async function runAgain() {
    if (!last?.done?.program) return;
    try { const r = await api.rerun(last.done.program); setRerunOut(r.ok ? "Run again: ok" : r.output.trim().split("\n").pop() ?? "failed"); }
    catch (e) { setRerunOut((e as Error).message); }
  }

  const ready = status.state === "ready";
  return (
    <div className="program-step">
      <div className="row">
        {status.state === "stopped" || status.state === "failed"
          ? <button onClick={start}><Play size={14} /> Start ROS</button>
          : <button onClick={async () => { await api.stopSession(); setStatus({ state: "stopped" }); }}><Square size={14} /> Stop ROS</button>}
        <span className="muted">
          {status.state === "starting" && "Starting ROS… (the first start builds the workspace)"}
          {ready && "ROS ready"}
          {status.state === "failed" && "ROS session stopped"}
        </span>
      </div>
      {status.state === "failed" && status.log && <pre className="log">{status.log}</pre>}
      {error && <p className="error">{error}</p>}
      <div className="chat">
        {cards.map((c, i) => (
          <div key={i} className="task-card">
            <p><b>{c.task}</b></p>
            {c.lines.map((l, k) => <p key={k} className="muted">{l}</p>)}
            {c.done && <p>{c.done.ok ? `Worked on run ${c.done.runs}/${MAX_RUNS}` : `Failed after ${c.done.runs} run${c.done.runs === 1 ? "" : "s"}`}</p>}
            {c.done?.summary && <p style={{ whiteSpace: "pre-wrap" }}>{c.done.summary}</p>}
            {c.error && <p className="error">{c.error}</p>}
          </div>
        ))}
      </div>
      <label>Task<textarea value={task} disabled={!ready || busy} onChange={(e) => setTask(e.target.value)}
        placeholder="e.g. Put a 5 cm box at (0.5, 0.2) on the floor, pick it up and place it at (0.4, -0.3)" /></label>
      <div className="row">
        <button onClick={send} disabled={!ready || busy || !task.trim()}><Send size={14} /> Send</button>
        <button onClick={() => setResume(null)} disabled={!resume || busy}>New conversation</button>
        <button onClick={runAgain} disabled={!ready || busy || !last?.done?.program}><RotateCcw size={14} /> Run again</button>
      </div>
      {rerunOut && <p className="muted">{rerunOut}</p>}
      {last?.done?.code && (<><p className="muted">{last.done.program}</p><pre className="code">{last.done.code}</pre></>)}
    </div>
  );
}
