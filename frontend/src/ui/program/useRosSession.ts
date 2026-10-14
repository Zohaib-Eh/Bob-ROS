import { useEffect, useRef, useState } from "react";
import * as api from "../../api/program";
import type { SessionStatus } from "../../api/program";
import { buildExportRequest } from "../../model/exportRequest";
import { FrameTree, SCENE_COMPONENTS, bodyPoses, homeFrames, sceneObjects } from "../../ros/liveRobot";
import { RosBridge } from "../../ros/rosbridge";
import type { Ctx } from "../ctx";
import { withNames } from "../useLinkMass";

export const POLL = { starting: 1000, ready: 5000 };  // ms; mutable so tests can shorten

export function useRosSession(ctx: Ctx) {
  const { model, robotName, viewer } = ctx;
  const [status, setStatus] = useState<SessionStatus>({ state: "stopped" });
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const ros = useRef<RosBridge | null>(null);
  const mounted = useRef(true);
  useEffect(() => { mounted.current = true; return () => { mounted.current = false; }; }, []);
  const msg = (e: unknown) => withNames((e as Error).message, model.links);

  async function start() {
    setError(null);
    try {
      const req = buildExportRequest(model);
      const s = await api.startSession(robotName, req.links, req.joints, req.bodiesByLink);
      if (mounted.current) setStatus(s);
    } catch (e) { if (mounted.current) setError(msg(e)); }
  }

  async function restart() {
    setBusy(true); setError(null);
    try { await api.stopSession(); await start(); }
    catch (e) { if (mounted.current) setError(msg(e)); }
    if (mounted.current) setBusy(false);
  }

  async function stop() {
    setBusy(true); setError(null);
    try { await api.stopSession(); if (mounted.current) setStatus({ state: "stopped" }); }
    catch (e) { if (mounted.current) setError(msg(e)); }
    if (mounted.current) setBusy(false);
  }

  async function refresh() {
    try {
      const s = await api.sessionStatus();
      // a dead container reports no stage: keep the last one seen so the panel can mark it
      if (mounted.current) setStatus((p) => (s.state === "failed" && !s.stage ? { ...s, stage: p.stage } : s));
    } catch (e) { if (mounted.current) setError((e as Error).message); }
  }

  useEffect(() => { void refresh(); }, []);  // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {  // poll while starting (fast) and ready (detects a dead session); never two at once
    if (status.state !== "starting" && status.state !== "ready") return;
    const delay = status.state === "starting" ? POLL.starting : POLL.ready;
    let stopped = false;
    let t = 0;
    const tick = async () => { await refresh(); if (!stopped) t = window.setTimeout(tick, delay); };
    t = window.setTimeout(tick, delay);
    return () => { stopped = true; clearTimeout(t); };
  }, [status.state]);  // eslint-disable-line react-hooks/exhaustive-deps

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

  return { status, error, setError, start, stop, restart, refresh, busy };
}
