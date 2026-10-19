import { Pause, Play, Square } from "lucide-react";
import { useEffect, useMemo, useState } from "react";
import type { Joint, Link } from "../../api/types";
import { at, duration, sceneWithPoses, toTf, type Trace } from "../../model/replay";
import { FrameTree, bodyPoses, homeFrames, sceneObjects } from "../../ros/liveRobot";
import type { Viewer } from "../../viewer/Viewer";

/** A finished run shown again in the 3D view: play/pause, scrub, 1×/2×, Stop (the live view comes back). */
export default function Replay({ trace, viewer, links, joints, rootLinkId, onStop }: {
  trace: Trace; viewer: Viewer | null; links: Link[]; joints: Joint[]; rootLinkId: string | null; onStop(): void;
}) {
  const [t, setT] = useState(0);
  const [playing, setPlaying] = useState(true);
  const [speed, setSpeed] = useState(1);
  const total = duration(trace);
  const home = useMemo(() => homeFrames(links, joints, rootLinkId ?? links[0]?.id ?? ""), [links, joints, rootLinkId]);

  useEffect(() => {   // a mobile base (recorded in map): the camera rides along with it
    if (!viewer || trace.base !== "map") return;
    viewer.setFollow(links.find((l) => l.id === (rootLinkId ?? links[0]?.id))?.body_ids[0] ?? null);
    return () => viewer.setFollow(null);
  }, [viewer, trace.base, links, rootLinkId]);

  useEffect(() => {   // the robot and the objects at time t, as the live view draws them
    if (!viewer) return;
    const frame = trace.frames[at(trace.frames, t)];
    const tree = new FrameTree();
    tree.update(toTf(trace.static));
    if (frame) tree.update(toTf(frame.tf));
    viewer.setBodyPoses(bodyPoses(links, tree, home, trace.base));
    const s = trace.scenes.length ? trace.scenes[at(trace.scenes, t)] : null;
    viewer.setSceneObjects(s && s.t <= t + 1e-9
      ? sceneObjects(sceneWithPoses(s.scene, frame?.poses ?? {}, trace.base), tree, trace.base) : []);
  }, [t, viewer, trace, links, home]);

  useEffect(() => {   // play at real speed (or 2×)
    if (!playing) return;
    let last = performance.now();
    let raf = requestAnimationFrame(function step(now) {
      const dt = (now - last) / 1000;
      last = now;
      setT((x) => {
        const next = Math.min(total, x + dt * speed);
        if (next >= total) setPlaying(false);
        return next;
      });
      raf = requestAnimationFrame(step);
    });
    return () => cancelAnimationFrame(raf);
  }, [playing, speed, total]);

  useEffect(() => () => { viewer?.setBodyPoses({}); viewer?.setSceneObjects([]); }, [viewer]);

  return (
    <div className="preview-run replay-bar" data-testid="replay">
      <button className="btn" aria-label={playing ? "Pause" : "Play"}
        onClick={() => { if (!playing && t >= total) setT(0); setPlaying(!playing); }}>
        {playing ? <Pause size={14} /> : <Play size={14} />}</button>
      <input type="range" aria-label="Replay time" min={0} max={total} step={0.01} value={t}
        onChange={(e) => { setPlaying(false); setT(Number(e.target.value)); }} />
      <span className="muted small">{t.toFixed(1)} / {total.toFixed(1)} s</span>
      <select aria-label="Replay speed" value={speed} onChange={(e) => setSpeed(Number(e.target.value))}>
        <option value={1}>1×</option><option value={2}>2×</option>
      </select>
      <span className={`small ${trace.result === "ok" ? "ok" : "bad"}`}>{trace.result === "ok" ? "✓ ok" : trace.result}</span>
      <button className="btn" aria-label="Stop replay" onClick={onStop}><Square size={14} />Stop</button>
    </div>
  );
}
