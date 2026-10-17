import { Check, Loader2, Play, X } from "lucide-react";
import { useEffect, useMemo, useRef, useState } from "react";
import { physicsCheck, type PhysicsResult } from "../../api/client";
import { buildExportRequest } from "../../model/exportRequest";
import type { Ctx } from "../ctx";
import { fmtElapsed, useElapsed } from "../useElapsed";

/** Holds the arm at home pose under gravity for 2 s in MuJoCo and says whether it stays put.
 *  Runs by itself whenever the robot changed since the last check. */
export default function PhysicsCard({ ctx }: { ctx: Ctx }) {
  const { model, robotName, viewer } = ctx;
  const req = useMemo(() => buildExportRequest(model), [model]);
  const key = useMemo(() => robotName + JSON.stringify(req), [robotName, req]);
  const [result, setResult] = useState<PhysicsResult | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [running, setRunning] = useState(false);
  const elapsed = useElapsed(running);
  const lastKey = useRef<string | null>(null);
  const token = useRef(0);

  async function run() {
    const t = ++token.current;
    setRunning(true);
    setError(null);
    try {
      const r = await physicsCheck(robotName, req.links, req.joints, req.bodiesByLink, req.gripper);
      if (t !== token.current) return;
      setResult(r);
      if (!r.ok && r.link) viewer?.focus(model.links.find((l) => l.name === r.link)?.body_ids ?? []);
    } catch (e) {
      if (t === token.current) setError((e as Error).message);
    }
    if (t === token.current) setRunning(false);
  }
  useEffect(() => { if (lastKey.current !== key) { lastKey.current = key; void run(); } }, [key]);  // eslint-disable-line react-hooks/exhaustive-deps

  const bad = !!error || (result !== null && !result.ok);
  return (
    <div className={`card physics-card${bad ? " bad" : result?.ok ? " ok" : ""}`} data-testid="physics-card">
      <div className="row">
        <strong>Physics</strong>
        {running ? <span className="muted"><Loader2 size={13} className="spin" /> Checking physics… {fmtElapsed(elapsed)}</span>
          : error ? <span><X size={13} /> {error}</span>
          : result ? <span>{result.ok ? <Check size={13} /> : <X size={13} />} {result.message}</span> : null}
        <span className="spacer" />
        <button className="btn btn-ghost" disabled={running} onClick={() => void run()}><Play size={13} />Run check</button>
      </div>
    </div>
  );
}
