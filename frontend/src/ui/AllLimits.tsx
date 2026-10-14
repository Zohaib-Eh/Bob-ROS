import { Loader2, Radar, Square } from "lucide-react";
import { useRef, useState } from "react";
import { sweepLimits } from "../api/client";
import { draftFromJoint, jointAtDefaultLimits } from "../model/jointDraft";
import type { LimitChange } from "../model/reducer";
import type { Ctx } from "./ctx";
import { fmtJointValue, sweepRequest, sweepSummary } from "./drive";
import Info from "./Info";

type Row = { jointId: string; name: string; status: "set" | "free" | "overlap" | "skipped" | "error"; text: string };

/** Sweep every joint still at the default range, one after another, and apply what was found. */
export default function AllLimits({ ctx }: { ctx: Ctx }) {
  const { model, dispatch } = ctx;
  const [rows, setRows] = useState<Row[]>([]);
  const [progress, setProgress] = useState<{ done: number; total: number } | null>(null);
  const cancelled = useRef(false);
  const pending = model.joints.filter(jointAtDefaultLimits);
  const linkName = (id: string) => model.links.find((l) => l.id === id)?.name ?? id;

  async function run() {
    cancelled.current = false;
    const out: Row[] = [];
    const changes: LimitChange[] = [];
    setRows([]);
    setProgress({ done: 0, total: pending.length });
    for (const j of pending) {
      if (cancelled.current) break;
      const slider = j.type === "prismatic";
      const row = (status: Row["status"], text: string) => out.push({ jointId: j.id, name: linkName(j.child_link_id), status, text });
      const req = sweepRequest(model, draftFromJoint(j));
      if (!req) row("skipped", "Needs an axis first.");
      else {
        try {
          const r = await sweepLimits(req.joint, req.moving, req.obstacles);
          if (r.at_rest_overlap.length) row("overlap", sweepSummary(r, slider, linkName));
          else if (r.free) row("free", "Turns freely: no collision in a full turn.");
          else if (r.lower === null || r.upper === null) row("skipped", sweepSummary(r, slider, linkName));
          else {
            changes.push({ jointId: j.id, lower: r.lower, upper: r.upper, continuous: false });
            row("set", `${fmtJointValue(r.lower, slider)} … ${fmtJointValue(r.upper, slider)} · ${sweepSummary(r, slider, linkName)}`);
          }
        } catch (e) {
          row("error", (e as Error).message);
        }
      }
      setRows([...out]);
      setProgress({ done: out.length, total: pending.length });
    }
    if (changes.length) dispatch({ type: "setLimits", changes });
    setProgress(null);
  }

  if (!pending.length && !rows.length) return null;
  return (
    <div className="card" data-testid="all-limits">
      <div className="row">
        {progress ? (
          <>
            <Loader2 size={14} className="spin" />Sweeping {progress.done + 1 > progress.total ? progress.total : progress.done + 1}/{progress.total}…
            <span className="spacer" />
            <button className="btn" data-testid="cancel-all-limits" onClick={() => { cancelled.current = true; }}><Square size={14} />Cancel</button>
          </>
        ) : pending.length ? (
          <button className="btn" data-testid="find-all-limits" onClick={() => void run()}>
            <Radar size={14} />Find limits for all joints ({pending.length} joint{pending.length === 1 ? "" : "s"} at the default)
          </button>
        ) : <span className="muted small">All joints have limits.</span>}
        <Info term="sweep" />
      </div>
      {rows.length > 0 && (
        <ul className="limit-rows small">
          {rows.map((r) => (
            <li key={r.jointId} data-testid={`limit-row-${r.jointId}`} className={r.status === "set" ? "ok" : r.status === "free" ? "" : "muted"}>
              <b>{r.name}</b>: {r.text}
              {r.status === "free" && model.joints.find((j) => j.id === r.jointId)?.type !== "continuous" && (
                <button className="btn" data-testid={`no-limits-${r.jointId}`}
                  onClick={() => dispatch({ type: "setLimits", changes: [{ jointId: r.jointId, lower: null, upper: null, continuous: true }] })}>
                  No limits
                </button>
              )}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
