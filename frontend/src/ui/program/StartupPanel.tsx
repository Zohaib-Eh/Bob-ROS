import { Check, Circle, Loader2, X } from "lucide-react";
import type { SessionStatus, Stage } from "../../api/program";

export const STAGES: { id: Stage; label: string }[] = [
  { id: "export", label: "Exporting robot" },
  { id: "collisions", label: "Checking collisions" },
  { id: "image", label: "Building Docker image (first time only)" },
  { id: "build", label: "Building workspace" },
  { id: "moveit", label: "Starting MoveIt" },
  { id: "bridge", label: "Starting bridge" },
  { id: "physics", label: "Starting physics (MuJoCo)" },
  { id: "ready", label: "Ready" },
];

const count = (s: SessionStatus) =>
  (s.total ? ` ${(s.done ?? 0).toLocaleString("en-US")} / ${s.total.toLocaleString("en-US")}` : "");

/** "Checking collisions 4,200 / 10,000": the current stage in words. */
export function stageText(s: SessionStatus): string {
  const st = STAGES.find((x) => x.id === s.stage);
  return st ? st.label + count(s) : "Starting";
}

/** The startup checklist: done, current (or failed) and pending stages, plus the error and log tail. */
export default function StartupPanel({ status }: { status: SessionStatus }) {
  const cur = STAGES.findIndex((x) => x.id === (status.stage ?? "export"));
  return (
    <div className="startup-panel" data-testid="startup-panel">
      <ol>
        {STAGES.map((x, i) => {
          const cls = status.state === "ready" || i < cur ? "done"
            : i > cur ? "todo" : status.state === "failed" ? "failed" : "current";
          return (
            <li key={x.id} data-stage={x.id} className={cls}>
              {cls === "done" ? <Check size={13} /> : cls === "failed" ? <X size={13} />
                : cls === "current" ? <Loader2 size={13} className="spin" /> : <Circle size={13} />}
              {x.label}{cls === "current" ? count(status) : ""}
            </li>
          );
        })}
      </ol>
      {status.error && <p className="error">{status.error}</p>}
      {status.log && <pre className="log">{status.log}</pre>}
    </div>
  );
}
