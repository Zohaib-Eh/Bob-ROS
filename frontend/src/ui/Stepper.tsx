import { Check } from "lucide-react";
import { exportChecklist, type StepId } from "../model/checklist";
import type { Progress } from "../model/detection";
import { chainDepth } from "../model/nextJoint";
import { partProblems } from "../model/problems";
import type { Ctx } from "./ctx";

/** Build's steps. "export" is shown as Review; Simulate ("program") is a workbench, not a step. */
export const STEPS: { id: StepId; label: string }[] = [
  { id: "parts", label: "Parts" }, { id: "group", label: "Group" },
  { id: "connect", label: "Connect" }, { id: "export", label: "Review" },
];

export function stepBadges(ctx: Pick<Ctx, "model" | "robotName">): Record<StepId, number> {
  const m = ctx.model;
  if (!m.links.length) return { parts: 0, group: 0, connect: 0, export: 0, program: 0 };
  const checks = exportChecklist(m, ctx.robotName);
  const depth = chainDepth(m);
  return {
    parts: partProblems(m).filter((p) => p.kind !== "warning").length,
    group: checks.find((c) => c.id === "names")!.linkIds.length,
    connect: m.links.filter((l) => !depth.has(l.id)).length,
    export: checks.filter((c) => !c.ok && !c.warning).length,
    program: 0,
  };
}

interface Props { ctx: Ctx; progress: Progress; onCancel: () => void; onResume: () => void }

export default function Stepper({ ctx, progress, onCancel, onResume }: Props) {
  const { model, step, goTo } = ctx;
  const badges = stepBadges(ctx);
  const working = progress.total > 0 && progress.done < progress.total;

  return (
    <header className="stepper">
      {STEPS.map((s, i) => (
        <span key={s.id} className="row">
          {i > 0 && <span className="step-sep" />}
          <button className={`step-tab${step === s.id ? " current" : ""}`} data-testid={`step-${s.id}`} onClick={() => goTo(s.id)}>
            <span className="num">{i + 1}</span>{s.label}
            {model.links.length > 0 && (badges[s.id] > 0
              ? <span className="badge danger">{badges[s.id]}</span>
              : <Check size={13} className="done" aria-label="done" />)}
          </button>
        </span>
      ))}
      <span className="spacer" />
      {working && (
        <span className="progress">
          Checking joints {progress.done}/{progress.total}
          {progress.paused
            ? <button className="btn btn-ghost" onClick={onResume}>Resume</button>
            : <button className="btn btn-ghost" onClick={onCancel}>Pause</button>}
        </span>
      )}
    </header>
  );
}
