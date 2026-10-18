import { useState } from "react";
import { gripSummary, gripperIssues, gripperWarnings } from "../model/gripper";
import type { Ctx } from "./ctx";
import Info from "./Info";

/** Mark two sliding joints as the gripper's fingers: the second mirrors the first. */
export default function GripperCard({ ctx }: { ctx: Ctx }) {
  const { model, dispatch } = ctx;
  const sliders = model.joints.filter((j) => j.type === "prismatic");
  const g = model.gripper;
  const [ticked, setTicked] = useState<string[]>(g ? [g.leader, g.follower] : []);
  if (sliders.length < 2) return null;
  const name = (linkId: string) => model.links.find((l) => l.id === linkId)?.name ?? linkId;

  const toggle = (id: string) => {
    const next = ticked.includes(id) ? ticked.filter((t) => t !== id) : [...ticked, id].slice(-2);
    setTicked(next);
    dispatch({ type: "setGripper", gripper: next.length === 2 ? { leader: next[0], follower: next[1], tcp_offset: g?.tcp_offset ?? 0 } : null });
  };
  const leader = g && model.joints.find((j) => j.id === g.leader);
  const issues = g ? [...gripperIssues(model, g), ...gripperWarnings(model, g)] : [];

  return (
    <div className="card" data-testid="gripper">
      <h3>Gripper <Info term="gripper" /></h3>
      <p className="muted">Tick the two sliding joints that are the gripper's fingers. The second finger mirrors the first.</p>
      {sliders.map((j) => (
        <label key={j.id} className="field-row">
          <input type="checkbox" checked={ticked.includes(j.id)} onChange={() => toggle(j.id)} />
          <span>{name(j.parent_link_id)} → {name(j.child_link_id)}</span>
        </label>
      ))}
      {g && leader && (
        <>
          <p data-testid="gripper-summary">{gripSummary(model, g)}</p>
          <label className="field-row">
            <span>Tool point offset (mm)</span>
            <input type="number" step={1} aria-label="Tool point offset (mm)" value={Math.round(g.tcp_offset * 1000)}
              onChange={(e) => { const v = Number(e.target.value); if (Number.isFinite(v)) dispatch({ type: "setGripper", gripper: { ...g, tcp_offset: v / 1000 } }); }} />
          </label>
        </>
      )}
      {issues.length > 0 && <div className="warn" role="alert">{issues.map((i) => <p key={i}>{i}</p>)}</div>}
    </div>
  );
}
