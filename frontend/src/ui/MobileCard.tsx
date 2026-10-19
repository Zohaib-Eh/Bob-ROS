import { RotateCw } from "lucide-react";
import { mobileSummary, mobileWarnings, sides, turnToDrive, wheelCandidates } from "../model/mobile";
import type { Ctx } from "./ctx";
import Info from "./Info";

/** Tick the joints that are a mobile base's driven wheels; the base then drives in Simulate. */
export default function MobileCard({ ctx }: { ctx: Ctx }) {
  const { model, dispatch } = ctx;
  const wheels = wheelCandidates(model);
  const sel = model.mobile ?? null;
  if (wheels.length < 2 && !sel) return null;
  const ticked = new Set(sel ? [...sel.left, ...sel.right] : []);
  const name = (jointId: string) => {
    const j = model.joints.find((x) => x.id === jointId);
    return model.links.find((l) => l.id === j?.child_link_id)?.name ?? jointId;
  };
  const toggle = (id: string) => {
    const next = ticked.has(id) ? [...ticked].filter((t) => t !== id) : [...ticked, id];
    const chosen = wheels.filter((w) => next.includes(w.jointId));
    dispatch({ type: "setMobile", mobile: chosen.length ? sides(chosen) : null });
    const j = model.joints.find((x) => x.id === id);
    if (j && !ticked.has(id) && j.type !== "continuous")   // a wheel turns for ever
      dispatch({ type: "updateJoint", joint: { ...j, type: "continuous", limit_lower: null, limit_upper: null } });
  };
  const turn = turnToDrive(wheels.filter((w) => ticked.has(w.jointId)).length ? wheels.filter((w) => ticked.has(w.jointId)) : wheels);
  const side = (id: string) => (sel?.left.includes(id) ? "left" : sel?.right.includes(id) ? "right" : "");
  const issues = sel ? mobileWarnings(model, sel) : [];

  return (
    <div className="card" data-testid="mobile">
      <h3>Mobile base <Info term="mobile" /></h3>
      <p className="muted">Tick the driven wheels. The base drives forward along +x; casters are found by themselves.</p>
      {wheels.map((w) => (
        <label key={w.jointId} className="field-row">
          <input type="checkbox" checked={ticked.has(w.jointId)} onChange={() => toggle(w.jointId)} />
          <span>{name(w.jointId)}</span>{side(w.jointId) && <span className="muted small">· {side(w.jointId)}</span>}
        </label>
      ))}
      {turn !== 0 && (
        <button className="btn" onClick={() => dispatch({ type: "turnRobot", yaw: turn })}
          title="Rotates every part and joint about the vertical axis, so the wheels roll along x">
          <RotateCw size={14} />Turn the robot to drive along +x</button>
      )}
      {sel && <p data-testid="mobile-summary">{mobileSummary(model, sel)}</p>}
      {issues.length > 0 && <div className="warn" role="alert">{issues.map((i) => <p key={i}>{i}</p>)}</div>}
    </div>
  );
}
