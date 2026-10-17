import { Anchor, ArrowRight, Check, Crosshair, Link2 } from "lucide-react";
import { draftForPair, emptyDraft } from "../../model/jointDraft";
import { chainDepth, nextJoint } from "../../model/nextJoint";
import { pairKey } from "../../model/pairs";
import AllLimits from "../AllLimits";
import type { Ctx } from "../ctx";
import { shortName } from "../format";
import GripperCard from "../GripperCard";
import Info from "../Info";

export default function ConnectStep({ ctx }: { ctx: Ctx }) {
  const { model, dispatch, pick, setPick, jointDraft, setJointDraft, goTo, detecting } = ctx;
  const name = (id: string) => model.links.find((l) => l.id === id)?.name ?? id;
  const depth = chainDepth(model);
  const connected = model.links.filter((l) => depth.has(l.id)).length;
  const next = nextJoint(model);
  const g = next ? model.suggestions[pairKey(next.fixed, next.moving)] : undefined;

  return (
    <section className="step">
      <h2>Connect <Info term="joint" /></h2>
      <p className="muted">Build the robot out from the base, one joint at a time.</p>
      <div className="card">
        <div className="row">
          <Anchor size={14} /><strong>Base</strong><Info term="base" /><span className="spacer" />
          <button className={`btn${pick === "base" ? " active" : ""}`} data-testid="pick-base"
            onClick={() => setPick(pick === "base" ? "select" : "base")}>
            <Crosshair size={14} />{pick === "base" ? "Click the part on the ground…" : "Change"}
          </button>
        </div>
        <div data-testid="base-name">{model.rootLinkId ? shortName(name(model.rootLinkId), 32) : "—"}</div>
        <p className="muted small">The part that sits on the ground. We guessed the lowest one.</p>
      </div>
      <div className="muted small" data-testid="connected-count">{connected} of {model.links.length} links connected</div>
      {!jointDraft && <AllLimits ctx={ctx} />}
      {jointDraft ? (
        <div className="card muted">Finish the joint in the Joint tool over the 3D view.</div>
      ) : next ? (
        <div className="card attention" data-testid="next-card">
          <h3>Next: {shortName(name(next.fixed), 18)} <ArrowRight size={12} /> {shortName(name(next.moving), 18)}</h3>
          <p className="muted small">{next.suggestion
            ? <>✨ Suggested from contact faces <Info term="suggested" /></>
            : detecting && !g ? "Checking this pair for contact faces…"
            : "No joint detected here — set it up by hand."}</p>
          <div className="row">
            <button className="btn btn-primary" data-testid="next-joint" onClick={() => setJointDraft(draftForPair(model, next))}>
              <Link2 size={14} />Set up joint
            </button>
            {g && <button className="btn" data-testid="not-connected" onClick={() => dispatch({ type: "rejectSuggestion", key: g.key })}>Not connected</button>}
          </div>
        </div>
      ) : connected === model.links.length ? (
        <div className="card ok" data-testid="all-connected">
          <div className="row"><Check size={14} />All {model.links.length} links are connected.
            <span className="spacer" /><button className="btn btn-primary" onClick={() => goTo("export")}>Go to Export</button></div>
        </div>
      ) : detecting ? (
        <div className="card muted" data-testid="checking">Checking which parts touch…</div>
      ) : (
        <div className="card attention" data-testid="stuck">
          Some links don't touch anything connected. Use <b>New joint (J)</b> to connect them by hand, or group them with a neighbour.
        </div>
      )}
      <button className="btn" data-testid="new-joint" onClick={() => { setJointDraft(emptyDraft()); setPick("fixed"); }}>
        <Link2 size={14} />New joint <kbd>J</kbd>
      </button>
      <GripperCard ctx={ctx} />
    </section>
  );
}
