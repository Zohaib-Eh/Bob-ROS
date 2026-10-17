import { Crosshair, RotateCw, Undo2 } from "lucide-react";
import { useEffect, useState } from "react";
import type { Face } from "../api/types";
import { invert, mountMatrix, type Mat4 } from "../model/mount";
import type { Ctx, PickedFace } from "./ctx";
import Info from "./Info";

/** Sources (imported files) in import order; bodies from before sources existed are left out. */
export function sourcesOf(bodies: Ctx["model"]["bodies"]): string[] {
  return [...new Set(Object.values(bodies).map((b) => b.source).filter((s): s is string => !!s))];
}

const faceOf = (ctx: Ctx, p: PickedFace | null): Face | null => (p ? ctx.model.bodies[p.bodyId]?.faces[p.faceIndex] ?? null : null);

/** Put a part from a second CAD file flush on a face of the robot (e.g. a gripper on the flange). */
export default function MountCard({ ctx }: { ctx: Ctx }) {
  const { model, dispatch, pick, setPick, pickedFace, setPickedFace } = ctx;
  const sources = sourcesOf(model.bodies);
  const [source, setSource] = useState(sources[sources.length - 1] ?? "");
  const [from, setFrom] = useState<PickedFace | null>(null);
  const [to, setTo] = useState<PickedFace | null>(null);
  const [turn, setTurn] = useState(0);
  const [applied, setApplied] = useState<{ M: Mat4; from: Face; to: Face } | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {   // a face clicked in the 3D view
    if (!pickedFace) return;
    if (pickedFace.mode === "mount-from") setFrom(pickedFace); else setTo(pickedFace);
    setPickedFace(null);
    setError(null);
  }, [pickedFace, setPickedFace]);

  if (sources.length < 2) return null;
  const inSource = (bodyId: string) => model.bodies[bodyId]?.source === source;
  const linksOfSource = new Set(model.links.filter((l) => l.body_ids.some(inSource)).map((l) => l.id));
  const hasJoints = model.joints.some((j) => linksOfSource.has(j.parent_link_id) || linksOfSource.has(j.child_link_id));

  const place = (fromFace: Face, toFace: Face, deg: number) => {
    const M = mountMatrix(fromFace, toFace, deg);
    if (applied) dispatch({ type: "transformSource", source, matrix: invert(applied.M) });
    dispatch({ type: "transformSource", source, matrix: M });
    setApplied({ M, from: fromFace, to: toFace });
  };
  const mount = () => {
    if (hasJoints) return setError("Remove this part's joints before moving it.");
    const f = faceOf(ctx, from), t = faceOf(ctx, to);
    if (!f || !t) return setError("Pick a face on this part and a face on the robot first.");
    if (!inSource(from!.bodyId) || inSource(to!.bodyId)) return setError("Pick the first face on this part and the second on the robot.");
    try {
      setApplied(null);
      const M = mountMatrix(f, t, turn);
      dispatch({ type: "transformSource", source, matrix: M });
      setApplied({ M, from: f, to: t });
      setError(null);
    } catch (e) {
      setError((e as Error).message);
    }
  };
  const turnBy = (deg: number) => {
    if (!applied) return;
    const next = turn + deg;
    setTurn(next);
    place(applied.from, applied.to, next);
  };
  const undo = () => {
    if (!applied) return;
    dispatch({ type: "transformSource", source, matrix: invert(applied.M) });
    setApplied(null);
    setTurn(0);
  };
  const pickBtn = (mode: PickedFace["mode"], chosen: PickedFace | null, label: string) => (
    <button className={`btn pick${pick === mode ? " active" : ""}`} onClick={() => setPick(pick === mode ? "select" : mode)}>
      <Crosshair size={13} />{pick === mode ? "Click a face in the 3D view…" : chosen ? `${model.bodies[chosen.bodyId]?.name} face` : label}
    </button>
  );

  return (
    <div className="card" data-testid="mount">
      <h3>Mount a part <Info term="mount" /></h3>
      <p className="muted">Puts a part from another file flush on the robot, e.g. a gripper's mounting plate on the flange.</p>
      <div className="field-row">
        <span>Part from</span>
        <select value={source} onChange={(e) => { setSource(e.target.value); setApplied(null); setTurn(0); }}>
          {sources.map((s) => <option key={s} value={s}>{s}</option>)}
        </select>
      </div>
      <div className="field-row"><span>① Face on this part</span>{pickBtn("mount-from", from, "Pick in 3D")}</div>
      <div className="field-row"><span>② Face on the robot</span>{pickBtn("mount-to", to, "Pick in 3D")}</div>
      <div className="row">
        <button className="btn primary" onClick={mount}>Mount</button>
        <button className="btn" disabled={!applied} onClick={() => turnBy(-15)}>Turn −15°</button>
        <button className="btn" disabled={!applied} onClick={() => turnBy(15)}><RotateCw size={13} />Turn +15°</button>
        <label className="muted">
          <input type="number" step={1} value={turn} disabled={!applied} aria-label="Turn angle (degrees)"
            onChange={(e) => { const v = Number(e.target.value); if (Number.isFinite(v)) turnBy(v - turn); }} />°
        </label>
        <button className="btn" disabled={!applied} onClick={undo}><Undo2 size={13} />Undo</button>
      </div>
      {error && <p className="warn" role="alert">{error}</p>}
    </div>
  );
}
