import { Crosshair, RotateCw, Undo2 } from "lucide-react";
import { useEffect, useState } from "react";
import type { Face } from "../api/types";
import { compose, invert, mountMatrix, sourceOf } from "../model/mount";
import type { Ctx, PickedFace } from "./ctx";
import Info from "./Info";

/** Sources (imported files) in import order; bodies from before sources existed are one source, "". */
export function sourcesOf(bodies: Ctx["model"]["bodies"]): string[] {
  return [...new Set(Object.values(bodies).map(sourceOf))];
}

const label = (source: string) => source || "Earlier parts";
const faceOf = (ctx: Ctx, p: PickedFace | null): Face | null => (p ? ctx.model.bodies[p.bodyId]?.faces[p.faceIndex] ?? null : null);

/** Put a part from a second CAD file flush on a face of the robot (e.g. a gripper on the flange). */
export default function MountCard({ ctx }: { ctx: Ctx }) {
  const { model, dispatch, pick, setPick, pickedFace, setPickedFace, viewer } = ctx;
  const sources = sourcesOf(model.bodies);
  const newest = sources[sources.length - 1] ?? "";
  const [source, setSource] = useState(newest);
  const [from, setFrom] = useState<PickedFace | null>(null);
  const [to, setTo] = useState<PickedFace | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {   // a file just added is the part to mount
    setSource(newest);
    setFrom(null);
    setTo(null);
    setError(null);
  }, [newest]);

  useEffect(() => {   // a face clicked in the 3D view
    if (!pickedFace) return;
    if (pickedFace.mode === "mount-from") setFrom(pickedFace); else setTo(pickedFace);
    setPickedFace(null);
    setError(null);
  }, [pickedFace, setPickedFace]);

  useEffect(() => {   // show the picked faces in 3D: a flat one gets an arrow along its normal
    const shown = [faceOf(ctx, from), faceOf(ctx, to)].filter((f): f is Face => !!f?.normal)
      .map((f) => ({ origin: f.center, axis: f.normal! }));
    viewer?.setPicks(shown);
    return () => viewer?.setPicks([]);
  }, [from, to, model.bodies, viewer]);   // eslint-disable-line react-hooks/exhaustive-deps

  if (sources.length < 2) return null;
  const applied = model.mounts?.[source] ?? null;   // in the model, so Edit > Undo keeps the card in step
  const turn = applied?.turn ?? 0;
  const inSource = (bodyId: string) => !!model.bodies[bodyId] && sourceOf(model.bodies[bodyId]) === source;
  const linksOfSource = new Set(model.links.filter((l) => l.body_ids.some(inSource)).map((l) => l.id));
  const hasJoints = model.joints.some((j) => linksOfSource.has(j.parent_link_id) || linksOfSource.has(j.child_link_id));
  const JOINTS = "Remove this part's joints before moving it.";

  const mount = () => {
    if (hasJoints) return setError(JOINTS);
    const f = faceOf(ctx, from), t = faceOf(ctx, to);
    if (!f || !t) return setError("Pick a face on this part and a face on the robot first.");
    if (!inSource(from!.bodyId) || inSource(to!.bodyId)) return setError("Pick the first face on this part and the second on the robot.");
    const curved = [[f, from!], [t, to!]].find(([face]) => (face as Face).kind !== "plane");
    if (curved) return setError(`The face picked on ${model.bodies[(curved[1] as PickedFace).bodyId]?.name} is curved. `
      + "Pick a flat face: the gripper's mounting plate and the flange.");
    try {
      const M = mountMatrix(f, t, 0);
      dispatch({ type: "transformSource", source, matrix: M,
        mount: { M, from: f, to: t, turn: 0, fromBody: from!.bodyId, toBody: to!.bodyId } });
      setError(null);
    } catch (e) {
      setError((e as Error).message);
    }
  };
  const turnBy = (deg: number) => {   // one history step: from the current mount straight to the turned one
    if (!applied) return;
    if (hasJoints) return setError(JOINTS);
    const M = mountMatrix(applied.from, applied.to, turn + deg);
    dispatch({ type: "transformSource", source, matrix: compose(M, invert(applied.M)), mount: { ...applied, M, turn: turn + deg } });
  };
  const undo = () => {
    if (!applied) return;
    if (hasJoints) return setError(JOINTS);
    dispatch({ type: "transformSource", source, matrix: invert(applied.M), mount: null });
  };
  const pickBtn = (mode: PickedFace["mode"], chosen: PickedFace | null, text: string) => (
    <button className={`btn pick${pick === mode ? " active" : ""}`} onClick={() => setPick(pick === mode ? "select" : mode)}>
      <Crosshair size={13} />{pick === mode ? "Click a face in the 3D view…" : chosen
        ? `${model.bodies[chosen.bodyId]?.name} · ${faceOf(ctx, chosen)?.kind === "plane" ? "flat" : "curved"} face` : text}
    </button>
  );

  return (
    <div className="card" data-testid="mount">
      <h3>Mount a part <Info term="mount" /></h3>
      <p className="muted">Puts a part from another file flush on the robot, e.g. a gripper's mounting plate on the flange.</p>
      <div className="field-row">
        <span>Part from</span>
        <select value={source} onChange={(e) => { setSource(e.target.value); setError(null); }}>
          {sources.map((s) => <option key={s} value={s}>{label(s)}</option>)}
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
