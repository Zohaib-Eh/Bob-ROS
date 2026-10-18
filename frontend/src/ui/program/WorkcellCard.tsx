import { Box, Check, Cylinder, Pencil, Table, Trash2 } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import * as api from "../../api/program";
import type { WorkcellObject } from "../../api/program";
import { newObject, outOfReach, reachAt, settle, top, type NewKind, type Reach } from "../../model/workcell";
import type { Viewer } from "../../viewer/Viewer";

const SAVE_DELAY_MS = 800;
const mm = (m: number) => Math.round(m * 1000);
const DENSITY = 500;   // kg/m³ for a loose object's default mass (a wood-like part)

/** The robot's workcell (<robot>_tasks/workcell.yaml: objects in every run), placed by dragging in the 3D view. */
export default function WorkcellCard({ robot, viewer }: { robot: string; viewer: Viewer | null }) {
  const [objects, setObjects] = useState<WorkcellObject[]>([]);
  const [reach, setReach] = useState<Reach>(null);
  const [error, setError] = useState<string | null>(null);
  const [editing, setEditing] = useState(false);
  const [selected, setSelected] = useState<string | null>(null);
  const dirty = useRef(false);

  useEffect(() => {
    let live = true;
    void api.getWorkcell(robot).then((w) => { if (!live) return; setObjects(w.objects); setReach(w.reach); setError(w.error); })
      .catch((e) => { if (live) setError((e as Error).message); });
    return () => { live = false; };
  }, [robot]);

  useEffect(() => {   // save a moment after the last change
    if (!dirty.current) return;
    const t = setTimeout(() => {
      dirty.current = false;
      api.putWorkcell(robot, objects).then(() => setError(null)).catch((e) => setError((e as Error).message));
    }, SAVE_DELAY_MS);
    return () => clearTimeout(t);
  }, [objects, robot]);

  const change = (next: WorkcellObject[]) => { dirty.current = true; setObjects(next); };
  const edit = (name: string, f: (o: WorkcellObject) => WorkcellObject) =>
    change(objects.map((o) => (o.name === name ? f(o) : o)));

  useEffect(() => {   // the 3D view: objects, the selection, the reach; drag to move
    if (!viewer) return;
    const ringZ = (() => { const s = objects.find((o) => o.name === selected); return s ? top(s) : 0.3; })();
    viewer.setWorkcell(editing ? {
      objects: objects.map((o) => ({ ...o, warn: outOfReach(o, reach) })), selected, reach: reachAt(reach, ringZ), ringZ,
      onPick: setSelected,
      onDrag: (name, x, y) => change(objects.map((o) => (o.name === name ? settle({ ...o, xyz: [x, y, o.xyz[2]] }, objects) : o))),
    } : null);
  }, [viewer, editing, objects, selected, reach]);   // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => () => viewer?.setWorkcell(null), [viewer]);

  const add = (kind: NewKind) => { const o = newObject(kind, objects, reach); change([...objects, o]); setSelected(o.name); };
  const sel = objects.find((o) => o.name === selected) ?? null;
  const far = objects.filter((o) => outOfReach(o, reach));
  const num = (label: string, value: number, set: (v: number) => void, step = 1) => (
    <label className="field-row small"><span>{label}</span>
      <input type="number" step={step} aria-label={label} value={value} style={{ width: 80 }}
        onChange={(e) => { const v = Number(e.target.value); if (Number.isFinite(v)) set(v); }} /></label>
  );

  return (
    <div className="card" data-testid="workcell">
      <div className="row"><strong>Workcell</strong><span className="spacer" />
        <button className="btn" onClick={() => { setEditing(!editing); setSelected(null); }}>
          {editing ? <><Check size={14} />Done</> : <><Pencil size={14} />Edit workcell</>}</button>
      </div>
      <p className="muted small">{objects.length} object{objects.length === 1 ? "" : "s"} in every run
        {editing && " · drag them in the 3D view; they settle onto what's under them"}</p>
      {editing && <>
        <div className="row small">Add:
          <button className="btn" onClick={() => add("box")}><Box size={14} />Box</button>
          <button className="btn" onClick={() => add("cylinder")}><Cylinder size={14} />Cylinder</button>
          <button className="btn" onClick={() => add("table")}><Table size={14} />Table</button>
        </div>
        <ul className="plain">
          {objects.map((o) => (
            <li key={o.name}><button className={`btn btn-ghost${o.name === selected ? " active" : ""}`} onClick={() => setSelected(o.name)}>
              {o.name} · {o.fixed ? "bolted down" : `${o.mass ?? "?"} kg`}</button></li>
          ))}
        </ul>
        {sel && <div className="sub-card">
          <label className="field-row small"><span>Name</span>
            <input aria-label="Name" value={sel.name} onChange={(e) => { const n = e.target.value; edit(sel.name, (o) => ({ ...o, name: n })); setSelected(n); }} /></label>
          {sel.shape === "box"
            ? ["x", "y", "z"].map((a, i) => <span key={a}>{num(`Size ${a} (mm)`, mm(sel.size[i]), (v) => edit(sel.name, (o) => ({ ...o, size: o.size.map((s, k) => (k === i ? v / 1000 : s)) })))}</span>)
            : <>{num("Radius (mm)", mm(sel.size[0]), (v) => edit(sel.name, (o) => ({ ...o, size: [v / 1000, o.size[1]] })))}
                {num("Height (mm)", mm(sel.size[1]), (v) => edit(sel.name, (o) => ({ ...o, size: [o.size[0], v / 1000] })))}</>}
          {["x", "y", "z"].map((a, i) => <span key={a}>{num(`Position ${a} (mm)`, mm(sel.xyz[i]), (v) => edit(sel.name, (o) => ({ ...o, xyz: o.xyz.map((s, k) => (k === i ? v / 1000 : s)) as WorkcellObject["xyz"] })))}</span>)}
          {num("Turn (°)", Math.round((sel.rpy[2] * 180) / Math.PI), (v) => edit(sel.name, (o) => ({ ...o, rpy: [o.rpy[0], o.rpy[1], (v * Math.PI) / 180] })))}
          <label className="field-row small"><input type="checkbox" aria-label="Bolted down" checked={sel.fixed}
            onChange={(e) => edit(sel.name, (o) => (e.target.checked ? { ...o, fixed: true, mass: null }
              : { ...o, fixed: false, mass: +(DENSITY * (o.shape === "box" ? o.size[0] * o.size[1] * o.size[2] : Math.PI * o.size[0] ** 2 * o.size[1])).toPrecision(2) }))} />
            <span>Bolted down</span></label>
          {!sel.fixed && num("Mass (kg)", sel.mass ?? 0, (v) => edit(sel.name, (o) => ({ ...o, mass: v })), 0.05)}
          <button className="btn" onClick={() => { change(objects.filter((o) => o.name !== sel.name)); setSelected(null); }}><Trash2 size={14} />Delete</button>
        </div>}
      </>}
      {far.map((o) => <p key={o.name} className="warn small">{o.name} is out of the arm's reach: it gets {(reachAt(reach, top(o)) ?? 0).toFixed(2)} m out at that height.</p>)}
      {error && <p className="warn small" role="alert">{error}</p>}
    </div>
  );
}
