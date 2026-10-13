import { AlertTriangle, Check, Download, Play, X } from "lucide-react";
import { useState } from "react";
import type { Joint } from "../../api/types";
import { exportRobot } from "../../api/client";
import { exportChecklist, type CheckItem } from "../../model/checklist";
import { splitExportError } from "../../model/errors";
import { buildExportRequest } from "../../model/exportRequest";
import { NAME_RE } from "../../model/types";
import { jointMotion, previewRange, subtreeLinks } from "../../viewer/preview";
import type { Viewer } from "../../viewer/Viewer";
import type { Ctx } from "../ctx";
import { downloadBlob } from "../download";
import Info from "../Info";
import { withNames } from "../useLinkMass";

/** One swing: rest → upper → rest → lower → rest, over 1.6 s. */
function swing(viewer: Viewer, j: Joint, bodyIds: string[], lo: number, hi: number): Promise<void> {
  return new Promise((done) => {
    const t0 = performance.now();
    const tick = (t: number) => {
      const k = Math.min(1, (t - t0) / 1600);
      const s = Math.sin(2 * Math.PI * k);
      viewer.setPreview(bodyIds, jointMotion(j, s >= 0 ? s * hi : -s * lo));
      if (k < 1) requestAnimationFrame(tick); else done();
    };
    requestAnimationFrame(tick);
  });
}

export default function ExportStep({ ctx }: { ctx: Ctx }) {
  const { model, robotName, setRobotName, goTo, setSelection, viewer, notify } = ctx;
  const [busy, setBusy] = useState(false);
  const [playing, setPlaying] = useState(false);
  const [server, setServer] = useState<string[]>([]);
  const items = exportChecklist(model, robotName);
  const req = buildExportRequest(model);
  const blocked = items.find((i) => !i.ok && !i.warning);
  const meshes = model.links.filter((l) => l.body_ids.length > 0).length;
  const linkName = (id: string) => model.links.find((l) => l.id === id)?.name ?? id;

  const open = (i: CheckItem) => {
    goTo(i.step);
    if (i.jointIds[0]) setSelection({ kind: "joint", id: i.jointIds[0] });
    else if (i.linkIds[0]) {
      setSelection({ kind: "link", id: i.linkIds[0] });
      viewer?.focus(model.links.find((l) => l.id === i.linkIds[0])?.body_ids ?? []);
    }
  };

  const onExport = async () => {
    setBusy(true);
    setServer([]);
    try {
      const { blob, warnings } = await exportRobot(robotName, req.links, req.joints, req.bodiesByLink);
      downloadBlob(blob, `${robotName}.zip`);
      notify(warnings.length ? `Exported with notes: ${warnings.map((w) => w.message).join(" ")}` : null);
    } catch (e) {
      const { byLink, byJoint, general } = splitExportError((e as Error).message);
      const label = (id: string) => {
        const j = req.joints.find((x) => x.id === id);
        return j ? `${linkName(j.parent_link_id)} → ${linkName(j.child_link_id)}: ` : "";
      };
      setServer([...Object.values(byLink).flat(),
        ...Object.entries(byJoint).flatMap(([id, ms]) => ms.map((m) => label(id) + m)),
        ...general].map((m) => withNames(m, model.links)));
    } finally {
      setBusy(false);
    }
  };

  const playAll = async () => {
    if (!viewer) return;
    setPlaying(true);
    for (const j of req.joints) {
      const [lo, hi] = previewRange(j);
      if (lo === hi) continue;
      const links = subtreeLinks(req.joints, j.id);
      await swing(viewer, j, model.links.filter((l) => links.includes(l.id)).flatMap((l) => l.body_ids), lo, hi);
    }
    viewer.setPreview([], null);
    setPlaying(false);
  };

  return (
    <section className="step">
      <h2>Export <Info term="urdf" /></h2>
      <p className="muted">Everything below must be ticked. Click an item to go and fix it.</p>
      <ul className="checklist">
        {items.map((i) => (
          <li key={i.id}>
            <button className={`check ${i.ok ? "ok" : i.warning ? "warn" : "bad"}`} data-testid={`check-${i.id}`} disabled={i.ok} onClick={() => open(i)}>
              {i.ok ? <Check size={14} /> : i.warning ? <AlertTriangle size={14} /> : <X size={14} />}
              <span><span>{i.label}</span>{!i.ok && i.detail && <span className="muted small block">{i.detail}</span>}</span>
            </button>
          </li>
        ))}
        {server.map((m, k) => (
          <li key={`server-${k}`}><div className="check bad" data-testid="server-error"><X size={14} /><span>{m}</span></div></li>
        ))}
      </ul>
      <button className="btn" data-testid="play-all" disabled={!viewer || playing || !req.joints.length} onClick={() => void playAll()}>
        <Play size={14} />{playing ? "Playing…" : "Play all joints"}
      </button>
      <div className="card">
        <div className="muted small" data-testid="summary">{model.links.length} links · {model.joints.length} joints · {meshes} meshes</div>
        <label className="row" style={{ margin: "6px 0" }}>Robot name <Info term="name" />
          <input className={`field${NAME_RE.test(robotName) ? "" : " invalid"}`} data-testid="robot-name" value={robotName}
            style={{ flex: 1 }} onChange={(e) => setRobotName(e.target.value)} />
        </label>
        <button className="btn btn-primary" data-testid="export" disabled={!!blocked || busy}
          title={blocked ? blocked.detail ?? blocked.label : ""} onClick={() => void onExport()}>
          <Download size={14} />{busy ? "Exporting…" : "Export URDF (.zip)"}
        </button>
      </div>
    </section>
  );
}
