import { AlertTriangle, Link2Off, Locate } from "lucide-react";
import { useEffect, useReducer } from "react";
import type { Viewer } from "../viewer/Viewer";
import type { Marker } from "./markers";

const ICON = { problem: AlertTriangle, joint: Locate, loose: Link2Off } as const;

/** HTML pins that follow points of the 3D model. */
export default function MarkerLayer({ viewer, markers, onClick }: {
  viewer: Viewer | null; markers: Marker[]; onClick: (m: Marker) => void;
}) {
  const [, relayout] = useReducer((n: number) => n + 1, 0);
  useEffect(() => viewer?.onCameraChange(relayout), [viewer]);
  if (!viewer) return null;
  return (
    <div className="markers">
      {markers.map((mk) => {
        const p = viewer.screenPoint(mk.at);
        if (!p) return null;
        const Icon = ICON[mk.kind];
        return (
          <button key={mk.id} className={`marker marker-${mk.kind}`} style={{ left: p.x, top: p.y }} title={mk.label}
            data-testid={`marker-${mk.id}`} onClick={() => onClick(mk)}>
            <Icon size={12} />{mk.kind === "problem" && <span>{mk.label}</span>}
          </button>
        );
      })}
    </div>
  );
}
