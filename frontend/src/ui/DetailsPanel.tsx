import type { Dispatch } from "react";
import type { Matrix4 } from "three";
import type { HistoryAction } from "../model/history";
import type { ModelState } from "../model/types";
import type { Viewer } from "../viewer/Viewer";
import JointDetails from "./JointDetails";
import LinkDetails from "./LinkDetails";
import Overview from "./Overview";
import PartDetails from "./PartDetails";
import type { Selection } from "./selection";

export interface DetailsProps {
  model: ModelState;
  dispatch: Dispatch<HistoryAction>;
  selection: Selection;
  setSelection: (s: Selection) => void;
  setBanner: (m: string | null) => void;
  viewer: Viewer | null;
  setPreview: (p: { bodyIds: string[]; matrix: Matrix4 } | null) => void;
  lastPick: { bodyId: string; triangle: number } | null;
  scale: number;   // import scale, also used for replacement parts
}

export default function DetailsPanel(props: DetailsProps) {
  const s = props.selection;
  if (s?.kind === "bodies" && s.ids.length === 1) return <PartDetails {...props} bodyId={s.ids[0]} />;
  if (s?.kind === "bodies") return (
    <div>
      <div>{s.ids.length} parts selected — use “Merge into link”.</div>
      <Overview {...props} />
    </div>
  );
  if (s?.kind === "link") return <LinkDetails {...props} linkId={s.id} />;
  if (s?.kind === "suggestion" || s?.kind === "joint") return <JointDetails {...props} />;
  return <Overview {...props} />;
}
