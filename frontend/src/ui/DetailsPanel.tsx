import type { Dispatch } from "react";
import type { HistoryAction } from "../model/history";
import type { ModelState } from "../model/types";
import LinkDetails from "./LinkDetails";
import PartDetails from "./PartDetails";
import type { Selection } from "./selection";

export interface DetailsProps {
  model: ModelState;
  dispatch: Dispatch<HistoryAction>;
  selection: Selection;
  setSelection: (s: Selection) => void;
  setBanner: (m: string | null) => void;
}

export default function DetailsPanel(props: DetailsProps) {
  const s = props.selection;
  if (s?.kind === "bodies" && s.ids.length === 1) return <PartDetails {...props} bodyId={s.ids[0]} />;
  if (s?.kind === "bodies") return <div>{s.ids.length} parts selected — use “Merge into link”.</div>;
  if (s?.kind === "link") return <LinkDetails {...props} linkId={s.id} />;
  return <div data-testid="details">{s ? s.kind : "Overview"}</div>;
}
