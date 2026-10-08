import type { Dispatch } from "react";
import type { HistoryAction } from "../model/history";
import type { ModelState } from "../model/types";
import type { Selection } from "./selection";

export interface DetailsProps {
  model: ModelState;
  dispatch: Dispatch<HistoryAction>;
  selection: Selection;
  setSelection: (s: Selection) => void;
  setBanner: (m: string | null) => void;
}

export default function DetailsPanel({ selection }: DetailsProps) {
  return <div data-testid="details">{selection ? selection.kind : "Overview"}</div>;
}
