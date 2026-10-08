import type { Dispatch } from "react";
import type { StepId } from "../model/checklist";
import type { HistoryAction } from "../model/history";
import type { JointDraft } from "../model/jointDraft";
import type { ModelState } from "../model/types";
import type { Viewer } from "../viewer/Viewer";
import type { Selection } from "./selection";

export type PickMode = "select" | "base" | "fixed" | "moving" | "origin";

/** Everything a panel needs, passed as one prop. */
export interface Ctx {
  model: ModelState;
  dispatch: Dispatch<HistoryAction>;
  canUndo: boolean;
  canRedo: boolean;
  undoDepth: number;
  step: StepId;
  goTo: (s: StepId) => void;
  selection: Selection;
  setSelection: (s: Selection) => void;
  pick: PickMode;
  setPick: (p: PickMode) => void;
  jointDraft: JointDraft | null;
  setJointDraft: (d: JointDraft | null) => void;
  viewer: Viewer | null;
  notify: (m: string | null) => void;
  scale: number;
  setScale: (s: number) => void;
  robotName: string;
  setRobotName: (n: string) => void;
  autoGrouped: boolean;
  setAutoGrouped: (v: boolean) => void;
  detecting: boolean;
}
