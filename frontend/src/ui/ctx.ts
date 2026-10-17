import type { Dispatch } from "react";
import type { StepId } from "../model/checklist";
import type { HistoryAction } from "../model/history";
import type { JointDraft } from "../model/jointDraft";
import type { ModelState } from "../model/types";
import type { Viewer } from "../viewer/Viewer";
import type { Selection } from "./selection";

export type PickMode = "select" | "base" | "fixed" | "moving" | "origin" | "mount-from" | "mount-to";

/** A face clicked while mounting a part (MountCard reads it). */
export interface PickedFace { mode: "mount-from" | "mount-to"; bodyId: string; faceIndex: number }

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
  pickedFace: PickedFace | null;
  setPickedFace: (f: PickedFace | null) => void;
  viewer: Viewer | null;
  notify: (m: string | null) => void;
  scale: number;
  setScale: (s: number) => void;
  robotName: string;
  setRobotName: (n: string) => void;
  autoGrouped: boolean;
  setAutoGrouped: (v: boolean) => void;
  detecting: boolean;
  /** Part under the mouse in the 3D view. */
  hovered: string | null;
}
