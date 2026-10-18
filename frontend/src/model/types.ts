import type { Body, DetectResponse, Face, GripperSel, Joint, Link, PartWarning } from "../api/types";
import type { Mat4 } from "./mount";

export interface Suggestion {
  key: string;
  linkA: string;
  linkB: string;
  membership: string;
  result: DetectResponse;
  status: "review" | "accepted" | "rejected";
}

export interface ModelState {
  bodies: Record<string, Body>;
  fileWarnings: PartWarning[];
  links: Link[];
  ignored: string[];
  rootLinkId: string | null;
  joints: Joint[];
  suggestions: Record<string, Suggestion>;
  gripper: GripperSel | null;
  /** The last mount per source (Mount card), kept in the model so Edit > Undo restores it too. */
  mounts: Record<string, Mount>;
}

/** M moved the part from `from` (on body fromBody) flush onto `to` (on body toBody), turned `turn` degrees. */
export interface Mount { M: Mat4; from: Face; to: Face; turn: number; fromBody?: string; toBody?: string }

export type { GripperSel };

export const emptyModel = (): ModelState => ({
  bodies: {}, fileWarnings: [], links: [], ignored: [], rootLinkId: null, joints: [], suggestions: {}, gripper: null,
  mounts: {},
});

export const NAME_RE = /^[A-Za-z0-9_][A-Za-z0-9_-]*$/;
export const DEFAULT_DENSITY = 1240;
export const DENSITY_PRESETS: Record<string, number> = {
  PLA: 1240, ABS: 1040, Aluminium: 2700, Steel: 7850,
};
