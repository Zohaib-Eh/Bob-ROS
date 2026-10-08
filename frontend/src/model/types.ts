import type { Body, DetectResponse, Joint, Link, PartWarning } from "../api/types";

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
}

export const emptyModel = (): ModelState => ({
  bodies: {}, fileWarnings: [], links: [], ignored: [], rootLinkId: null, joints: [], suggestions: {},
});

export const NAME_RE = /^[A-Za-z0-9_][A-Za-z0-9_-]*$/;
export const DEFAULT_DENSITY = 1240;
export const DENSITY_PRESETS: Record<string, number> = {
  PLA: 1240, ABS: 1040, Aluminium: 2700, Steel: 7850,
};
