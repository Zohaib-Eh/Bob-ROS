import { modelReducer, type Action } from "./reducer";
import { emptyModel, type ModelState } from "./types";

export const HISTORY_CAP = 100;
const NOT_UNDOABLE = new Set<Action["type"]>(["setSuggestion"]);

export interface History { past: ModelState[]; present: ModelState; future: ModelState[] }
export type HistoryAction = Action | { type: "undo" } | { type: "redo" };

export const initialHistory = (): History => ({ past: [], present: emptyModel(), future: [] });

export function historyReducer(h: History, a: HistoryAction): History {
  if (a.type === "undo") {
    if (!h.past.length) return h;
    return { past: h.past.slice(0, -1), present: h.past[h.past.length - 1], future: [h.present, ...h.future] };
  }
  if (a.type === "redo") {
    if (!h.future.length) return h;
    return { past: [...h.past, h.present], present: h.future[0], future: h.future.slice(1) };
  }
  const next = modelReducer(h.present, a);
  if (next === h.present) return h;
  if (a.type === "load") return { past: [], present: next, future: [] };
  if (NOT_UNDOABLE.has(a.type)) return { ...h, present: next };
  return { past: [...h.past, h.present].slice(-HISTORY_CAP), present: next, future: [] };
}
