export type Shortcut = "undo" | "redo" | "joint" | "group" | "animate" | "focus" | "back";

type KeyLike = Pick<KeyboardEvent, "key" | "ctrlKey" | "metaKey" | "shiftKey"> & { target: EventTarget | null };

const PLAIN: Record<string, Shortcut> = { j: "joint", g: "group", " ": "animate", f: "focus", escape: "back" };

export function shortcutFor(e: KeyLike): Shortcut | null {
  const k = e.key.toLowerCase();
  const t = e.target as HTMLElement | null;
  const typing = !!t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.tagName === "SELECT" || t.isContentEditable);
  if (typing) return k === "escape" ? "back" : null;
  if (e.ctrlKey || e.metaKey) {
    if (k === "z") return e.shiftKey ? "redo" : "undo";
    if (k === "y") return "redo";
    return null;
  }
  return PLAIN[k] ?? null;
}
