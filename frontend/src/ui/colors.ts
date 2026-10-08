import type { StepId } from "../model/checklist";
import { draftPreviewJoint, type JointDraft } from "../model/jointDraft";
import type { ModelState } from "../model/types";
import type { JointGlyph } from "../viewer/overlays";

export const LINK_HUES = [0x93c5fd, 0xfca5a5, 0x86efac, 0xfde68a, 0xc4b5fd, 0x99f6e4, 0xf9a8d4, 0xfdba74];
export const FIXED_TINT = 0x2563eb;
export const MOVING_TINT = 0xf59e0b;
export const hex = (n: number) => `#${n.toString(16).padStart(6, "0")}`;

export function tintsFor(step: StepId, m: ModelState, draft: JointDraft | null): Record<string, number> {
  const t: Record<string, number> = {};
  const paint = (linkId: string | null, c: number) => {
    for (const b of m.links.find((l) => l.id === linkId)?.body_ids ?? []) t[b] = c;
  };
  if (step === "connect" && draft) {
    paint(draft.fixed, FIXED_TINT);
    paint(draft.moving, MOVING_TINT);
  } else if (step === "group") {
    m.links.forEach((l, i) => paint(l.id, LINK_HUES[i % LINK_HUES.length]));
  }
  return t;
}

export function glyphsFor(step: StepId, m: ModelState, draft: JointDraft | null): JointGlyph[] {
  if (step !== "connect" && step !== "export") return [];
  const out: JointGlyph[] = m.joints.filter((j) => j.id !== draft?.id).map((joint) => ({ joint, state: "done" }));
  if (step === "connect") {
    if (!draft) {
      for (const g of Object.values(m.suggestions)) {
        if (g.status === "review" && g.result.joint) out.push({ joint: g.result.joint, state: "suggested" });
      }
    }
    const p = draft ? draftPreviewJoint(draft) : null;
    if (p) out.push({ joint: p, state: "active" });
  }
  return out;
}
