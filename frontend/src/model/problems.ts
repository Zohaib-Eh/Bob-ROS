import type { PartWarning } from "../api/types";
import type { ModelState } from "./types";

export type Problem =
  | { kind: "failed"; bodyId: string }
  | { kind: "approximate"; bodyId: string; linkId: string }
  | { kind: "warning"; bodyId: string; warning: PartWarning };

/** Problems the Parts step shows, in body order. Ignored parts and resolved approximate parts drop out. */
export function partProblems(m: ModelState): Problem[] {
  const out: Problem[] = [];
  for (const b of Object.values(m.bodies)) {
    if (m.ignored.includes(b.id)) continue;
    const link = m.links.find((l) => l.body_ids.includes(b.id));
    if (b.status === "failed") {
      out.push({ kind: "failed", bodyId: b.id });
    } else if (b.status === "approximate") {
      if (link && !link.accept_approximate_mass && link.mass_override_kg === null) {
        out.push({ kind: "approximate", bodyId: b.id, linkId: link.id });
      }
    } else {
      for (const w of b.warnings) out.push({ kind: "warning", bodyId: b.id, warning: w });
    }
  }
  return out;
}
