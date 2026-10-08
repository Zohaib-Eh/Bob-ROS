import type { Joint, Link } from "../api/types";

export interface TreeError { linkIds: string[]; jointId?: string; message: string }

export function directJoints(links: Link[], joints: Joint[], rootId: string | null):
  { joints: Joint[]; errors: TreeError[] } {
  const errors: TreeError[] = [];
  if (!rootId || !links.some((l) => l.id === rootId)) {
    return { joints: [], errors: [{ linkIds: [], message: "No base link chosen." }] };
  }
  const seenPairs = new Set<string>();
  const usable: Joint[] = [];
  for (const j of joints) {
    const key = [j.parent_link_id, j.child_link_id].sort().join("|");
    if (seenPairs.has(key)) {
      errors.push({ linkIds: [j.parent_link_id, j.child_link_id], jointId: j.id,
        message: "These links have more than one joint between them." });
    } else {
      seenPairs.add(key);
      usable.push(j);
    }
  }
  const visited = new Set([rootId]);
  const directed: Joint[] = [];
  const used = new Set<string>();
  const queue = [rootId];
  while (queue.length) {
    const cur = queue.shift()!;
    for (const j of usable) {
      if (used.has(j.id)) continue;
      const other = j.parent_link_id === cur ? j.child_link_id : j.child_link_id === cur ? j.parent_link_id : null;
      if (other === null) continue;
      used.add(j.id);
      if (visited.has(other)) {
        errors.push({ linkIds: [cur, other], jointId: j.id,
          message: "This joint closes a loop; a URDF must be a tree." });
        continue;
      }
      visited.add(other);
      directed.push({ ...j, parent_link_id: cur, child_link_id: other });
      queue.push(other);
    }
  }
  const unreached = links.filter((l) => !visited.has(l.id)).map((l) => l.id);
  if (unreached.length) {
    errors.push({ linkIds: unreached, message: "Not connected to the base link by any joint." });
  }
  return { joints: directed, errors };
}
