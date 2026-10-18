import type { Body, Joint, Link, PartWarning } from "../api/types";
import { baseGuess } from "./geometry";
import { sourceOf, transformBody, type Mat4 } from "./mount";
import { membership } from "./pairs";
import { DEFAULT_DENSITY, emptyModel, type GripperSel, type ModelState, type Mount, type Suggestion } from "./types";

export type LinkProps = Partial<Pick<Link, "density_kg_m3" | "mass_override_kg" | "accept_approximate_mass">>;

export type Action =
  | { type: "import"; bodies: Body[]; warnings: PartWarning[]; source?: string }
  | { type: "transformSource"; source: string; matrix: Mat4; mount?: Mount | null }
  | { type: "setGripper"; gripper: GripperSel | null }
  | { type: "merge"; linkIds: string[] }
  | { type: "mergeSmall"; pairs: { linkId: string; neighbourId: string }[] }
  | { type: "moveBody"; bodyId: string; toLinkId: string }
  | { type: "ignoreBody"; bodyId: string }
  | { type: "ignoreBodies"; bodyIds: string[] }
  | { type: "renameBody"; bodyId: string; name: string }
  | { type: "ungroupLink"; linkId: string }
  | { type: "unignoreBody"; bodyId: string }
  | { type: "renameLink"; linkId: string; name: string }
  | { type: "setRoot"; linkId: string }
  | { type: "setLinkProps"; linkId: string; props: LinkProps }
  | { type: "replaceBody"; body: Body }
  | { type: "setSuggestion"; suggestion: Suggestion }
  | { type: "acceptSuggestion"; key: string; joint: Joint }
  | { type: "rejectSuggestion"; key: string }
  | { type: "addJoint"; joint: Joint }
  | { type: "updateJoint"; joint: Joint }
  | { type: "removeJoint"; jointId: string }
  | { type: "setLimits"; changes: LimitChange[] }
  | { type: "load"; model: ModelState };

/** New limits for one joint; `continuous` turns a revolute that spins freely into a no-limit joint. */
export interface LimitChange { jointId: string; lower: number | null; upper: number | null; continuous: boolean }

/** `name`, or `name (2)`, `name (3)`… when an earlier import already used it. */
function uniqueSource(name: string, bodies: Record<string, Body>): string {
  const used = new Set(Object.values(bodies).map((b) => b.source));
  let n = name;
  for (let k = 2; used.has(n); k++) n = `${name} (${k})`;
  return n;
}

export function sanitizeName(raw: string): string {
  let n = raw.trim().replace(/[^A-Za-z0-9_-]/g, "_");
  if (!n) return "part";
  if (!/^[A-Za-z0-9_]/.test(n)) n = "p" + n;
  return n;
}

function uniqueName(base: string, taken: Set<string>): string {
  let name = base;
  for (let k = 2; taken.has(name); k++) name = `${base}_${k}`;
  taken.add(name);
  return name;
}

const newId = () => crypto.randomUUID();

function newLink(body: Body, taken: Set<string>): Link {
  return {
    id: newId(), name: uniqueName(sanitizeName(body.name), taken), body_ids: [body.id],
    density_kg_m3: DEFAULT_DENSITY, mass_override_kg: null, accept_approximate_mass: false,
  };
}

/** Drop empty links, joints/suggestions on missing links, and fix the base link. */
function tidy(s: ModelState): ModelState {
  const links = s.links.filter((l) => l.body_ids.length > 0);
  const ids = new Set(links.map((l) => l.id));
  const joints = s.joints.filter(
    (j) => ids.has(j.parent_link_id) && ids.has(j.child_link_id) && j.parent_link_id !== j.child_link_id);
  const suggestions = Object.fromEntries(
    Object.entries(s.suggestions).filter(([, g]) => ids.has(g.linkA) && ids.has(g.linkB)));
  let rootLinkId = s.rootLinkId && ids.has(s.rootLinkId) ? s.rootLinkId : null;
  if (!rootLinkId && links.length) rootLinkId = baseGuess(links, s.bodies);
  return { ...s, links, joints, suggestions, rootLinkId };
}

function removeBodyFromLinks(links: Link[], bodyId: string): Link[] {
  return links.map((l) => (l.body_ids.includes(bodyId)
    ? { ...l, body_ids: l.body_ids.filter((b) => b !== bodyId) } : l));
}

function mergeLinks(s: ModelState, target: string, others: string[]): ModelState {
  const moved = s.links.filter((l) => others.includes(l.id)).flatMap((l) => l.body_ids);
  const links = s.links
    .filter((l) => !others.includes(l.id))
    .map((l) => (l.id === target ? { ...l, body_ids: [...l.body_ids, ...moved] } : l));
  const remap = (id: string) => (others.includes(id) ? target : id);
  const joints = s.joints.map((j) => ({
    ...j, parent_link_id: remap(j.parent_link_id), child_link_id: remap(j.child_link_id) }));
  return { ...s, links, joints };
}

export function modelReducer(s: ModelState, a: Action): ModelState {
  const next = reduce(s, a);
  return next.gripper && !gripperJointsExist(next) ? { ...next, gripper: null } : next;
}

/** The gripper refers to joints; once either is gone or no longer sliding, the gripper goes too. */
function gripperJointsExist(s: ModelState): boolean {
  const g = s.gripper!;
  return [g.leader, g.follower].every((id) => s.joints.some((j) => j.id === id && j.type === "prismatic"));
}

function reduce(s: ModelState, a: Action): ModelState {
  switch (a.type) {
    case "import": {
      const taken = new Set(s.links.map((l) => l.name));
      const bodies = { ...s.bodies };
      const source = a.source && uniqueSource(a.source, s.bodies);
      for (const b of a.bodies) bodies[b.id] = source ? { ...b, source } : b;
      const links = [...s.links, ...a.bodies.map((b) => newLink(b, taken))];
      return tidy({ ...s, bodies, links, fileWarnings: [...s.fileWarnings, ...a.warnings] });
    }
    case "setGripper":
      return { ...s, gripper: a.gripper };
    case "transformSource": {
      const bodies = { ...s.bodies };
      for (const b of Object.values(s.bodies)) if (sourceOf(b) === a.source) bodies[b.id] = transformBody(b, a.matrix);
      // suggestions hold world-space origins/axes: drop the moved links' ones so they are detected again
      const moved = new Set(s.links.filter((l) => l.body_ids.some((id) => s.bodies[id] && sourceOf(s.bodies[id]) === a.source)).map((l) => l.id));
      const suggestions = Object.fromEntries(Object.entries(s.suggestions)
        .filter(([, g]) => !moved.has(g.linkA) && !moved.has(g.linkB)));
      const mounts = { ...s.mounts };
      if (a.mount) mounts[a.source] = a.mount; else if (a.mount === null) delete mounts[a.source];
      return { ...s, bodies, suggestions, mounts };
    }
    case "merge": {
      const [target, ...others] = [...new Set(a.linkIds)];
      if (!target || !others.length || !s.links.some((l) => l.id === target)) return s;
      return tidy(mergeLinks(s, target, others));
    }
    case "mergeSmall": {
      // Pairs run in order; a link merged earlier resolves to where its bodies went.
      const into = new Map<string, string>();
      const resolve = (id: string) => { while (into.has(id)) id = into.get(id)!; return id; };
      const exists = (id: string) => s.links.some((l) => l.id === id);
      let next = s;
      for (const p of a.pairs) {
        const from = resolve(p.linkId);
        const to = resolve(p.neighbourId);
        if (from === to || !exists(from) || !exists(to)) continue;
        next = mergeLinks(next, to, [from]);
        into.set(from, to);
      }
      return next === s ? s : tidy(next);
    }
    case "moveBody": {
      if (!s.bodies[a.bodyId] || s.ignored.includes(a.bodyId)) return s;
      if (!s.links.some((l) => l.id === a.toLinkId)) return s;
      const links = removeBodyFromLinks(s.links, a.bodyId).map((l) =>
        (l.id === a.toLinkId ? { ...l, body_ids: [...l.body_ids, a.bodyId] } : l));
      return tidy({ ...s, links });
    }
    case "ignoreBody": {
      if (!s.bodies[a.bodyId] || s.ignored.includes(a.bodyId)) return s;
      return tidy({ ...s, links: removeBodyFromLinks(s.links, a.bodyId), ignored: [...s.ignored, a.bodyId] });
    }
    case "ignoreBodies": {
      const ids = [...new Set(a.bodyIds)].filter((id) => s.bodies[id] && !s.ignored.includes(id));
      if (!ids.length) return s;
      let links = s.links;
      for (const id of ids) links = removeBodyFromLinks(links, id);
      return tidy({ ...s, links, ignored: [...s.ignored, ...ids] });
    }
    case "renameBody": {
      const b = s.bodies[a.bodyId];
      const name = a.name.trim();
      if (!b || !name || name === b.name) return s;
      const bodies = { ...s.bodies, [b.id]: { ...b, name } };   // same geometry arrays: no mesh rebuild
      const link = s.links.find((l) => l.body_ids.length === 1 && l.body_ids[0] === b.id);
      const base = sanitizeName(b.name);
      // a single-part link still named after its part follows the part's new name
      const isDefault = (n: string) => n === base || (n.startsWith(`${base}_`) && /^\d+$/.test(n.slice(base.length + 1)));
      if (!link || !isDefault(link.name)) {
        return { ...s, bodies };
      }
      const taken = new Set(s.links.filter((l) => l.id !== link.id).map((l) => l.name));
      const linkName = uniqueName(sanitizeName(name), taken);
      return { ...s, bodies, links: s.links.map((l) => (l.id === link.id ? { ...l, name: linkName } : l)) };
    }
    case "ungroupLink": {
      const link = s.links.find((l) => l.id === a.linkId);
      if (!link || link.body_ids.length < 2) return s;
      const taken = new Set(s.links.map((l) => l.name));
      const [first, ...rest] = link.body_ids;
      const links = s.links.flatMap((l) => (l.id === link.id
        ? [{ ...l, body_ids: [first] }, ...rest.map((id) => newLink(s.bodies[id], taken))] : [l]));
      const joints = s.joints.filter((j) => j.parent_link_id !== link.id && j.child_link_id !== link.id);
      return tidy({ ...s, links, joints });
    }
    case "unignoreBody": {
      if (!s.ignored.includes(a.bodyId)) return s;
      const taken = new Set(s.links.map((l) => l.name));
      return tidy({ ...s, ignored: s.ignored.filter((b) => b !== a.bodyId),
        links: [...s.links, newLink(s.bodies[a.bodyId], taken)] });
    }
    case "renameLink":
      if (!s.links.some((l) => l.id === a.linkId)) return s;
      return { ...s, links: s.links.map((l) => (l.id === a.linkId ? { ...l, name: a.name } : l)) };
    case "setRoot":
      return s.links.some((l) => l.id === a.linkId) ? { ...s, rootLinkId: a.linkId } : s;
    case "setLinkProps":
      if (!s.links.some((l) => l.id === a.linkId)) return s;
      return { ...s, links: s.links.map((l) => (l.id === a.linkId ? { ...l, ...a.props } : l)) };
    case "replaceBody": {
      if (!s.bodies[a.body.id]) return s;
      const lid = s.links.find((l) => l.body_ids.includes(a.body.id))?.id;   // geometry changed, membership didn't: re-detect
      const suggestions = Object.fromEntries(Object.entries(s.suggestions)
        .filter(([, g]) => g.linkA !== lid && g.linkB !== lid));
      return { ...s, bodies: { ...s.bodies, [a.body.id]: a.body }, suggestions };
    }
    case "setSuggestion": {
      const la = s.links.find((l) => l.id === a.suggestion.linkA);
      const lb = s.links.find((l) => l.id === a.suggestion.linkB);
      if (!la || !lb || membership(la, lb) !== a.suggestion.membership) return s;   // stale result
      return { ...s, suggestions: { ...s.suggestions, [a.suggestion.key]: a.suggestion } };
    }
    case "acceptSuggestion": {
      const g = s.suggestions[a.key];
      if (!g) return s;
      return { ...s, joints: [...s.joints.filter((j) => j.id !== a.joint.id), { ...a.joint, source: "confirmed" }],
        suggestions: { ...s.suggestions, [a.key]: { ...g, status: "accepted" } } };
    }
    case "rejectSuggestion": {
      const g = s.suggestions[a.key];
      return g ? { ...s, suggestions: { ...s.suggestions, [a.key]: { ...g, status: "rejected" } } } : s;
    }
    case "addJoint":
      return tidy({ ...s, joints: [...s.joints, a.joint] });
    case "updateJoint":
      return { ...s, joints: s.joints.map((j) => (j.id === a.joint.id ? a.joint : j)) };
    case "setLimits": {
      if (!a.changes.length) return s;
      const byId = new Map(a.changes.map((c) => [c.jointId, c]));
      return { ...s, joints: s.joints.map((j) => {
        const c = byId.get(j.id);
        if (!c) return j;
        return c.continuous ? { ...j, type: "continuous", limit_lower: null, limit_upper: null }
          : { ...j, limit_lower: c.lower, limit_upper: c.upper };
      }) };
    }
    case "removeJoint": {
      const suggestions = Object.fromEntries(Object.entries(s.suggestions).map(([k, g]) =>
        [k, g.result.joint?.id === a.jointId && g.status === "accepted" ? { ...g, status: "review" as const } : g]));
      return { ...s, joints: s.joints.filter((j) => j.id !== a.jointId), suggestions };
    }
    case "load":
      return { ...emptyModel(), ...a.model };
  }
}
