import type { Body, Joint, Link, PartWarning } from "../api/types";
import { baseGuess } from "./geometry";
import { membership } from "./pairs";
import { DEFAULT_DENSITY, emptyModel, type ModelState, type Suggestion } from "./types";

export type LinkProps = Partial<Pick<Link, "density_kg_m3" | "mass_override_kg" | "accept_approximate_mass">>;

export type Action =
  | { type: "import"; bodies: Body[]; warnings: PartWarning[] }
  | { type: "merge"; linkIds: string[] }
  | { type: "mergeSmall"; pairs: { linkId: string; neighbourId: string }[] }
  | { type: "moveBody"; bodyId: string; toLinkId: string }
  | { type: "ignoreBody"; bodyId: string }
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
  | { type: "load"; model: ModelState };

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
  switch (a.type) {
    case "import": {
      const taken = new Set(s.links.map((l) => l.name));
      const bodies = { ...s.bodies };
      for (const b of a.bodies) bodies[b.id] = b;
      const links = [...s.links, ...a.bodies.map((b) => newLink(b, taken))];
      return tidy({ ...s, bodies, links, fileWarnings: [...s.fileWarnings, ...a.warnings] });
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
    case "removeJoint": {
      const suggestions = Object.fromEntries(Object.entries(s.suggestions).map(([k, g]) =>
        [k, g.result.joint?.id === a.jointId && g.status === "accepted" ? { ...g, status: "review" as const } : g]));
      return { ...s, joints: s.joints.filter((j) => j.id !== a.jointId), suggestions };
    }
    case "load":
      return { ...emptyModel(), ...a.model };
  }
}
