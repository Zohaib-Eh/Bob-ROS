import { jointAtDefaultLimits } from "./jointDraft";
import { partProblems } from "./problems";
import { directJoints } from "./tree";
import { NAME_RE, type ModelState } from "./types";

export type StepId = "parts" | "group" | "connect" | "export";

export interface CheckItem {
  id: "connected" | "mass" | "names" | "joints" | "limits";
  label: string;
  ok: boolean;
  /** advice only: shown, but does not block export */
  warning?: boolean;
  detail: string | null;
  linkIds: string[];
  jointIds: string[];
  step: StepId;
}

export function exportChecklist(m: ModelState, robotName: string): CheckItem[] {
  const name = (id: string) => m.links.find((l) => l.id === id)?.name ?? id;
  const tree = directJoints(m.links, m.joints, m.rootLinkId);
  const massLinks = [...new Set(partProblems(m).filter((p) => p.kind !== "warning")
    .map((p) => m.links.find((l) => l.body_ids.includes(p.bodyId))?.id)
    .filter((x): x is string => x !== undefined))];
  const seen = new Set<string>();
  const badNames: string[] = [];
  for (const l of m.links) {
    if (!NAME_RE.test(l.name) || seen.has(l.name)) badNames.push(l.id);
    seen.add(l.name);
  }
  const robotOk = NAME_RE.test(robotName);
  const badJoints = m.joints.filter((j) => (j.type !== "fixed" && !j.axis) || j.type === "cylindrical"
    || j.type === "spherical" || ((j.type === "revolute" || j.type === "prismatic")
      && (j.limit_lower === null || j.limit_upper === null || j.limit_lower > j.limit_upper)));
  const defaults = m.joints.filter(jointAtDefaultLimits);
  const empty = m.links.length === 0;
  return [
    { id: "connected", label: "Every link is connected to the base", ok: !empty && tree.errors.length === 0,
      detail: empty ? "Import a model first." : tree.errors[0]?.message ?? null,
      linkIds: [...new Set(tree.errors.flatMap((e) => e.linkIds))],
      jointIds: tree.errors.flatMap((e) => (e.jointId ? [e.jointId] : [])), step: "connect" },
    { id: "mass", label: "Every link has a usable mass", ok: massLinks.length === 0,
      detail: massLinks.length ? `Needs a mass decision: ${massLinks.map(name).join(", ")}` : null,
      linkIds: massLinks, jointIds: [], step: "parts" },
    { id: "names", label: "Link and robot names are valid", ok: robotOk && badNames.length === 0,
      detail: !robotOk ? `Robot name "${robotName}" is not valid.`
        : badNames.length ? `Invalid or duplicate link names: ${badNames.map(name).join(", ")}` : null,
      linkIds: badNames, jointIds: [], step: "group" },
    { id: "joints", label: "Every moving joint has an axis and limits", ok: badJoints.length === 0,
      detail: badJoints.length ? `${badJoints.length} joint(s) need an axis or limits.` : null,
      linkIds: [], jointIds: badJoints.map((j) => j.id), step: "connect" },
    { id: "limits", label: "Joint limits found from geometry (recommended)", ok: defaults.length === 0, warning: true,
      detail: defaults.length
        ? `${defaults.length} joint(s) still use the default limits; find them from geometry in the Connect step.` : null,
      linkIds: [], jointIds: defaults.map((j) => j.id), step: "connect" },
  ];
}
