import type { Body, Joint, Link } from "../api/types";
import { directJoints, type TreeError } from "./tree";
import type { GripperSel, ModelState } from "./types";

export function buildExportRequest(m: ModelState): { links: Link[]; joints: Joint[]; bodiesByLink: Record<string, Body[]>;
  errors: TreeError[]; gripper: GripperSel | null } {
  const { joints, errors } = directJoints(m.links, m.joints, m.rootLinkId);
  const bodiesByLink = Object.fromEntries(m.links.map((l) => [l.id, l.body_ids.map((id) => m.bodies[id])]));
  return { links: m.links, joints, bodiesByLink, errors, gripper: m.gripper ?? null };
}
