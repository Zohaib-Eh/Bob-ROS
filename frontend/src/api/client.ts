import type { Body, DetectResponse, ImportResponse, Joint, Link, LinkMass, PartWarning, SweepResult, Vec3 } from "./types";

export class ApiError extends Error {
  constructor(public status: number, message: string) {
    super(message);
  }
}

export function errorText(detail: unknown): string {
  if (typeof detail === "string") return detail;
  if (Array.isArray(detail)) {
    return detail.map((d: { loc?: unknown[]; msg?: string }) =>
      `${(d.loc ?? []).join(".")}: ${d.msg ?? ""}`).join("; ");
  }
  return "Request failed";
}

export async function send(path: string, init: RequestInit): Promise<Response> {
  let res: Response;
  try {
    res = await fetch(`/api${path}`, init);
  } catch (e) {
    if ((e as Error).name === "AbortError") throw e;
    throw new ApiError(0, "Cannot reach the backend — is it running on localhost:8000?");
  }
  if (!res.ok) {
    let detail: unknown = res.statusText;
    try { detail = (await res.json()).detail; } catch { /* non-JSON error body */ }
    throw new ApiError(res.status, errorText(detail));
  }
  return res;
}

export const json = (body: unknown): RequestInit => ({
  method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
});

export async function importFile(file: File, scale: number): Promise<ImportResponse> {
  const form = new FormData();
  form.append("file", file);
  return (await send(`/import?scale=${scale}`, { method: "POST", body: form })).json();
}

export async function replacePart(
  file: File, bodyId: string, name: string, bbox: { min: Vec3; max: Vec3 } | null, scale: number,
): Promise<Body> {
  const form = new FormData();
  form.append("file", file);
  form.append("body_id", bodyId);
  form.append("name", name);
  form.append("scale", String(scale));
  if (bbox) {
    form.append("bbox_min", bbox.min.join(","));
    form.append("bbox_max", bbox.max.join(","));
  }
  return (await send("/replace-part", { method: "POST", body: form })).json();
}

export async function detectJoints(
  a: Body[], b: Body[], linkA: string, linkB: string, signal?: AbortSignal,
): Promise<DetectResponse> {
  const init = json({ bodies_a: a, bodies_b: b, parent_link_id: linkA, child_link_id: linkB });
  return (await send("/detect-joints", { ...init, signal })).json();
}

export async function linkMass(link: Link, bodies: Body[]): Promise<LinkMass> {
  return (await send("/link-mass", json({ link, bodies }))).json();
}

export async function exportRobot(
  robotName: string, links: Link[], joints: Joint[], bodiesByLink: Record<string, Body[]>,
): Promise<{ blob: Blob; warnings: PartWarning[] }> {
  const res = await send("/export", json({
    robot_name: robotName, links, joints, bodies_by_link: bodiesByLink }));
  const header = res.headers.get("X-Export-Warnings");
  return { blob: await res.blob(), warnings: header ? JSON.parse(header) : [] };
}

export async function sweepLimits(
  joint: { type: "revolute" | "prismatic"; origin_xyz: Vec3; axis: Vec3 },
  moving: Body[], obstacles: { link_id: string; bodies: Body[] }[],
): Promise<SweepResult> {
  return (await send("/sweep-limits", json({ joint, moving, obstacles }))).json();
}
