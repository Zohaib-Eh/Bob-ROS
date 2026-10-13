import { json, send } from "./client";
import type { Body, Joint, Link } from "./types";

export type ProgramEvent =
  | { kind: "text"; text: string }
  | { kind: "file"; path: string }
  | { kind: "run"; n: number }
  | { kind: "run_result"; ok: boolean; message: string }
  | { kind: "done"; summary: string; runs: number; ok: boolean; session_id: string | null; program: string | null; code: string | null }
  | { kind: "error"; message: string };
export interface SessionStatus { state: "stopped" | "starting" | "ready" | "failed"; robot?: string | null; log?: string }

export async function startSession(robotName: string, links: Link[], joints: Joint[],
  bodiesByLink: Record<string, Body[]>): Promise<SessionStatus> {
  return (await send("/program/session", json({ robot_name: robotName, links, joints, bodies_by_link: bodiesByLink }))).json();
}
export async function sessionStatus(): Promise<SessionStatus> {
  return (await send("/program/session", { method: "GET" })).json();
}
export async function stopSession(): Promise<void> {
  await send("/program/session", { method: "DELETE" });
}
export async function rerun(program: string): Promise<{ ok: boolean; output: string }> {
  return (await send("/program/rerun", json({ program }))).json();
}

export function parseSse(buffer: string): { events: ProgramEvent[]; rest: string } {
  const parts = buffer.split("\n\n");
  const rest = parts.pop() ?? "";
  const events = parts.filter((p) => p.startsWith("data: ")).map((p) => JSON.parse(p.slice(6)) as ProgramEvent);
  return { events, rest };
}

export async function runTask(task: string, resume: string | null, onEvent: (e: ProgramEvent) => void,
  signal?: AbortSignal): Promise<void> {
  const res = await send("/program/run", { ...json({ task, resume }), signal });
  const reader = res.body!.getReader();
  const decoder = new TextDecoder();
  let buffer = "";
  for (;;) {
    const { done, value } = await reader.read();
    if (value) buffer += decoder.decode(value, { stream: true });
    const { events, rest } = parseSse(buffer);
    buffer = rest;
    events.forEach(onEvent);
    if (done) break;
  }
}
