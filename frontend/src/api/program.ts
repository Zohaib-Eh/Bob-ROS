import { json, parseSse as sse, readEvents, send } from "./client";
import type { Message } from "../model/chat";
import type { Body, Joint, Link } from "./types";

export type ProgramEvent =
  | { kind: "text"; text: string }
  | { kind: "file"; path: string }
  | { kind: "run"; n: number }
  | { kind: "run_result"; ok: boolean; message: string; output?: string }
  | { kind: "done"; summary: string; runs: number; ok: boolean; session_id: string | null; program: string | null; code: string | null }
  | { kind: "error"; message: string };
export type Stage = "export" | "collisions" | "image" | "build" | "physics" | "moveit" | "bridge" | "ready";
export interface SessionStatus { state: "stopped" | "starting" | "ready" | "failed"; robot?: string | null;
  stage?: Stage; done?: number; total?: number; log?: string; error?: string;
  physics?: string }  // when ready: "mujoco" | "off: <reason>"

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

export interface ConversationInfo { id: string; title: string; created?: number; updated?: number; broken?: boolean }
export interface Conversation { id: string; title: string; created: number; updated: number;
  claude_session_id: string | null; edited: string[]; messages: Message[] }
export interface FileEntry { path: string; editable: boolean }

const put = (body: unknown): RequestInit => ({ ...json(body), method: "PUT" });
const patch = (body: unknown): RequestInit => ({ ...json(body), method: "PATCH" });

const q = (robot: string) => `?robot=${encodeURIComponent(robot)}`;
export async function listConversations(robot: string): Promise<ConversationInfo[]> {
  return (await send(`/program/conversations${q(robot)}`, { method: "GET" })).json();
}
export async function createConversation(robot: string, title?: string): Promise<Conversation> {
  return (await send(`/program/conversations${q(robot)}`, json(title ? { title } : {}))).json();
}
export async function getConversation(robot: string, id: string): Promise<Conversation> {
  return (await send(`/program/conversations/${id}${q(robot)}`, { method: "GET" })).json();
}
export async function renameConversation(robot: string, id: string, title: string): Promise<Conversation> {
  return (await send(`/program/conversations/${id}${q(robot)}`, patch({ title }))).json();
}
export async function deleteConversation(robot: string, id: string): Promise<void> {
  await send(`/program/conversations/${id}${q(robot)}`, { method: "DELETE" });
}
export async function runConversation(id: string, task: string, onEvent: (e: ProgramEvent) => void,
  signal?: AbortSignal): Promise<void> {
  await streamEvents(`/program/conversations/${id}/run`, { task }, onEvent, signal);
}
export async function listFiles(): Promise<FileEntry[]> {
  return (await send("/program/files", { method: "GET" })).json();
}
export async function readFile(path: string): Promise<{ path: string; content: string; editable: boolean }> {
  return (await send(`/program/files/content?path=${encodeURIComponent(path)}`, { method: "GET" })).json();
}
export async function saveFile(path: string, content: string, conversation: string | null): Promise<void> {
  await send("/program/files/content", put({ path, content, conversation }));
}
export async function deleteFile(path: string): Promise<void> {
  await send(`/program/files/content?path=${encodeURIComponent(path)}`, { method: "DELETE" });
}
export async function runFile(path: string, conversation: string | null): Promise<{ ok: boolean; output: string }> {
  return (await send("/program/files/run", json({ path, conversation }))).json();
}

export const parseSse = (buffer: string) => sse<ProgramEvent>(buffer);

async function streamEvents(path: string, body: unknown, onEvent: (e: ProgramEvent) => void,
  signal?: AbortSignal): Promise<void> {
  await readEvents<ProgramEvent>(await send(path, { ...json(body), signal }), onEvent);
}
