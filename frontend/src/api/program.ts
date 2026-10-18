import { gripperBody, json, mobileBody, parseSse as sse, readEvents, send, type GripperSel, type MobileSel } from "./client";
import type { Message } from "../model/chat";
import type { Body, Joint, Link } from "./types";

export type ProgramEvent =
  | { kind: "text"; text: string }
  | { kind: "file"; path: string }
  | { kind: "run"; n: number }
  | { kind: "run_result"; ok: boolean; message: string; output?: string; trace?: string | null }
  | { kind: "done"; summary: string; runs: number; ok: boolean; session_id: string | null; program: string | null; code: string | null }
  | { kind: "error"; message: string };
export type Stage = "export" | "collisions" | "image" | "build" | "physics" | "moveit" | "bridge" | "ready";
export interface SessionStatus { state: "stopped" | "starting" | "ready" | "failed"; robot?: string | null;
  stage?: Stage; done?: number; total?: number; log?: string; error?: string;
  physics?: string }  // when ready: "mujoco" | "off: <reason>"

export async function startSession(robotName: string, links: Link[], joints: Joint[],
  bodiesByLink: Record<string, Body[]>, gripper: GripperSel | null = null, mobile: MobileSel | null = null): Promise<SessionStatus> {
  return (await send("/program/session", json({ robot_name: robotName, links, joints, bodies_by_link: bodiesByLink,
    gripper: gripperBody(gripper), mobile: mobileBody(mobile) }))).json();
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
/** One object of <robot>_tasks/workcell.yaml (base frame, metres; box size = full x y z, cylinder = radius, height). */
export interface WorkcellObject {
  name: string; shape: "box" | "cylinder" | "mesh"; size: number[]; xyz: [number, number, number]; rpy: [number, number, number];
  fixed: boolean; mass: number | null; friction: number | null;
  file?: string; pieces?: number;   // a CAD fixture (shape mesh): its <name>.stl and convex piece count; size = bounding box
}
export interface FixtureMesh { vertices: [number, number, number][]; triangles: [number, number, number][] }
/** A CAD file (STEP, IGES, STL, OBJ...) as a bolted-down fixture; scale applies to unitless files (STL: mm = 0.001). */
export async function importFixture(robot: string, file: File, scale: number): Promise<Workcell> {
  const form = new FormData();
  form.append("robot", robot);
  form.append("scale", String(scale));
  form.append("file", file);
  return (await send("/program/workcell/fixture", { method: "POST", body: form })).json();
}
/** Workcell > Save: workcell.yaml plus its fixtures' meshes, zipped. */
export async function getWorkcellBundle(robot: string): Promise<Blob> {
  return (await send(`/program/workcell/bundle${q(robot)}`, { method: "GET" })).blob();
}
/** Workcell > Load: a zip from Save, or a workcell.yaml, in place of the current cell. */
export async function loadWorkcellBundle(robot: string, file: File): Promise<Workcell> {
  const form = new FormData();
  form.append("file", file);
  return (await send(`/program/workcell/bundle${q(robot)}`, { method: "POST", body: form })).json();
}
export async function getFixtureMesh(robot: string, file: string): Promise<FixtureMesh> {
  return (await send(`/program/workcell/mesh${q(robot)}&file=${encodeURIComponent(file)}`, { method: "GET" })).json();
}
/** reach: [height, how far out the tool point gets there] per 10 cm band, bottom up; null until ROS has set the robot up. */
export interface Workcell { objects: WorkcellObject[]; error: string | null; reach: [number, number][] | null }
export async function getWorkcell(robot: string): Promise<Workcell> {
  return (await send(`/program/workcell${q(robot)}`, { method: "GET" })).json();
}
export async function putWorkcell(robot: string, objects: WorkcellObject[]): Promise<WorkcellObject[]> {
  return (await (await send("/program/workcell", put({ robot, objects }))).json()).objects;
}
/** A run's recording (runs/<program>-<UTC>.json, from the run's `trace`), read from the robot's workspace. */
export async function getTrace(robot: string, trace: string): Promise<import("../model/replay").Trace> {
  const name = trace.replace(/^runs\//, "");
  return (await send(`/program/traces/${encodeURIComponent(name)}${q(robot)}`, { method: "GET" })).json();
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
export async function runFile(path: string, conversation: string | null): Promise<{ ok: boolean; output: string; trace?: string | null }> {
  return (await send("/program/files/run", json({ path, conversation }))).json();
}

export const parseSse = (buffer: string) => sse<ProgramEvent>(buffer);

async function streamEvents(path: string, body: unknown, onEvent: (e: ProgramEvent) => void,
  signal?: AbortSignal): Promise<void> {
  await readEvents<ProgramEvent>(await send(path, { ...json(body), signal }), onEvent);
}
