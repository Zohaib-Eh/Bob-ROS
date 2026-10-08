import { emptyModel, type ModelState } from "./types";

export const PROJECT_FORMAT = "cad-to-urdf-project";
export const PROJECT_VERSION = 1;

export const saveProject = (m: ModelState) =>
  JSON.stringify({ format: PROJECT_FORMAT, version: PROJECT_VERSION, model: m });

export function loadProject(text: string): ModelState {
  let data: { format?: unknown; version?: unknown; model?: Partial<ModelState> };
  try {
    data = JSON.parse(text);
  } catch {
    throw new Error("This is not a project file (it isn't valid JSON).");
  }
  if (data?.format !== PROJECT_FORMAT) throw new Error("This is not a project file from this app.");
  if (data.version !== PROJECT_VERSION) {
    throw new Error(`Project file version ${String(data.version)} is not supported (expected ${PROJECT_VERSION}).`);
  }
  return { ...emptyModel(), ...data.model };
}
