import { execFileSync } from "node:child_process";
import { mkdirSync } from "node:fs";
import { homedir } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));

export const PY = process.env.CAD_TO_URDF_PYTHON ?? join(homedir(), "miniforge3", "envs", "cad-to-urdf", "python.exe");
export const FIXTURES = resolve(here, "fixtures");

export default function globalSetup() {
  mkdirSync(FIXTURES, { recursive: true });
  execFileSync(PY, ["-m", "corpus.generate", FIXTURES], {
    cwd: resolve(here, "../../backend"), env: { ...process.env, OPENBLAS_NUM_THREADS: "1" } });
}
