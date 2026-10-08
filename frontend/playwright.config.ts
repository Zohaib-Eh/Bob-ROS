import { defineConfig } from "@playwright/test";
import { homedir } from "node:os";
import { join } from "node:path";

const py = process.env.CAD_TO_URDF_PYTHON ?? join(homedir(), "miniforge3", "envs", "cad-to-urdf", "python.exe");

export default defineConfig({
  testDir: "e2e",
  timeout: 120_000,
  globalSetup: "./e2e/globalSetup.ts",
  use: { baseURL: "http://localhost:5173", acceptDownloads: true },
  webServer: [
    { command: `"${py}" -m uvicorn app.main:app --port 8000`, cwd: "../backend", url: "http://localhost:8000/health",
      reuseExistingServer: true, env: { OPENBLAS_NUM_THREADS: "1" }, timeout: 120_000 },
    { command: "npm run dev", url: "http://localhost:5173", reuseExistingServer: true, timeout: 120_000 },
  ],
});
