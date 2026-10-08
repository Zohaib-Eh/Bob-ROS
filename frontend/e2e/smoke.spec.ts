import { expect, test } from "@playwright/test";
import { execFileSync } from "node:child_process";
import { join } from "node:path";
import { FIXTURES, PY } from "./globalSetup";

test("peg-in-hole: import → connect (suggested) → undo/redo → export", async ({ page }) => {
  await page.goto("/");
  await page.getByTestId("import-input").setInputFiles(join(FIXTURES, "peg_in_hole.step"));
  await expect(page.getByTestId(/^link-/)).toHaveCount(2);

  await page.getByTestId("step-connect").click();
  await expect(page.getByTestId("next-card")).toContainText("Suggested", { timeout: 60_000 });
  await page.getByTestId("next-joint").click();
  await expect(page.getByTestId("suggested")).toContainText("cylindrical");
  await expect(page.getByTestId("motion-revolute")).toHaveClass(/active/);
  await page.getByTestId("animate").click();
  await page.getByTestId("animate").click();
  await page.getByTestId("ok").click();
  await expect(page.getByTestId(/^jointrow-/)).toHaveCount(1);
  await expect(page.getByTestId("all-connected")).toBeVisible();

  await page.getByTestId("undo").click();
  await expect(page.getByTestId(/^jointrow-/)).toHaveCount(0);
  await page.getByTestId("redo").click();
  await expect(page.getByTestId(/^jointrow-/)).toHaveCount(1);

  await page.getByTestId("step-export").click();
  await expect(page.getByTestId("check-connected")).toHaveClass(/ok/);
  await page.getByTestId("robot-name").fill("peg_robot");
  const [download] = await Promise.all([page.waitForEvent("download"), page.getByTestId("export").click()]);
  const path = join(FIXTURES, "peg_robot.zip");
  await download.saveAs(path);
  const names = execFileSync(PY, ["-c", "import sys, zipfile; print('\\n'.join(zipfile.ZipFile(sys.argv[1]).namelist()))", path])
    .toString().trim().split(/\r?\n/);
  expect(names).toContain("peg_robot/urdf/peg_robot.urdf");
  expect(names.filter((n: string) => n.startsWith("peg_robot/meshes/"))).toHaveLength(2);
});
