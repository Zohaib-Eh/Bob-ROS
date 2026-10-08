import { expect, test } from "@playwright/test";
import { execFileSync } from "node:child_process";
import { join } from "node:path";
import { FIXTURES, PY } from "./globalSetup";

test("peg-in-hole: import → suggestion → accept → preview → undo/redo → export", async ({ page }) => {
  await page.goto("/");
  await page.getByTestId("import-input").setInputFiles(join(FIXTURES, "peg_in_hole.step"));
  await expect(page.getByTestId(/^link-/)).toHaveCount(2);

  const suggestion = page.getByTestId(/^suggestion-/);
  await expect(suggestion).toHaveCount(1, { timeout: 60_000 });
  await suggestion.click();
  await page.getByTestId("joint-type").selectOption("continuous");
  await page.getByTestId("accept").click();

  const slider = page.getByTestId("preview-slider");
  await slider.fill("1");
  await expect(slider).toHaveAttribute("data-preview-value", "1");

  await page.getByTestId("undo").click();
  await expect(page.getByTestId("accept")).toBeEnabled();      // back to review
  await page.getByTestId("redo").click();
  await expect(page.getByTestId("accept")).toHaveCount(0);      // accepted again

  await page.getByTestId("robot-name").fill("peg_robot");
  const [download] = await Promise.all([page.waitForEvent("download"), page.getByTestId("export").click()]);
  const path = join(FIXTURES, "peg_robot.zip");
  await download.saveAs(path);
  const names = execFileSync(PY, ["-c", "import sys, zipfile; print('\\n'.join(zipfile.ZipFile(sys.argv[1]).namelist()))", path])
    .toString().trim().split(/\r?\n/);
  expect(names).toContain("peg_robot/urdf/peg_robot.urdf");
  expect(names.filter((n) => n.startsWith("peg_robot/meshes/"))).toHaveLength(2);
});
