// SPDX-License-Identifier: Apache-2.0
import { expect, test } from "@playwright/test";

const base = process.env.PLAYWRIGHT_BASE_URL ?? "http://127.0.0.1:4173/rosbag-eval-bench/";

test("shows degraded FAIL rows", async ({ page }) => {
  await page.goto(base);
  await expect(page.getByText("Degraded candidate")).toBeVisible();
  await expect(page.locator("td.fail").first()).toContainText("FAIL");
});
