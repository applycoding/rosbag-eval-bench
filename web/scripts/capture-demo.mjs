// SPDX-License-Identifier: Apache-2.0
import { chromium } from "playwright";
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../..");
const base = process.argv[2] ?? "http://127.0.0.1:4173/rosbag-eval-bench/";

const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 960, height: 720 } });
await page.goto(base, { waitUntil: "networkidle" });
await page.waitForSelector("table.diff");
const out = path.join(root, "docs", "demo.png");
await mkdir(path.dirname(out), { recursive: true });
await page.screenshot({ path: out, fullPage: true });
await browser.close();
console.log(`wrote ${out}`);
