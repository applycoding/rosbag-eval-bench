// SPDX-License-Identifier: Apache-2.0
import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./tests",
  use: {
    baseURL: process.env.PLAYWRIGHT_BASE_URL ?? "http://127.0.0.1:4173/rosbag-eval-bench/",
  },
});
