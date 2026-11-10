// SPDX-License-Identifier: Apache-2.0
import { defineConfig } from "vite";

export default defineConfig({
  base: "/rosbag-eval-bench/",
  publicDir: "../fixtures/web",
  build: {
    outDir: "dist",
    emptyOutDir: true,
  },
});
