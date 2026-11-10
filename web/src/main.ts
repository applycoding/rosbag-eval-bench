// SPDX-License-Identifier: Apache-2.0
import "./style.css";

type CompareRow = {
  name: string;
  baseline: number | null;
  candidate: number | null;
  delta: number | null;
  passed: boolean;
  gated?: boolean;
};

type Results = {
  suite: string;
  comparison_passed?: boolean;
  compare_rows?: CompareRow[];
  runs: Array<{ name: string; metrics: Record<string, number | null>; series?: Record<string, unknown> }>;
};

async function loadJson(name: string): Promise<Results> {
  const url = `${import.meta.env.BASE_URL}${name}`;
  const res = await fetch(url);
  if (!res.ok) {
    throw new Error(`failed to load ${url}`);
  }
  return res.json() as Promise<Results>;
}

function renderTable(title: string, data: Results, root: HTMLElement) {
  const h2 = document.createElement("h2");
  h2.textContent = title;
  root.appendChild(h2);
  const p = document.createElement("p");
  p.textContent = `Suite: ${data.suite} · comparison ${data.comparison_passed ? "PASS" : "FAIL"}`;
  root.appendChild(p);
  const table = document.createElement("table");
  table.className = "diff";
  const head = document.createElement("tr");
  head.innerHTML = "<th>Metric</th><th>Baseline</th><th>Candidate</th><th>Delta</th><th>Result</th>";
  table.appendChild(head);
  for (const row of data.compare_rows ?? []) {
    if (!row.gated) continue;
    const tr = document.createElement("tr");
    const cls = row.passed ? "pass" : "fail";
    tr.innerHTML = `<td>${row.name}</td><td>${fmt(row.baseline)}</td><td>${fmt(row.candidate)}</td><td>${fmt(row.delta)}</td><td class="${cls}">${row.passed ? "PASS" : "FAIL"}</td>`;
    table.appendChild(tr);
  }
  root.appendChild(table);
}

function fmt(v: number | null | undefined): string {
  if (v === null || v === undefined) return "n/a";
  return String(v);
}

function drawCanvas(canvas: HTMLCanvasElement, data: Results) {
  const run = data.runs[0];
  if (!run?.series?.path_poses) return;
  const poses = run.series.path_poses as Array<{ x: number; y: number }>;
  const ctx = canvas.getContext("2d");
  if (!ctx || poses.length === 0) return;
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  const xs = poses.map((p) => p.x);
  const ys = poses.map((p) => p.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const pad = 20;
  const sx = (w - 2 * pad) / Math.max(maxX - minX, 0.1);
  const sy = (h - 2 * pad) / Math.max(maxY - minY, 0.1);
  const s = Math.min(sx, sy);
  ctx.strokeStyle = "#06c";
  ctx.beginPath();
  poses.forEach((p, i) => {
    const x = pad + (p.x - minX) * s;
    const y = h - pad - (p.y - minY) * s;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

async function main() {
  const root = document.getElementById("app");
  if (!root) return;
  const [baseline, degraded] = await Promise.all([
    loadJson("baseline.json"),
    loadJson("degraded.json"),
  ]);
  renderTable("Baseline (nominal candidate)", baseline, root);
  renderTable("Degraded candidate", degraded, root);
  const canvas = document.getElementById("path-canvas") as HTMLCanvasElement | null;
  if (canvas) drawCanvas(canvas, degraded);
}

main().catch((err) => {
  const root = document.getElementById("app");
  if (root) root.textContent = String(err);
});
