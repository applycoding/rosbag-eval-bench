# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from pathlib import Path

from jinja2 import Environment, FileSystemLoader, select_autoescape

_TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"


def write_markdown(results: dict, path: Path) -> None:
    env = Environment(
        loader=FileSystemLoader(str(_TEMPLATE_DIR)),
        autoescape=select_autoescape(enabled_extensions=()),
    )
    tpl = env.get_template("report.md.j2")
    path.write_text(tpl.render(results=results), encoding="utf-8")


def write_html(results: dict, path: Path, figures_dir: Path) -> None:
    env = Environment(
        loader=FileSystemLoader(str(_TEMPLATE_DIR)),
        autoescape=select_autoescape(["html", "xml"]),
    )
    tpl = env.get_template("report.html.j2")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(tpl.render(results=results, figures=figures_dir.name), encoding="utf-8")


def write_figures(results: dict, out_dir: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    for run in results.get("runs", []):
        series = run.get("series", {})
        ate = series.get("ate_series")
        if ate:
            fig, ax = plt.subplots(figsize=(6, 3))
            ax.plot([p["t_s"] for p in ate], [p["err_m"] for p in ate])
            ax.set_xlabel("time (s)")
            ax.set_ylabel("aligned error (m)")
            ax.set_title(f"ATE series — {run['name']}")
            fig.tight_layout()
            fig.savefig(out_dir / f"ate_{run['name']}.png")
            plt.close(fig)
        path_poses = series.get("path_poses")
        if path_poses:
            fig, ax = plt.subplots(figsize=(5, 5))
            ax.plot([p["x"] for p in path_poses], [p["y"] for p in path_poses], "-o", markersize=3)
            ax.set_aspect("equal")
            ax.set_title(f"Path — {run['name']}")
            fig.tight_layout()
            fig.savefig(out_dir / f"path_{run['name']}.png")
            plt.close(fig)
