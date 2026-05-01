#!/usr/bin/env python3
"""Plot the dissertation figures from aggregated_cells.csv.

Reads results_web/full_matrix_<TS>/aggregated_cells.csv and writes
PDF/PNG pairs into results_web/figures/:

    fig1_loss_decomposition       success/deadline/uplink/no_assoc per scenario
    fig2_reliability_vs_density   reliability vs density with 95% CI bars
    fig3_delay_breakdown          queue/migration/transport per scenario
    fig4_reliability_delay_dual   reliability + delay vs density (dual axis)
    fig5_task_throughput          offered/sent/uplink_rx/success per scenario
    fig7_migration_comparison     migration delay, CFN vs VFN

fig6 CDF is omitted: per-task latency CDFs need raw per-packet logs
pooled across seeds.
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DENSITIES = (50, 100, 150)
ARCHS = ("CFN", "VFN")
ARCH_COLOR = {"CFN": "#1f77b4", "VFN": "#d62728"}


def find_root() -> Path:
    base = Path(__file__).resolve().parent.parent / "results_web"
    candidates = sorted(base.glob("full_matrix_*"))
    if not candidates:
        sys.exit(f"No full_matrix_* directory found under {base}")
    return candidates[-1]


def load_cells(root: Path) -> List[Dict[str, float]]:
    p = root / "aggregated_cells.csv"
    if not p.is_file():
        sys.exit(f"Missing {p}; run tools/aggregate_full_matrix.py first")
    rows = []
    with p.open() as f:
        reader = csv.DictReader(f)
        for r in reader:
            cell = {"density": int(r["density"]), "arch": r["arch"], "n_seeds": int(r["n_seeds"])}
            for k, v in r.items():
                if k in ("density", "arch", "n_seeds"):
                    continue
                try:
                    cell[k] = float(v)
                except (TypeError, ValueError):
                    pass
            rows.append(cell)
    return rows


def get_cell(cells, density, arch):
    return next((c for c in cells if c["density"] == density and c["arch"] == arch), None)


def setup_plot():
    # Match the dissertation typeface (Calibri) when available.
    import matplotlib.font_manager as fm
    candidates = [
        "/Library/Fonts/Calibri/CALIBRI.TTF",
        "/Library/Fonts/Calibri/CALIBRIB.TTF",
        "/Library/Fonts/Calibri/CALIBRII.TTF",
        "/Library/Fonts/Calibri/CALIBRIZ.TTF",
        "/Applications/Microsoft Word.app/Contents/Resources/DFonts/Calibri.ttf",
    ]
    for path in candidates:
        if Path(path).is_file():
            try:
                fm.fontManager.addfont(path)
            except Exception:
                pass
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Calibri", "Carlito", "DejaVu Sans"],
        "font.size": 12,
        "axes.labelsize": 13,
        "axes.titlesize": 13,
        "legend.fontsize": 11,
        "xtick.labelsize": 11,
        "ytick.labelsize": 11,
        "figure.dpi": 120,
        "savefig.dpi": 300,
    })


def fig_reliability_vs_density(cells, fig_dir: Path):
    fig, ax = plt.subplots(figsize=(3.6, 3.4))
    for arch in ARCHS:
        means = []
        cis = []
        for d in DENSITIES:
            c = get_cell(cells, d, arch)
            if c is None or c["n_seeds"] == 0:
                means.append(np.nan)
                cis.append(0)
                continue
            means.append(c["offered_load_reliability_mean"] * 100)
            cis.append(c["offered_load_reliability_ci95"] * 100)
        ax.errorbar(DENSITIES, means, yerr=cis, marker="o", linewidth=2,
                    capsize=4, color=ARCH_COLOR[arch], label=arch)
    ax.set_xlabel("Number of vehicles")
    ax.set_ylabel("End-to-end reliability (%)")
    ax.set_xticks(DENSITIES)
    ax.grid(True, alpha=0.3)
    # Legend below the plot in two columns, matching the layout of the
    # adjacent loss-decomposition subfigure.
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22),
              ncol=2, frameon=True)
    fig.tight_layout()
    out = fig_dir / "fig2_reliability_vs_density.pdf"
    fig.savefig(out, bbox_inches="tight")
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {out}")


def fig_loss_decomposition(cells, fig_dir: Path):
    fig, ax = plt.subplots(figsize=(3.6, 3.4))
    n_scenarios = len(DENSITIES) * len(ARCHS)
    labels = []
    success_pct, deadline_pct, uplink_pct, no_assoc_pct = [], [], [], []
    for d in DENSITIES:
        for arch in ARCHS:
            c = get_cell(cells, d, arch)
            labels.append(f"{d}\n{arch}")
            if c is None or c["n_seeds"] == 0:
                success_pct.append(0); deadline_pct.append(0); uplink_pct.append(0); no_assoc_pct.append(0)
                continue
            offered = c["offered_tasks_mean"]
            if offered <= 0:
                success_pct.append(0); deadline_pct.append(0); uplink_pct.append(0); no_assoc_pct.append(0)
                continue
            success_pct.append(c["success_tasks_mean"] / offered * 100)
            deadline_pct.append(c["deadline_miss_tasks_mean"] / offered * 100)
            uplink_pct.append(c["uplink_loss_tasks_mean"] / offered * 100)
            no_assoc_pct.append(c["no_association_drops_mean"] / offered * 100)

    x = np.arange(n_scenarios)
    w = 0.75
    ax.bar(x, success_pct, w, label="Success",
           color="#4CAF50")
    ax.bar(x, deadline_pct, w, bottom=success_pct, label="Deadline miss",
           color="#FFC107")
    bottom2 = [s + d for s, d in zip(success_pct, deadline_pct)]
    ax.bar(x, uplink_pct, w, bottom=bottom2, label="Pre-fog (uplink) loss",
           color="#FF7043")
    bottom3 = [b + u for b, u in zip(bottom2, uplink_pct)]
    ax.bar(x, no_assoc_pct, w, bottom=bottom3, label="No association",
           color="#9E9E9E")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Percentage of offered tasks")
    ax.set_ylim(0, 105)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22),
              ncol=2, frameon=True)
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    out = fig_dir / "fig1_loss_decomposition.pdf"
    fig.savefig(out, bbox_inches="tight")
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {out}")


def fig_migration_comparison(cells, fig_dir: Path):
    fig, ax = plt.subplots(figsize=(5.5, 3.0))
    x = np.arange(len(DENSITIES))
    w = 0.35
    for i, arch in enumerate(ARCHS):
        means = []
        cis = []
        for d in DENSITIES:
            c = get_cell(cells, d, arch)
            if c is None or c["n_seeds"] == 0:
                means.append(0); cis.append(0); continue
            means.append(c["sum_avg_migration_delay_s_mean"] * 1000)
            cis.append(c["sum_avg_migration_delay_s_ci95"] * 1000)
        offset = (i - 0.5) * w
        ax.bar(x + offset, means, w, yerr=cis, capsize=3,
               label=arch, color=ARCH_COLOR[arch])
    ax.set_xticks(x)
    ax.set_xticklabels([str(d) for d in DENSITIES])
    ax.set_xlabel("Number of vehicles")
    ax.set_ylabel("Average migration delay (ms)")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    out = fig_dir / "fig7_migration_comparison.pdf"
    fig.savefig(out)
    fig.savefig(out.with_suffix(".png"), dpi=150)
    plt.close(fig)
    print(f"  wrote {out}")


def fig_throughput(cells, fig_dir: Path):
    fig, ax = plt.subplots(figsize=(5.5, 3.6))
    n_scenarios = len(DENSITIES) * len(ARCHS)
    x = np.arange(n_scenarios)
    w = 0.20
    metrics = [
        ("offered_tasks", "Offered", "#90A4AE"),
        ("sent_tasks", "Sent", "#42A5F5"),
        ("uplink_rx_packets", "Uplink Rx", "#FFA726"),
        ("success_tasks", "Success", "#66BB6A"),
    ]
    labels = []
    for d in DENSITIES:
        for arch in ARCHS:
            labels.append(f"{d}\n{arch}")
    for i, (key, name, color) in enumerate(metrics):
        means = []
        cis = []
        for d in DENSITIES:
            for arch in ARCHS:
                c = get_cell(cells, d, arch)
                if c is None or c["n_seeds"] == 0:
                    means.append(0); cis.append(0); continue
                means.append(c[f"{key}_mean"])
                cis.append(c[f"{key}_ci95"])
        offset = (i - 1.5) * w
        ax.bar(x + offset, means, w, yerr=cis, capsize=2,
               label=name, color=color)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Number of tasks")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    out = fig_dir / "fig5_task_throughput.pdf"
    fig.savefig(out)
    fig.savefig(out.with_suffix(".png"), dpi=150)
    plt.close(fig)
    print(f"  wrote {out}")


def fig_dual_axis(cells, fig_dir: Path):
    from matplotlib.lines import Line2D
    fig, ax1 = plt.subplots(figsize=(5.5, 3.8))
    ax2 = ax1.twinx()
    for arch in ARCHS:
        rel_m, rel_ci, del_m, del_ci = [], [], [], []
        for d in DENSITIES:
            c = get_cell(cells, d, arch)
            if c is None or c["n_seeds"] == 0:
                rel_m.append(np.nan); rel_ci.append(0); del_m.append(np.nan); del_ci.append(0)
                continue
            rel_m.append(c["offered_load_reliability_mean"] * 100)
            rel_ci.append(c["offered_load_reliability_ci95"] * 100)
            del_m.append(c["sum_avg_completion_delay_s_mean"] * 1000)
            del_ci.append(c["sum_avg_completion_delay_s_ci95"] * 1000)
        ax1.errorbar(DENSITIES, rel_m, yerr=rel_ci, marker="o", linewidth=2,
                     capsize=3, color=ARCH_COLOR[arch])
        ax2.errorbar(DENSITIES, del_m, yerr=del_ci, marker="s", linewidth=1.5,
                     linestyle="--", capsize=3, color=ARCH_COLOR[arch], alpha=0.85)
    ax1.set_xlabel("Number of vehicles")
    ax1.set_ylabel("End-to-end reliability (%)")
    ax2.set_ylabel("Avg completion delay (ms)")
    ax1.set_xticks(DENSITIES)
    ax1.grid(True, alpha=0.3)
    # Use Line2D proxies for the legend so each entry is a single clean
    # swatch instead of the half-solid/half-dashed errorbar handle.
    proxies = []
    labels = []
    for arch in ARCHS:
        proxies.append(Line2D([0], [0], color=ARCH_COLOR[arch],
                              linewidth=2, marker="o", linestyle="-"))
        labels.append(f"{arch} reliability")
    for arch in ARCHS:
        proxies.append(Line2D([0], [0], color=ARCH_COLOR[arch],
                              linewidth=1.5, marker="s", linestyle="--",
                              alpha=0.85))
        labels.append(f"{arch} delay")
    fig.legend(proxies, labels, loc="lower center",
               bbox_to_anchor=(0.5, -0.02), ncol=2, frameon=True)
    fig.tight_layout(rect=[0, 0.18, 1, 1])
    out = fig_dir / "fig4_reliability_delay_dual.pdf"
    fig.savefig(out, bbox_inches="tight")
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {out}")


def fig_delay_breakdown(cells, fig_dir: Path):
    fig, ax = plt.subplots(figsize=(5.5, 3.5))
    n_scenarios = len(DENSITIES) * len(ARCHS)
    x = np.arange(n_scenarios)
    w = 0.7
    labels = []
    queue, mig, transport_plus_service = [], [], []
    for d in DENSITIES:
        for arch in ARCHS:
            c = get_cell(cells, d, arch)
            labels.append(f"{d}\n{arch}")
            if c is None or c["n_seeds"] == 0:
                queue.append(0); mig.append(0); transport_plus_service.append(0); continue
            q = c["sum_avg_queue_delay_s_mean"] * 1000
            m = c["sum_avg_migration_delay_s_mean"] * 1000
            total = c["sum_avg_completion_delay_s_mean"] * 1000
            queue.append(q); mig.append(m)
            transport_plus_service.append(max(0, total - q - m))
    ax.bar(x, queue, w, label="Queue", color="#90CAF9")
    ax.bar(x, mig, w, bottom=queue, label="Migration", color="#FFAB91")
    bottom2 = [q + m for q, m in zip(queue, mig)]
    ax.bar(x, transport_plus_service, w, bottom=bottom2,
           label="Transport + service", color="#A5D6A7")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Average delay component (ms)")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    out = fig_dir / "fig3_delay_breakdown.pdf"
    fig.savefig(out)
    fig.savefig(out.with_suffix(".png"), dpi=150)
    plt.close(fig)
    print(f"  wrote {out}")


def main():
    setup_plot()
    root = find_root()
    print(f"Reading aggregated CSV from: {root}")
    cells = load_cells(root)
    fig_dir = Path(__file__).resolve().parent.parent / "results_web" / "figures"
    fig_dir.mkdir(exist_ok=True)
    fig_reliability_vs_density(cells, fig_dir)
    fig_loss_decomposition(cells, fig_dir)
    fig_throughput(cells, fig_dir)
    fig_migration_comparison(cells, fig_dir)
    fig_dual_axis(cells, fig_dir)
    fig_delay_breakdown(cells, fig_dir)
    print(f"\nDone. Figures in: {fig_dir}")


if __name__ == "__main__":
    main()
