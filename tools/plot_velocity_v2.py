#!/usr/bin/env python3
"""fig10: uplink failure rate against relative velocity, pooled across
the 5 substreams of the full-matrix sweep.

Reads results_methodology_a.csv from each cell of the most recent
full_matrix_<TS>/ directory, bins Tx/Rx events by velocity, and plots
the failure rate per bin in a 2x2 panel (50/150 vehicles x VFN/CFN).
"""
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


BIN_WIDTH = 10  # km/h
VFN_COLOR = "#AB47BC"
CFN_COLOR = "#42A5F5"
SEEDS = (1, 2, 3, 4, 5)


def find_root() -> Path:
    base = Path(__file__).resolve().parent.parent / "results_web"
    candidates = sorted(base.glob("full_matrix_*"))
    if not candidates:
        raise SystemExit("no full_matrix_* directory found")
    return candidates[-1]


def load_pooled_bins(root: Path, density: int, arch: str) -> dict:
    """Pool methodology-A events across the 5 substreams of one cell.

    Only cells whose simulation reached the end (task_counters.csv
    exists) contribute. Crashed cells have a partial methodology_a.csv
    and must be skipped, otherwise their bins skew the failure rate.
    """
    bins = defaultdict(lambda: {"tx": 0, "rx": 0})
    cells_used = 0
    for seed in SEEDS:
        cell_dir = root / f"{density}veh_{arch}_seed{seed}"
        if not (cell_dir / "task_counters.csv").is_file():
            continue
        path = cell_dir / "results_methodology_a.csv"
        if not path.is_file():
            continue
        cells_used += 1
        with path.open() as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    vel = float(row["Relative_Velocity_kmh"])
                except (KeyError, ValueError):
                    continue
                state = row["Packet_State"]
                bin_start = int(vel // BIN_WIDTH) * BIN_WIDTH
                key = f"{bin_start}-{bin_start + BIN_WIDTH}"
                if state == "Tx":
                    bins[key]["tx"] += 1
                elif state == "Rx":
                    bins[key]["rx"] += 1
    out = {}
    for label, c in sorted(bins.items(), key=lambda x: int(x[0].split("-")[0])):
        if c["tx"] > 0:
            fr = (c["tx"] - c["rx"]) / c["tx"] if c["tx"] >= c["rx"] else 0.0
            out[label] = {"tx": c["tx"], "rx": c["rx"], "failure_rate": fr}
    return out, cells_used


def main():
    import matplotlib.font_manager as fm
    for path in [
        "/Library/Fonts/Calibri/CALIBRI.TTF",
        "/Library/Fonts/Calibri/CALIBRIB.TTF",
        "/Library/Fonts/Calibri/CALIBRII.TTF",
        "/Library/Fonts/Calibri/CALIBRIZ.TTF",
        "/Applications/Microsoft Word.app/Contents/Resources/DFonts/Calibri.ttf",
    ]:
        if Path(path).is_file():
            try:
                fm.fontManager.addfont(path)
            except Exception:
                pass
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Calibri", "Carlito", "DejaVu Sans"],
        "font.size": 11,
        "axes.labelsize": 12,
        "axes.titlesize": 12,
        "legend.fontsize": 10,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "figure.dpi": 120,
        "savefig.dpi": 300,
    })

    root = find_root()
    print(f"Reading: {root}")

    fig, axes = plt.subplots(2, 2, figsize=(6.4, 5.2), sharey=True)
    panels = [
        ("50 VFN",  axes[0, 0], VFN_COLOR, 50, "VFN"),
        ("150 VFN", axes[0, 1], VFN_COLOR, 150, "VFN"),
        ("50 CFN",  axes[1, 0], CFN_COLOR, 50, "CFN"),
        ("150 CFN", axes[1, 1], CFN_COLOR, 150, "CFN"),
    ]
    for (label, ax, color, density, arch) in panels:
        data, n = load_pooled_bins(root, density, arch)
        if not data:
            ax.set_title(f"{label} (no data)")
            continue
        # Drop bins with fewer than 100 transmissions; below that the
        # per-bin failure rate is dominated by sample noise.
        kept = [(b, data[b]["failure_rate"], data[b]["tx"])
                for b in data if data[b]["tx"] >= 100]
        if not kept:
            ax.set_title(f"{label} (no data)")
            continue
        bin_labels      = [b for (b, _, _) in kept]
        failure_rates   = [fr for (_, fr, _) in kept]
        tx_counts       = [tx for (_, _, tx) in kept]
        x = np.arange(len(bin_labels))
        ax.bar(x, failure_rates, color=color, alpha=0.85, edgecolor="white", linewidth=0.5)
        ax.set_xticks(x)
        ax.set_xticklabels(bin_labels, rotation=45, ha="right")
        # Extra headroom so the rotated n= labels sit inside the panel
        # box, even on the high-failure-rate bars (~0.65 in 150 CFN).
        ax.set_ylim(0, 1.15)
        ax.set_title(f"{label} (pooled, $n_{{cells}}$={n})", fontweight="bold")
        ax.grid(True, alpha=0.2, axis="y")
        ax.set_xlabel("Relative velocity (km/h)")
        for i, (fr, tx) in enumerate(zip(failure_rates, tx_counts)):
            if fr > 0.02:
                ax.text(i, fr + 0.03, f"n={tx}",
                        ha="left", va="bottom",
                        rotation=45, rotation_mode="anchor",
                        fontsize=10, color="black")

    axes[0, 0].set_ylabel("Failure rate")
    axes[1, 0].set_ylabel("Failure rate")

    fig.suptitle("Uplink failure rate by relative velocity (pooled across substreams)",
                 fontsize=12, fontweight="bold", y=0.99)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = Path(__file__).resolve().parent.parent / "results_web" / "figures" / "fig10_velocity_failure.pdf"
    fig.savefig(out)
    fig.savefig(out.with_suffix(".png"))
    plt.close(fig)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
