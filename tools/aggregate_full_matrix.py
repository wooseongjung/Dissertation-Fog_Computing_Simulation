#!/usr/bin/env python3
"""Aggregate the full-matrix output into mean +/- 95% CI per cell.

Reads results_web/full_matrix_<TS>/<veh>veh_<arch>_seed<N>/task_counters.csv
along with the per-run summary CSVs, and writes a single aggregated CSV
that the plot scripts read.

Usage:
    python3 tools/aggregate_full_matrix.py
    python3 tools/aggregate_full_matrix.py --root results_web/full_matrix_<TS>
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional


DENSITIES = (50, 100, 150)
ARCHS = ("CFN", "VFN")
SEEDS = (1, 2, 3, 4, 5)


# Per-run metrics
TASK_COUNTER_KEYS = (
    "offered_tasks",
    "sent_tasks",
    "no_association_drops",
    "uplink_rx_packets",
    "uplink_loss_tasks",
    "success_tasks",
    "deadline_miss_tasks",
    "return_path_loss_tasks",
    "unclassified_loss_tasks",
    "offered_load_reliability",
    "sent_load_reliability",
    "uplink_delivery_rate",
)

SUMMARY_KEYS = (
    "total_tasks",
    "success_tasks",
    "success_rate",
    "avg_queue_delay_s",
    "avg_migration_delay_s",
    "avg_completion_delay_s",
)


def find_root() -> Path:
    base = Path(__file__).resolve().parent.parent / "results_web"
    candidates = sorted(base.glob("full_matrix_*"))
    if not candidates:
        sys.exit(f"No full_matrix_* directory found under {base}")
    return candidates[-1]


def load_kv_csv(path: Path) -> Dict[str, float]:
    out: Dict[str, float] = {}
    if not path.is_file():
        return out
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                out[row["metric"]] = float(row["value"])
            except (KeyError, ValueError):
                pass
    return out


def load_summary(path: Path) -> Dict[str, float]:
    out: Dict[str, float] = {}
    if not path.is_file():
        return out
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                if k is None:
                    continue
                try:
                    out[k] = float(v)
                except (TypeError, ValueError):
                    pass
            break
    return out


def ci95(values: List[float]) -> float:
    """95% CI half-width using two-sided Student-t (df = n-1)."""
    n = len(values)
    if n < 2:
        return 0.0
    t_crit = {
        1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
        6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
    }
    df = n - 1
    crit = t_crit.get(df, 2.0)
    s = statistics.stdev(values)
    return crit * s / math.sqrt(n)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=None,
                        help="Path to a full_matrix_* directory (default: latest)")
    args = parser.parse_args()

    root = args.root or find_root()
    print(f"Aggregating: {root}")

    rows = []
    raw_rows = []

    for veh in DENSITIES:
        for arch in ARCHS:
            cell_runs: Dict[str, List[float]] = {}
            for seed in SEEDS:
                cell_dir = root / f"{veh}veh_{arch}_seed{seed}"
                if not cell_dir.is_dir():
                    print(f"  WARN: missing dir {cell_dir}")
                    continue
                # task_counters.csv is the end-of-sim marker. Crashed
                # runs leave partial CSVs without it and must be skipped
                # so n_seeds is not inflated.
                tc_path = cell_dir / "task_counters.csv"
                if not tc_path.is_file():
                    print(f"  SKIP: {cell_dir.name} has no task_counters.csv (sim did not complete)")
                    continue
                tc = load_kv_csv(tc_path)
                summary_files = list(cell_dir.glob("*-summary.csv"))
                summary = load_summary(summary_files[0]) if summary_files else {}

                raw = {"density": veh, "arch": arch, "seed": seed}
                for k in TASK_COUNTER_KEYS:
                    raw[k] = tc.get(k, 0.0)
                for k in SUMMARY_KEYS:
                    raw[f"sum_{k}"] = summary.get(k, 0.0)
                raw_rows.append(raw)

                for k in TASK_COUNTER_KEYS:
                    cell_runs.setdefault(k, []).append(tc.get(k, 0.0))
                for k in SUMMARY_KEYS:
                    cell_runs.setdefault(f"sum_{k}", []).append(summary.get(k, 0.0))

            cell_summary = {"density": veh, "arch": arch, "n_seeds": len(cell_runs.get("offered_tasks", []))}
            for k, vals in cell_runs.items():
                if not vals:
                    cell_summary[f"{k}_mean"] = 0.0
                    cell_summary[f"{k}_std"] = 0.0
                    cell_summary[f"{k}_ci95"] = 0.0
                    continue
                cell_summary[f"{k}_mean"] = statistics.mean(vals)
                cell_summary[f"{k}_std"] = statistics.stdev(vals) if len(vals) > 1 else 0.0
                cell_summary[f"{k}_ci95"] = ci95(vals)
            rows.append(cell_summary)

    cell_csv = root / "aggregated_cells.csv"
    if rows:
        keys = list(rows[0].keys())
        with cell_csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        print(f"  wrote {cell_csv}")

    raw_csv = root / "raw_runs.csv"
    if raw_rows:
        keys = list(raw_rows[0].keys())
        with raw_csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(raw_rows)
        print(f"  wrote {raw_csv}")

    print()
    print("=== Offered-load reliability (mean +/- 95% CI) ===")
    print(f"{'density':>7}  {'CFN':>20}  {'VFN':>20}")
    for veh in DENSITIES:
        line = [f"{veh:>7}"]
        for arch in ARCHS:
            cell = next((r for r in rows if r["density"] == veh and r["arch"] == arch), None)
            if cell is None or cell["n_seeds"] == 0:
                line.append("              -      ")
                continue
            m = cell["offered_load_reliability_mean"] * 100
            ci = cell["offered_load_reliability_ci95"] * 100
            line.append(f"  {m:6.2f} +/- {ci:5.2f}%")
        print("  ".join(line))

    print()
    print("=== Sent-load reliability (mean +/- 95% CI) ===")
    print(f"{'density':>7}  {'CFN':>20}  {'VFN':>20}")
    for veh in DENSITIES:
        line = [f"{veh:>7}"]
        for arch in ARCHS:
            cell = next((r for r in rows if r["density"] == veh and r["arch"] == arch), None)
            if cell is None or cell["n_seeds"] == 0:
                line.append("              -      ")
                continue
            m = cell["sent_load_reliability_mean"] * 100
            ci = cell["sent_load_reliability_ci95"] * 100
            line.append(f"  {m:6.2f} +/- {ci:5.2f}%")
        print("  ".join(line))

    print()
    print("=== Avg completion delay (mean +/- 95% CI, ms) ===")
    print(f"{'density':>7}  {'CFN':>20}  {'VFN':>20}")
    for veh in DENSITIES:
        line = [f"{veh:>7}"]
        for arch in ARCHS:
            cell = next((r for r in rows if r["density"] == veh and r["arch"] == arch), None)
            if cell is None or cell["n_seeds"] == 0:
                line.append("              -      ")
                continue
            m = cell["sum_avg_completion_delay_s_mean"] * 1000
            ci = cell["sum_avg_completion_delay_s_ci95"] * 1000
            line.append(f"  {m:6.2f} +/- {ci:5.2f} ms")
        print("  ".join(line))

    print()
    print("=== Avg migration delay (mean +/- 95% CI, ms) ===")
    print(f"{'density':>7}  {'CFN':>20}  {'VFN':>20}")
    for veh in DENSITIES:
        line = [f"{veh:>7}"]
        for arch in ARCHS:
            cell = next((r for r in rows if r["density"] == veh and r["arch"] == arch), None)
            if cell is None or cell["n_seeds"] == 0:
                line.append("              -      ")
                continue
            m = cell["sum_avg_migration_delay_s_mean"] * 1000
            ci = cell["sum_avg_migration_delay_s_ci95"] * 1000
            line.append(f"  {m:6.2f} +/- {ci:5.2f} ms")
        print("  ".join(line))


if __name__ == "__main__":
    main()
