#!/usr/bin/env python3
"""Read the 12-cell calibration sweep and decide which factor (SNR
threshold or fog density) carries the reliability gap.

Reads results_web/calibration_<TS>/<arch>_<fogs>fog_<thr>dB/task_counters.csv
across the 12 cells and prints the 2x2x3 matrix plus the verdict.

Verdict keys:
  threshold_dominant : delta(thr=0 -> thr=-10) >= 30 pp at numFogs=2
  density_dominant   : delta(numFogs=2 -> numFogs=4) >= 30 pp at thr=0
  both_matter        : both swings >= 30 pp
  neither_closes_gap : both relaxations leave reliability below 60 %
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path
from typing import Dict, Optional


ARCHS = ("CFN", "VFN")
FOG_COUNTS = (2, 4)
THRESHOLDS = (0, -5, -10)


def find_calibration_root() -> Path:
    base = Path(__file__).resolve().parent.parent / "results_web"
    candidates = sorted(base.glob("calibration_*"))
    if not candidates:
        sys.exit(f"No calibration_* directory found under {base}")
    return candidates[-1]


def cell_dir(root: Path, arch: str, fogs: int, threshold: int) -> Path:
    safe = str(threshold).replace("-", "m")
    return root / f"{arch}_{fogs}fog_{safe}dB"


def load_counters(path: Path) -> Optional[Dict[str, float]]:
    counters: Dict[str, float] = {}
    if not path.is_file():
        return None
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                counters[row["metric"]] = float(row["value"])
            except (KeyError, ValueError):
                continue
    return counters


def gather(root: Path):
    out = {}
    for arch in ARCHS:
        out[arch] = {}
        for fogs in FOG_COUNTS:
            out[arch][fogs] = {}
            for thr in THRESHOLDS:
                d = cell_dir(root, arch, fogs, thr)
                tc = load_counters(d / "task_counters.csv")
                if tc is None:
                    print(f"WARN: missing task_counters.csv in {d}")
                    continue
                assoc_csv = d / "assoc_diagnostics.csv"
                ever = total = 0
                if assoc_csv.is_file():
                    with assoc_csv.open() as f:
                        reader = csv.DictReader(f)
                        for row in reader:
                            if row.get("row_type") == "vehicle_summary":
                                total += 1
                                if row.get("ever_associated") == "1":
                                    ever += 1
                tc["assoc_rate"] = ever / total if total else 0.0
                out[arch][fogs][thr] = tc
    return out


def fmt_pct(value: float) -> str:
    return f"{value*100:5.1f}%"


def print_matrix(data, metric_name: str, label: str):
    print()
    print(f"=== {label} ({metric_name}) ===")
    header = f"{'arch':>4} {'fogs':>5}  " + "  ".join(f"thr={t:>3} dB" for t in THRESHOLDS)
    print(header)
    for arch in ARCHS:
        for fogs in FOG_COUNTS:
            row = [f"{arch:>4} {fogs:>5}"]
            for thr in THRESHOLDS:
                cell = data[arch][fogs].get(thr)
                if cell is None:
                    row.append("    -    ")
                else:
                    row.append(f"  {fmt_pct(cell.get(metric_name, 0)):>7}")
            print("  ".join(row))


def delta_threshold(data, arch: str, fogs: int, metric: str = "offered_load_reliability") -> float:
    a = data[arch][fogs].get(0)
    b = data[arch][fogs].get(-10)
    if not a or not b:
        return float("nan")
    return (b[metric] - a[metric]) * 100.0


def delta_density(data, arch: str, thr: int, metric: str = "offered_load_reliability") -> float:
    a = data[arch][2].get(thr)
    b = data[arch][4].get(thr)
    if not a or not b:
        return float("nan")
    return (b[metric] - a[metric]) * 100.0


def classify(data) -> str:
    thr_swings = [delta_threshold(data, a, 2) for a in ARCHS]
    den_swings = [delta_density(data, a, 0) for a in ARCHS]
    thr_swing = sum(thr_swings) / len(thr_swings)
    den_swing = sum(den_swings) / len(den_swings)

    best = []
    for arch in ARCHS:
        cell = data[arch][4].get(-10)
        if cell:
            best.append(cell.get("offered_load_reliability", 0))
    best_rel = (sum(best) / len(best)) * 100.0 if best else 0.0

    print()
    print("=== Decision criteria ===")
    print(f"  delta(threshold 0 -> -10 dB), at fogs=2, mean over archs : {thr_swing:+.1f} pp")
    print(f"  delta(fogs 2 -> 4),           at threshold=0, mean over archs : {den_swing:+.1f} pp")
    print(f"  best-case (fogs=4, thr=-10) offered-load reliability         : {best_rel:.1f}%")

    if best_rel < 60:
        return (
            "NEITHER_CLOSES_GAP: both relaxations leave reliability < 60%. "
            "Re-examine trace geography, gNB count, and simulation duration."
        )
    if thr_swing >= 30 and den_swing < 30:
        return (
            "THRESHOLD_DOMINANT: relaxing the SNR floor closes the gap. "
            "Run the full matrix at sinrThresholdDb=-5 with 2 fogs "
            "(3GPP TS 38.214 cell-edge MCS-0 SINR ~ -6.7 dB)."
        )
    if den_swing >= 30 and thr_swing < 30:
        return (
            "DENSITY_DOMINANT: increasing fog count closes the gap. "
            "Run the full matrix at numFogs=4 with sinrThresholdDb=0."
        )
    if thr_swing >= 30 and den_swing >= 30:
        return (
            "BOTH_MATTER: both relaxations produce >=30 pp swings. "
            "Prefer threshold=-5 with fogs=2 since it stays consistent with Table 4."
        )
    return (
        "INCONCLUSIVE: neither factor produces a >=30 pp swing on its own. "
        f"Best-case relaxation gives {best_rel:.1f}% which is above the 60% floor; "
        "review the full matrix manually."
    )


def main():
    root = find_calibration_root()
    print(f"Calibration root: {root}")
    data = gather(root)

    print_matrix(data, "offered_load_reliability", "Offered-load reliability")
    print_matrix(data, "sent_load_reliability", "Sent-load reliability")
    print_matrix(data, "uplink_delivery_rate", "Uplink delivery (sent -> fog)")
    print_matrix(data, "assoc_rate", "Association rate (ever-associated vehicles)")

    print()
    print("=== Raw counters per cell ===")
    print(f"{'cell':<22} {'offered':>9} {'sent':>9} {'no_assoc':>9} {'success':>9} {'late':>6} {'unclassif':>10}")
    for arch in ARCHS:
        for fogs in FOG_COUNTS:
            for thr in THRESHOLDS:
                cell = data[arch][fogs].get(thr)
                if not cell:
                    continue
                cell_id = f"{arch}_{fogs}fog_{thr:>3}dB"
                print(
                    f"{cell_id:<22} "
                    f"{int(cell.get('offered_tasks',0)):>9} "
                    f"{int(cell.get('sent_tasks',0)):>9} "
                    f"{int(cell.get('no_association_drops',0)):>9} "
                    f"{int(cell.get('success_tasks',0)):>9} "
                    f"{int(cell.get('deadline_miss_tasks',0)):>6} "
                    f"{int(cell.get('unclassified_loss_tasks',0)):>10}"
                )

    verdict = classify(data)
    print()
    print("=== VERDICT ===")
    print(verdict)


if __name__ == "__main__":
    main()
