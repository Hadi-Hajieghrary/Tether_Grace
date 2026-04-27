#!/usr/bin/env python3
"""P2-E analysis: wind-magnitude x seed sweep.

Converts "works for one seed" into a statistical robustness statement.
Box-plots of headline metrics across seeds at each wind level.

Output: output/p2e_wind_seed_sweep/_summary/
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, DOUBLE_COL, setup_style
from plot_capability_demo import num_drones, peak_tension, tracking_error

setup_style()

ROOT = Path("/workspaces/Tether_Grace/output/p2e_wind_seed_sweep")
WINDS = [0, 4, 6, 8, 10]
SEEDS = [42, 43, 44]


def _load(w, s):
    tag = f"p2e_w{w}_s{s}"
    csv = ROOT / tag / f"scenario_{tag}.csv"
    if not csv.exists():
        return None
    return pd.read_csv(csv)


def main():
    out = ROOT / "_summary"
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    for w in WINDS:
        for s in SEEDS:
            df = _load(w, s)
            if df is None:
                continue
            mask = df["time"].values >= 0.5
            d = df[mask]
            N = num_drones(d)
            e = tracking_error(d)
            tp = peak_tension(d, N)
            rows.append({
                "wind_mps":   w,
                "seed":       s,
                "rms_err_m":  round(float(np.sqrt(np.mean(e * e))), 4),
                "peak_err_m": round(float(np.max(e)), 4),
                "peak_T_N":   round(float(np.max(tp)), 2),
            })
    summary = pd.DataFrame(rows)
    summary.to_csv(out / "p2e_summary.csv", index=False)
    print("\n=== P2-E summary ===")
    print(summary.to_string(index=False))

    # Boxplot: RMS tracking error per wind level.
    for metric, ylabel, fname in [
        ("rms_err_m", r"RMS $\|e_p\|$ [m]", "p2e_rms_vs_wind.png"),
        ("peak_err_m", r"peak $\|e_p\|$ [m]", "p2e_peak_vs_wind.png"),
        ("peak_T_N",   r"peak $T$ [N]", "p2e_peakT_vs_wind.png"),
    ]:
        fig, ax = plt.subplots(figsize=DOUBLE_COL)
        data_by_wind = [
            summary[summary["wind_mps"] == w][metric].values for w in WINDS]
        bp = ax.boxplot(data_by_wind, positions=WINDS, widths=0.8,
                        patch_artist=True, showmeans=True,
                        medianprops=dict(color=COLORS[0], linewidth=1.2),
                        meanprops=dict(marker="x", markeredgecolor=COLORS[3],
                                       markersize=6))
        for patch in bp["boxes"]:
            patch.set_facecolor(COLORS[1] + "40")
            patch.set_edgecolor(COLORS[1])
            patch.set_linewidth(0.8)
        ax.set_xlabel("mean wind [m/s]")
        ax.set_ylabel(ylabel)
        ax.set_title(f"P2-E: {ylabel} across seeds", fontsize=9)
        ax.set_xticks(WINDS)
        ax.grid(True, linewidth=0.3, alpha=0.3, axis="y")
        fig.tight_layout()
        fig.savefig(out / fname, dpi=180)
        plt.close(fig)

    # Summary statistics: mean ± std per wind level.
    stats = summary.groupby("wind_mps").agg(
        rms_mean=("rms_err_m", "mean"),
        rms_std=("rms_err_m", "std"),
        peak_mean=("peak_err_m", "mean"),
        peak_std=("peak_err_m", "std"),
        peakT_mean=("peak_T_N", "mean"),
        peakT_std=("peak_T_N", "std"),
    ).reset_index()
    stats.to_csv(out / "p2e_stats.csv", index=False)
    print("\n=== P2-E stats (mean ± std across seeds) ===")
    print(stats.round(4).to_string(index=False))

    print(f"\nAll P2-E outputs under: {out}")


if __name__ == "__main__":
    main()
