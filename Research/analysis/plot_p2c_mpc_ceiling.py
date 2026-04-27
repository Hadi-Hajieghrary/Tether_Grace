#!/usr/bin/env python3
"""P2-C analysis: reduced MPC tension-ceiling sweep.

Compares the baseline reference against MPC at successively tighter
tension ceilings (100, 90, 80, 70, 60 N).  Produces:

    - Peak-tension vs ceiling plot.
    - Tracking-cost vs ceiling plot.
    - Per-rope tension time-series grouped by ceiling.
    - Violation-rate table: fraction of ticks where
      max_j T_j > T_ceiling for each (mode, ceiling).

Output: output/p2c_mpc_ceiling_sweep/_summary/
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import (  # noqa: E402
    COLORS, SINGLE_COL, DOUBLE_COL, annotate_faults, setup_style,
)
from plot_capability_demo import num_drones, peak_tension, tracking_error
from plot_review_augmentation import first_fault_per_rope

setup_style()

ROOT = Path("/workspaces/Tether_Grace/output/p2c_mpc_ceiling_sweep")
CEILINGS = [60, 70, 80, 90, 100]


def _load(tag):
    csv = ROOT / tag / f"scenario_{tag}.csv"
    if not csv.exists():
        print(f"  MISSING: {csv}")
        return None
    return pd.read_csv(csv)


def main():
    out = ROOT / "_summary"
    out.mkdir(parents=True, exist_ok=True)

    base = _load("p2c_baseline")
    mpcs = {T: _load(f"p2c_mpc_T{T}") for T in CEILINGS}

    if base is None or any(v is None for v in mpcs.values()):
        print("Incomplete data; aborting.")
        return

    # Headline: peak tension vs ceiling.
    rows = []
    faults = first_fault_per_rope(base, num_drones(base))
    base_peak = float(np.max(peak_tension(base, num_drones(base))))
    rows.append({"mode": "baseline", "T_ceiling_N": np.nan,
                 "peak_T_N": round(base_peak, 2),
                 "rms_err_m": round(float(np.sqrt(np.mean(
                     tracking_error(base) ** 2))), 4),
                 "violation_frac": round(float(np.mean(
                     peak_tension(base, num_drones(base)) > 100.0)), 4)})
    for T, df in mpcs.items():
        N = num_drones(df)
        tp = peak_tension(df, N)
        rows.append({
            "mode": f"mpc_T{T}",
            "T_ceiling_N": T,
            "peak_T_N": round(float(np.max(tp)), 2),
            "rms_err_m": round(float(np.sqrt(np.mean(
                tracking_error(df) ** 2))), 4),
            "violation_frac": round(float(np.mean(tp > T)), 4),
        })
    summary = pd.DataFrame(rows)
    summary.to_csv(out / "p2c_summary.csv", index=False)
    print("\n=== P2-C summary ===")
    print(summary.to_string(index=False))

    # Plot: peak tension vs ceiling.
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    xs = CEILINGS
    mpc_peaks = [summary[summary["mode"] == f"mpc_T{T}"]["peak_T_N"].values[0]
                 for T in CEILINGS]
    ax.plot(xs, [base_peak] * len(xs), color=COLORS[0], lw=1.2,
            linestyle="--", label=f"baseline (fixed, {base_peak:.1f} N)")
    ax.plot(xs, mpc_peaks, color=COLORS[1], lw=1.4, marker="o",
            label="MPC peak")
    ax.plot(xs, xs, color="black", linestyle=":", lw=0.8,
            label="ceiling line")
    ax.set_xlabel(r"$T_\text{max,safe}$ [N]")
    ax.set_ylabel(r"peak $\max_j T_j$ [N]")
    ax.set_title("P2-C: peak tension vs MPC ceiling", fontsize=9)
    ax.legend(loc="lower right", fontsize=7.5)
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "p2c_peak_vs_ceiling.png", dpi=180)
    plt.close(fig)

    # Plot: tracking RMS vs ceiling.
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    base_rms = summary[summary["mode"] == "baseline"]["rms_err_m"].values[0]
    mpc_rms = [summary[summary["mode"] == f"mpc_T{T}"]["rms_err_m"].values[0]
               for T in CEILINGS]
    ax.plot(xs, [base_rms] * len(xs), color=COLORS[0], lw=1.2,
            linestyle="--", label=f"baseline ({base_rms:.3f} m)")
    ax.plot(xs, mpc_rms, color=COLORS[1], lw=1.4, marker="o",
            label="MPC")
    ax.set_xlabel(r"$T_\text{max,safe}$ [N]")
    ax.set_ylabel(r"RMS $\|e_p\|$ [m]")
    ax.set_title("P2-C: tracking cost of tighter ceiling", fontsize=9)
    ax.legend(loc="upper left", fontsize=7.5)
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "p2c_rms_vs_ceiling.png", dpi=180)
    plt.close(fig)

    # Plot: peak-tension time-series for baseline + each ceiling.
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i, (label, df) in enumerate(
            [("baseline", base)] + [(f"MPC T={T}", mpcs[T]) for T in CEILINGS]):
        N = num_drones(df)
        t = df["time"].values
        ax.plot(t, peak_tension(df, N), lw=0.9,
                color=COLORS[i % len(COLORS)], label=label)
    annotate_faults(ax, faults)
    ax.axhline(100, color="red", linestyle="--", lw=0.6)
    ax.set_xlabel("t [s]")
    ax.set_ylabel(r"peak $T$ [N]")
    ax.set_title("P2-C: peak tension time series", fontsize=9)
    ax.legend(loc="upper right", ncol=2, fontsize=7)
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "p2c_timeseries_peakT.png", dpi=180)
    plt.close(fig)

    print(f"\nAll P2-C outputs under: {out}")


if __name__ == "__main__":
    main()
