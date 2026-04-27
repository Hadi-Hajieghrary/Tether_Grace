#!/usr/bin/env python3
"""P2-D analysis: lemniscate-period sweep for the reshape supervisor.

Compares baseline-only against baseline+reshape at lemniscate
periods T in {8, 10, 12} s.  The reshape supervisor's theoretical
peak-tension benefit scales with centripetal load (~1/T^2).

Produces:
    - Peak tension vs period (grouped: no-reshape, reshape).
    - Max-rope-tension during post-fault recovery vs period.
    - Reshape benefit (% reduction) vs period.

Output: output/p2d_period_sweep/_summary/
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

ROOT = Path("/workspaces/Tether_Grace/output/p2d_period_sweep")
PERIODS = [8, 10, 12]


def _load(tag):
    csv = ROOT / tag / f"scenario_{tag}.csv"
    if not csv.exists():
        print(f"  MISSING: {csv}")
        return None
    return pd.read_csv(csv)


def main():
    out = ROOT / "_summary"
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    for T in PERIODS:
        for mode in ("noreshape", "reshape"):
            tag = f"p2d_T{T}_{mode}"
            df = _load(tag)
            if df is None:
                continue
            N = num_drones(df)
            t = df["time"].values
            tp = peak_tension(df, N)
            # Post-fault peak (t >= 12).
            post = tp[t >= 12]
            rows.append({
                "T_period_s": T,
                "mode": mode,
                "peak_T_all_N": round(float(np.max(tp)), 2),
                "peak_T_post_fault_N": round(float(np.max(post))
                    if len(post) else float("nan"), 2),
                "rms_err_m": round(float(np.sqrt(np.mean(
                    tracking_error(df) ** 2))), 4),
            })
    summary = pd.DataFrame(rows)
    summary.to_csv(out / "p2d_summary.csv", index=False)
    print("\n=== P2-D summary ===")
    print(summary.to_string(index=False))

    # Plot peak (post-fault) vs period grouped by mode.
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i, mode in enumerate(("noreshape", "reshape")):
        sub = summary[summary["mode"] == mode].sort_values("T_period_s")
        ax.plot(sub["T_period_s"], sub["peak_T_post_fault_N"],
                color=COLORS[i], lw=1.4, marker="o", label=mode)
    ax.set_xlabel("lemniscate period $T$ [s]")
    ax.set_ylabel(r"peak post-fault $T$ [N]")
    ax.set_title("P2-D: peak post-fault tension vs period", fontsize=9)
    ax.legend(loc="upper right", fontsize=7.5)
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "p2d_peak_vs_period.png", dpi=180)
    plt.close(fig)

    # Plot % reduction vs period.
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    xs, ys = [], []
    for T in PERIODS:
        nr = summary[(summary["T_period_s"] == T)
                     & (summary["mode"] == "noreshape")]
        re = summary[(summary["T_period_s"] == T)
                     & (summary["mode"] == "reshape")]
        if len(nr) == 0 or len(re) == 0:
            continue
        nrv = nr["peak_T_post_fault_N"].values[0]
        rev = re["peak_T_post_fault_N"].values[0]
        pct = 100.0 * (nrv - rev) / nrv if nrv > 0 else 0.0
        xs.append(T); ys.append(pct)
    ax.bar(xs, ys, color=COLORS[2], edgecolor="black", linewidth=0.5,
           width=0.8)
    for x, y in zip(xs, ys):
        ax.text(x, y + 0.2, f"{y:+.1f}%", ha="center", fontsize=8)
    ax.axhline(25.8, color="red", linestyle="--", lw=0.7,
               label="theory: 25.8% at T=12 s")
    ax.set_xlabel("lemniscate period $T$ [s]")
    ax.set_ylabel("peak-tension reduction [%]")
    ax.set_title("P2-D: reshape supervisor benefit vs period", fontsize=9)
    ax.legend(loc="upper right", fontsize=7.5)
    ax.grid(True, linewidth=0.3, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out / "p2d_reduction_vs_period.png", dpi=180)
    plt.close(fig)

    print(f"\nAll P2-D outputs under: {out}")


if __name__ == "__main__":
    main()
