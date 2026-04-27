#!/usr/bin/env python3
"""Grouped bar chart of per-scenario headline metrics with bootstrap
95 % confidence intervals. Consumes the aggregated
`publication_metrics.csv` written by `run_transactions_campaign.sh`
and emits the Figure 13 variant used by the IEEE-Transactions draft.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, DOUBLE_COL, setup_style  # noqa: E402
from statistics import bootstrap_ci  # noqa: E402

setup_style()

CONFIG_ORDER = ["baseline", "l1", "mpc5", "mpc10", "fullstack",
                "fullstack_t60"]
CONFIG_LABEL = {
    "baseline":      "baseline",
    "l1":            "+L1",
    "mpc5":          "+MPC(Np=5)",
    "mpc10":         "+MPC(Np=10)",
    "fullstack":     "fullstack",
    "fullstack_t60": "fullstack, Tmax=60",
}

SCENARIOS = ["A_nominal", "B_single_15s", "C_dual_5s",
             "D_dual_10s", "E_cascade_3fault"]

METRICS = [
    ("rms_error",      "RMS tracking error [m]"),
    ("peak_error",     "peak tracking error [m]"),
    ("peak_tension",   "peak tension [N]"),
    ("rms_sigma_T",    "RMS σ_T [N]"),
    ("qp_solve_us_p99","QP p99 [µs]"),
]


def plot_metric(df: pd.DataFrame, metric: str, ylabel: str,
                out_pdf: Path) -> None:
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    n_groups = len(SCENARIOS)
    n_cfg = len(CONFIG_ORDER)
    width = 0.8 / n_cfg
    x = np.arange(n_groups)
    for k, cfg in enumerate(CONFIG_ORDER):
        means, los, his = [], [], []
        for s in SCENARIOS:
            samples = df[(df.config == cfg)
                         & (df.scenario == s)][metric].dropna().values
            ci = bootstrap_ci(samples)
            means.append(ci.mean)
            los.append(ci.mean - ci.lo)
            his.append(ci.hi - ci.mean)
        pos = x + (k - (n_cfg - 1) / 2.0) * width
        ax.bar(pos, means, width,
               yerr=[los, his],
               capsize=2, color=COLORS[k % len(COLORS)],
               edgecolor="black", linewidth=0.4,
               label=CONFIG_LABEL[cfg])
    ax.set_xticks(x)
    ax.set_xticklabels(SCENARIOS, rotation=15, fontsize=7)
    ax.set_ylabel(ylabel)
    ax.legend(ncol=3, fontsize=6, loc="upper left")
    ax.grid(axis="y", linewidth=0.3, alpha=0.3)
    fig.savefig(out_pdf)
    plt.close(fig)


def main() -> None:
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    metrics_csv = Path(sys.argv[1])
    out_dir = Path(sys.argv[2])
    out_dir.mkdir(parents=True, exist_ok=True)
    df = pd.read_csv(metrics_csv)
    for metric, label in METRICS:
        if metric not in df.columns:
            continue
        for ext in ("pdf", "png"):
            plot_metric(df, metric, label,
                        out_dir / f"F13_{metric}_ci.{ext}")


if __name__ == "__main__":
    main()
