#!/usr/bin/env python3
"""
F13 — cross-configuration ablation bar charts.

Reads the `summary_metrics.csv` produced by `run_ablation_campaign.sh`
and emits grouped-bar plots that compare each of the four
configurations (baseline / +L1 / +MPC / fullstack) across every
scenario (A/B/C/D) on a handful of headline metrics.

Usage:
    python3 plot_ablation.py <ablation_root> <output_dir>
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, DOUBLE_COL, DOUBLE_COL_TALL, setup_style

setup_style()


# Metric -> axis-label mapping.
METRICS = [
    ("rms_error",        r"RMS $\|e_p\|$ [m]"),
    ("peak_error",       r"peak $\|e_p\|$ [m]"),
    ("peak_tension",     r"peak $T$ [N]"),
    ("rms_sigma_T",      r"RMS $\sigma_T$ [N]"),
    ("qp_solve_us_p99",  r"QP $p_{99}$ [$\mu$s]"),
]

# Canonical presentation order (left to right).
CONFIG_ORDER = ["baseline", "l1", "mpc", "fullstack"]
CONFIG_LABEL = {
    "baseline":  "baseline",
    "l1":        "+L1",
    "mpc":       "+MPC",
    "fullstack": "L1+MPC+reshape",
}
# Scenario order.
SCENARIO_ORDER = [
    "A_nominal",
    "B_single_fault",
    "C_dual_5sec",
    "D_dual_10sec",
]
SCENARIO_LABEL = {
    "A_nominal":       "A nominal",
    "B_single_fault":  "B single",
    "C_dual_5sec":     "C dual 5 s",
    "D_dual_10sec":    "D dual 10 s",
}


def load_summary(root: Path) -> pd.DataFrame:
    """Load summary_metrics.csv produced by the ablation runner."""
    path = root / "summary_metrics.csv"
    if not path.exists():
        raise FileNotFoundError(
            f"{path} not found — run `run_ablation_campaign.sh` first.")
    df = pd.read_csv(path)
    # Accept either 'scenario' being either the raw ID (A_nominal) or
    # the campaign filename form (scenario_A_5drone_nominal).
    df["scenario_clean"] = df["scenario"].str.replace(
        "_5drone", "", regex=False).str.replace("scenario_", "", regex=False)
    return df


def plot_grouped_bars(df: pd.DataFrame, metric: str, ylabel: str,
                      title: str, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    scenarios = [s for s in SCENARIO_ORDER
                  if s in df["scenario_clean"].values]
    configs = [c for c in CONFIG_ORDER if c in df["config"].values]
    x = np.arange(len(scenarios))
    bar_w = 0.8 / max(len(configs), 1)
    colours = COLORS
    for k, cfg in enumerate(configs):
        vals = []
        for s in scenarios:
            row = df[(df.config == cfg) & (df.scenario_clean == s)]
            vals.append(float(row[metric].iloc[0]) if not row.empty else np.nan)
        offs = (k - (len(configs) - 1) / 2.0) * bar_w
        ax.bar(x + offs, vals, bar_w,
               color=colours[k % len(colours)],
               edgecolor="black", linewidth=0.4,
               label=CONFIG_LABEL.get(cfg, cfg))
        # Inline value labels.
        for xi, v in zip(x + offs, vals):
            if np.isfinite(v):
                ax.text(xi, v * 1.01, f"{v:.2g}", ha="center",
                        va="bottom", fontsize=5.5)
    ax.set_xticks(x)
    ax.set_xticklabels([SCENARIO_LABEL[s] for s in scenarios],
                       rotation=10, fontsize=7)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=9)
    ax.legend(ncol=len(configs), fontsize=7, loc="upper left")
    ax.grid(axis="y", linewidth=0.3, alpha=0.3)
    fig.savefig(out_path)
    plt.close(fig)


def plot_summary_heatmap(df: pd.DataFrame, out_path: Path) -> None:
    """Config × Metric normalised heat-map (baseline = 1.0)."""
    configs = [c for c in CONFIG_ORDER if c in df["config"].values]
    metric_keys = [m[0] for m in METRICS]
    # Pool across scenarios by taking the mean for each (config, metric).
    pooled = (df.groupby("config")[metric_keys].mean()
                .reindex(configs))
    if "baseline" not in pooled.index:
        return
    normalised = pooled / pooled.loc["baseline"]
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    im = ax.imshow(normalised.values, cmap="RdYlGn_r", aspect="auto",
                    vmin=0.5, vmax=1.5)
    ax.set_xticks(range(len(metric_keys)))
    ax.set_xticklabels([m[1] for m in METRICS], rotation=25, ha="right",
                       fontsize=7)
    ax.set_yticks(range(len(configs)))
    ax.set_yticklabels([CONFIG_LABEL[c] for c in configs], fontsize=8)
    for i in range(len(configs)):
        for j in range(len(metric_keys)):
            v = normalised.values[i, j]
            ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                    color="black" if 0.7 < v < 1.3 else "white",
                    fontsize=6.5)
    fig.colorbar(im, ax=ax, pad=0.02).set_label("ratio vs baseline")
    ax.set_title("Ablation summary — mean metric ratio vs baseline",
                 fontsize=9)
    fig.savefig(out_path)
    plt.close(fig)


def main() -> None:
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    root = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    df = load_summary(root)
    for key, label in METRICS:
        if key not in df.columns:
            continue
        for ext in ("pdf", "png"):
            plot_grouped_bars(
                df, key, label,
                f"Ablation bars — {label}",
                out / f"F13_{key}.{ext}")
    for ext in ("pdf", "png"):
        plot_summary_heatmap(df, out / f"F13_summary_heatmap.{ext}")
    print(f"F13 ablation figures written to {out}")


if __name__ == "__main__":
    main()
