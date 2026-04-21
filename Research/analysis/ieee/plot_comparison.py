#!/usr/bin/env python3
"""
Cross-scenario IEEE comparison figures (one PNG per metric) + overlays.

Usage:
    python3 plot_comparison.py <scenarios_root> <output_dir>
"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import (setup_style, COLORS, DOUBLE_COL, DOUBLE_COL_TALL,
                        SINGLE_COL, trim_start, detect_faults)

setup_style()


def load_scenarios(root: Path) -> dict[str, pd.DataFrame]:
    """Search each scenario sub-folder for its CSV in ../08_source_data/."""
    dfs = {}
    data_root = root / "08_source_data"
    if not data_root.exists():
        # Fallback — CSVs might be alongside HTML in the old layout
        data_root = root
    for csv in sorted(data_root.glob("scenario_*.csv")):
        name = csv.stem.replace("scenario_", "")
        df = trim_start(pd.read_csv(csv), 0.5)
        dfs[name] = df
    return dfs


def compute_metrics(df: pd.DataFrame) -> dict:
    t = df["time"].values
    err = df[["ref_x", "ref_y", "ref_z"]].values - df[["payload_x", "payload_y", "payload_z"]].values
    en = np.linalg.norm(err, axis=1)

    T = np.column_stack([df[f"tension_{i}"] for i in range(4)])
    F = np.column_stack([
        np.sqrt(df[f"fx_{i}"]**2 + df[f"fy_{i}"]**2 + df[f"fz_{i}"]**2).values
        for i in range(4)])
    dt = np.diff(t, prepend=t[0])
    imp = np.sum(F * dt[:, None])
    return dict(
        rms_error=float(np.sqrt(np.mean(en**2))),
        peak_error=float(en.max()),
        max_tension=float(T.max()),
        peak_thrust=float(F.max()),
        mean_thrust=float(F.mean()),
        total_impulse=float(imp),
        tension_imbalance_rms=float(np.sqrt(np.mean(np.std(T, axis=1)**2))),
        num_faults=len(detect_faults(df)),
    )


# --- bar-chart helpers -------------------------------------------------
def _bar(ax, names, vals, ylabel, title=""):
    x = np.arange(len(names))
    palette = plt.cm.tab10(np.linspace(0, 1, max(len(names), 3)))
    ax.bar(x, vals, color=palette, edgecolor="black", linewidth=0.4)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=30, ha="right", fontsize=7)
    ax.set_ylabel(ylabel)
    if title: ax.set_title(title, fontsize=9)
    for i, v in enumerate(vals):
        ax.text(i, v * 1.01, f"{v:.2f}", ha="center", va="bottom", fontsize=6.5)
    ax.grid(axis="y", linewidth=0.3, alpha=0.3)


def plot_bar(out_dir, names, vals, ylabel, title, fname):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    _bar(ax, names, vals, ylabel, title)
    fig.savefig(out_dir / fname); plt.close(fig)


# ------ overlay plots --------------------------------------------------
def plot_tracking_overlay(dfs, out):
    palette = plt.cm.tab10(np.linspace(0, 1, len(dfs)))
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for (name, df), c in zip(dfs.items(), palette):
        err = df[["ref_x", "ref_y", "ref_z"]].values - df[["payload_x", "payload_y", "payload_z"]].values
        ax.plot(df["time"], np.linalg.norm(err, axis=1),
                lw=1.0, color=c, label=name)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$\|e_p\|$ [m]")
    ax.legend(ncol=3, fontsize=7)
    ax.set_title("Payload Tracking Error — all scenarios", fontsize=9)
    fig.savefig(out); plt.close(fig)


def plot_imbalance_overlay(dfs, out):
    palette = plt.cm.tab10(np.linspace(0, 1, len(dfs)))
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for (name, df), c in zip(dfs.items(), palette):
        T = np.column_stack([df[f"tension_{i}"] for i in range(4)])
        ax.plot(df["time"], np.std(T, axis=1), lw=1.0, color=c, label=name)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$\sigma_T$ [N]")
    ax.legend(ncol=3, fontsize=7)
    ax.set_title("Load-Sharing Imbalance — all scenarios", fontsize=9)
    fig.savefig(out); plt.close(fig)


def plot_fault_signature_overlay(dfs, out):
    """Faulted-drone tension for scenarios that have faults, showing the
    clean collapse to zero."""
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    palette = plt.cm.tab10(np.linspace(0, 1, len(dfs)))
    idx = 0
    for name, df in dfs.items():
        faults = detect_faults(df)
        if not faults: continue
        for (drone, t_f) in faults[:1]:   # only first fault per scenario
            ax.plot(df["time"], df[f"tension_{drone}"], lw=1.0,
                    color=palette[idx],
                    label=f"{name} — drone {drone}")
        idx += 1
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$T_{\mathrm{faulted}}$ [N]")
    ax.legend(ncol=2, fontsize=7)
    ax.set_title("Faulted-rope Tension — clean collapse after $T_{\\mathrm{fault}}$",
                 fontsize=9)
    fig.savefig(out); plt.close(fig)


# ----------------------------------------------------------------------
def main():
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    scen_root = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)

    dfs = load_scenarios(scen_root)
    metrics = {n: compute_metrics(df) for n, df in dfs.items()}
    metrics_df = pd.DataFrame(metrics).T
    metrics_df.to_csv(out / "campaign_metrics.csv")
    print(f"Wrote: {out / 'campaign_metrics.csv'}")
    print(metrics_df)

    names = list(metrics.keys())

    plot_bar(out, names, [metrics[n]["rms_error"] for n in names],
             r"$\|e_p\|$ RMS [m]", "Payload Tracking RMS per Scenario",
             "fig_compare_rms_error.png")
    plot_bar(out, names, [metrics[n]["peak_error"] for n in names],
             r"$\|e_p\|$ peak [m]", "Peak Tracking Error",
             "fig_compare_peak_error.png")
    plot_bar(out, names, [metrics[n]["max_tension"] for n in names],
             r"$T_{\max}$ [N]", "Peak Rope Tension",
             "fig_compare_max_tension.png")
    plot_bar(out, names, [metrics[n]["peak_thrust"] for n in names],
             r"$\|F\|_{\max}$ [N]", "Peak Thrust Command per Drone",
             "fig_compare_peak_thrust.png")
    plot_bar(out, names, [metrics[n]["total_impulse"] for n in names],
             r"$\Sigma \int \|F\| dt$ [N$\cdot$s]",
             "Total Thrust Impulse (energy proxy)",
             "fig_compare_total_impulse.png")
    plot_bar(out, names, [metrics[n]["tension_imbalance_rms"] for n in names],
             r"RMS $\sigma_T$ [N]", "Load-Sharing Imbalance (low = balanced)",
             "fig_compare_imbalance.png")

    plot_tracking_overlay(dfs,       out / "fig_overlay_tracking_error.png")
    plot_imbalance_overlay(dfs,      out / "fig_overlay_imbalance.png")
    plot_fault_signature_overlay(dfs, out / "fig_overlay_fault_signature.png")

    print(f"All comparison plots written to {out}")


if __name__ == "__main__":
    main()
