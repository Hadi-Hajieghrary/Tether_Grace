#!/usr/bin/env python3
"""
Cross-scenario comparison: summary metrics + overlay plots.

Loads every scenario_*.csv in the output directory, computes per-scenario
performance metrics, and produces:
  - A bar-chart figure comparing key metrics across scenarios
  - A CSV table of all metrics
  - An overlay plot of tracking errors across scenarios

Usage:
    python3 campaign_summary.py <replays_dir>
"""

from __future__ import annotations

import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def num_drones(df):
    n = 0
    while f"quad{n}_x" in df.columns: n += 1
    return n


def detect_faults(df, N):
    faults = []
    for i in range(N):
        t = df[f"tension_{i}"].values
        time = df["time"].values
        near_zero = np.abs(t) < 0.2
        if near_zero.any():
            for j in range(len(t) - 1, 0, -1):
                if not near_zero[j]:
                    if j < len(t) - 1:
                        faults.append((i, float(time[j + 1])))
                    break
    return faults


def compute_metrics(df: pd.DataFrame) -> dict:
    N = num_drones(df)
    t = df["time"].values
    dt = np.diff(t, prepend=t[0])

    payload = df[["payload_x", "payload_y", "payload_z"]].values
    ref = df[["ref_x", "ref_y", "ref_z"]].values
    err = ref - payload
    err_norm = np.linalg.norm(err, axis=1)

    tensions = np.column_stack([df[f"tension_{i}"].values for i in range(N)])
    fz = np.column_stack([df[f"fz_{i}"].values for i in range(N)])
    cum_impulse = np.sum(fz * dt[:, None])

    thrust_mag = np.column_stack([
        np.sqrt(df[f"fx_{i}"] ** 2 + df[f"fy_{i}"] ** 2 + df[f"fz_{i}"] ** 2).values
        for i in range(N)
    ])

    faults = detect_faults(df, N)
    post_settle_mask = (t >= 3.0)  # after pickup completes

    return {
        "num_drones": N,
        "duration": float(t[-1]),
        "num_faults": len(faults),
        "fault_times": [f[1] for f in faults],
        "rms_error": float(np.sqrt(np.mean(err_norm ** 2))),
        "rms_error_post_pickup": float(np.sqrt(np.mean(err_norm[post_settle_mask] ** 2))),
        "max_error": float(err_norm.max()),
        "final_error": float(np.mean(err_norm[-100:])),
        "max_tension": float(tensions.max()),
        "max_thrust": float(thrust_mag.max()),
        "mean_thrust": float(thrust_mag.mean()),
        "total_impulse": float(cum_impulse),
        "tension_imbalance_rms": float(np.sqrt(np.mean(np.std(tensions, axis=1) ** 2))),
    }


def make_summary_figure(metrics: dict[str, dict], out_path: Path):
    names = list(metrics.keys())
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(names), 3)))

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # Panel 1: Tracking accuracy
    ax = axes[0, 0]
    rms = [metrics[n]["rms_error"] for n in names]
    rms_post = [metrics[n]["rms_error_post_pickup"] for n in names]
    x = np.arange(len(names))
    w = 0.35
    ax.bar(x - w / 2, rms, w, label="Overall RMS", color="#1f77b4")
    ax.bar(x + w / 2, rms_post, w, label="Post-pickup (t>3s) RMS", color="#ff7f0e")
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("RMS tracking error [m]")
    ax.set_title("Payload Tracking Accuracy")
    ax.legend(); ax.grid(alpha=0.3, axis="y")

    # Panel 2: Peak errors
    ax = axes[0, 1]
    max_err = [metrics[n]["max_error"] for n in names]
    final_err = [metrics[n]["final_error"] for n in names]
    ax.bar(x - w / 2, max_err, w, label="Peak error", color="#d62728")
    ax.bar(x + w / 2, final_err, w, label="Final (last 100 samples)", color="#2ca02c")
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("Error [m]")
    ax.set_title("Peak and Final Tracking Error")
    ax.legend(); ax.grid(alpha=0.3, axis="y")

    # Panel 3: Max tension (stress indicator)
    ax = axes[0, 2]
    max_ten = [metrics[n]["max_tension"] for n in names]
    ax.bar(x, max_ten, color=colors)
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("Peak rope tension [N]")
    ax.set_title("Maximum Rope Tension\n(stress on surviving ropes)")
    ax.grid(alpha=0.3, axis="y")

    # Panel 4: Thrust stats
    ax = axes[1, 0]
    max_thrust = [metrics[n]["max_thrust"] for n in names]
    mean_thrust = [metrics[n]["mean_thrust"] for n in names]
    ax.bar(x - w / 2, max_thrust, w, label="Peak", color="#d62728")
    ax.bar(x + w / 2, mean_thrust, w, label="Mean", color="#2ca02c")
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("Thrust magnitude [N]")
    ax.set_title("Peak vs. Mean Thrust per Drone")
    ax.legend(); ax.grid(alpha=0.3, axis="y")

    # Panel 5: Total impulse (energy proxy)
    ax = axes[1, 1]
    impulse = [metrics[n]["total_impulse"] for n in names]
    ax.bar(x, impulse, color=colors)
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("Σ ∫ f_z dt [N·s]")
    ax.set_title("Total Vertical Impulse (energy proxy)")
    ax.grid(alpha=0.3, axis="y")

    # Panel 6: Tension imbalance
    ax = axes[1, 2]
    imb = [metrics[n]["tension_imbalance_rms"] for n in names]
    ax.bar(x, imb, color=colors)
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("RMS std(T) [N]")
    ax.set_title("Load-Sharing Imbalance\n(low = balanced)")
    ax.grid(alpha=0.3, axis="y")

    fig.suptitle("Campaign Summary — Decentralized Fault-Aware Lift",
                 fontsize=14, weight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def make_overlay_figure(scenarios: dict[str, pd.DataFrame], out_path: Path):
    """Overlay tracking error vs time across all scenarios."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 9))
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(scenarios), 3)))

    # Overlay: tracking error norm
    ax = axes[0]
    for (name, df), c in zip(scenarios.items(), colors):
        t = df["time"].values
        err = df[["ref_x", "ref_y", "ref_z"]].values - df[["payload_x", "payload_y", "payload_z"]].values
        e = np.linalg.norm(err, axis=1)
        ax.plot(t, e, lw=1.2, label=name, color=c)
    ax.set_xlabel("t [s]"); ax.set_ylabel("||e|| [m]")
    ax.set_title("Payload Tracking Error Norm — all scenarios")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # Overlay: tension imbalance
    ax = axes[1]
    for (name, df), c in zip(scenarios.items(), colors):
        N = num_drones(df)
        t = df["time"].values
        T = np.column_stack([df[f"tension_{i}"].values for i in range(N)])
        imb = np.std(T, axis=1)
        ax.plot(t, imb, lw=1.2, label=name, color=c)
    ax.set_xlabel("t [s]"); ax.set_ylabel("std(T) [N]")
    ax.set_title("Load-Sharing Imbalance — all scenarios")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    dir_path = Path(sys.argv[1])
    csv_files = sorted(dir_path.glob("scenario_*.csv"))
    if not csv_files:
        print(f"No scenario CSVs in {dir_path}"); sys.exit(1)

    metrics = {}
    scenarios = {}
    for p in csv_files:
        name = p.stem.replace("scenario_", "")
        print(f"Loading {name}...")
        df = pd.read_csv(p)
        scenarios[name] = df
        metrics[name] = compute_metrics(df)

    # Save CSV table
    metrics_df = pd.DataFrame(metrics).T
    metrics_df.to_csv(dir_path / "campaign_metrics.csv")
    print(f"\nMetrics table:\n{metrics_df.to_string()}\n")

    # Save figures
    make_summary_figure(metrics, dir_path / "campaign_summary.png")
    make_overlay_figure(scenarios, dir_path / "campaign_overlay.png")
    print(f"Saved:")
    print(f"  {dir_path / 'campaign_metrics.csv'}")
    print(f"  {dir_path / 'campaign_summary.png'}")
    print(f"  {dir_path / 'campaign_overlay.png'}")


if __name__ == "__main__":
    main()
