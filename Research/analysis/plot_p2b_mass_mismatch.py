#!/usr/bin/env python3
"""P2-B analysis: payload-mass mismatch for the L1 adaptive layer.

Three modes per (payload) mass:
    (A) baseline with tension feed-forward on;
    (B) baseline with tension feed-forward disabled;
    (C) mode-(B) + L1 augmentation.

Expected: mode A is insensitive to mass (FF reacts via T_measured);
mode B degrades linearly with mass error; mode C recovers toward
mode A.

Output: output/p2b_mass_mismatch/_summary/
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, DOUBLE_COL, setup_style
from plot_capability_demo import num_drones, tracking_error

setup_style()

ROOT = Path("/workspaces/Tether_Grace/output/p2b_mass_mismatch")
MASSES = [2.5, 3.0, 3.5, 3.9]
MODES = [("ff_on", "FF on"),
         ("ff_off", "FF off"),
         ("ff_off_l1", "FF off + L1")]


def _load(tag):
    csv = ROOT / tag / f"scenario_{tag}.csv"
    if not csv.exists():
        return None
    return pd.read_csv(csv)


def altitude_sag_rms(df):
    mask = df["time"].values >= 5.0  # post-pickup
    return float(np.sqrt(np.mean(
        (df["payload_z"].values[mask] - df["ref_z"].values[mask]) ** 2)))


def main():
    out = ROOT / "_summary"
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    for m in MASSES:
        mu = str(m).replace(".", "p")
        for mode, label in MODES:
            tag = f"p2b_m{mu}_{mode}"
            df = _load(tag)
            if df is None:
                continue
            mask = df["time"].values >= 5.0
            d = df[mask]
            N = num_drones(d)
            e = tracking_error(d)
            sag = d["payload_z"].values - d["ref_z"].values
            rows.append({
                "mass_kg":        m,
                "mode":           mode,
                "mode_label":     label,
                "rms_err_m":      round(float(np.sqrt(np.mean(e * e))), 4),
                "mean_alt_sag_m": round(float(np.mean(sag)), 4),
                "rms_alt_sag_m":  round(float(np.sqrt(np.mean(sag * sag))), 4),
            })
    summary = pd.DataFrame(rows)
    summary.to_csv(out / "p2b_summary.csv", index=False)
    print("\n=== P2-B summary ===")
    print(summary.to_string(index=False))

    # Plot RMS altitude-sag vs mass, one line per mode.
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i, (mode, label) in enumerate(MODES):
        sub = summary[summary["mode"] == mode].sort_values("mass_kg")
        ax.plot(sub["mass_kg"], sub["rms_alt_sag_m"],
                color=COLORS[i], lw=1.4, marker="o", label=label)
    ax.axvline(3.0, color="black", linestyle=":", lw=0.7,
               label="controller nominal = 3 kg")
    ax.set_xlabel("actual payload mass [kg]")
    ax.set_ylabel(r"RMS altitude sag [m]")
    ax.set_title("P2-B: L1 rejection of mass-mismatch disturbance",
                 fontsize=9)
    ax.legend(loc="upper left", fontsize=7.5)
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "p2b_altsag_vs_mass.png", dpi=180)
    plt.close(fig)

    # Plot mean altitude sag (signed) vs mass.
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i, (mode, label) in enumerate(MODES):
        sub = summary[summary["mode"] == mode].sort_values("mass_kg")
        ax.plot(sub["mass_kg"], sub["mean_alt_sag_m"],
                color=COLORS[i], lw=1.4, marker="o", label=label)
    ax.axvline(3.0, color="black", linestyle=":", lw=0.7)
    ax.axhline(0.0, color="black", linestyle="-", lw=0.5)
    ax.set_xlabel("actual payload mass [kg]")
    ax.set_ylabel(r"mean altitude sag $z_L - z_\text{ref}$ [m]")
    ax.set_title("P2-B: altitude bias vs mass", fontsize=9)
    ax.legend(loc="upper right", fontsize=7.5)
    ax.grid(True, linewidth=0.3, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "p2b_altbias_vs_mass.png", dpi=180)
    plt.close(fig)

    print(f"\nAll P2-B outputs under: {out}")


if __name__ == "__main__":
    main()
