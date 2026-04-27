#!/usr/bin/env python3
"""A2: Actuator-margin audit.

For each capability variant V3..V6, report:
  - per-drone f_i(t) / f_max ratio time series
  - max(f_i / f_max) per drone per variant
  - % of cruise time with f_i > 0.9 f_max  (saturation hazard)
  - QP active-set transition fraction

Outputs:
  IEEE_T-CST/Figures/fig_actuator_margin.png
  output/actuator_margin/summary.csv
"""
from __future__ import annotations
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, setup_style  # noqa: E402
from plot_capability_demo import num_drones  # noqa: E402

setup_style()

CAP = Path("/workspaces/Tether_Grace/output/capability_demo")
OUT = Path("/workspaces/Tether_Grace/output/actuator_margin")
FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")
OUT.mkdir(parents=True, exist_ok=True)

F_MAX = 150.0   # per-drone thrust ceiling [N]
T_LO = 8.0
T_HI = 30.0


def _load(tag: str) -> pd.DataFrame | None:
    csv = CAP / tag / f"scenario_{tag}.csv"
    return pd.read_csv(csv) if csv.exists() else None


def _surviving(df: pd.DataFrame, N: int) -> np.ndarray:
    s = np.zeros((len(df), N), dtype=bool)
    for i in range(N):
        s[:, i] = df[f"tension_{i}"].values > 0.5
    return s


def main():
    rows = []
    fig, axes = plt.subplots(4, 1, figsize=(7.5, 7.5), sharex=True)
    variants = [("V3_single_wind", "V3", axes[0]),
                ("V4_dual_5s_wind", "V4", axes[1]),
                ("V5_dual_10s_wind", "V5", axes[2]),
                ("V6_dual_5s_fullstack", "V6", axes[3])]
    for tag, label, ax in variants:
        df = _load(tag)
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values
        cruise = (t >= T_LO) & (t <= T_HI)
        surv = _surviving(df, N)
        max_ratios = []
        sat_fracs = []
        for i in range(N):
            f_ratio = df[f"thrust_cmd_{i}"].values / F_MAX
            ax.plot(t[cruise], f_ratio[cruise],
                    color=COLORS[i % len(COLORS)],
                    linewidth=0.7,
                    label=f"drone {i}" if label == "V3" else None)
            # only count survivors and cruise window
            mask = cruise & surv[:, i]
            if mask.any():
                mr = float(np.max(f_ratio[mask]))
                sf = 100 * float(np.mean(f_ratio[mask] > 0.90))
            else:
                mr = sf = float("nan")
            max_ratios.append(mr)
            sat_fracs.append(sf)
        ax.axhline(0.90, color="red", linestyle="--", linewidth=0.6,
                   alpha=0.7)
        ax.axhline(1.00, color="black", linestyle=":", linewidth=0.6,
                   alpha=0.7)
        ax.set_ylabel(rf"{label}: $f_i/f_{{\max}}$",
                      fontsize=9)
        ax.set_ylim(0, 1.05)
        rows.append({
            "variant": label,
            "max_ratio_per_variant": float(np.nanmax(max_ratios)),
            "argmax_drone": int(np.nanargmax(max_ratios)),
            "max_sat_frac_pct":
                float(np.nanmax([sf for sf in sat_fracs
                                 if not np.isnan(sf)] or [0.0])),
        })
    axes[0].legend(loc="upper right", fontsize=7.5, ncol=5,
                   framealpha=0.92)
    axes[-1].set_xlabel(r"time $[\mathrm{s}]$", fontsize=10)
    # Title kept short — full description belongs in the caption.
    fig.suptitle(r"Commanded thrust ratio $f_i(t)/f_{\max}$, "
                 r"V3--V6 (red: $0.9$ sat.; black: ceiling)",
                 fontsize=9, y=0.995)
    fig.tight_layout(pad=0.4, h_pad=0.3, rect=[0, 0, 1, 0.97])
    fig.savefig(FIG / "fig_actuator_margin.png", dpi=200,
                bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print(f"  wrote fig_actuator_margin.png")

    sdf = pd.DataFrame(rows)
    sdf.to_csv(OUT / "summary.csv", index=False)
    print(sdf.round(3).to_string(index=False))


if __name__ == "__main__":
    main()
