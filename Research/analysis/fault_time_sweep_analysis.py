#!/usr/bin/env python3
"""WP4.2: Fault-time permutation analysis.

Process the t1 in {8, 10, 12, 14, 16} s sweep on the V4 dual-fault
schedule (preserved 5 s inter-fault gap, drones 0 and 2). Tests
trajectory-phase invariance: does recovery quality depend on
where in the lemniscate cycle the first fault occurs?

Outputs:
    output/fault_time_sweep/summary.csv
    IEEE_T-CST/Figures/fig_fault_time_sweep.png
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

ROOT = Path("/workspaces/Tether_Grace/output/fault_time_sweep")
FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")
T1S = [8, 10, 12, 14, 16]


def metrics(df: pd.DataFrame, t1: float) -> dict:
    t = df["time"].values
    cruise = (t >= max(t1 - 4.0, 4.0)) & (t <= 30.0)
    dx = df["payload_x"].values[cruise] - df["ref_x"].values[cruise]
    dy = df["payload_y"].values[cruise] - df["ref_y"].values[cruise]
    dz = df["payload_z"].values[cruise] - df["ref_z"].values[cruise]
    rmse3d = float(np.sqrt(np.mean(dx*dx + dy*dy + dz*dz)))
    # Peak post-fault sag, detrended by 0.2 s pre-fault DC offset
    bm = (t >= max(0.0, t1 - 0.2)) & (t < t1)
    sag = df["ref_z"].values - df["payload_z"].values
    offset = float(np.mean(sag[bm])) if bm.any() else 0.0
    pf = (t >= t1) & (t <= min(t1 + 3.0, 30.0))
    peak_sag_mm = float(np.max(np.abs(sag[pf] - offset))) * 1000
    # Peak rope tension across all surviving ropes post-fault
    N = num_drones(df)
    tp = np.max([df[f"tension_{i}"].values for i in range(N)], axis=0)
    peak_T_post = float(np.max(tp[pf]))
    return {"rmse3d_m": rmse3d, "peak_sag_mm": peak_sag_mm,
            "peak_T_post_N": peak_T_post}


def main():
    rows = []
    for t1 in T1S:
        tag = f"t1_{t1}s"
        csv = ROOT / tag / f"scenario_{tag}.csv"
        if not csv.exists():
            print(f"  {tag}: missing"); continue
        df = pd.read_csv(csv)
        m = metrics(df, float(t1))
        rows.append({"t1_s": t1, **m})
        print(f"  t1={t1}s: RMSE={m['rmse3d_m']*1000:.0f}mm  "
              f"peak_sag={m['peak_sag_mm']:.1f}mm  "
              f"peak_T={m['peak_T_post_N']:.1f}N")
    sdf = pd.DataFrame(rows)
    sdf.to_csv(ROOT / "summary.csv", index=False)
    print(f"\nwrote {ROOT/'summary.csv'}")

    # Figure: dual-axis. Peak sag (left, blue) + RMSE (right, orange).
    fig, ax_main = plt.subplots(figsize=(7.0, 4.0))
    sag_color = "#1F6F6F"
    rmse_color = "#D3580B"
    ax_main.plot(sdf["t1_s"], sdf["peak_sag_mm"],
                 color=sag_color, marker="o", linewidth=1.7,
                 markersize=8, label="peak post-fault altitude sag")
    ax_main.set_xlabel(r"first-fault time $t_1$ $[\mathrm{s}]$",
                       fontsize=10)
    ax_main.set_ylabel(r"peak post-fault sag $[\mathrm{mm}]$ "
                       r"(left axis)", color=sag_color, fontsize=10)
    ax_main.tick_params(axis="y", labelcolor=sag_color)
    ax_main.spines["left"].set_color(sag_color)

    ax_sec = ax_main.twinx()
    ax_sec.plot(sdf["t1_s"], sdf["rmse3d_m"] * 1000,
                color=rmse_color, marker="s", linewidth=1.4,
                linestyle="--", markersize=7,
                label="3-D cruise RMSE")
    ax_sec.set_ylabel(r"3-D cruise RMSE $[\mathrm{mm}]$ "
                      r"(right axis)", color=rmse_color, fontsize=10)
    ax_sec.tick_params(axis="y", labelcolor=rmse_color)
    ax_sec.spines["right"].set_color(rmse_color)

    main_h, main_l = ax_main.get_legend_handles_labels()
    sec_h, sec_l = ax_sec.get_legend_handles_labels()
    ax_main.legend(main_h + sec_h, main_l + sec_l,
                   loc="upper left", fontsize=8, framealpha=0.92)
    ax_main.set_title(r"Fault-time permutation: peak sag and "
                      r"RMSE vs $t_1$ ($\Delta t{=}5$ s)",
                      fontsize=9, pad=4)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIG / "fig_fault_time_sweep.png", dpi=200,
                bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print("  wrote fig_fault_time_sweep.png")


if __name__ == "__main__":
    main()
