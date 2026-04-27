#!/usr/bin/env python3
"""WP4.1: Fault-index permutation analysis.

Process the 6-pair sweep (drone pairs covering every geometric
relationship in a 5-drone formation) on the V4 schedule.

Outputs:
    output/fault_index_sweep/summary.csv
    IEEE_T-CST/Figures/fig_fault_index_sweep.png
"""
from __future__ import annotations
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import setup_style  # noqa: E402
from plot_capability_demo import num_drones  # noqa: E402

setup_style()

ROOT = Path("/workspaces/Tether_Grace/output/fault_index_sweep")
FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")
PAIRS = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
T1, T2 = 12.0, 17.0


def metrics(df: pd.DataFrame) -> dict:
    t = df["time"].values
    cruise = (t >= 8.0) & (t <= 30.0)
    dx = df["payload_x"].values[cruise] - df["ref_x"].values[cruise]
    dy = df["payload_y"].values[cruise] - df["ref_y"].values[cruise]
    dz = df["payload_z"].values[cruise] - df["ref_z"].values[cruise]
    rmse3d = float(np.sqrt(np.mean(dx*dx + dy*dy + dz*dz)))
    bm = (t >= max(0.0, T1 - 0.2)) & (t < T1)
    sag = df["ref_z"].values - df["payload_z"].values
    offset = float(np.mean(sag[bm])) if bm.any() else 0.0
    pf = (t >= T1) & (t <= 30.0)
    peak_sag_mm = float(np.max(np.abs(sag[pf] - offset))) * 1000
    N = num_drones(df)
    tp = np.max([df[f"tension_{i}"].values for i in range(N)], axis=0)
    peak_T_post = float(np.max(tp[pf]))
    return {"rmse3d_m": rmse3d, "peak_sag_mm": peak_sag_mm,
            "peak_T_post_N": peak_T_post}


def main():
    rows = []
    for i1, i2 in PAIRS:
        tag = f"pair_{i1}_{i2}"
        csv = ROOT / tag / f"scenario_{tag}.csv"
        if not csv.exists():
            print(f"  {tag}: missing"); continue
        df = pd.read_csv(csv)
        m = metrics(df)
        rows.append({"i1": i1, "i2": i2, "pair": f"({i1},{i2})", **m})
        print(f"  pair=({i1},{i2}): RMSE={m['rmse3d_m']*1000:.0f}mm  "
              f"peak_sag={m['peak_sag_mm']:.1f}mm  "
              f"peak_T={m['peak_T_post_N']:.1f}N")
    if not rows:
        return
    sdf = pd.DataFrame(rows)
    sdf.to_csv(ROOT / "summary.csv", index=False)
    print(f"\nwrote {ROOT/'summary.csv'}")

    # Figure: dual-axis horizontal bar (peak sag + RMSE) per pair.
    fig, ax_main = plt.subplots(figsize=(7.0, 4.0))
    sag_color = "#1F6F6F"
    rmse_color = "#D3580B"
    x = np.arange(len(sdf))
    w = 0.36
    ax_main.bar(x - w/2, sdf["peak_sag_mm"], width=w,
                color=sag_color, edgecolor="black", linewidth=0.5,
                label="peak post-fault sag")
    ax_main.set_xticks(x)
    ax_main.set_xticklabels(sdf["pair"])
    ax_main.set_xlabel(r"failed-drone pair $(i_1, i_2)$",
                       fontsize=10)
    ax_main.set_ylabel(r"peak post-fault sag $[\mathrm{mm}]$ "
                       r"(left axis)",
                       color=sag_color, fontsize=10)
    ax_main.tick_params(axis="y", labelcolor=sag_color)

    ax_sec = ax_main.twinx()
    ax_sec.bar(x + w/2, sdf["rmse3d_m"] * 1000, width=w,
               color=rmse_color, edgecolor="black", linewidth=0.5,
               hatch="//", label="3-D cruise RMSE")
    ax_sec.set_ylabel(r"3-D cruise RMSE $[\mathrm{mm}]$ "
                      r"(right axis)",
                      color=rmse_color, fontsize=10)
    ax_sec.tick_params(axis="y", labelcolor=rmse_color)

    main_h, main_l = ax_main.get_legend_handles_labels()
    sec_h, sec_l = ax_sec.get_legend_handles_labels()
    ax_main.legend(main_h + sec_h, main_l + sec_l,
                   loc="upper right", fontsize=8, framealpha=0.92)
    ax_main.set_title("Fault-index permutation (WP4.1): "
                      "geometric-invariance test on the 5-drone "
                      r"formation, $\Delta t=5$ s, $t_1=12$ s",
                      fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG / "fig_fault_index_sweep.png", dpi=200)
    plt.close(fig)
    print("  wrote fig_fault_index_sweep.png")


if __name__ == "__main__":
    main()
