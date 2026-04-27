#!/usr/bin/env python3
"""WP2.3: N-cross-validation analysis.

Process the N=3 (m_L=6kg, F=1) and N=7 (m_L=14kg, F=2) runs and
compare against the canonical N=5 V4 (m_L=10kg, F=2). Demonstrates
that the architecture's recovery envelope is qualitatively
invariant in N within the actuator-margin envelope.

Outputs:
    output/n_cross_validation/summary.csv
    IEEE_T-CST/Figures/fig_n_cross_validation.png
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

NCROSS = Path("/workspaces/Tether_Grace/output/n_cross_validation")
CANON = Path("/workspaces/Tether_Grace/output/capability_demo/V4_dual_5s_wind")
FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")


def metrics(csv: Path, t1: float = 12.0):
    df = pd.read_csv(csv)
    t = df["time"].values
    cruise = (t >= 8.0) & (t <= 30.0)
    dx = df["payload_x"].values[cruise] - df["ref_x"].values[cruise]
    dy = df["payload_y"].values[cruise] - df["ref_y"].values[cruise]
    dz = df["payload_z"].values[cruise] - df["ref_z"].values[cruise]
    rmse3d = float(np.sqrt(np.mean(dx*dx + dy*dy + dz*dz)))
    bm = (t >= max(0.0, t1 - 0.2)) & (t < t1)
    sag = df["ref_z"].values - df["payload_z"].values
    offset = float(np.mean(sag[bm])) if bm.any() else 0.0
    pf = (t >= t1) & (t <= 30.0)
    peak_sag_mm = float(np.max(np.abs(sag[pf] - offset))) * 1000
    N = num_drones(df)
    tp = np.max([df[f"tension_{i}"].values for i in range(N)], axis=0)
    peak_T_post = float(np.max(tp[pf]))
    return rmse3d, peak_sag_mm, peak_T_post


def main():
    rows = []
    canonical = CANON / "scenario_V4_dual_5s_wind.csv"
    if canonical.exists():
        r, s, t = metrics(canonical)
        rows.append({"label": "N=5, m=10kg (V4 canonical, F=2)",
                     "N": 5, "m_L": 10, "F": 2,
                     "rmse3d_m": r, "peak_sag_mm": s,
                     "peak_T_N": t})

    for tag, N, m, F in [("N3_m6kg_F1", 3, 6, 1),
                          ("N7_m14kg_F2", 7, 14, 2)]:
        csv = NCROSS / tag / f"scenario_{tag}.csv"
        if not csv.exists():
            print(f"  {tag}: missing"); continue
        r, s, t = metrics(csv)
        rows.append({"label": f"N={N}, m={m}kg (F={F})",
                     "N": N, "m_L": m, "F": F,
                     "rmse3d_m": r, "peak_sag_mm": s,
                     "peak_T_N": t})

    if len(rows) < 2:
        print("insufficient data")
        return
    sdf = pd.DataFrame(rows)
    sdf.to_csv(NCROSS / "summary.csv", index=False)
    print(sdf.round(2).to_string(index=False))

    # Figure: triple-bar chart per configuration.
    fig, ax_main = plt.subplots(figsize=(7.0, 4.0))
    sag_color = "#1F6F6F"
    rmse_color = "#D3580B"
    x = np.arange(len(sdf))
    w = 0.36
    ax_main.bar(x - w/2, sdf["peak_sag_mm"], width=w,
                color=sag_color, edgecolor="black", linewidth=0.5,
                label="peak post-fault sag")
    ax_main.set_xticks(x)
    ax_main.set_xticklabels(sdf["label"], rotation=0, fontsize=8)
    ax_main.set_xlabel("configuration", fontsize=10)
    ax_main.set_ylabel(r"peak post-fault sag $[\mathrm{mm}]$ "
                       r"(left)", color=sag_color, fontsize=10)
    ax_main.tick_params(axis="y", labelcolor=sag_color)

    ax_sec = ax_main.twinx()
    ax_sec.bar(x + w/2, sdf["rmse3d_m"] * 1000, width=w,
               color=rmse_color, edgecolor="black", linewidth=0.5,
               hatch="//", label="3-D cruise RMSE")
    ax_sec.set_ylabel(r"3-D cruise RMSE $[\mathrm{mm}]$ "
                      r"(right)", color=rmse_color, fontsize=10)
    ax_sec.tick_params(axis="y", labelcolor=rmse_color)

    main_h, main_l = ax_main.get_legend_handles_labels()
    sec_h, sec_l = ax_sec.get_legend_handles_labels()
    ax_main.legend(main_h + sec_h, main_l + sec_l,
                   loc="upper left", fontsize=8, framealpha=0.92)
    ax_main.set_title("Team-size cross-validation (WP2.3): "
                      "architecture invariant in $N$ within the "
                      "actuator-margin envelope", fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG / "fig_n_cross_validation.png", dpi=200)
    plt.close(fig)
    print("  wrote fig_n_cross_validation.png")


if __name__ == "__main__":
    main()
