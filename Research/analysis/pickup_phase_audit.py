#!/usr/bin/env python3
"""WP4.6: Pickup-phase exercise.

The simulator already starts each capability run with
drone-to-payload distance ~1.17 m versus rope rest length 1.25 m,
so the rope is initially slack and the pickup phase --- the slack
-> taut engagement on the rope-tension channel --- is exercised
over t in [0, ~3] s in every run. Capability evaluation discards
that window via the [8,30]s analysis cut. Here we analyse it
directly to demonstrate the controller engages payload pickup
without intervention or supervisor.

Outputs:
    IEEE_T-CST/Figures/fig_pickup_engagement_V3.png  (tension + altitude)
    output/pickup_phase/V3_pickup_metrics.csv
"""
from __future__ import annotations
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, DOUBLE_COL, setup_style  # noqa: E402
from plot_capability_demo import num_drones  # noqa: E402

setup_style()

CAP = Path("/workspaces/Tether_Grace/output/capability_demo")
OUT = Path("/workspaces/Tether_Grace/output/pickup_phase")
FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")
OUT.mkdir(parents=True, exist_ok=True)
FIG.mkdir(parents=True, exist_ok=True)

TAG = "V3_single_wind"


def main():
    csv = CAP / TAG / f"scenario_{TAG}.csv"
    if not csv.exists():
        print(f"missing {csv}")
        return
    df = pd.read_csv(csv)
    N = num_drones(df)
    t = df["time"].values
    pickup_window = t <= 5.0

    # Tension threshold: 5 N is "engaged" per the controller's
    # pickup-target tension defaults.
    THR = 5.0
    engagement_times = []
    for i in range(N):
        T = df[f"tension_{i}"].values
        idx = np.argmax((T > THR).astype(int))
        if T[idx] > THR:
            engagement_times.append(t[idx])

    # Lift initiation: payload altitude crossing 0.05 m above start.
    payload_z = df["payload_z"].values
    z0 = float(payload_z[0])
    lift_idx = np.argmax(payload_z - z0 > 0.05)
    lift_time = float(t[lift_idx]) if (payload_z[lift_idx] - z0) > 0.05 else np.nan

    # Save metrics
    metrics = pd.DataFrame({
        "rope_id": list(range(N)),
        "engagement_time_s": engagement_times + [np.nan]*(N-len(engagement_times))
    })
    metrics.to_csv(OUT / "V3_pickup_metrics.csv", index=False)
    print(f"  wrote {OUT/'V3_pickup_metrics.csv'}")
    print(f"  engagement times (s): {engagement_times}")
    print(f"  payload-lift initiation: {lift_time:.3f} s")

    # Single canvas, dual y-axis. Rope tensions on the LEFT axis
    # (purple/teal family). Payload altitude on the RIGHT axis
    # (orange + black-dashed reference). Threshold horizontal
    # guide on left, lift-time vertical guide spanning both.
    fig, ax_main = plt.subplots(figsize=(7.5, 4.5))

    rope_cmap = plt.colormaps["viridis"]
    rope_colors = [rope_cmap(0.05 + 0.22 * i) for i in range(N)]
    rope_axis_color = "#1F6F6F"
    for i in range(N):
        ax_main.plot(t[pickup_window],
                     df[f"tension_{i}"].values[pickup_window],
                     color=rope_colors[i], linewidth=1.2,
                     label=rf"rope $T_{i}$")
    ax_main.axhline(THR, color="#444444", linestyle=":",
                    linewidth=1.0,
                    label=r"engagement threshold "
                          r"$T_\mathrm{thr}=5$ N")
    ax_main.set_xlabel(r"time $[\mathrm{s}]$", fontsize=10)
    ax_main.set_ylabel(r"rope tension $T_i$ $[\mathrm{N}]$ "
                       r"(left axis)",
                       color=rope_axis_color, fontsize=10)
    ax_main.tick_params(axis="y", labelcolor=rope_axis_color)
    ax_main.spines["left"].set_color(rope_axis_color)

    # Payload altitude on the RIGHT axis.
    ax_alt = ax_main.twinx()
    alt_color = "#D3580B"   # strong orange, distinct from rope family
    ax_alt.plot(t[pickup_window], payload_z[pickup_window],
                color=alt_color, linewidth=2.4,
                label=r"payload altitude $z_L$")
    ax_alt.plot(t[pickup_window], df["ref_z"].values[pickup_window],
                color="black", linestyle="--", linewidth=1.0,
                label=r"reference altitude $z_L^*$")
    if not np.isnan(lift_time):
        ax_alt.axvline(lift_time, color=alt_color, linestyle="-.",
                       linewidth=1.0, alpha=0.65,
                       label=rf"lift initiation $t={lift_time:.2f}$ s")
    ax_alt.set_ylabel(r"payload altitude $z_L$ $[\mathrm{m}]$ "
                      r"(right axis)",
                      color=alt_color, fontsize=10)
    ax_alt.tick_params(axis="y", labelcolor=alt_color)
    ax_alt.spines["right"].set_color(alt_color)

    # Two compact legends, one per axis colour family, placed
    # inside the panel where space allows.
    leg_left = ax_main.legend(loc="upper left", fontsize=7.5,
                              ncol=2, framealpha=0.92,
                              title="left axis (rope tension)",
                              title_fontsize=8)
    leg_left.get_frame().set_edgecolor(rope_axis_color)
    leg_right = ax_alt.legend(loc="lower right", fontsize=7.5,
                              framealpha=0.92,
                              title="right axis (payload altitude)",
                              title_fontsize=8)
    leg_right.get_frame().set_edgecolor(alt_color)
    ax_main.set_title("V3 pickup phase: rope-tension engagement "
                      "and payload lift",
                      fontsize=9, pad=4)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIG / "fig_pickup_engagement_V3.png", dpi=200,
                bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print(f"  wrote fig_pickup_engagement_V3.png "
          f"(single canvas, dual y-axis)")


if __name__ == "__main__":
    main()
