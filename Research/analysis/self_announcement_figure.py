#!/usr/bin/env python3
"""A1: Self-announcement mechanism figure.

Single canvas, four stacked rows on a shared time axis covering a
fault-zoom window around fault 1 (t1=12s) of the V4 dual-fault
schedule:
  Row 1: per-rope tension T_i(t)         (FF-on solid vs FF-off dashed)
  Row 2: feed-forward command T_i^ff(t)
  Row 3: per-drone commanded thrust f_i(t)
  Row 4: payload altitude error e_{L,z}(t)

This single figure exposes the causal chain: peer cable severs ->
surviving T_i jumps -> the identity T_i^ff = T_i propagates the
jump -> thrust steps up at the plant's native rate -> altitude
error stays bounded. With FF off, the same fault triggers the
same T_i jump but T_i^ff and thrust do NOT step, and the altitude
error grows.

Output: IEEE_T-CST/Figures/fig_self_announcement_V4.png
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
P2A = Path("/workspaces/Tether_Grace/output/p2a_tension_ff_ablation")
FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")

T1 = 12.0       # canonical V4 first-fault time
T_LO = 11.5     # window start
T_HI = 14.5     # window end
DRONE_FOCUS = 1   # surviving drone whose tension is the cleanest
                  # signal of the fault (drone 0 fails at t1)


def _load(root: Path, tag: str) -> pd.DataFrame | None:
    csv = root / tag / f"scenario_{tag}.csv"
    return pd.read_csv(csv) if csv.exists() else None


def main():
    ff_on = _load(CAP, "V4_dual_5s_wind")
    ff_off = _load(P2A, "V4_dual_5s_wind_nff")
    if ff_on is None or ff_off is None:
        print("missing required data")
        return

    fig, axes = plt.subplots(4, 1, figsize=(7.5, 8.0), sharex=True)
    ax_T, ax_Tff, ax_f, ax_e = axes

    for ax in axes:
        ax.axvline(T1, color="red", linestyle=":", linewidth=1.0,
                   alpha=0.65)

    on_color = "#1F6F6F"     # teal for FF-on (the claim)
    off_color = "#D3580B"    # orange for FF-off (the counterfactual)

    # Row 1 — surviving rope tension (drone 1, the clean signal).
    for df, lab, c, ls in [(ff_on, "FF-on", on_color, "-"),
                            (ff_off, "FF-off", off_color, "--")]:
        t = df["time"].values
        m = (t >= T_LO) & (t <= T_HI)
        ax_T.plot(t[m], df[f"tension_{DRONE_FOCUS}"].values[m],
                  color=c, linewidth=1.6, linestyle=ls,
                  label=lab)
    ax_T.set_ylabel(rf"$T_{DRONE_FOCUS}(t)$ $[\mathrm{{N}}]$  (peer-cable"
                    rf" severance announces here)", fontsize=9)
    ax_T.legend(loc="upper right", fontsize=8, framealpha=0.92)
    # Short panel title; full description in caption.
    ax_T.set_title(r"V4 self-announcement chain "
                   r"$T_i\!\to\!T_i^{\mathrm{ff}}\!\to\!f_i\!\to\!e_{L,z}$",
                   fontsize=9, pad=4)

    # Row 2 — feed-forward command (the identity T_ff = T applied,
    # or NOT applied in the FF-off case).
    for df, lab, c, ls in [(ff_on, "FF-on", on_color, "-"),
                            (ff_off, "FF-off", off_color, "--")]:
        t = df["time"].values
        m = (t >= T_LO) & (t <= T_HI)
        ax_Tff.plot(t[m], df[f"T_ff_{DRONE_FOCUS}"].values[m],
                    color=c, linewidth=1.6, linestyle=ls)
    ax_Tff.set_ylabel(rf"$T_{DRONE_FOCUS}^{{\mathrm{{ff}}}}(t)$ $[\mathrm{{N}}]$  "
                      r"(identity translates announcement)",
                      fontsize=9)

    # Row 3 — commanded thrust (the response).
    for df, lab, c, ls in [(ff_on, "FF-on", on_color, "-"),
                            (ff_off, "FF-off", off_color, "--")]:
        t = df["time"].values
        m = (t >= T_LO) & (t <= T_HI)
        ax_f.plot(t[m], df[f"thrust_cmd_{DRONE_FOCUS}"].values[m],
                  color=c, linewidth=1.6, linestyle=ls)
    ax_f.set_ylabel(rf"$f_{DRONE_FOCUS}(t)$ $[\mathrm{{N}}]$  "
                    r"(commanded thrust steps up)",
                    fontsize=9)

    # Row 4 — payload altitude error.
    for df, lab, c, ls in [(ff_on, "FF-on", on_color, "-"),
                            (ff_off, "FF-off", off_color, "--")]:
        t = df["time"].values
        m = (t >= T_LO) & (t <= T_HI)
        ez = df["ref_z"].values[m] - df["payload_z"].values[m]
        ax_e.plot(t[m], ez * 1000, color=c, linewidth=1.6,
                  linestyle=ls)
    ax_e.set_ylabel(r"$e_{L,z}(t)$ $[\mathrm{mm}]$  "
                    r"(payload altitude error)", fontsize=9)
    ax_e.set_xlabel(r"time $[\mathrm{s}]$", fontsize=10)
    ax_e.axhline(0, color="black", linewidth=0.4, linestyle=":",
                 alpha=0.6)

    fig.tight_layout(pad=0.4, h_pad=0.25)
    fig.savefig(FIG / "fig_self_announcement_V4.png", dpi=200,
                bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print("  wrote fig_self_announcement_V4.png")

    # Compact metrics for the §VI text: tension-rise latency,
    # thrust-response latency, peak sag, recovery time.
    def _latency(df, channel: str) -> float:
        """First time after t1 at which channel exceeds its
        pre-fault running mean by 5x its pre-fault std (a robust
        first-step detector)."""
        t = df["time"].values
        x = df[channel].values
        pre = (t >= T1 - 0.2) & (t < T1)
        mu, sigma = float(np.mean(x[pre])), float(np.std(x[pre]) + 1e-3)
        post = t >= T1
        for ti, xi in zip(t[post], x[post]):
            if abs(xi - mu) > 5 * sigma:
                return float(ti - T1)
        return float("nan")

    print("\n  Tension-rise / thrust-response latencies (FF-on):")
    print(f"    {DRONE_FOCUS} tension rise:  "
          f"{_latency(ff_on, f'tension_{DRONE_FOCUS}')*1000:.1f} ms")
    print(f"    {DRONE_FOCUS} thrust step:   "
          f"{_latency(ff_on, f'thrust_cmd_{DRONE_FOCUS}')*1000:.1f} ms")
    # Peak sag and recovery (FF on vs off).
    for label, df in [("FF-on", ff_on), ("FF-off", ff_off)]:
        t = df["time"].values
        m = (t >= T1) & (t <= T1 + 3.0)
        sag = (df["ref_z"].values[m] - df["payload_z"].values[m])
        peak_sag_mm = float(np.max(np.abs(sag))) * 1000
        # Recovery time: first time after t1 at which |e_z| returns
        # below 10 mm continuously for 0.3 s.
        rec_t = np.nan
        thr = 0.010
        hold = 0.3
        post = t >= T1
        for i, ti in enumerate(t[post]):
            j_end = i + int(hold / 1e-3)  # 1 ms-spaced log
            if j_end >= len(t[post]):
                break
            window_e = np.abs(df["ref_z"].values[post][i:j_end]
                              - df["payload_z"].values[post][i:j_end])
            if window_e.size and np.all(window_e < thr):
                rec_t = float(ti - T1)
                break
        print(f"    {label}: peak_sag={peak_sag_mm:.1f}mm  "
              f"t_rec(10mm)={rec_t:.2f}s")


if __name__ == "__main__":
    main()
