#!/usr/bin/env python3
"""A1: Self-announcement mechanism figure.

Four panels on a common fault-zoom window around fault 1 (t1=12s)
of the V4 dual-fault schedule, each saved as its own PNG:
  (a): per-rope tension T_i(t)         (FF-on solid vs FF-off dashed)
  (b): feed-forward command T_i^ff(t)
  (c): per-drone commanded thrust f_i(t)
  (d): payload altitude error e_{L,z}(t)

These four panels expose the causal chain: peer cable severs ->
surviving T_i jumps -> the identity T_i^ff = T_i propagates the
jump -> thrust steps up at the plant's native rate -> altitude
error stays bounded. With FF off, the same fault triggers the
same T_i jump but T_i^ff and thrust do NOT step, and the altitude
error grows.

Output: IEEE_T-CST/Figures/fig_self_announcement_V4_{a,b,c,d}.png
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

    on_color = "#1F6F6F"     # teal for FF-on (the claim)
    off_color = "#D3580B"    # orange for FF-off (the counterfactual)

    def _trim(df):
        t = df["time"].values
        m = (t >= T_LO) & (t <= T_HI)
        return t[m], m

    def _new_axes(width_in: float = 7.5, height_in: float = 1.6):
        fig, ax = plt.subplots(figsize=(width_in, height_in))
        ax.axvline(T1, color="red", linestyle=":", linewidth=1.0,
                   alpha=0.65)
        ax.set_xlim(T_LO, T_HI)
        return fig, ax

    def _save(fig, suffix: str):
        out = FIG / f"fig_self_announcement_V4_{suffix}.png"
        fig.tight_layout(pad=0.4)
        fig.savefig(out, dpi=200, bbox_inches="tight",
                    pad_inches=0.05)
        plt.close(fig)
        print(f"  wrote {out.name}")

    # Panel (a) — surviving rope tension.
    fig, ax = _new_axes()
    for df, lab, c, ls in [(ff_on, "FF-on", on_color, "-"),
                            (ff_off, "FF-off", off_color, "--")]:
        t, m = _trim(df)
        ax.plot(t, df[f"tension_{DRONE_FOCUS}"].values[m],
                color=c, linewidth=1.6, linestyle=ls, label=lab)
    ax.set_ylabel(rf"$T_{DRONE_FOCUS}(t)$ [N]", fontsize=9)
    ax.set_xlabel(r"time [s]", fontsize=9)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.92)
    _save(fig, "a")

    # Panel (b) — feed-forward command.
    fig, ax = _new_axes()
    for df, lab, c, ls in [(ff_on, "FF-on", on_color, "-"),
                            (ff_off, "FF-off", off_color, "--")]:
        t, m = _trim(df)
        ax.plot(t, df[f"T_ff_{DRONE_FOCUS}"].values[m],
                color=c, linewidth=1.6, linestyle=ls)
    ax.set_ylabel(rf"$T_{DRONE_FOCUS}^{{\mathrm{{ff}}}}(t)$ [N]",
                  fontsize=9)
    ax.set_xlabel(r"time [s]", fontsize=9)
    _save(fig, "b")

    # Panel (c) — commanded thrust.
    fig, ax = _new_axes()
    for df, lab, c, ls in [(ff_on, "FF-on", on_color, "-"),
                            (ff_off, "FF-off", off_color, "--")]:
        t, m = _trim(df)
        ax.plot(t, df[f"thrust_cmd_{DRONE_FOCUS}"].values[m],
                color=c, linewidth=1.6, linestyle=ls)
    ax.set_ylabel(rf"$f_{DRONE_FOCUS}(t)$ [N]", fontsize=9)
    ax.set_xlabel(r"time [s]", fontsize=9)
    _save(fig, "c")

    # Panel (d) — payload altitude error.
    fig, ax = _new_axes()
    for df, lab, c, ls in [(ff_on, "FF-on", on_color, "-"),
                            (ff_off, "FF-off", off_color, "--")]:
        t, m = _trim(df)
        ez = df["ref_z"].values[m] - df["payload_z"].values[m]
        ax.plot(t, ez * 1000, color=c, linewidth=1.6, linestyle=ls)
    ax.axhline(0, color="black", linewidth=0.4, linestyle=":",
               alpha=0.6)
    ax.set_ylabel(r"$e_{L,z}(t)$ [mm]", fontsize=9)
    ax.set_xlabel(r"time [s]", fontsize=9)
    _save(fig, "d")

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
