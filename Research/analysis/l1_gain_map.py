#!/usr/bin/env python3
"""Tier-2F: L1 Adaptive-Gain Stability Map.

Post-processes output/l1_gain_map/gamma_*/ runs and produces:
    output/l1_gain_map/summary.csv
    IEEE_T-CST/Figures/fig_l1_rmse_vs_gamma.png
    IEEE_T-CST/Figures/fig_l1_altitude_sag_vs_gamma.png
    IEEE_T-CST/Figures/fig_l1_phase_portrait_gamma.png

Closed-form Euler-discretisation bound from Proposition 1:
    Gamma^* = 2 / (T_s * p_{22})  ~  4.75e4
with T_s = 5e-3 s and p_{22} from the discrete Lyapunov equation
solved at the canonical altitude operating point.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import COLORS, DOUBLE_COL, setup_style  # noqa: E402

setup_style()

ROOT = Path("/workspaces/Tether_Grace/output/l1_gain_map")
FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")
GAMMA_STAR = 4.75e4   # closed-form bound from Proposition 1
T_START = 3.0         # s — exclude startup transient
T_END = 20.0


def _load(tag: str) -> pd.DataFrame | None:
    csv = ROOT / tag / f"scenario_{tag}.csv"
    return pd.read_csv(csv) if csv.exists() else None


def _metrics(df: pd.DataFrame) -> dict:
    t = df["time"].values
    m = (t >= T_START) & (t <= T_END)
    dx = df["payload_x"].values[m] - df["ref_x"].values[m]
    dy = df["payload_y"].values[m] - df["ref_y"].values[m]
    dz = df["payload_z"].values[m] - df["ref_z"].values[m]
    rmse3d = float(np.sqrt(np.mean(dx * dx + dy * dy + dz * dz)))
    rmse_alt = float(np.sqrt(np.mean(dz * dz)))
    sag = df["ref_z"].values - df["payload_z"].values
    peak_sag_mm = float(np.max(np.abs(sag[m]))) * 1000
    # Altitude-error variance growth: stability proxy.
    # Compare first 25% of cruise window to last 25%.
    n = len(dz)
    early = dz[: n // 4]
    late = dz[-n // 4:]
    var_growth = float(np.std(late) / max(np.std(early), 1e-6))
    return {"rmse3d_m": rmse3d, "rmse_alt_m": rmse_alt,
            "peak_sag_mm": peak_sag_mm, "var_growth_ratio": var_growth}


def main():
    gammas = sorted([int(p.name.replace("gamma_", ""))
                     for p in ROOT.glob("gamma_*") if p.is_dir()])
    if not gammas:
        print("no L1 gain-map runs yet")
        return
    rows = []
    for g in gammas:
        tag = f"gamma_{g}"
        df = _load(tag)
        if df is None:
            print(f"  missing {tag}, skipping")
            continue
        m = _metrics(df)
        rows.append({"gamma": g, "gamma_ratio": g / GAMMA_STAR, **m})
        print(f"  {tag}: RMSE={m['rmse3d_m']*1000:6.1f}mm  "
              f"alt={m['rmse_alt_m']*1000:6.1f}mm  "
              f"peak_sag={m['peak_sag_mm']:6.1f}mm  "
              f"var_growth={m['var_growth_ratio']:.2f}")
    sdf = pd.DataFrame(rows).sort_values("gamma")
    sdf.to_csv(ROOT / "summary.csv", index=False)
    print(f"\nwrote {ROOT/'summary.csv'}")
    print(sdf.round(3).to_string(index=False))

    _fig_stability_dashboard(sdf)


def _fig_stability_dashboard(sdf: pd.DataFrame):
    """Combined L1 stability dashboard: altitude RMSE (left axis,
    primary metric) and peak post-fault sag (right axis,
    secondary) versus Gamma, with the Proposition-1 sufficiency
    boundary as a shaded region. Replaces three separate plots."""
    g = sdf["gamma"].values
    rmse_alt_mm = sdf["rmse_alt_m"].values * 1000
    peak_sag = sdf["peak_sag_mm"].values
    var_growth = sdf["var_growth_ratio"].values

    fig, ax_main = plt.subplots(figsize=(7.0, 4.0))

    # Shaded regions: green wash to the left of Gamma_star
    # ("contraction guaranteed"), grey wash beyond ("beyond
    # sufficient bound, practical stability holds empirically").
    ax_main.axvspan(g.min() * 0.5, GAMMA_STAR,
                    color="#cfe8d3", alpha=0.55,
                    label=r"contraction guarantee region "
                          r"($\Gamma < \Gamma^*$)")
    ax_main.axvspan(GAMMA_STAR, g.max() * 1.5,
                    color="#eaeaea", alpha=0.55,
                    label=r"beyond sufficient bound "
                          r"(empirical stability)")

    # Primary: altitude RMSE on the left axis (orange).
    rmse_color = COLORS[1]
    line_rmse, = ax_main.plot(g, rmse_alt_mm,
                              color=rmse_color, marker="o",
                              linewidth=1.6, markersize=7,
                              label=r"altitude RMSE")
    ax_main.set_xscale("log")
    ax_main.set_xlabel(r"$L_1$ adaptive gain $\Gamma$",
                       fontsize=10)
    ax_main.set_ylabel(r"altitude RMSE $[\mathrm{mm}]$",
                       color=rmse_color, fontsize=10)
    ax_main.tick_params(axis="y", labelcolor=rmse_color)
    ax_main.axvline(GAMMA_STAR, color="red", linestyle="--",
                    linewidth=1.0,
                    label=r"$\Gamma^* = 4.75\!\times\!10^4$ "
                          r"(Prop.~1)")

    # Secondary: peak post-fault altitude sag on the right axis (blue).
    sag_color = COLORS[0]
    ax_sec = ax_main.twinx()
    line_sag, = ax_sec.plot(g, peak_sag,
                            color=sag_color, marker="s",
                            linewidth=1.4, markersize=6,
                            linestyle="--",
                            label=r"peak altitude sag")
    ax_sec.set_ylabel(r"peak altitude sag $[\mathrm{mm}]$",
                      color=sag_color, fontsize=10)
    ax_sec.tick_params(axis="y", labelcolor=sag_color)

    # Annotate var_growth near each point as a small text label,
    # carrying the stability-proxy info (was its own figure).
    for gi, vg, ri in zip(g, var_growth, rmse_alt_mm):
        ax_main.annotate(f"{vg:.2f}", xy=(gi, ri),
                         xytext=(0, 10), textcoords="offset points",
                         fontsize=6.5, ha="center",
                         color="#444444")

    ax_main.set_title(r"$L_1$ stability dashboard"
                      r" (italics: variance-growth ratio)",
                      fontsize=9, pad=4)
    # Combined legend: pull handles from both axes + the regions.
    main_handles, main_labels = ax_main.get_legend_handles_labels()
    sec_handles, sec_labels = ax_sec.get_legend_handles_labels()
    ax_main.legend(main_handles + sec_handles,
                   main_labels + sec_labels,
                   loc="upper center", fontsize=7.5, ncol=2,
                   framealpha=0.92)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIG / "fig_l1_stability_dashboard.png", dpi=200,
                bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print("  wrote fig_l1_stability_dashboard.png")


def _fig_rmse_vs_gamma(sdf: pd.DataFrame):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(sdf["gamma"], sdf["rmse_alt_m"] * 1000,
            color=COLORS[0], marker="o", linewidth=1.4,
            label="altitude RMSE")
    ax.axvline(GAMMA_STAR, color="red", linestyle="--", linewidth=0.9,
               label=r"$\Gamma^* = 4.75 \times 10^4$ (Prop.~1)")
    ax.set_xscale("log")
    ax.set_xlabel(r"L1 adaptive gain $\Gamma$")
    ax.set_ylabel(r"altitude RMSE $[\mathrm{mm}]$")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.set_title("L1 stability map: altitude RMSE vs adaptive gain",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG / "fig_l1_rmse_vs_gamma.png", dpi=180)
    plt.close(fig)
    print("  wrote fig_l1_rmse_vs_gamma.png")


def _fig_altitude_sag_vs_gamma(sdf: pd.DataFrame):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(sdf["gamma"], sdf["peak_sag_mm"],
            color=COLORS[2], marker="s", linewidth=1.4,
            label="peak altitude sag")
    ax.axvline(GAMMA_STAR, color="red", linestyle="--", linewidth=0.9,
               label=r"$\Gamma^*$ (Prop.~1)")
    ax.set_xscale("log")
    ax.set_xlabel(r"L1 adaptive gain $\Gamma$")
    ax.set_ylabel(r"peak altitude sag $[\mathrm{mm}]$")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.set_title("L1 stability map: peak altitude sag vs gain",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG / "fig_l1_altitude_sag_vs_gamma.png", dpi=180)
    plt.close(fig)
    print("  wrote fig_l1_altitude_sag_vs_gamma.png")


def _fig_var_growth(sdf: pd.DataFrame):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(sdf["gamma"], sdf["var_growth_ratio"],
            color=COLORS[1], marker="^", linewidth=1.4,
            label="late/early std ratio")
    ax.axhline(1.0, color="black", linestyle=":", linewidth=0.9,
               label="stationary regime")
    ax.axvline(GAMMA_STAR, color="red", linestyle="--", linewidth=0.9,
               label=r"$\Gamma^*$ (Prop.~1)")
    ax.set_xscale("log")
    ax.set_xlabel(r"L1 adaptive gain $\Gamma$")
    ax.set_ylabel("altitude-error std growth ratio")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.set_title("L1 stability map: error-variance growth",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG / "fig_l1_var_growth_vs_gamma.png", dpi=180)
    plt.close(fig)
    print("  wrote fig_l1_var_growth_vs_gamma.png")


if __name__ == "__main__":
    main()
