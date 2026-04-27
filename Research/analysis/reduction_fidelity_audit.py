#!/usr/bin/env python3
"""Tier-1A Reduction Fidelity Audit.

Post-processes V1--V6 capability-demo CSVs to audit the taut-cable
reduction theorem (Thm 1). The reduction replaces the
3N_b-dimensional bead-chain state with a scalar per-rope tension;
its core approximation is that within the admissibility domain
Omega_tau^dwell, the surviving-cable tensions can be described by
a single effective value T_eff(t) up to bounded geometric and
dynamic residuals. We measure that residual directly.

The reduction residual per rope is

    eps_i(t) = T_i^KV(t) - T_eff(t)       (rope i taut)

where T_eff(t) = mean over surviving ropes of T_j. For the
reduction's O(delta + eta_max) error bound to be empirically
supported, eps_i must stay small compared to the nominal tension
m_L g / |S| ~ 20--30 N across the cruise window.

Additionally, we log the payload force-balance residual

    F_res(t) = || sum_i T_i cable_hat_i  -  m_L (a_ref - g e_z) ||

which is the net force the payload sees beyond what the reference
trajectory demands; a small F_res means the surviving ropes are
indeed supplying the reduction-predicted load to leading order.

Outputs:
    output/reduction_fidelity/{V*}_audit.csv
    output/reduction_fidelity/summary.csv
    IEEE_T-CST/Figures/fig_reduction_overlay_V4.png
    IEEE_T-CST/Figures/fig_reduction_error_cdf.png
    IEEE_T-CST/Figures/fig_reduction_error_vs_slack.png
    IEEE_T-CST/Figures/fig_force_balance_residual_V4.png
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
OUT = Path("/workspaces/Tether_Grace/output/reduction_fidelity")
FIG = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")
OUT.mkdir(parents=True, exist_ok=True)
FIG.mkdir(parents=True, exist_ok=True)

M_L = 10.0
G = 9.81
SLACK_THRESHOLD = 0.2   # N — below this the rope is slack
T_START = 8.0           # s — exclude pre-cruise startup transient
T_END = 30.0

VARIANTS = ["V1_nominal_nowind", "V2_nominal_wind", "V3_single_wind",
            "V4_dual_5s_wind", "V5_dual_10s_wind",
            "V6_dual_5s_fullstack"]


def _load(tag: str) -> pd.DataFrame | None:
    csv = CAP / tag / f"scenario_{tag}.csv"
    return pd.read_csv(csv) if csv.exists() else None


def _surviving(df: pd.DataFrame, N: int) -> np.ndarray:
    surv = np.zeros((len(df), N), dtype=bool)
    for i in range(N):
        surv[:, i] = df[f"tension_{i}"].values > SLACK_THRESHOLD
    return surv


def _cable_dir(df: pd.DataFrame, i: int) -> np.ndarray:
    """Unit vector (T, 3) from payload toward drone i."""
    dx = df[f"quad{i}_x"].values - df["payload_x"].values
    dy = df[f"quad{i}_y"].values - df["payload_y"].values
    dz = df[f"quad{i}_z"].values - df["payload_z"].values
    d = np.sqrt(dx * dx + dy * dy + dz * dz) + 1e-9
    return np.stack([dx / d, dy / d, dz / d], axis=1)


def _cruise_mask(df: pd.DataFrame) -> np.ndarray:
    t = df["time"].values
    return (t >= T_START) & (t <= T_END)


def audit_variant(tag: str) -> pd.DataFrame | None:
    df = _load(tag)
    if df is None:
        print(f"{tag}: MISSING")
        return None
    N = num_drones(df)
    mask = _cruise_mask(df)
    df_c = df[mask].reset_index(drop=True)
    T_i = np.stack([df_c[f"tension_{i}"].values for i in range(N)],
                   axis=1)
    surv = np.zeros_like(T_i, dtype=bool)
    for i in range(N):
        surv[:, i] = T_i[:, i] > SLACK_THRESHOLD

    # Effective lumped tension: mean over surviving ropes at each time.
    # Where no rope is taut, T_eff undefined — masked out below.
    surv_count = surv.sum(axis=1)
    T_eff = np.zeros(len(df_c))
    good = surv_count > 0
    T_eff[good] = np.where(surv[good], T_i[good], 0).sum(axis=1) / \
        surv_count[good]

    rows = []
    for i in range(N):
        for k in range(len(df_c)):
            if not surv[k, i]:
                continue
            rows.append((df_c["time"].values[k], i,
                         float(T_i[k, i]), float(T_eff[k]),
                         float(T_i[k, i] - T_eff[k])))
    adf = pd.DataFrame(rows, columns=["time", "rope",
                                      "T_KV", "T_eff", "eps"])
    adf.to_csv(OUT / f"{tag}_audit.csv", index=False)
    return adf


def _force_balance_residual_V4() -> tuple[np.ndarray, np.ndarray] | None:
    df = _load("V4_dual_5s_wind")
    if df is None:
        return None
    N = num_drones(df)
    mask = _cruise_mask(df)
    df_c = df[mask].reset_index(drop=True)
    t = df_c["time"].values

    # Net cable force on the payload from the surviving set.
    F = np.zeros((len(df_c), 3))
    for i in range(N):
        T = df_c[f"tension_{i}"].values
        d_hat = _cable_dir(df_c, i)
        # Cable pulls payload toward drone => +d_hat
        F += (T[:, None]) * d_hat

    # What payload needs: m_L * (a_ref + g*e_z)
    ax = np.gradient(df_c["ref_vx"].values, t)
    ay = np.gradient(df_c["ref_vy"].values, t)
    az = np.gradient(df_c["ref_vz"].values, t)
    F_expected = M_L * np.stack([ax, ay, az + G], axis=1)

    residual_mag = np.linalg.norm(F - F_expected, axis=1)
    return t, residual_mag


def main():
    per_variant_summary = []
    audits: dict[str, pd.DataFrame] = {}
    for tag in VARIANTS:
        adf = audit_variant(tag)
        if adf is None:
            continue
        audits[tag] = adf
        # Clean NaNs for stats
        eps = adf["eps"].dropna().values
        if len(eps) == 0:
            continue
        per_variant_summary.append((tag,
                                    float(np.sqrt(np.mean(eps ** 2))),
                                    float(np.percentile(np.abs(eps), 95)),
                                    float(np.max(np.abs(eps)))))
        print(f"  audited {tag}: "
              f"rms={np.sqrt(np.mean(eps ** 2)):.2f} N, "
              f"p95={np.percentile(np.abs(eps), 95):.2f} N, "
              f"peak={np.max(np.abs(eps)):.2f} N")

    sdf = pd.DataFrame(per_variant_summary,
                       columns=["variant", "rms_eps_N",
                                "p95_eps_N", "peak_eps_N"])
    sdf.to_csv(OUT / "summary.csv", index=False)
    print(f"\nwrote {OUT/'summary.csv'}")
    print(sdf.round(2).to_string(index=False))

    _fig_overlay_V4(audits)
    _fig_error_cdf(audits)
    _fig_error_vs_slack(audits)


def _fig_overlay_V4(audits):
    v4 = audits.get("V4_dual_5s_wind")
    if v4 is None or len(v4) == 0:
        return
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for rid in sorted(v4["rope"].unique()):
        sub = v4[v4["rope"] == rid].sort_values("time")
        ax.plot(sub["time"], sub["T_KV"],
                color=COLORS[int(rid) % len(COLORS)],
                linewidth=0.8,
                label=rf"$T_{int(rid)}^{{\mathrm{{KV}}}}$")
    # overlay T_eff once (identical across ropes)
    sub0 = v4[v4["rope"] == sorted(v4["rope"].unique())[0]].sort_values("time")
    ax.plot(sub0["time"], sub0["T_eff"], color="black",
            linestyle="--", linewidth=1.0,
            label=r"$T_{\mathrm{eff}}$ (lumped)")
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"rope tension $[\mathrm{N}]$")
    ax.set_title(r"V4 reduction audit: per-rope $T_i^{\mathrm{KV}}$ "
                 r"vs lumped $T_{\mathrm{eff}}$", fontsize=9)
    ax.legend(loc="upper right", fontsize=7.5, ncol=2)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIG / "fig_reduction_overlay_V4.png", dpi=180, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print("  wrote fig_reduction_overlay_V4.png")


def _fig_error_cdf(audits):
    all_eps = np.concatenate([
        np.abs(a["eps"].dropna().values)
        for a in audits.values() if a is not None and len(a) > 0])
    if len(all_eps) == 0:
        return
    all_eps = all_eps[all_eps > 0]  # log scale
    x = np.sort(all_eps)
    y = np.arange(1, len(x) + 1) / len(x)
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(x, y, color=COLORS[0], linewidth=1.2,
            label="empirical CDF (V1--V6)")
    # Nominal tension m_L g / N = 10*9.81/5 = 19.6 N => 10% ~ 2 N.
    ax.axvline(2.0, color="red", linestyle="--", linewidth=0.9,
               label=r"$10\%$ of nominal $\approx 2$ N")
    ax.axvline(5.0, color="orange", linestyle=":", linewidth=0.9,
               label=r"$25\%$ of nominal $\approx 5$ N")
    ax.set_xscale("log")
    ax.set_xlabel(r"$|\varepsilon_i| = "
                  r"|T_i^{\mathrm{KV}} - T_{\mathrm{eff}}|$ "
                  r"$[\mathrm{N}]$")
    ax.set_ylabel("empirical CDF")
    ax.legend(loc="lower right", fontsize=7.5)
    ax.set_title("Reduction residual CDF, aggregated V1--V6",
                 fontsize=9)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIG / "fig_reduction_error_cdf.png", dpi=180, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print("  wrote fig_reduction_error_cdf.png")


def _fig_error_vs_slack(audits):
    v4 = audits.get("V4_dual_5s_wind")
    if v4 is None or len(v4) == 0:
        return
    df = _load("V4_dual_5s_wind")
    if df is None:
        return
    N = num_drones(df)
    mask = _cruise_mask(df)
    df_c = df[mask].reset_index(drop=True)
    t = df_c["time"].values
    surv = np.zeros((len(df_c), N), dtype=bool)
    for i in range(N):
        surv[:, i] = df_c[f"tension_{i}"].values > SLACK_THRESHOLD
    bins = np.arange(int(T_START), int(T_END), 1.0)
    xs, ys = [], []
    for b in bins:
        m = (t >= b) & (t < b + 1.0)
        if not m.any():
            continue
        slack_frac = float(np.mean(~surv[m])) * 100
        sub = v4[(v4["time"] >= b) & (v4["time"] < b + 1.0)]
        if len(sub) == 0:
            continue
        rms_eps = float(np.sqrt(np.mean(sub["eps"].dropna() ** 2)))
        xs.append(slack_frac)
        ys.append(rms_eps)
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.scatter(xs, ys, color=COLORS[1], s=18)
    ax.set_xlabel("slack duty cycle (per 1-s window) [%]")
    ax.set_ylabel(r"RMS $|\varepsilon_i|$ $[\mathrm{N}]$")
    ax.set_title("V4 reduction residual vs slack duty cycle",
                 fontsize=9)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIG / "fig_reduction_error_vs_slack.png", dpi=180, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print("  wrote fig_reduction_error_vs_slack.png")


def _fig_force_balance():
    r = _force_balance_residual_V4()
    if r is None:
        return
    t, res = r
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(t, res, color=COLORS[2], linewidth=0.8)
    ax.axhline(M_L * G * 0.05, color="red", linestyle="--",
               linewidth=0.9,
               label=r"$5\%$ of $m_L g \approx 4.9$ N")
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"$\|\sum_i T_i \hat{\mathbf{d}}_i "
                  r"- m_L (\mathbf{a}_{\mathrm{ref}} + g\hat z)\|$ "
                  r"$[\mathrm{N}]$")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("V4 payload force-balance residual (reduction "
                 "theorem leading-order check)", fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG / "fig_force_balance_residual_V4.png", dpi=180)
    plt.close(fig)
    print("  wrote fig_force_balance_residual_V4.png")


if __name__ == "__main__":
    main()
