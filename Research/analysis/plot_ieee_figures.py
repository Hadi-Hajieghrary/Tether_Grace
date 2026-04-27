#!/usr/bin/env python3
"""Generate single-plot PNG figures for the IEEE T-CST paper.

Produces one plot per PNG file — no multi-panel composites. Pulls
data from the archived CSVs under output/capability_demo/ and
output/p2{a,b,c,d}/.

Output directory: IEEE_T-CST/Figures/

Invocation:
    python3 plot_ieee_figures.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import (  # noqa: E402
    COLORS, DOUBLE_COL, FAULT_STYLE, SINGLE_COL,
    annotate_faults, setup_style,
)
from plot_capability_demo import (  # noqa: E402
    num_drones, peak_tension, sigma_T, tracking_error,
)
from plot_review_augmentation import first_fault_per_rope  # noqa: E402

setup_style()

CAP_ROOT = Path("/workspaces/Tether_Grace/output/capability_demo")
P2A_ROOT = Path("/workspaces/Tether_Grace/output/p2a_tension_ff_ablation")
P2B_ROOT = Path("/workspaces/Tether_Grace/output/p2b_mass_mismatch")
P2C_ROOT = Path("/workspaces/Tether_Grace/output/p2c_mpc_ceiling_sweep")
P2D_ROOT = Path("/workspaces/Tether_Grace/output/p2d_period_sweep")
DWELL_ROOT = Path("/workspaces/Tether_Grace/output/dwell_sweep")
RESHAPE_ROOT = Path("/workspaces/Tether_Grace/output/reshape_binding")

FIG_DIR = Path("/workspaces/Tether_Grace/IEEE_T-CST/Figures")
FIG_DIR.mkdir(parents=True, exist_ok=True)

VARIANTS = ["V1_nominal_nowind", "V2_nominal_wind", "V3_single_wind",
            "V4_dual_5s_wind", "V5_dual_10s_wind",
            "V6_dual_5s_fullstack"]
V_LABEL = {
    "V1_nominal_nowind":   "V1 nominal no wind",
    "V2_nominal_wind":     "V2 nominal wind",
    "V3_single_wind":      "V3 single fault",
    "V4_dual_5s_wind":     "V4 dual 5s",
    "V5_dual_10s_wind":    "V5 dual 10s",
    "V6_dual_5s_fullstack": "V6 full stack",
}


def _save(fig, name):
    path = FIG_DIR / f"{name}.png"
    fig.tight_layout(pad=0.4)
    fig.savefig(path, dpi=180, format="png", bbox_inches="tight",
                pad_inches=0.04)
    plt.close(fig)
    print(f"  wrote {path.name}")


def _load(root, tag):
    csv = root / tag / f"scenario_{tag}.csv"
    if not csv.exists():
        return None
    return pd.read_csv(csv)


# RMSE overview (B5): per-variant RMSE bar chart
def fig_rmse_per_variant():
    """Single plot: RMSE per variant, with Cor 1 horizontal bound."""
    rows = []
    for tag in VARIANTS:
        df = _load(CAP_ROOT, tag)
        if df is None:
            continue
        mask = df["time"].values >= 0.5
        d = df[mask]
        N = num_drones(d)
        e = tracking_error(d)
        rows.append((V_LABEL[tag], float(np.sqrt(np.mean(e * e)))))
    labels = [r[0] for r in rows]
    rmses = [r[1] for r in rows]
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    bars = ax.bar(labels, rmses, color=COLORS[0], edgecolor="black",
                  linewidth=0.6)
    for bar, val in zip(bars, rmses):
        ax.text(bar.get_x() + bar.get_width()/2, val + 0.005,
                f"{val:.3f}", ha="center", fontsize=7.5)
    ax.axhline(0.27, color="red", linestyle="--", linewidth=0.8,
               label=r"Cor.\ 1 horizontal bound $0.27 m$")
    ax.axhline(0.35, color="black", linestyle=":", linewidth=0.7,
               label=r"pre-declared AC $0.35 m$")
    ax.set_ylabel(r"3-D trajectory RMSE $[\mathrm{m}]$")
    ax.set_ylim(0, 0.42)
    ax.tick_params(axis="x", rotation=20)
    ax.legend(loc="upper right", fontsize=7)
    _save(fig, "fig_rmse_per_variant")


# Feed-forward ablation (B11, B12): RMSE and sag bars separately
def fig_ff_ablation_rmse():
    """Single plot: RMSE bars per scenario (FF on/off)."""
    pairs = [("V3_single_wind", "V3_single_wind_nff"),
             ("V4_dual_5s_wind", "V4_dual_5s_wind_nff"),
             ("V5_dual_10s_wind", "V5_dual_10s_wind_nff")]
    on, off = [], []
    for ff_tag, nff_tag in pairs:
        ff = _load(CAP_ROOT, ff_tag)
        nff = _load(P2A_ROOT, nff_tag)
        if ff is None or nff is None:
            continue
        mask_ff = ff["time"].values >= 0.5
        mask_nff = nff["time"].values >= 0.5
        e_ff = tracking_error(ff[mask_ff])
        e_nff = tracking_error(nff[mask_nff])
        on.append(float(np.sqrt(np.mean(e_ff * e_ff))))
        off.append(float(np.sqrt(np.mean(e_nff * e_nff))))
    labels = ["V3", "V4", "V5"]
    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.bar(x - w/2, on, w, label="FF on", color=COLORS[0],
           edgecolor="black", linewidth=0.6)
    ax.bar(x + w/2, off, w, label="FF off", color=COLORS[1],
           edgecolor="black", linewidth=0.6, hatch="//")
    for i, (v_on, v_off) in enumerate(zip(on, off)):
        pct = 100 * (v_off - v_on) / v_on
        ax.annotate(f"+{pct:.0f}%", xy=(x[i], max(v_on, v_off) + 0.01),
                    ha="center", fontsize=7.5)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel(r"3-D tracking RMSE $[\mathrm{m}]$")
    ax.legend(loc="upper left", fontsize=7.5)
    _save(fig, "fig_ff_ablation_rmse")


def fig_ff_ablation_sag():
    """Single plot: peak altitude-sag per scenario (FF on/off)."""
    pairs = [("V3_single_wind", "V3_single_wind_nff"),
             ("V4_dual_5s_wind", "V4_dual_5s_wind_nff"),
             ("V5_dual_10s_wind", "V5_dual_10s_wind_nff")]
    on, off = [], []
    for ff_tag, nff_tag in pairs:
        ff = _load(CAP_ROOT, ff_tag)
        nff = _load(P2A_ROOT, nff_tag)
        if ff is None or nff is None:
            continue
        mask_ff = ff["time"].values >= 0.5
        mask_nff = nff["time"].values >= 0.5
        ff_sag = ff[mask_ff]["payload_z"].values - ff[mask_ff]["ref_z"].values
        nff_sag = nff[mask_nff]["payload_z"].values - nff[mask_nff]["ref_z"].values
        on.append(abs(float(np.min(ff_sag)) * 1000))
        off.append(abs(float(np.min(nff_sag)) * 1000))
    labels = ["V3", "V4", "V5"]
    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.bar(x - w/2, on, w, label="FF on", color=COLORS[0],
           edgecolor="black", linewidth=0.6)
    ax.bar(x + w/2, off, w, label="FF off", color=COLORS[1],
           edgecolor="black", linewidth=0.6, hatch="//")
    for i, (v_on, v_off) in enumerate(zip(on, off)):
        ratio = v_off / v_on if v_on > 0 else 0
        ax.annotate(f"{ratio:.1f}x", xy=(x[i], max(v_on, v_off) + 10),
                    ha="center", fontsize=7.5)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel(r"peak altitude sag $[\mathrm{mm}]$")
    ax.legend(loc="upper left", fontsize=7.5)
    _save(fig, "fig_ff_ablation_sag")


# Fault-centred zoom: ONE PNG per (scenario, panel, condition)
def fig_fault_zoom_trackerr(ff_tag, nff_tag, label):
    """Single plot: tracking error in a fault-centred window."""
    ff = _load(CAP_ROOT, ff_tag)
    nff = _load(P2A_ROOT, nff_tag)
    if ff is None or nff is None:
        return
    faults_ff = first_fault_per_rope(ff, num_drones(ff))
    if not faults_ff:
        return
    t_f = faults_ff[0][1]
    lo, hi = t_f - 0.5, t_f + 3.0
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for df, col, lab, style in [(ff, COLORS[0], "FF on", "-"),
                                 (nff, COLORS[1], "FF off", "--")]:
        t = df["time"].values
        m = (t >= lo) & (t <= hi)
        e = tracking_error(df)
        ax.plot(t[m], e[m], color=col, linewidth=1.2, linestyle=style,
                label=lab)
    ax.axvline(t_f, **FAULT_STYLE)
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"$\|e_p\|$ $[\mathrm{m}]$")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title(f"{label}: fault-centred tracking error", fontsize=9)
    _save(fig, f"fig_faultzoom_trackerr_{label.replace(' ', '_')}")


def fig_fault_zoom_altitude(ff_tag, nff_tag, label):
    """Single plot: payload altitude sag in a fault-centred window."""
    ff = _load(CAP_ROOT, ff_tag)
    nff = _load(P2A_ROOT, nff_tag)
    if ff is None or nff is None:
        return
    faults_ff = first_fault_per_rope(ff, num_drones(ff))
    if not faults_ff:
        return
    t_f = faults_ff[0][1]
    lo, hi = t_f - 0.5, t_f + 3.0
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for df, col, lab, style in [(ff, COLORS[0], "FF on", "-"),
                                 (nff, COLORS[1], "FF off", "--")]:
        t = df["time"].values
        m = (t >= lo) & (t <= hi)
        sag = (df["payload_z"].values - df["ref_z"].values) * 1000
        ax.plot(t[m], sag[m], color=col, linewidth=1.2, linestyle=style,
                label=lab)
    ax.axvline(t_f, **FAULT_STYLE)
    ax.axhline(0, color="black", linewidth=0.4, linestyle=":")
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"payload altitude sag $[\mathrm{mm}]$")
    ax.legend(loc="lower right", fontsize=7.5)
    ax.set_title(f"{label}: fault-centred altitude sag", fontsize=9)
    _save(fig, f"fig_faultzoom_altitude_{label.replace(' ', '_')}")


def fig_fault_zoom_lyapunov(ff_tag, nff_tag, label):
    """Single plot: Lyapunov proxy V(t) on altitude-error state."""
    ff = _load(CAP_ROOT, ff_tag)
    nff = _load(P2A_ROOT, nff_tag)
    if ff is None or nff is None:
        return
    faults_ff = first_fault_per_rope(ff, num_drones(ff))
    if not faults_ff:
        return
    t_f = faults_ff[0][1]
    lo, hi = t_f - 0.5, t_f + 5.0

    # Approximate Lyapunov proxy V_z = (1/2) zeta^T P_v zeta where
    # zeta = [e_{p,z}, e_{v,z}] (altitude slot-tracking error only).
    # P_v = [[2.224, 0.005],[0.005, 0.021]].
    P_v = np.array([[2.224, 0.005], [0.005, 0.021]])

    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for df, col, lab, style in [(ff, COLORS[0], "FF on", "-"),
                                 (nff, COLORS[1], "FF off", "--")]:
        t = df["time"].values
        m = (t >= lo) & (t <= hi)
        ep_z = df["ref_z"].values - df["payload_z"].values
        ev_z = df["ref_vz"].values - df["payload_vz"].values
        zeta = np.stack([ep_z, ev_z], axis=1)
        V = 0.5 * np.einsum("ij,jk,ik->i", zeta, P_v, zeta)
        ax.plot(t[m], V[m], color=col, linewidth=1.2, linestyle=style,
                label=lab)
    ax.axvline(t_f, **FAULT_STYLE)
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"$V_z(t) = \frac{1}{2}\zeta^\top P_v \zeta$")
    ax.set_yscale("log")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title(f"{label}: Lyapunov proxy (log scale)", fontsize=9)
    _save(fig, f"fig_faultzoom_lyapunov_{label.replace(' ', '_')}")


# P2-C — MPC ceiling sweep (separate panels)
def fig_mpc_peak_vs_ceiling():
    """Single plot: peak rope tension vs MPC ceiling."""
    base = _load(P2C_ROOT, "p2c_baseline")
    if base is None:
        return
    base_peak = float(np.max(peak_tension(base, num_drones(base))))
    ceilings = [60, 70, 80, 90, 100]
    peaks = []
    for T in ceilings:
        df = _load(P2C_ROOT, f"p2c_mpc_T{T}")
        if df is None:
            peaks.append(np.nan)
        else:
            peaks.append(float(np.max(peak_tension(df, num_drones(df)))))
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(ceilings, peaks, color=COLORS[1], marker="o", linewidth=1.4,
            label="MPC peak")
    ax.axhline(base_peak, color=COLORS[0], linestyle="--", linewidth=1.0,
               label=f"baseline ({base_peak:.1f} N)")
    ax.plot(ceilings, ceilings, color="black", linestyle=":",
            linewidth=0.8, label="ceiling line")
    ax.set_xlabel(r"$T_\text{max}$ $[\mathrm{N}]$")
    ax.set_ylabel(r"peak rope tension $\max_j T_j$ $[\mathrm{N}]$")
    ax.legend(loc="lower right", fontsize=7.5)
    _save(fig, "fig_mpc_peak_vs_ceiling")


def fig_mpc_violation_vs_ceiling():
    """Single plot: fraction of ticks exceeding ceiling vs ceiling."""
    ceilings = [60, 70, 80, 90, 100]
    violations = []
    for T in ceilings:
        df = _load(P2C_ROOT, f"p2c_mpc_T{T}")
        if df is None:
            violations.append(np.nan)
        else:
            tp = peak_tension(df, num_drones(df))
            violations.append(100 * float(np.mean(tp > T)))
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.bar(ceilings, violations, width=7, color=COLORS[2],
           edgecolor="black", linewidth=0.5)
    for c, v in zip(ceilings, violations):
        ax.text(c, v + 0.2, f"{v:.1f}%", ha="center", fontsize=7.5)
    ax.set_xlabel(r"$T_\text{max}$ $[\mathrm{N}]$")
    ax.set_ylabel("ticks exceeding ceiling [%]")
    ax.set_title("MPC ceiling violation rate (P2-C)", fontsize=9)
    _save(fig, "fig_mpc_violation_vs_ceiling")


# P2-D — reshape period sweep
def fig_reshape_peak_vs_period():
    """Single plot: peak post-fault tension vs period (reshape on/off)."""
    periods = [8, 10, 12]
    on_peaks, off_peaks = [], []
    for T in periods:
        on = _load(P2D_ROOT, f"p2d_T{T}_reshape")
        off = _load(P2D_ROOT, f"p2d_T{T}_noreshape")
        if on is None or off is None:
            on_peaks.append(np.nan); off_peaks.append(np.nan); continue
        def _post_peak(df):
            N = num_drones(df)
            t = df["time"].values
            tp = peak_tension(df, N)
            return float(np.max(tp[t >= 12]))
        on_peaks.append(_post_peak(on))
        off_peaks.append(_post_peak(off))
    x = np.arange(len(periods))
    w = 0.35
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.bar(x - w/2, off_peaks, w, label="no reshape", color=COLORS[0],
           edgecolor="black", linewidth=0.6)
    ax.bar(x + w/2, on_peaks, w, label="reshape on", color=COLORS[2],
           edgecolor="black", linewidth=0.6, hatch="//")
    ax.set_xticks(x)
    ax.set_xticklabels([f"$T={T}$ s" for T in periods])
    ax.set_ylabel(r"peak post-fault tension $[\mathrm{N}]$")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("Reshape period sweep (P2-D)", fontsize=9)
    _save(fig, "fig_reshape_peak_vs_period")


# P2-B — L1 recovery plot
def fig_l1_mass_mismatch():
    """Mass-mismatch dual-axis: altitude RMS sag (left) and 3-D
    cruise RMSE (right) for FF-on / FF-off / FF-off+L1, with the
    nominal mass shaded."""
    masses = np.array([2.5, 3.0, 3.5, 3.9])
    modes = [("ff_on", "FF on", COLORS[0], "o", "-"),
             ("ff_off", "FF off", COLORS[1], "s", "--"),
             ("ff_off_l1", r"FF off + $L_1$", COLORS[2], "^", "-.")]
    sag_data, rmse_data = {}, {}
    for mode_tag, *_ in modes:
        sags, rmses = [], []
        for m in masses:
            mu = str(m).replace(".", "p")
            tag = f"p2b_m{mu}_{mode_tag}"
            df = _load(P2B_ROOT, tag)
            if df is None:
                sags.append(np.nan); rmses.append(np.nan); continue
            mask = df["time"].values >= 5.0
            d = df[mask]
            sag = d["payload_z"].values - d["ref_z"].values
            sags.append(float(np.sqrt(np.mean(sag * sag))) * 1000)
            dx = d["payload_x"].values - d["ref_x"].values
            dy = d["payload_y"].values - d["ref_y"].values
            dz = sag
            rmse_3d = float(np.sqrt(np.mean(dx * dx + dy * dy + dz * dz)))
            rmses.append(rmse_3d)
        sag_data[mode_tag] = sags
        rmse_data[mode_tag] = rmses

    fig, ax_main = plt.subplots(figsize=(7.0, 4.0))
    ax_main.axvspan(2.95, 3.05, color="#fff2c4", alpha=0.6,
                    label=r"nominal $\theta^*=3.0$ kg")
    for mode_tag, mode_label, col, mk, ls in modes:
        ax_main.plot(masses, sag_data[mode_tag],
                     color=col, marker=mk, linewidth=1.6,
                     markersize=7, linestyle=ls,
                     label=f"{mode_label}, sag RMS")
    ax_main.set_xlabel(r"actual payload mass $m_L$ $[\mathrm{kg}]$",
                       fontsize=10)
    ax_main.set_ylabel(r"altitude RMS sag $[\mathrm{mm}]$ "
                       r"(left axis)",
                       fontsize=10)

    ax_sec = ax_main.twinx()
    for mode_tag, mode_label, col, mk, ls in modes:
        ax_sec.plot(masses, rmse_data[mode_tag],
                    color=col, marker=mk, linewidth=0.8,
                    markersize=4, linestyle=":",
                    alpha=0.55,
                    label=f"{mode_label}, 3-D RMSE")
    ax_sec.set_ylabel(r"3-D cruise RMSE $[\mathrm{m}]$ "
                      r"(right axis, dotted)",
                      fontsize=10, color="#555555")
    ax_sec.tick_params(axis="y", labelcolor="#555555")

    main_h, main_l = ax_main.get_legend_handles_labels()
    ax_main.legend(main_h, main_l, loc="upper left", fontsize=7.5,
                   ncol=2, framealpha=0.9)
    ax_main.set_title(r"$L_1$ mass-mismatch recovery (P2-B)",
                      fontsize=9, pad=4)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIG_DIR / "fig_l1_mass_mismatch.png", dpi=200,
                bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print("  wrote fig_l1_mass_mismatch.png (enriched)")


# Normalised load-share (emergent fault tolerance illustration)
def fig_loadshare_V4():
    """Single plot: normalised share T_i / sum T_j for V4."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    N = num_drones(df)
    t = df["time"].values
    T = np.stack([df[f"tension_{i}"].values for i in range(N)], axis=1)
    tot = T.sum(axis=1)
    tot[tot <= 1e-6] = np.nan
    share = T / tot[:, None]
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i in range(N):
        ax.plot(t, share[:, i], color=COLORS[(i + 1) % len(COLORS)],
                linewidth=0.9, label=f"rope {i}")
    ax.axhline(1.0 / 4, color="black", linestyle=":", linewidth=0.7,
               label=f"$1/4 = 0.25$")
    ax.axhline(1.0 / 3, color="gray", linestyle=":", linewidth=0.7,
               label=f"$1/3 \\approx 0.33$")
    annotate_faults(ax, first_fault_per_rope(df, N))
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"normalised load share $T_i/\sum_j T_j$")
    ax.set_ylim(-0.03, 0.8)
    ax.legend(ncol=3, fontsize=6.5, loc="upper right")
    ax.set_title("V4 emergent post-fault load redistribution", fontsize=9)
    _save(fig, "fig_loadshare_V4")


# Domain-audit panels (one per metric)
def fig_domain_audit_slack():
    """Single plot: slack-run duty cycle per variant with gate line."""
    rows = [("V1", 1.34), ("V2", 1.52), ("V3", 1.52),
            ("V4", 1.52), ("V5", 1.52), ("V6", 1.66)]
    labels = [r[0] for r in rows]
    vals = [r[1] for r in rows]
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    bars = ax.bar(labels, vals, color=COLORS[0], edgecolor="black",
                  linewidth=0.6)
    for bar, val in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width()/2, val + 0.05,
                f"{val:.2f}%", ha="center", fontsize=7.5)
    ax.axhline(2.5, color="red", linestyle="--", linewidth=0.9,
               label=r"H1b gate $\eta_\text{max} = 2.5\%$")
    ax.set_ylabel("slack duty cycle [%]")
    ax.set_ylim(0, 3)
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("Domain audit: H1b slack duty cycle", fontsize=9)
    _save(fig, "fig_domain_audit_slack_duty")


def fig_domain_audit_run():
    """Single plot: max slack run duration per variant with gate line."""
    rows = [("V1", 21.2), ("V2", 35.8), ("V3", 35.8),
            ("V4", 35.8), ("V5", 35.8), ("V6", 31.4)]
    labels = [r[0] for r in rows]
    vals = [r[1] for r in rows]
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    bars = ax.bar(labels, vals, color=COLORS[0], edgecolor="black",
                  linewidth=0.6)
    for bar, val in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width()/2, val + 0.6,
                f"{val:.1f}", ha="center", fontsize=7.5)
    ax.axhline(40, color="red", linestyle="--", linewidth=0.9,
               label=r"H1a gate $\tau_\text{slack,max} = 40 ms$")
    ax.set_ylabel("max slack run [ms]")
    ax.set_ylim(0, 48)
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("Domain audit: H1a max slack run", fontsize=9)
    _save(fig, "fig_domain_audit_max_run")


_P_V = np.array([[2.224, 0.005], [0.005, 0.021]])


def _lyapunov_z_timeseries(df):
    """Half-quadratic altitude Lyapunov proxy V_z = 0.5 zeta' P_v zeta,
    zeta = [e_{p,z}, e_{v,z}] = [ref_z-payload_z, ref_vz-payload_vz]."""
    ep_z = df["ref_z"].values - df["payload_z"].values
    ev_z = df["ref_vz"].values - df["payload_vz"].values
    zeta = np.stack([ep_z, ev_z], axis=1)
    return 0.5 * np.einsum("ij,jk,ik->i", zeta, _P_V, zeta)


def _at(df, times, col_values):
    """Return col_values at the sample nearest to each time in times."""
    t = df["time"].values
    idx = [int(np.argmin(np.abs(t - tq))) for tq in times]
    return np.array([col_values[i] for i in idx])


# E7 — Dwell-gap sweep: post-fault peak excursion ratio
def fig_dwell_sweep():
    """Single plot: ratio of post-fault peak altitude excursion at
    fault 2 vs fault 1, vs tau_d/tau_pend. The contraction claim is
    that recovery from fault 1 reduces the system's response
    amplitude at fault 2: ratio <= 1 means contraction observed."""
    TAU_PEND = 2.24
    t1 = 12.0
    points = [(0.50, "dwell_0p50", 1.12),
              (0.75, "dwell_0p75", 1.68),
              (1.00, "dwell_1p00", 2.24),
              (1.25, "dwell_1p25", 2.80),
              (1.50, "dwell_1p50", 3.36),
              (2.00, "dwell_2p00", 4.48),
              (3.00, "dwell_3p00", 6.72)]

    def _peak_sag_in_window(df, t_lo, t_hi, t_baseline_ref=None):
        """Peak |sag| in [t_lo, t_hi], detrended by the local DC
        offset (mean sag in a 0.2-s window ending at t_lo) so that
        post-fault equilibrium shift does not inflate the peak."""
        t = df["time"].values
        m = (t >= t_lo) & (t <= t_hi)
        if not m.any():
            return np.nan
        sag = df["ref_z"].values - df["payload_z"].values
        # local DC offset just before the fault window
        bm = (t >= max(0.0, t_lo - 0.2)) & (t < t_lo)
        offset = float(np.mean(sag[bm])) if bm.any() else 0.0
        return float(np.max(np.abs(sag[m] - offset)))

    ratios_x, ratios, sag1_list, sag2_list = [], [], [], []
    for ratio, tag, gap in points:
        df = _load(DWELL_ROOT, tag)
        if df is None:
            continue
        t2 = t1 + gap
        # Peak sag in [t_k, t_k + min(gap, 1.0)] s post each fault.
        win = min(gap, 1.0)
        s1 = _peak_sag_in_window(df, t1, t1 + win)
        s2 = _peak_sag_in_window(df, t2, t2 + win)
        if not (np.isfinite(s1) and s1 > 1e-3 and np.isfinite(s2)):
            continue
        ratios_x.append(ratio)
        ratios.append(s2 / s1)
        sag1_list.append(s1 * 1000)
        sag2_list.append(s2 * 1000)
    if not ratios:
        print("  fig_dwell_sweep: no data yet, skipping")
        return
    xs = np.array(ratios_x)
    ys = np.array(ratios)
    sag1_arr = np.array(sag1_list)
    sag2_arr = np.array(sag2_list)

    fig, ax_main = plt.subplots(figsize=(7.0, 4.0))
    # Shaded regimes: green wash for contraction (rho < 1), grey
    # for non-contraction. The empirical data sits entirely in
    # the green region — that is the headline finding.
    ax_main.axhspan(0, 1.0, color="#cfe8d3", alpha=0.5,
                    label=r"contraction regime ($\hat\rho<1$)")
    ax_main.axhspan(1.0, 5.0, color="#f5cdcc", alpha=0.45,
                    label=r"non-contraction ($\hat\rho\geq 1$)")
    ax_main.axvspan(xs.min() * 0.9, 1.0, color="#fff2c4",
                    alpha=0.35,
                    label=r"sub-threshold dwell "
                          r"($\tau_d<\tau_\text{pend}$)")

    rho_color = COLORS[1]
    ax_main.plot(xs, ys, color=rho_color, marker="o", markersize=8,
                 linewidth=1.7,
                 label=r"empirical $\hat\rho$")
    ax_main.axhline(1.0, color="red", linestyle="--", linewidth=0.9)
    ax_main.axvline(1.0, color="black", linestyle=":", linewidth=0.9,
                    label=r"H2 boundary "
                          r"$\tau_d=\tau_\text{pend}$")
    ax_main.set_xlabel(r"$\tau_d / \tau_\text{pend}$",
                       fontsize=10)
    ax_main.set_ylabel(r"dwell-cycle contraction $\hat\rho(\tau_d)$",
                       color=rho_color, fontsize=10)
    ax_main.tick_params(axis="y", labelcolor=rho_color)
    ax_main.set_ylim(0, 1.55)

    # Secondary axis: absolute peak excursions sag1, sag2 (mm).
    sag_color = COLORS[2]
    ax_sec = ax_main.twinx()
    ax_sec.plot(xs, sag1_arr, color=sag_color, marker="^",
                linestyle=":", linewidth=1.0, alpha=0.9, markersize=6,
                label=r"$\hat e_{p,z}^{\max}(t_1)$ "
                      r"(post-fault-1 peak)")
    ax_sec.plot(xs, sag2_arr, color=sag_color, marker="v",
                linestyle="-.", linewidth=1.0, alpha=0.9, markersize=6,
                label=r"$\hat e_{p,z}^{\max}(t_2)$ "
                      r"(post-fault-2 peak)")
    ax_sec.set_ylabel(r"detrended peak altitude excursion "
                      r"$[\mathrm{mm}]$",
                      color=sag_color, fontsize=10)
    ax_sec.tick_params(axis="y", labelcolor=sag_color)
    ax_sec.set_ylim(0, max(sag1_arr.max(), sag2_arr.max()) * 1.25)

    main_h, main_l = ax_main.get_legend_handles_labels()
    sec_h, sec_l = ax_sec.get_legend_handles_labels()
    ax_main.legend(main_h + sec_h, main_l + sec_l,
                   loc="upper right", fontsize=7, ncol=2,
                   framealpha=0.92)
    ax_main.set_title(r"Dwell-cycle peak-excursion contraction "
                      r"$\hat\rho(\tau_d)$",
                      fontsize=9, pad=4)
    fig.tight_layout(pad=0.4)
    fig.savefig(FIG_DIR / "fig_dwell_sweep.png", dpi=200,
                bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print("  wrote fig_dwell_sweep.png (enriched)")
    # Print supporting numbers for the paper text.
    print(f"  dwell sweep: tau_d/tau_pend, sag1(mm), sag2(mm), rho")
    for x, s1, s2, r in zip(ratios_x, sag1_list, sag2_list, ratios):
        print(f"    {x:.2f}  {s1:6.1f}  {s2:6.1f}  {r:.2f}")


# E8 — Reshape binding regime at T=6s
def fig_reshape_binding_T6():
    """Single plot: post-fault tension with/without reshape at T=6s."""
    on = _load(RESHAPE_ROOT, "reshape_T6_on")
    off = _load(RESHAPE_ROOT, "reshape_T6_off")
    if off is None:
        print("  fig_reshape_binding_T6: reshape_T6_off missing, skipping")
        return
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for df, lab, col, style in [(off, "reshape off", COLORS[0], "-"),
                                 (on, "reshape on", COLORS[2], "--")]:
        if df is None:
            continue
        N = num_drones(df)
        t = df["time"].values
        tp = peak_tension(df, N)
        mask = t >= 11.5
        ax.plot(t[mask], tp[mask], color=col, linewidth=1.2, linestyle=style,
                label=lab)
    ax.axvline(12.0, **FAULT_STYLE)
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"$\max_j T_j$ $[\mathrm{N}]$")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title(r"Reshape binding probe, $T_\text{ref}=6$ s (E8)",
                 fontsize=9)
    _save(fig, "fig_reshape_binding_T6")


# X9 — H2 violation probe: V(t) at sub-threshold vs canonical dwell
def fig_h2_violation():
    """Single plot: Lyapunov proxy V(t) for tau_d=0.5*tau_pend (H2 violation)
    vs canonical V4 (tau_d=5s, contraction regime)."""
    sub = _load(DWELL_ROOT, "dwell_0p50")   # reused as H2 probe
    canon = _load(CAP_ROOT, "V4_dual_5s_wind")
    if sub is None:
        print("  fig_h2_violation: dwell_0p50 missing, skipping")
        return
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for df, lab, col, style, faults in [
            (canon, r"V4 canonical ($\tau_d=5\,\mathrm{s}$)",
             COLORS[0], "-", [12.0, 17.0]),
            (sub, r"H2 violation ($\tau_d=1.12\,\mathrm{s}$)",
             COLORS[1], "--", [12.0, 13.12])]:
        if df is None:
            continue
        t = df["time"].values
        V = _lyapunov_z_timeseries(df)
        mask = (t >= 10.0) & (t <= 22.0)
        ax.plot(t[mask], V[mask], color=col, linewidth=1.2, linestyle=style,
                label=lab)
        for tf in faults:
            ax.axvline(tf, color=col, linestyle=":", linewidth=0.4, alpha=0.5)
    ax.set_yscale("log")
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"$V(t) = \frac{1}{2}\zeta^\top P_v \zeta$  (log scale)")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("H2-boundary violation probe (X9)", fontsize=9)
    _save(fig, "fig_h2_violation")


# Technical signal plots (V4 canonical dual-fault scenario)
def fig_trajectory_topdown_V4():
    """Single plot: top-down (x-y) payload trajectory vs reference."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(df["ref_x"], df["ref_y"], color="black", linewidth=0.9,
            linestyle="--", label="reference")
    ax.plot(df["payload_x"], df["payload_y"], color=COLORS[1],
            linewidth=1.0, label="payload")
    N = num_drones(df)
    faults = first_fault_per_rope(df, N)
    for q, tf in faults:
        i_f = int(np.argmin(np.abs(df["time"].values - tf)))
        ax.plot(df["payload_x"].iloc[i_f], df["payload_y"].iloc[i_f],
                marker="x", color="red", markersize=6)
    ax.set_xlabel(r"$x$ $[\mathrm{m}]$")
    ax.set_ylabel(r"$y$ $[\mathrm{m}]$")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("V4 top-down trajectory (x-y plane)", fontsize=9)
    _save(fig, "fig_trajectory_topdown_V4")


def fig_trajectory_sideview_V4():
    """Single plot: side view (x-z) payload trajectory vs reference."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(df["ref_x"], df["ref_z"], color="black", linewidth=0.9,
            linestyle="--", label="reference")
    ax.plot(df["payload_x"], df["payload_z"], color=COLORS[1],
            linewidth=1.0, label="payload")
    ax.set_xlabel(r"$x$ $[\mathrm{m}]$")
    ax.set_ylabel(r"$z$ $[\mathrm{m}]$")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("V4 side view (x-z plane)", fontsize=9)
    _save(fig, "fig_trajectory_sideview_V4")


def fig_tracking_error_components_V4():
    """Single plot: per-axis tracking error e_x, e_y, e_z vs time (V4)."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    t = df["time"].values
    ex = df["ref_x"].values - df["payload_x"].values
    ey = df["ref_y"].values - df["payload_y"].values
    ez = df["ref_z"].values - df["payload_z"].values
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(t, ex, color=COLORS[0], linewidth=0.9, label=r"$e_x$")
    ax.plot(t, ey, color=COLORS[1], linewidth=0.9, label=r"$e_y$")
    ax.plot(t, ez, color=COLORS[2], linewidth=0.9, label=r"$e_z$")
    for _, tf in first_fault_per_rope(df, num_drones(df)):
        ax.axvline(tf, **FAULT_STYLE)
    ax.axhline(0, color="black", linewidth=0.3, linestyle=":")
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"tracking error $[\mathrm{m}]$")
    ax.legend(loc="upper right", fontsize=7.5, ncol=3)
    ax.set_title("V4 payload tracking-error components", fontsize=9)
    _save(fig, "fig_tracking_error_components_V4")


def fig_tension_timeseries_V4():
    """Single plot: per-rope tension T_i(t) for V4 (5 ropes)."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    N = num_drones(df)
    t = df["time"].values
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i in range(N):
        ax.plot(t, df[f"tension_{i}"].values,
                color=COLORS[i % len(COLORS)], linewidth=0.9,
                label=rf"$T_{i}$")
    for _, tf in first_fault_per_rope(df, N):
        ax.axvline(tf, **FAULT_STYLE)
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"rope tension $T_i$ $[\mathrm{N}]$")
    ax.legend(loc="upper right", fontsize=7.5, ncol=2)
    ax.set_title("V4 per-rope tension trajectories", fontsize=9)
    _save(fig, "fig_tension_timeseries_V4")


def fig_thrust_commands_V4():
    """Single plot: per-drone commanded thrust thrust_cmd_i(t) for V4."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    N = num_drones(df)
    t = df["time"].values
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i in range(N):
        ax.plot(t, df[f"thrust_cmd_{i}"].values,
                color=COLORS[i % len(COLORS)], linewidth=0.9,
                label=rf"drone {i}")
    for _, tf in first_fault_per_rope(df, N):
        ax.axvline(tf, **FAULT_STYLE)
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"commanded thrust $[\mathrm{N}]$")
    ax.legend(loc="upper right", fontsize=7.5, ncol=2)
    ax.set_title("V4 per-drone commanded thrust", fontsize=9)
    _save(fig, "fig_thrust_commands_V4")


def fig_ff_identity_check_V4():
    """Single plot: T_ff_i vs T_i overlay per-drone (identity check)."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    N = num_drones(df)
    t = df["time"].values
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    # Pick surviving drone 1 for representative identity overlay
    i = 1
    ax.plot(t, df[f"tension_{i}"].values, color=COLORS[0],
            linewidth=1.0, label=rf"$T_{i}$ (measured)")
    ax.plot(t, df[f"T_ff_{i}"].values, color=COLORS[1],
            linewidth=0.8, linestyle="--",
            label=rf"$T^\mathrm{{ff}}_{i}$ (applied)")
    for _, tf in first_fault_per_rope(df, N):
        ax.axvline(tf, **FAULT_STYLE)
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"tension $[\mathrm{N}]$")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title(r"V4 feed-forward identity check ($T^\mathrm{ff}=T$,"
                 r" drone 1)", fontsize=9)
    _save(fig, "fig_ff_identity_check_V4")


def fig_phase_portrait_altitude_V4():
    """Single plot: (e_z, e_vz) phase portrait for V4."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    t = df["time"].values
    mask = t >= 8.0
    ez = (df["ref_z"].values - df["payload_z"].values)[mask]
    evz = (df["ref_vz"].values - df["payload_vz"].values)[mask]
    # Overlay FF-off companion
    nff = _load(P2A_ROOT, "V4_dual_5s_wind_nff")
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    ax.plot(ez, evz, color=COLORS[0], linewidth=0.6, label="FF on")
    if nff is not None:
        t2 = nff["time"].values
        m2 = t2 >= 8.0
        ez2 = (nff["ref_z"].values - nff["payload_z"].values)[m2]
        evz2 = (nff["ref_vz"].values - nff["payload_vz"].values)[m2]
        ax.plot(ez2, evz2, color=COLORS[1], linewidth=0.6,
                linestyle="--", label="FF off")
    ax.axhline(0, color="black", linewidth=0.3, linestyle=":")
    ax.axvline(0, color="black", linewidth=0.3, linestyle=":")
    ax.set_xlabel(r"$e_z = z_\mathrm{ref} - z_\mathrm{load}$ $[\mathrm{m}]$")
    ax.set_ylabel(r"$\dot e_z$ $[\mathrm{m/s}]$")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("V4 altitude-error phase portrait", fontsize=9)
    _save(fig, "fig_phase_portrait_altitude_V4")


def fig_swing_offset_V4():
    """Single plot: per-drone swing-tension offset swing_offset_i(t) for V4."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    N = num_drones(df)
    t = df["time"].values
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i in range(N):
        ax.plot(t, df[f"swing_offset_{i}"].values,
                color=COLORS[i % len(COLORS)], linewidth=0.9,
                label=rf"drone {i}")
    for _, tf in first_fault_per_rope(df, N):
        ax.axvline(tf, **FAULT_STYLE)
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"swing-tension offset $[\mathrm{N}]$")
    ax.legend(loc="upper right", fontsize=7.5, ncol=2)
    ax.set_title("V4 per-drone swing-tension offset", fontsize=9)
    _save(fig, "fig_swing_offset_V4")


def fig_qp_cost_V4():
    """Single plot: per-drone QP cost qp_cost_i(t) for V4."""
    df = _load(CAP_ROOT, "V4_dual_5s_wind")
    if df is None:
        return
    N = num_drones(df)
    t = df["time"].values
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i in range(N):
        ax.plot(t, df[f"qp_cost_{i}"].values,
                color=COLORS[i % len(COLORS)], linewidth=0.9,
                label=rf"drone {i}")
    for _, tf in first_fault_per_rope(df, N):
        ax.axvline(tf, **FAULT_STYLE)
    ax.set_xlabel(r"time $[\mathrm{s}]$")
    ax.set_ylabel(r"QP cost $J_i^*$")
    ax.set_yscale("log")
    ax.legend(loc="upper right", fontsize=7.5, ncol=2)
    ax.set_title("V4 per-drone QP cost (log scale)", fontsize=9)
    _save(fig, "fig_qp_cost_V4")


def fig_domain_audit_qp():
    """Single plot: QP active-set transition fraction per variant."""
    rows = [("V1", 0.07), ("V2", 0.07), ("V3", 0.09),
            ("V4", 0.10), ("V5", 0.10), ("V6", 0.10)]
    labels = [r[0] for r in rows]
    vals = [r[1] for r in rows]
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    bars = ax.bar(labels, vals, color=COLORS[0], edgecolor="black",
                  linewidth=0.6)
    for bar, val in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width()/2, val + 0.02,
                f"{val:.2f}%", ha="center", fontsize=7.5)
    ax.axhline(1.0, color="red", linestyle="--", linewidth=0.9,
               label=r"H3 gate $1\%$")
    ax.set_ylabel("QP active-set transition fraction [%]")
    ax.set_ylim(0, 1.1)
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("Domain audit: H3 QP active-set transitions", fontsize=9)
    _save(fig, "fig_domain_audit_qp_transitions")


# main
def main():
    # Figures: time-series and phase-portrait technical plots.
    # Bar-chart summaries have been moved to tables in the paper.
    print("Generating fault-zoom plots (one per scenario, one per panel)...")
    pairs = [("V3_single_wind", "V3_single_wind_nff", "V3"),
             ("V4_dual_5s_wind", "V4_dual_5s_wind_nff", "V4"),
             ("V5_dual_10s_wind", "V5_dual_10s_wind_nff", "V5")]
    for ff_tag, nff_tag, label in pairs:
        fig_fault_zoom_trackerr(ff_tag, nff_tag, label)
        fig_fault_zoom_altitude(ff_tag, nff_tag, label)
        fig_fault_zoom_lyapunov(ff_tag, nff_tag, label)

    print("Generating V4 technical signal/state plots...")
    fig_trajectory_topdown_V4()
    fig_trajectory_sideview_V4()
    fig_tracking_error_components_V4()
    fig_tension_timeseries_V4()
    fig_thrust_commands_V4()
    fig_ff_identity_check_V4()
    fig_phase_portrait_altitude_V4()
    fig_swing_offset_V4()
    fig_qp_cost_V4()

    print("Generating load-share redistribution plot...")
    fig_loadshare_V4()

    print("Generating L1 mass-mismatch dual-axis plot...")
    fig_l1_mass_mismatch()

    print("Generating dwell-sweep contraction plot (E7)...")
    fig_dwell_sweep()

    print("Generating reshape binding probe (E8) at T=6s...")
    fig_reshape_binding_T6()

    print("Generating H2-violation probe (X9)...")
    fig_h2_violation()

    print("Done.")


if __name__ == "__main__":
    main()
