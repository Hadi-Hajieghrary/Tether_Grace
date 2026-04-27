"""
gen_tcst_figures.py
===================
Generate all 9 IEEE T-CST Section VI publication figures as PNG.
Outputs go to: IEEE_T-CST/Figures/
Run from workspace root:
    python3 Research/analysis/gen_tcst_figures.py

Figures produced
----------------
  fig_rmse_overview.png          – V1-V6 RMSE bar + config matrix
  fig_reduction_fidelity.png     – seg_T vs tension time-series + CDF
  fig_fault_zoom.png             – 3-row fault panels (V3/V4/V5)
  fig_paired_delta.png           – paired ablation delta summary
  fig_dwell_sweep.png            – dwell-gap sweep (τ_d / τ_pend)
  fig_ff_ablation.png            – P2-A grouped bar
  fig_robustness_surface.png     – P2-B 2×2 robustness surface
  fig_extensions.png             – MPC ceiling + reshape period sweep
  fig_failure_gallery.png        – H1/H2/H3 boundary violations
"""
from __future__ import annotations
import os, sys, json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Patch
from scipy.optimize import curve_fit

# ── paths ──────────────────────────────────────────────────────────────────────
ROOT   = "/workspaces/Tether_Grace"
OUT    = "/workspaces/Tether_Grace/IEEE_T-CST/Figures"
CAP    = os.path.join(ROOT, "output/capability_demo")
P2A    = os.path.join(ROOT, "output/p2a_tension_ff_ablation")
P2B    = os.path.join(ROOT, "output/p2b_mass_mismatch")
P2C    = os.path.join(ROOT, "output/p2c_mpc_ceiling_sweep")
P2D    = os.path.join(ROOT, "output/p2d_period_sweep")
SWEEP  = os.path.join(ROOT, "output/5drone_baseline_campaign/inter_fault_sweep")
os.makedirs(OUT, exist_ok=True)

sys.path.insert(0, os.path.join(ROOT, "Research/analysis"))
from ieee_style import setup_style, COLORS, FAULT_STYLE, SINGLE_COL, DOUBLE_COL, DOUBLE_COL_TALL, annotate_faults
setup_style()

# ── colour palette for this paper ─────────────────────────────────────────────
C_FFON  = "#0072B2"   # FF-on / baseline  (blue)
C_FFOFF = "#D55E00"   # FF-off            (vermilion)
C_L1    = "#009E73"   # FF-off + L1       (green)
C_REF   = "#000000"   # reference         (black)
C_FAULT = "#CC0000"   # fault tick        (red)
C_LYAP  = "#CC79A7"   # Lyapunov proxy    (purple)

TAU_PEND = 2.24  # s

# ── helpers ────────────────────────────────────────────────────────────────────
def load_cap(variant: str) -> pd.DataFrame:
    csvs = [f for f in os.listdir(os.path.join(CAP, variant)) if f.endswith(".csv")]
    return pd.read_csv(os.path.join(CAP, variant, csvs[0]))

def error_vec(df):
    ex = df["payload_x"] - df["ref_x"]
    ey = df["payload_y"] - df["ref_y"]
    ez = df["payload_z"] - df["ref_z"]
    return np.sqrt(ex**2 + ey**2 + ez**2)

def alt_error(df):
    return df["payload_z"] - df["ref_z"]

def lyapunov_proxy(df):
    """V(t) = e_z^2 * p11 + 2*e_z*dot_ez*p12 + dot_ez^2 * p22
    P_v from Section IV: p11=2.224, p12=0.005, p22=0.021"""
    p11, p12, p22 = 2.224, 0.005, 0.021
    ez   = (df["payload_z"] - df["ref_z"]).values
    dvz  = (df["payload_vz"] - df["ref_vz"]).values
    return p11*ez**2 + 2*p12*ez*dvz + p22*dvz**2

def fit_envelope(t_rel, e_post):
    """Fit Γ(t) = A*exp(-λ*t) + floor to post-fault window."""
    def model(t, A, lam, fl): return A*np.exp(-lam*t) + fl
    try:
        A0 = float(e_post[0]) - float(np.percentile(e_post, 5))
        p0 = [max(A0, 1e-3), 0.5, float(np.percentile(e_post, 5))]
        popt, _ = curve_fit(model, t_rel, e_post, p0=p0,
                            bounds=([0,0,0],[np.inf,20,1]), maxfev=5000)
        return popt
    except Exception:
        return None

def save(name):
    path = os.path.join(OUT, name)
    plt.savefig(path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"  saved → {path}")

# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 1: fig_rmse_overview — V1-V6 RMSE bar + config matrix
# ══════════════════════════════════════════════════════════════════════════════
def fig_rmse_overview():
    print("Generating fig_rmse_overview …")
    variants = ["V1", "V2", "V3", "V4", "V5", "V6"]
    labels   = ["V1\n(no wind)", "V2\n(wind)", "V3\n(fault 1)", "V4\n(fault 1+2\nΔt=5s)",
                "V5\n(fault 1+2\nΔt=10s)", "V6\n(full stack)"]
    rmse_vals = []
    for v in variants:
        tag = {"V1":"V1_nominal_nowind","V2":"V2_nominal_wind",
               "V3":"V3_single_wind","V4":"V4_dual_5s_wind",
               "V5":"V5_dual_10s_wind","V6":"V6_dual_5s_fullstack"}[v]
        df = load_cap(tag)
        mask = df["time"] >= 8
        rmse_vals.append(float(np.sqrt(np.mean(error_vec(df[mask])**2))))

    # Config matrix:  rows = subsystems, cols = variants
    subsystems = ["Baseline QP", "$L_1$ adapt.", "MPC ceiling", "Reshape"]
    config = np.array([
        [1,1,1,1,1,1],   # Baseline QP — always on
        [0,0,0,0,0,1],   # L1
        [0,0,0,0,0,1],   # MPC
        [0,0,0,0,0,1],   # Reshape
    ])

    fig = plt.figure(figsize=(DOUBLE_COL[0], 4.0))
    gs  = gridspec.GridSpec(2, 1, hspace=0.45, height_ratios=[2.5, 1])
    ax_bar = fig.add_subplot(gs[0])
    ax_cfg = fig.add_subplot(gs[1])

    x = np.arange(len(variants))
    bar_colors = [C_FFON if v != "V6" else "#CC79A7" for v in variants]
    bars = ax_bar.bar(x, rmse_vals, color=bar_colors, width=0.6, zorder=3)
    ax_bar.axhline(0.27, color=C_FAULT, linestyle="--", linewidth=1.1,
                   label="Cor.\u202f1 horiz. bound (0.27 m)")
    ax_bar.axhline(0.35, color="gray", linestyle=":", linewidth=0.9,
                   label="Acceptance threshold (0.35 m)")
    for bar, val in zip(bars, rmse_vals):
        ax_bar.text(bar.get_x() + bar.get_width()/2, val + 0.004,
                    f"{val:.3f}", ha="center", va="bottom", fontsize=7)
    ax_bar.set_ylabel("3-D trajectory RMSE (m)")
    ax_bar.set_xticks(x); ax_bar.set_xticklabels(labels, fontsize=7.5)
    ax_bar.set_ylim(0, 0.42)
    ax_bar.legend(fontsize=7, loc="upper left")
    ax_bar.set_title("(a) Trajectory tracking RMSE — V1 through V6", fontsize=8.5)

    # Config matrix heatmap
    ax_cfg.imshow(config, cmap="Blues", aspect="auto", vmin=0, vmax=1.5)
    ax_cfg.set_xticks(x); ax_cfg.set_xticklabels(["V1","V2","V3","V4","V5","V6"], fontsize=7.5)
    ax_cfg.set_yticks(range(4)); ax_cfg.set_yticklabels(subsystems, fontsize=7)
    ax_cfg.set_title("(b) Active subsystems per variant", fontsize=8.5)
    ax_cfg.tick_params(left=False, bottom=False)
    ax_cfg.spines[:].set_visible(False)
    ax_cfg.grid(False)

    save("fig_rmse_overview.png")

# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 2: fig_reduction_fidelity — seg_T vs tension_i + CDF
# ══════════════════════════════════════════════════════════════════════════════
def fig_reduction_fidelity():
    print("Generating fig_reduction_fidelity …")
    # Use V4 for time-series (hardest scenario), aggregate all V1-V6 for CDF
    cap_map = {
        "V1":"V1_nominal_nowind","V2":"V2_nominal_wind","V3":"V3_single_wind",
        "V4":"V4_dual_5s_wind","V5":"V5_dual_10s_wind","V6":"V6_dual_5s_fullstack"
    }
    fault_times_v4 = [12.0, 17.0]

    df4 = load_cap("V4_dual_5s_wind")
    # seg_T_{i}_{seg} last segment = tip = actual Kelvin-Voigt tension
    # tension_i = T_ff_i at controller tick = lumped model tension
    # Use the last bead segment (index 8) as "truth"
    N_ropes = 5
    t = df4["time"].values

    fig, axes = plt.subplots(1, 2, figsize=(DOUBLE_COL[0], 2.6))

    # --- Left panel: time series V4, all surviving ropes ---
    ax = axes[0]
    surviving = [0,1,2,3,4]  # all start alive; 0 severs @12, 2 @17
    rope_colors = COLORS[:5]
    for i in surviving:
        T_kv  = df4[f"seg_T_{i}_8"].values   # tip segment = KV truth
        T_lum = df4[f"T_ff_{i}"].values       # feedforward = lumped model
        # after fault, tension → 0; skip severed
        alive_mask = T_kv > 0.1
        t_show = t[alive_mask]
        ax.plot(t_show, T_kv[alive_mask],  color=rope_colors[i], lw=0.9, alpha=0.85)
        ax.plot(t_show, T_lum[alive_mask], color=rope_colors[i], lw=0.9,
                linestyle="--", alpha=0.55)
    # Fault ticks
    for tf in fault_times_v4:
        ax.axvline(tf, **FAULT_STYLE)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Rope tension (N)")
    ax.set_xlim(8, 28); ax.set_ylim(-2, 50)
    ax.set_title("(a) V4: $T_i^{\\mathrm{KV}}$ (solid) vs $T_i^{\\mathrm{lumped}}$ (dash)", fontsize=8)

    # Custom legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0],[0], color="gray", lw=0.9, label="$T^{\\mathrm{KV}}$ (KV truth)"),
        Line2D([0],[0], color="gray", lw=0.9, ls="--", alpha=0.55,
               label="$T^{\\mathrm{lumped}}$ (controller FF)"),
        Line2D([0],[0], color=C_FAULT, ls=":", lw=1.0, label="Fault event"),
    ]
    ax.legend(handles=legend_elements, fontsize=6.5)

    # --- Right panel: empirical CDF of |T_KV - T_lumped| ---
    ax2 = axes[1]
    all_errors = []
    for vtag in cap_map.values():
        df = load_cap(vtag)
        for i in range(5):
            T_kv  = df[f"seg_T_{i}_8"].values
            T_lum = df[f"T_ff_{i}"].values
            alive = T_kv > 0.1
            eps   = np.abs(T_kv[alive] - T_lum[alive])
            all_errors.append(eps)
    eps_all = np.concatenate(all_errors)
    eps_sorted = np.sort(eps_all)
    cdf = np.arange(1, len(eps_sorted)+1) / len(eps_sorted)

    # Analytical bound approximation: O(δ + η_max)
    # δ ≈ 0.003 (from domain audit), η_max ≈ 0.1 kg (bead mass), g=9.81 → ~1 N
    analytical_bound = 2.0  # conservative 2 N

    ax2.plot(eps_sorted, cdf, color=C_FFON, lw=1.2)
    ax2.axvline(analytical_bound, color=C_FAULT, ls="--", lw=1.0,
                label=f"Analytical bound $O(\\delta+\\eta_{{\\max}})$ ≈ {analytical_bound} N")
    ax2.set_xlabel(r"$|\,T_i^{\mathrm{KV}} - T_i^{\mathrm{lumped}}\,|$ (N)")
    ax2.set_ylabel("Empirical CDF")
    ax2.set_xlim(0, max(analytical_bound * 2.5, float(np.percentile(eps_sorted, 99.5))))
    ax2.set_ylim(0, 1.02)
    ax2.set_title("(b) CDF of reduction error (all variants)", fontsize=8)
    ax2.legend(fontsize=6.5)

    plt.tight_layout()
    save("fig_reduction_fidelity.png")

# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 3: fig_fault_zoom — 3-row panels (V3/V4/V5)
# ══════════════════════════════════════════════════════════════════════════════
def fig_fault_zoom():
    print("Generating fig_fault_zoom …")
    cases = [
        ("V3_single_wind",    [(12.0, 0)],         "V3 — single fault"),
        ("V4_dual_5s_wind",   [(12.0, 0),(17.0,2)], "V4 — dual fault Δt=5s"),
        ("V5_dual_10s_wind",  [(12.0, 0),(22.0,2)], "V5 — dual fault Δt=10s"),
    ]
    nff_map = {
        "V3_single_wind":   "V3_single_wind_nff",
        "V4_dual_5s_wind":  "V4_dual_5s_wind_nff",
        "V5_dual_10s_wind": "V5_dual_10s_wind_nff",
    }
    W = 5.0  # seconds either side of first fault

    fig, axes = plt.subplots(3, 3, figsize=(DOUBLE_COL[0], 5.8),
                             sharex="col", sharey="row")

    for col, (tag, faults, title) in enumerate(cases):
        df_on  = load_cap(tag)
        # Load nff
        nff_tag = nff_map[tag]
        nff_csvs = [f for f in os.listdir(os.path.join(P2A, nff_tag)) if f.endswith(".csv")]
        df_off = pd.read_csv(os.path.join(P2A, nff_tag, nff_csvs[0]))

        t_f1 = faults[0][0]
        t0, t1 = t_f1 - W, t_f1 + W * 2.5
        for df_use, df_off_use in [(df_on, df_off)]:
            pass  # just for scoping

        mask_on  = (df_on["time"] >= t0)  & (df_on["time"] <= t1)
        mask_off = (df_off["time"] >= t0) & (df_off["time"] <= t1)

        t_on  = df_on.loc[mask_on,  "time"].values
        t_off = df_off.loc[mask_off, "time"].values

        # ── Row 0: altitude error ──────────────────────────────────────────
        ax = axes[0, col]
        ez_on  = alt_error(df_on[mask_on]).values
        ez_off = alt_error(df_off[mask_off]).values
        ax.plot(t_on,  ez_on,  color=C_FFON,  lw=1.1, label="FF-on")
        ax.plot(t_off, ez_off, color=C_FFOFF, lw=1.0, ls="--", label="FF-off")

        # Fit exponential envelope to FF-on post-fault window
        post_mask = (t_on >= t_f1) & (t_on <= t_f1 + W*2)
        t_post = t_on[post_mask] - t_f1
        e_post = np.abs(ez_on[post_mask])
        if len(t_post) > 20:
            popt = fit_envelope(t_post, e_post)
            if popt is not None:
                A, lam, fl = popt
                t_env = np.linspace(0, W*2, 300)
                env   = A * np.exp(-lam * t_env) + fl
                ax.fill_between(t_env + t_f1, -env, env, alpha=0.15, color=C_FFON)
                if col == 0:
                    ax.annotate(f"$\\hat{{\\lambda}}$={lam:.2f} s$^{{-1}}$",
                                xy=(t_f1 + 1.0, A*0.4 + fl),
                                fontsize=6.5, color=C_FFON)

        for tf, _ in faults:
            ax.axvline(tf, **FAULT_STYLE)
        if col == 0:
            ax.set_ylabel("Alt. error $e_z$ (m)", fontsize=7.5)
            ax.legend(fontsize=6.5, loc="lower right")
        ax.set_title(title, fontsize=8)
        ax.set_ylim(-0.6, 0.6)

        # ── Row 1: commanded thrust per drone ─────────────────────────────
        ax = axes[1, col]
        for i in range(5):
            thrust = df_on.loc[mask_on, f"thrust_cmd_{i}"].values
            alive = df_on.loc[mask_on, f"tension_{i}"].values > 0.1
            t_alive = t_on[alive]
            th_alive = thrust[alive]
            ax.plot(t_alive, th_alive, color=COLORS[i], lw=0.8, alpha=0.8)
        for tf, _ in faults:
            ax.axvline(tf, **FAULT_STYLE)
        if col == 0:
            ax.set_ylabel("Thrust cmd (N)", fontsize=7.5)
        ax.set_ylim(0, 40)

        # ── Row 2: Lyapunov proxy V(t) ────────────────────────────────────
        ax = axes[2, col]
        Vt_on  = lyapunov_proxy(df_on[mask_on])
        Vt_off = lyapunov_proxy(df_off[mask_off])
        ax.semilogy(t_on,  Vt_on,  color=C_FFON,  lw=1.1, label="FF-on")
        ax.semilogy(t_off, Vt_off, color=C_FFOFF, lw=1.0, ls="--", label="FF-off")
        # Horizontal reference at pre-fault mean
        pre_mask = (t_on < t_f1) & (t_on >= t_f1 - 2.0)
        V_pre = float(np.mean(Vt_on[t_on < t_f1][t_on[t_on < t_f1] >= t_f1 - 2.0])) if pre_mask.any() else None
        if V_pre and V_pre > 0:
            ax.axhline(V_pre, color="gray", ls=":", lw=0.7, alpha=0.6)
        for tf, _ in faults:
            ax.axvline(tf, **FAULT_STYLE)
        if col == 0:
            ax.set_ylabel("$V(t)=\\xi^\\top P_v \\xi$", fontsize=7.5)
        ax.set_xlabel("Time (s)", fontsize=7.5)

    plt.tight_layout()
    save("fig_fault_zoom.png")

# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 4: fig_paired_delta — FF ablation paired-delta summary (single-seed)
# ══════════════════════════════════════════════════════════════════════════════
def fig_paired_delta():
    print("Generating fig_paired_delta …")
    cases = [
        ("V3_single_wind",   "V3_single_wind_nff",   "V3"),
        ("V4_dual_5s_wind",  "V4_dual_5s_wind_nff",  "V4"),
        ("V5_dual_10s_wind", "V5_dual_10s_wind_nff", "V5"),
    ]
    metrics = {
        "ΔRMSE (m)":          [],
        "ΔPeak sag (m)":      [],
        "ΔPeak tension (N)":  [],
    }

    for on_tag, off_tag, _ in cases:
        df_on  = load_cap(on_tag)
        nff_csvs = [f for f in os.listdir(os.path.join(P2A, off_tag)) if f.endswith(".csv")]
        df_off = pd.read_csv(os.path.join(P2A, off_tag, nff_csvs[0]))

        mask = df_on["time"] >= 8
        rmse_on  = float(np.sqrt(np.mean(error_vec(df_on[mask])**2)))
        rmse_off = float(np.sqrt(np.mean(error_vec(df_off[df_off["time"]>=8])**2)))

        sag_on  = float(np.min((df_on["payload_z"] - df_on["ref_z"]).values))
        sag_off = float(np.min((df_off["payload_z"] - df_off["ref_z"]).values))

        t_on = [df_on[f"tension_{i}"].values.max() for i in range(5)]
        t_off= [df_off[f"tension_{i}"].values.max() for i in range(5)]
        peak_on  = float(np.max(t_on))
        peak_off = float(np.max(t_off))

        metrics["ΔRMSE (m)"].append(rmse_off - rmse_on)
        metrics["ΔPeak sag (m)"].append(sag_off - sag_on)
        metrics["ΔPeak tension (N)"].append(peak_off - peak_on)

    xlabels = ["V3", "V4", "V5"]
    fig, axes = plt.subplots(1, 3, figsize=(DOUBLE_COL[0], 2.5), sharey=False)
    for ax, (label, vals) in zip(axes, metrics.items()):
        x = np.arange(len(vals))
        colors = [C_FFOFF if v >= 0 else C_FFON for v in vals]
        bars = ax.bar(x, vals, color=colors, width=0.55, zorder=3)
        ax.axhline(0, color="black", lw=0.8)
        for b, v in zip(bars, vals):
            ax.text(b.get_x()+b.get_width()/2,
                    v + np.sign(v)*abs(v)*0.03,
                    f"{v:+.3f}", ha="center", va="bottom" if v>0 else "top", fontsize=7)
        ax.set_xticks(x); ax.set_xticklabels(xlabels, fontsize=8)
        ax.set_title(label, fontsize=8)
        ax.set_xlabel("Scenario")

    # Note about single-seed
    fig.text(0.5, -0.04, "Single seed (deterministic); paired-seed CRN bootstrap planned (n=32).",
             ha="center", fontsize=7, style="italic", color="gray")
    plt.tight_layout()
    save("fig_paired_delta.png")

# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 5: fig_dwell_sweep — inter-fault gap sweep results
# ══════════════════════════════════════════════════════════════════════════════
def fig_dwell_sweep():
    print("Generating fig_dwell_sweep …")

    sweep_dir = SWEEP
    dts, rho_vals, rmse_vals, success = [], [], [], []

    if not os.path.isdir(sweep_dir):
        print("  ⚠ sweep data not yet available; generating placeholder")
        _fig_dwell_sweep_placeholder()
        return

    for fname in sorted(os.listdir(sweep_dir)):
        if not fname.endswith(".json"):
            continue
        meta = json.load(open(os.path.join(sweep_dir, fname)))
        dt   = meta["inter_fault_gap"]
        csv_name = fname.replace(".json", ".csv")
        csv_path = os.path.join(sweep_dir, csv_name)
        if not os.path.exists(csv_path):
            continue
        df = pd.read_csv(csv_path)
        t  = df["time"].values

        # Lyapunov proxy at fault events
        V_all = lyapunov_proxy(df)
        t_f1  = meta.get("t_fault_0", 15.0)
        t_f2  = meta.get("t_fault_1", t_f1 + dt)

        # V at each fault tick (just before)
        def V_at(t_fault):
            idx = np.searchsorted(t, t_fault) - 1
            if idx < 0: idx = 0
            return float(V_all[max(0, idx)])

        V1 = V_at(t_f1)
        V2 = V_at(t_f2)
        rho = V2 / V1 if V1 > 0 else float("nan")

        # RMSE over [8, end]
        mask  = df["time"] >= 8
        rmse_ = float(np.sqrt(np.mean(error_vec(df[mask])**2)))

        dts.append(dt)
        rho_vals.append(rho)
        rmse_vals.append(rmse_)
        # Success: rho < 1 and rmse < 0.35
        success.append(1 if (rho < 1.0 and rmse_ < 0.35) else 0)

    if not dts:
        print("  ⚠ no sweep CSV found; generating placeholder")
        _fig_dwell_sweep_placeholder()
        return

    dts_norm = [d / TAU_PEND for d in dts]

    # Theory: rho_theory = exp(-alpha_min * tau_d) with alpha_min = lambda_Lyap = 0.45
    alpha_min = 0.45
    mu2 = 1.05  # conservative Lyapunov jump ratio at fault (mu² from Lemma 4)
    dt_range = np.linspace(min(dts)*0.5, max(dts)*1.1, 200)
    rho_theory = np.exp(-alpha_min * dt_range) * mu2

    fig, axes = plt.subplots(2, 1, figsize=(SINGLE_COL[0], 4.5), sharex=True)

    # ── Top: hat_rho vs tau_d / tau_pend (semilog) ──────────────────────────
    ax = axes[0]
    ax.semilogy([d/TAU_PEND for d in dts], rho_vals,
                "o-", color=C_FFON, markersize=5, lw=1.2, label="Empirical $\\hat{\\rho}$")
    ax.semilogy(dt_range/TAU_PEND, rho_theory,
                "--", color="gray", lw=1.0, label="$\\rho^{\\rm theory}=e^{-\\alpha_{\\min}\\tau_d}\\mu^2$")
    ax.axhline(1.0, color=C_FAULT, ls=":", lw=1.0, label="$\\hat{\\rho}=1$ threshold")
    ax.axvline(1.0, color="gray", ls="--", lw=0.7, alpha=0.7)
    ax.set_ylabel("Dwell-cycle $\\hat{\\rho} = V(t_{k+1}^-)/V(t_k^-)$")
    ax.set_title("Dwell-time boundary characterisation")
    ax.legend(fontsize=6.5)

    # ── Bottom: success probability (single-seed binary, no CI yet) ─────────
    ax2 = axes[1]
    ax2.plot([d/TAU_PEND for d in dts], success,
             "s-", color=C_FFON, markersize=5, lw=1.2, label="Pass (single seed)")
    ax2.axvline(1.0, color="gray", ls="--", lw=0.7, alpha=0.7,
                label="$\\tau_d = \\tau_{\\mathrm{pend}}$")
    ax2.set_ylim(-0.1, 1.2)
    ax2.set_yticks([0, 1]); ax2.set_yticklabels(["Fail", "Pass"])
    ax2.set_xlabel("$\\tau_d / \\tau_{\\mathrm{pend}}$")
    ax2.set_ylabel("Success")
    ax2.legend(fontsize=6.5)
    fig.text(0.5, -0.01, "Wilson 95% CI planned (n=300 trials per point).",
             ha="center", fontsize=6.5, style="italic", color="gray")

    plt.tight_layout()
    save("fig_dwell_sweep.png")

def _fig_dwell_sweep_placeholder():
    """Theoretical prediction only when sweep data unavailable."""
    dts_norm = np.linspace(0.4, 4.0, 200)
    alpha_min, mu2 = 0.45, 1.05
    rho_theory = np.exp(-alpha_min * dts_norm * TAU_PEND) * mu2

    fig, axes = plt.subplots(2, 1, figsize=(SINGLE_COL[0], 4.5), sharex=True)
    ax = axes[0]
    ax.semilogy(dts_norm, rho_theory, "--", color="gray", lw=1.2,
                label="$\\rho^{\\rm theory}=e^{-\\alpha_{\\min}\\tau_d}\\mu^2$")
    ax.axhline(1.0, color=C_FAULT, ls=":", lw=1.0)
    ax.axvline(1.0, color="gray", ls="--", lw=0.7, alpha=0.7)
    ax.set_ylabel("$\\hat{\\rho}$")
    ax.set_title("Dwell-time boundary (theory; data pending)")
    ax.legend(fontsize=7)
    axes[1].axvline(1.0, color="gray", ls="--", lw=0.7)
    axes[1].set_xlabel("$\\tau_d / \\tau_{\\mathrm{pend}}$")
    axes[1].set_ylabel("Success")
    axes[1].set_ylim(-0.1, 1.2)
    axes[1].text(0.5, 0.5, "[Sweep data pending]", transform=axes[1].transAxes,
                 ha="center", va="center", fontsize=9, color="gray", style="italic")
    plt.tight_layout()
    save("fig_dwell_sweep.png")

# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 6: fig_ff_ablation — P2-A grouped bar
# ══════════════════════════════════════════════════════════════════════════════
def fig_ff_ablation():
    print("Generating fig_ff_ablation …")
    cases = [
        ("V3_single_wind",  "V3_single_wind_nff",  "V3"),
        ("V4_dual_5s_wind", "V4_dual_5s_wind_nff", "V4"),
        ("V5_dual_10s_wind","V5_dual_10s_wind_nff","V5"),
    ]
    rmse_on, rmse_off, sag_on, sag_off = [], [], [], []
    for on_tag, off_tag, _ in cases:
        df_on  = load_cap(on_tag)
        nff_csvs = [f for f in os.listdir(os.path.join(P2A, off_tag)) if f.endswith(".csv")]
        df_off = pd.read_csv(os.path.join(P2A, off_tag, nff_csvs[0]))

        mask_on  = df_on["time"]  >= 8
        mask_off = df_off["time"] >= 8
        rmse_on.append(float(np.sqrt(np.mean(error_vec(df_on[mask_on])**2))))
        rmse_off.append(float(np.sqrt(np.mean(error_vec(df_off[mask_off])**2))))
        sag_on.append(float(np.abs(np.min((df_on["payload_z"] - df_on["ref_z"]).values))))
        sag_off.append(float(np.abs(np.min((df_off["payload_z"] - df_off["ref_z"]).values))))

    x   = np.arange(3)
    w   = 0.35
    xlabels = ["V3", "V4", "V5"]

    fig, axes = plt.subplots(1, 2, figsize=(DOUBLE_COL[0], 2.6))

    # RMSE
    ax = axes[0]
    b1 = ax.bar(x - w/2, rmse_on,  w, color=C_FFON,  label="FF enabled",  zorder=3)
    b2 = ax.bar(x + w/2, rmse_off, w, color=C_FFOFF, label="FF disabled", zorder=3,
                hatch="//", edgecolor="white")
    for bars in [b1, b2]:
        for bar in bars:
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.003,
                    f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=6.5)
    # Annotate % change
    for i, (a, b) in enumerate(zip(rmse_on, rmse_off)):
        pct = (b-a)/a*100
        ax.annotate(f"+{pct:.0f}%", xy=(x[i]+w/2, b+0.005),
                    fontsize=6.5, color=C_FFOFF, ha="center")
    ax.set_xticks(x); ax.set_xticklabels(xlabels)
    ax.set_ylabel("3-D RMSE (m)")
    ax.set_title("(a) Trajectory RMSE — FF ablation")
    ax.legend(fontsize=7)
    ax.set_ylim(0, max(rmse_off)*1.25)

    # Peak sag
    ax2 = axes[1]
    b3 = ax2.bar(x - w/2, sag_on,  w, color=C_FFON,  label="FF enabled",  zorder=3)
    b4 = ax2.bar(x + w/2, sag_off, w, color=C_FFOFF, label="FF disabled", zorder=3,
                 hatch="//", edgecolor="white")
    for bars in [b3, b4]:
        for bar in bars:
            ax2.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.003,
                     f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=6.5)
    for i, (a, b) in enumerate(zip(sag_on, sag_off)):
        mult = b/a if a > 0 else float("nan")
        if not np.isnan(mult):
            ax2.annotate(f"×{mult:.1f}", xy=(x[i]+w/2, b+0.005),
                         fontsize=6.5, color=C_FFOFF, ha="center")
    ax2.set_xticks(x); ax2.set_xticklabels(xlabels)
    ax2.set_ylabel("Peak payload alt. sag |min $e_z$| (m)")
    ax2.set_title("(b) Peak sag — FF ablation")
    ax2.legend(fontsize=7)
    ax2.set_ylim(0, max(sag_off)*1.3)

    plt.tight_layout()
    save("fig_ff_ablation.png")

# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 7: fig_robustness_surface — P2-B mass sweep + 2D robustness panel
# ══════════════════════════════════════════════════════════════════════════════
def fig_robustness_surface():
    print("Generating fig_robustness_surface …")
    df = pd.read_csv(os.path.join(P2B, "_summary/p2b_summary.csv"))

    masses = sorted(df["mass_kg"].unique())
    mode_col = df["mode"]
    rmse_on   = [df[(df["mass_kg"]==m) & (mode_col=="ff_on")  ]["rms_err_m"].values[0] for m in masses]
    rmse_off  = [df[(df["mass_kg"]==m) & (mode_col=="ff_off") ]["rms_err_m"].values[0] for m in masses]
    rmse_l1   = [df[(df["mass_kg"]==m) & (mode_col=="ff_off_l1")]["rms_err_m"].values[0] for m in masses]
    sag_on    = [abs(df[(df["mass_kg"]==m) & (mode_col=="ff_on")  ]["mean_alt_sag_m"].values[0]) for m in masses]
    sag_off   = [abs(df[(df["mass_kg"]==m) & (mode_col=="ff_off") ]["mean_alt_sag_m"].values[0]) for m in masses]
    sag_l1    = [abs(df[(df["mass_kg"]==m) & (mode_col=="ff_off_l1")]["mean_alt_sag_m"].values[0]) for m in masses]

    fig, axes = plt.subplots(1, 2, figsize=(DOUBLE_COL[0], 2.8))

    ax = axes[0]
    ax.plot(masses, rmse_on,  "o-", color=C_FFON,  lw=1.2, label="FF-on")
    ax.plot(masses, rmse_off, "s--", color=C_FFOFF, lw=1.1, label="FF-off")
    ax.plot(masses, rmse_l1,  "^-.", color=C_L1,   lw=1.1, label="FF-off + $L_1$")
    ax.axvline(3.0, color="gray", ls=":", lw=0.9, label="$\\theta^\\star=3.0$ kg")
    ax.axhline(0.35, color="gray", ls="--", lw=0.7, alpha=0.7, label="Threshold 0.35 m")
    ax.set_xlabel("True payload mass $m_L$ (kg)")
    ax.set_ylabel("3-D RMSE (m)")
    ax.set_title("(a) P2-B: RMSE vs mass mismatch")
    ax.legend(fontsize=6.5)
    ax.set_xticks(masses)

    ax2 = axes[1]
    ax2.plot(masses, sag_on,  "o-", color=C_FFON,  lw=1.2, label="FF-on")
    ax2.plot(masses, sag_off, "s--", color=C_FFOFF, lw=1.1, label="FF-off")
    ax2.plot(masses, sag_l1,  "^-.", color=C_L1,   lw=1.1, label="FF-off + $L_1$")
    ax2.axvline(3.0, color="gray", ls=":", lw=0.9)
    # Annotate recovery fraction at 3.9 kg
    idx = masses.index(3.9)
    sag_total = sag_off[idx] - sag_on[idx]
    sag_l1_rec= sag_off[idx] - sag_l1[idx]
    frac = sag_l1_rec / sag_total if sag_total > 0 else 0
    ax2.annotate(f"$L_1$ recovers\n{frac*100:.0f}% of sag\n@ 3.9 kg",
                 xy=(3.9, sag_l1[idx]),
                 xytext=(3.55, sag_off[idx]*0.55),
                 fontsize=6.5, arrowprops=dict(arrowstyle="->", lw=0.7),
                 color=C_L1)
    ax2.set_xlabel("True payload mass $m_L$ (kg)")
    ax2.set_ylabel("Mean alt. sag |$\\bar{e}_z$| (m)")
    ax2.set_title("(b) P2-B: Mean sag vs mass mismatch")
    ax2.legend(fontsize=6.5)
    ax2.set_xticks(masses)

    fig.text(0.5, -0.04,
             "P2-B operates at $m_L \\in [2.5, 3.9]$ kg (design range). "
             "Robustness surface over $(m_L, k_s, c_{\\rm drag}, \\bar{w})$ requires E5 sweep (pending).",
             ha="center", fontsize=6.5, style="italic", color="gray", wrap=True)
    plt.tight_layout()
    save("fig_robustness_surface.png")

# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 8: fig_extensions — MPC + segmented tension + reshape
# ══════════════════════════════════════════════════════════════════════════════
def fig_extensions():
    print("Generating fig_extensions …")
    df_mpc    = pd.read_csv(os.path.join(P2C, "_summary/p2c_summary.csv"))
    df_period = pd.read_csv(os.path.join(P2D, "_summary/p2d_summary.csv"))

    # Segmented tension: compute pre-fault vs post-fault peak for baseline and mpc
    def seg_tensions(variant_dir, variant_csv):
        csv_path = os.path.join(variant_dir, variant_csv)
        df = pd.read_csv(csv_path)
        t_fault1 = 12.0
        pre  = df[df["time"] < t_fault1]
        post = df[(df["time"] >= t_fault1) & (df["time"] < t_fault1 + 8)]
        peak_pre  = float(np.max([df[f"tension_{i}"].max() for i in range(5)]))
        peak_post = float(np.max([df[df["time"] >= t_fault1][f"tension_{i}"].max() for i in range(5)]))
        return peak_pre, peak_post

    # Baseline (V4) and two MPC variants
    base_pre, base_post = seg_tensions(
        os.path.join(CAP, "V4_dual_5s_wind"),
        "scenario_V4_dual_5s_wind.csv")

    # MPC variants
    mpc_dirs = [d for d in os.listdir(P2C) if not d.startswith("_")]
    mpc_seg = {}
    for d in mpc_dirs:
        csvs = [f for f in os.listdir(os.path.join(P2C, d)) if f.endswith(".csv")]
        if csvs:
            try:
                pre_, post_ = seg_tensions(os.path.join(P2C, d), csvs[0])
                ceiling_str = d.replace("p2c_mpc_T", "").replace("p2c_", "")
                mpc_seg[d] = (pre_, post_)
            except Exception:
                pass

    fig = plt.figure(figsize=(DOUBLE_COL[0], 4.8))
    gs  = gridspec.GridSpec(2, 2, hspace=0.55, wspace=0.38)
    ax_rmse  = fig.add_subplot(gs[0, 0])
    ax_viol  = fig.add_subplot(gs[0, 1])
    ax_seg   = fig.add_subplot(gs[1, 0])   # segmented tension
    ax_resh  = fig.add_subplot(gs[1, 1])

    # ── (a) MPC RMSE vs ceiling ─────────────────────────────────────────────
    ceil_vals = df_mpc[df_mpc["mode"] != "baseline"]["T_ceiling_N"].values
    rmse_mpc  = df_mpc[df_mpc["mode"] != "baseline"]["rms_err_m"].values
    base_rmse = float(df_mpc[df_mpc["mode"] == "baseline"]["rms_err_m"].iloc[0])

    ax_rmse.bar(ceil_vals, rmse_mpc, width=6, color=C_FFON, zorder=3)
    ax_rmse.axhline(base_rmse, color=C_FAULT, ls="--", lw=1.0,
                    label=f"Baseline {base_rmse:.4f} m")
    ax_rmse.set_xlabel("MPC tension ceiling $T_{\\max}$ (N)")
    ax_rmse.set_ylabel("3-D RMSE (m)")
    ax_rmse.set_title("(a) MPC: RMSE vs ceiling (NF2)")
    ax_rmse.legend(fontsize=6.5)
    ax_rmse.set_ylim(base_rmse * 0.98, base_rmse * 1.04)

    # ── (b) MPC violation fraction ─────────────────────────────────────────
    viol = df_mpc[df_mpc["mode"] != "baseline"]["violation_frac"].values
    ax_viol.bar(ceil_vals, viol * 100, width=6, color="#CC79A7", zorder=3)
    ax_viol.set_xlabel("$T_{\\max}$ (N)")
    ax_viol.set_ylabel("Violation fraction (%)")
    ax_viol.set_title("(b) MPC: constraint violation frac.")

    # ── (c) Segmented tension analysis (pre vs post fault) ─────────────────
    ax_seg.bar([0], [base_pre],  width=0.35, color=C_FFON,  label="Pre-fault peak")
    ax_seg.bar([0.4],[base_post], width=0.35, color=C_FFOFF, label="Post-fault peak")
    ax_seg.axhline(60, color="gray", ls="--", lw=0.7, alpha=0.7)
    ax_seg.axhline(100, color="gray", ls=":", lw=0.7, alpha=0.7)
    ax_seg.set_xticks([0.2]); ax_seg.set_xticklabels(["V4 baseline"], fontsize=7.5)
    ax_seg.set_ylabel("Peak rope tension (N)")
    ax_seg.set_title("(c) Pre/post-fault segmented tension\n(NF2: ceiling non-binding post-fault)")
    ax_seg.legend(fontsize=7)
    ax_seg.text(0.05, base_pre + 2, f"{base_pre:.1f} N\n(traj. spike)", fontsize=6.5, color=C_FFON)
    ax_seg.text(0.05, base_post + 2, f"{base_post:.1f} N\n(redistributed)", fontsize=6.5, color=C_FFOFF)

    # ── (d) Reshape period sweep ────────────────────────────────────────────
    periods   = sorted(df_period["T_period_s"].unique())
    rmse_nr   = [df_period[(df_period["T_period_s"]==p) & (df_period["mode"]=="noreshape")]["rms_err_m"].values[0]
                 for p in periods]
    rmse_r    = [df_period[(df_period["T_period_s"]==p) & (df_period["mode"]=="reshape")]["rms_err_m"].values[0]
                 for p in periods]
    ax_resh.plot(periods, rmse_nr, "o-",  color=C_FFON,  lw=1.2, label="No reshape")
    ax_resh.plot(periods, rmse_r,  "s--", color=C_FFOFF, lw=1.1, label="Reshape on")

    # Theoretical prediction line at T=6s (placeholder annotation)
    ax_resh.axvline(6.0, color="gray", ls=":", lw=0.9, alpha=0.8,
                    label="E8 binding regime ($T=6$ s, data pending)")
    ax_resh.annotate("25.8% reduction\npredicted\n(Theorem 4)",
                     xy=(6.0, min(rmse_nr)*0.95), fontsize=6, color="gray",
                     xytext=(6.5, min(rmse_nr)*0.85),
                     arrowprops=dict(arrowstyle="->", lw=0.6))
    ax_resh.set_xlabel("Lemniscate period $T_{\\rm ref}$ (s)")
    ax_resh.set_ylabel("3-D RMSE (m)")
    ax_resh.set_title("(d) Reshape: period sweep (NF3)")
    ax_resh.set_xticks(periods)
    ax_resh.legend(fontsize=6.5)

    save("fig_extensions.png")

# ══════════════════════════════════════════════════════════════════════════════
# FIGURE 9: fig_failure_gallery — H1/H2/H3 boundary violations
# ══════════════════════════════════════════════════════════════════════════════
def fig_failure_gallery():
    print("Generating fig_failure_gallery …")
    # We construct synthetic boundary demonstrations from existing data
    # H1: excessive slack — use tension data near zero (existing V2/V3)
    # H2: sub-threshold dwell — use sweep dt=2 if available, else V4 transient analysis
    # H3: QP saturation — illustrate from domain audit fractions
    df_v4 = load_cap("V4_dual_5s_wind")
    t = df_v4["time"].values
    # Slack excursion: tension_0 near zero after t=12
    T0 = df_v4["tension_0"].values

    fig, axes = plt.subplots(1, 3, figsize=(DOUBLE_COL[0], 2.8))

    # ── H1: Slack excursion ─────────────────────────────────────────────────
    ax = axes[0]
    # Show tension of rope 0 going to zero (H1 illustration)
    window = (t >= 10) & (t <= 18)
    ax.plot(t[window], T0[window], color=C_FFON, lw=1.2, label="Rope 0 tension")
    ax.axhline(0.5, color=C_FAULT, ls="--", lw=1.0, label="Slack threshold 0.5 N")
    ax.axvline(12.0, **FAULT_STYLE, label="Fault @12 s")
    ax.fill_between(t[window], 0, np.where(T0[window] < 0.5, T0[window], 0.5),
                    alpha=0.25, color=C_FAULT)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Tension (N)")
    ax.set_title("H1: Slack excursion\n(rope 0, V4 fault window)")
    ax.legend(fontsize=6.5)
    ax.text(0.35, 0.72, "H1 violated:\n$\\eta > \\eta_{\\max}$\nfor severed rope",
            transform=ax.transAxes, fontsize=7, color=C_FAULT,
            bbox=dict(fc="white", ec=C_FAULT, lw=0.7, alpha=0.85))

    # ── H2: Sub-threshold dwell (use sweep data or V4 Lyapunov proxy) ──────
    ax2 = axes[1]
    sweep_avail = False
    if os.path.isdir(SWEEP):
        csvs = sorted([f for f in os.listdir(SWEEP) if f.endswith(".csv")])
        if csvs:
            dt2_csv = [c for c in csvs if "dt_2" in c]
            dt10_csv= [c for c in csvs if "dt_10" in c or "dt_14" in c or "dt_20" in c]
            if dt2_csv and dt10_csv:
                df_dt2  = pd.read_csv(os.path.join(SWEEP, dt2_csv[0]))
                df_dt10 = pd.read_csv(os.path.join(SWEEP, dt10_csv[-1]))
                V_dt2  = lyapunov_proxy(df_dt2)
                V_dt10 = lyapunov_proxy(df_dt10)
                t2  = df_dt2["time"].values
                t10 = df_dt10["time"].values
                win2  = (t2  >= 10) & (t2  <= 25)
                win10 = (t10 >= 10) & (t10 <= 25)
                ax2.semilogy(t2[win2],   V_dt2[win2],   color=C_FFOFF, lw=1.1,
                             label=f"$\\tau_d=2$ s < $\\tau_{{\\rm pend}}$ (H2 violated)")
                ax2.semilogy(t10[win10], V_dt10[win10], color=C_FFON,  lw=1.1,
                             label=f"$\\tau_d\\geq\\tau_{{\\rm pend}}$ (H2 satisfied)")
                ax2.axvline(15.0, **FAULT_STYLE)
                sweep_avail = True

    if not sweep_avail:
        # Illustrative: show V(t) from V4 and V5 as surrogates
        V4 = lyapunov_proxy(df_v4)
        df_v5 = load_cap("V5_dual_10s_wind")
        V5 = lyapunov_proxy(df_v5)
        t5 = df_v5["time"].values
        win4 = (t  >= 10) & (t  <= 27)
        win5 = (t5 >= 10) & (t5 <= 27)
        ax2.semilogy(t[win4],  V4[win4],  color=C_FFOFF, lw=1.1,
                     label="V4: $\\Delta t=5$ s (margin 2.76 s)")
        ax2.semilogy(t5[win5], V5[win5],  color=C_FFON,  lw=1.1,
                     label="V5: $\\Delta t=10$ s (margin 7.76 s)")
        ax2.axvline(12.0, **FAULT_STYLE)
        ax2.axvline(17.0, color=C_FAULT, ls=":", lw=0.8)

    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("$V(t)=\\xi^\\top P_v \\xi$ (log scale)")
    ax2.set_title("H2: Sub-threshold dwell\n(Lyapunov contraction proxy)")
    ax2.legend(fontsize=6.5)
    ax2.text(0.05, 0.72, "$\\tau_{{\\rm pend}}=2.24$ s\nH2 satisfied: $V(t_k^-)\\downarrow$\nH2 violated: $V(t_k^-)\\uparrow$",
             transform=ax2.transAxes, fontsize=6.5, color="gray")

    # ── H3: QP active-set transition ────────────────────────────────────────
    ax3 = axes[2]
    # Show QP solve time spike as proxy for H3 (from V4)
    qp_t = df_v4.loc[(df_v4["time"] >= 10) & (df_v4["time"] <= 25), "time"].values
    # Use max across drones
    qp_max = np.max(np.stack(
        [df_v4.loc[(df_v4["time"] >= 10) & (df_v4["time"] <= 25), f"qp_solve_us_{i}"].values
         for i in range(5)], axis=0), axis=0)
    ax3.plot(qp_t, qp_max, color=C_FFON, lw=0.8, alpha=0.85, label="QP solve time (μs)")
    # Running H3 fraction
    from numpy.lib.stride_tricks import sliding_window_view
    window_size = 1000
    if len(qp_max) > window_size:
        n_active = sliding_window_view(
            (df_v4.loc[(df_v4["time"]>=10)&(df_v4["time"]<=25)]["act_az_lo_0"].abs() < 1).values.astype(float),
            window_size).mean(axis=1)
        t_slide = qp_t[window_size-1:]
        ax3_twin = ax3.twinx()
        ax3_twin.plot(t_slide, n_active * 100, color=C_LYAP, lw=0.9,
                      ls="--", alpha=0.7, label="QP constraint active (%)")
        ax3_twin.axhline(1.0, color=C_FAULT, ls=":", lw=0.8)
        ax3_twin.set_ylabel("QP active frac. (%)", color=C_LYAP, fontsize=7)
        ax3_twin.tick_params(axis="y", labelcolor=C_LYAP)
        ax3_twin.legend(fontsize=6.5, loc="upper right")
    ax3.axvline(12.0, **FAULT_STYLE)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("QP solve time (μs)")
    ax3.set_title("H3: QP active-set\n(transition proxy, V4)")
    ax3.legend(fontsize=6.5, loc="upper left")

    fig.text(0.5, -0.02,
             "H1: rope slack excursion post-fault (Kelvin–Voigt regime). "
             "H2: sub-threshold dwell using sweep data (E4 pending for full Wilson CI). "
             "H3: QP active-set fractions from domain audit.",
             ha="center", fontsize=6, style="italic", color="gray", wrap=True)
    plt.tight_layout()
    save("fig_failure_gallery.png")

# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print(f"Output directory: {OUT}\n")
    fig_rmse_overview()
    fig_reduction_fidelity()
    fig_fault_zoom()
    fig_paired_delta()
    fig_dwell_sweep()
    fig_ff_ablation()
    fig_robustness_surface()
    fig_extensions()
    fig_failure_gallery()
    print("\nAll figures generated.")
