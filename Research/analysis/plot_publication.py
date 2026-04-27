#!/usr/bin/env python3
"""
Publication-grade figure suite for the decentralized fault-tolerant
cooperative-lift manuscript. Produces the 12-figure IEEE-style set
requested by the multi-agent reviewer consultation:

    F01  — System architecture block diagram (matplotlib rendering)
    F02  — 3-D trajectory with reference tube (per scenario)
    F03  — Payload tracking-error time series + fault annotations
    F04  — Per-rope tension waterfall (segment × time)
    F05  — Load-sharing imbalance σ_T(t) cross-scenario overlay
    F06  — STFT spectrogram of σ_T (fault detectability)
    F07  — QP active-constraint occupancy stackplot
    F08  — Thrust / tilt saturation timeline
    F09  — Closed-loop pole migration (N_alive sweep) — linearised
    F10  — Peak rope-tension vs inter-fault gap Δt (MC scatter, if present)
    F11  — Ablation / headline-metrics heatmap
    F12  — Cross-scenario grouped bars with bootstrap CIs
    S01  — Pickup-phase bead-chain stretch (supplementary)
    S02  — Settling-time ECDF per scenario (supplementary)

Usage:
    python3 plot_publication.py <campaign_root> <output_dir>

Where <campaign_root> is e.g.
    /workspaces/Tether_Grace/output/Tether_Grace_5drone

and must contain
    08_source_data/scenario_*.csv
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal as sp_signal

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import (
    BURN_IN_SECONDS, COLORS, DOUBLE_COL, DOUBLE_COL_TALL, FAULT_STYLE,
    PAYLOAD_STYLE, REF_STYLE, SINGLE_COL, SQUARE, annotate_faults,
    detect_faults, setup_style, trim_start,
)

setup_style()

# HELPERS
def num_drones(df: pd.DataFrame) -> int:
    n = 0
    while f"quad{n}_x" in df.columns:
        n += 1
    return n


def num_segments(df: pd.DataFrame, drone_idx: int = 0) -> int:
    n = 0
    while f"seg_T_{drone_idx}_{n}" in df.columns:
        n += 1
    return n


def tracking_error(df: pd.DataFrame) -> np.ndarray:
    err = df[["ref_x", "ref_y", "ref_z"]].values - df[
        ["payload_x", "payload_y", "payload_z"]
    ].values
    return np.linalg.norm(err, axis=1)


def tension_matrix(df: pd.DataFrame) -> np.ndarray:
    N = num_drones(df)
    return np.column_stack([df[f"tension_{i}"].values for i in range(N)])


def sigma_T(df: pd.DataFrame) -> np.ndarray:
    """Load-sharing imbalance: population std-dev across surviving drones."""
    return np.std(tension_matrix(df), axis=1)


def load_scenarios(root: Path) -> Dict[str, pd.DataFrame]:
    """Load every scenario CSV under <root>/08_source_data/."""
    data_root = root / "08_source_data"
    if not data_root.exists():
        data_root = root
    dfs: Dict[str, pd.DataFrame] = {}
    for csv in sorted(data_root.glob("scenario_*.csv")):
        name = csv.stem.replace("scenario_", "")
        dfs[name] = pd.read_csv(csv)
    return dfs


def _bootstrap_ci(
    x: np.ndarray, n_boot: int = 2000, ci: float = 0.95, seed: int = 0
) -> Tuple[float, float]:
    rng = np.random.default_rng(seed)
    n = len(x)
    means = np.empty(n_boot)
    for i in range(n_boot):
        means[i] = rng.choice(x, size=n, replace=True).mean()
    lo = np.quantile(means, (1 - ci) / 2)
    hi = np.quantile(means, 1 - (1 - ci) / 2)
    return lo, hi


# F01 — System architecture block diagram
def plot_architecture_diagram(out: Path):
    fig, ax = plt.subplots(figsize=(7.16, 4.4))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6.2)
    ax.axis("off")

    def box(xy, w, h, text, colour="#f0f0f0", edge="#333", fontsize=8):
        rect = plt.Rectangle(xy, w, h, facecolor=colour, edgecolor=edge,
                             linewidth=1.1, zorder=2)
        ax.add_patch(rect)
        ax.text(xy[0] + w / 2, xy[1] + h / 2, text,
                ha="center", va="center", fontsize=fontsize, zorder=3)

    def arrow(xy0, xy1, label=None, offset=(0, 0.12), **kw):
        ax.annotate("", xy=xy1, xytext=xy0,
                    arrowprops=dict(arrowstyle="->", lw=0.8, color="#333"),
                    zorder=1)
        if label:
            ax.text((xy0[0] + xy1[0]) / 2 + offset[0],
                    (xy0[1] + xy1[1]) / 2 + offset[1],
                    label, ha="center", fontsize=6.5, color="#555", zorder=3)

    # Top row — control stack
    box((0.2, 5.1), 2.3, 0.8,
        "Shared payload\nreference $r(t)$", colour="#e8f5e9")
    box((3.1, 4.6), 3.8, 1.4,
        "Drone $i$  —  Local QP (3 vars, box-constrained)\n"
        r"$\min\; w_t\|a - a_{\mathrm{tgt}}\|^2 + w_e\|a\|^2$"
        "\n"
        r"$a_{\mathrm{tgt}}=a_{\mathrm{track}} + w_s\,a_{\mathrm{swing}}$",
        colour="#e3f2fd", fontsize=7.5)
    box((7.5, 5.1), 2.3, 0.8,
        "Attitude PD\n(SO(3) geom.)", colour="#e3f2fd")

    # Plant (middle, wide)
    box((2.1, 1.9), 5.8, 1.5,
        "Drake MultibodyPlant + SceneGraph\n"
        "$N$ quadrotors  ·  $N \\times 8$ bead-chain ropes  ·  "
        "payload  ·  ground",
        colour="#fff3e0")

    # Feedback signals (left and right)
    box((0.2, 2.5), 1.8, 0.8, "Payload state\n$p_L, v_L$",
        colour="#fce4ec")
    box((8.0, 2.5), 1.8, 0.8, "Own rope\ntension $T_i$",
        colour="#fce4ec")

    # Fault-gate band
    box((2.1, 0.3), 5.8, 0.7,
        "Four-gate fault model:  physical / telemetry / visual / supervisory",
        colour="#ffebee")

    # Arrows
    arrow((2.5, 5.5), (3.1, 5.3), "$r(t)$")
    arrow((6.9, 5.3), (7.5, 5.5), "$a^{\\star}$")
    arrow((8.6, 5.1), (5.0, 3.4), "thrust $f$, torques $\\tau$")
    arrow((5.0, 1.9), (5.0, 1.0), "supervisory gating")
    arrow((2.0, 2.9), (3.1, 5.0), "$p_L, v_L$", offset=(-0.25, 0.0))
    arrow((8.0, 2.9), (6.9, 5.0), "$T_i$", offset=(0.15, 0.0))
    arrow((2.1, 2.6), (2.0, 2.9), None)
    arrow((7.9, 2.6), (8.0, 2.9), None)

    # Title caption
    ax.text(5, 6.0,
            "Figure 1.  Decentralised fault-aware cooperative-lift control: "
            "each drone executes an\nindependent local QP; no peer-tension "
            "channel and no global fault signal cross drone boundaries.",
            ha="center", fontsize=8, style="italic")
    fig.savefig(out)
    plt.close(fig)


# F02 — 3-D trajectory with reference tube
def plot_3d_trajectory_tube(df: pd.DataFrame, out: Path, scenario: str,
                            tube_radius: float = 0.3):
    from mpl_toolkits.mplot3d import art3d  # noqa: F401
    fig = plt.figure(figsize=DOUBLE_COL)
    ax = fig.add_subplot(111, projection="3d")

    # Reference + tube
    ax.plot(df["ref_x"], df["ref_y"], df["ref_z"], **REF_STYLE)
    # Construct a rough "tube" by plotting two offset helices.
    # Skip if reference is stationary for long stretches.
    ax.plot(df["payload_x"], df["payload_y"], df["payload_z"],
            color=PAYLOAD_STYLE["color"], lw=1.4, label="Payload")
    for i in range(num_drones(df)):
        ax.plot(df[f"quad{i}_x"], df[f"quad{i}_y"], df[f"quad{i}_z"],
                color=COLORS[i], lw=0.7, alpha=0.75, label=f"Drone {i}")
    # Mark faults as red X markers on the payload trace
    faults = detect_faults(df, num_drones(df))
    for (drone_i, t_f) in faults:
        row = df.iloc[(df["time"] - t_f).abs().argmin()]
        ax.scatter([row.payload_x], [row.payload_y], [row.payload_z],
                   s=40, c="#CC0000", marker="x",
                   label=f"Fault t={t_f:.1f}s (D{drone_i})")

    ax.set_xlabel("$x$ [m]"); ax.set_ylabel("$y$ [m]"); ax.set_zlabel("$z$ [m]")
    ax.set_title(f"3-D Trajectory — {scenario}", fontsize=9)
    ax.legend(loc="upper left", fontsize=6, ncol=2)
    ax.view_init(elev=24, azim=-62)
    fig.savefig(out)
    plt.close(fig)


# F03 — Tracking-error time series
def plot_tracking_error_overlay(dfs: Dict[str, pd.DataFrame], out: Path):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    palette = plt.cm.tab10(np.linspace(0, 1, len(dfs)))
    for (name, df), colour in zip(dfs.items(), palette):
        ax.plot(df["time"], tracking_error(df), lw=0.9, color=colour, label=name)
        faults = detect_faults(df, num_drones(df))
        for (_, t_f) in faults:
            ax.axvline(t_f, color=colour, lw=0.5, alpha=0.5, ls=":")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$\|e_p\|$ [m]")
    ax.set_title("Payload Tracking Error — cross-scenario overlay", fontsize=9)
    ax.legend(ncol=3, fontsize=7)
    fig.savefig(out); plt.close(fig)


# F04 — Per-rope tension waterfall (segment × time for each drone)
def plot_tension_waterfall(df: pd.DataFrame, out: Path, scenario: str):
    N = num_drones(df)
    S = num_segments(df)
    if S == 0:
        # Legacy CSV without per-segment data.
        return

    # Post-fault, the physical cable is severed (CableFaultGate zeros the
    # spatial forces applied to the plant) but the bead bodies still exist
    # in plant state and fall independently under gravity. The geometric
    # tension-probe output on those beads is therefore meaningless after
    # the fault — we mask it to zero for display, reflecting the actual
    # physical fact that the cable is severed. Detect fault times from the
    # scalar top-segment tension (which is the gated signal delivered to
    # the controller) and zero out that drone's segment tensions after
    # that time.
    faults = detect_faults(df, N)
    fault_time = {drone_i: t_f for (drone_i, t_f) in faults}
    t = df["time"].values

    fig, axes = plt.subplots(
        nrows=(N + 1) // 2, ncols=2,
        figsize=(7.16, 2.0 * ((N + 1) // 2)),
        sharex=True, sharey=True, constrained_layout=True)
    axes = np.atleast_1d(axes).ravel()

    segment_matrices = []
    for i in range(N):
        T = np.column_stack([df[f"seg_T_{i}_{j}"].values for j in range(S)])
        if i in fault_time:
            mask = t >= fault_time[i]
            T[mask, :] = 0.0
        segment_matrices.append(T)
    # Compute shared colour scale from the post-masking matrices.
    vmax = max(np.quantile(T, 0.995) for T in segment_matrices)
    if vmax <= 0:
        vmax = 1.0

    for i, T in enumerate(segment_matrices):
        ax = axes[i]
        im = ax.imshow(T.T, aspect="auto", origin="lower",
                       extent=[t[0], t[-1], 0, S],
                       cmap="inferno", vmin=0, vmax=vmax)
        ax.set_title(f"Drone {i} — rope segments", fontsize=8)
        ax.set_ylabel("seg idx")
        if i in fault_time:
            ax.axvline(fault_time[i], color="#9bffcc", ls=":", lw=0.9,
                       label=f"fault t={fault_time[i]:.1f} s")
    for ax in axes[N:]:
        ax.axis("off")
    axes[-1 if N % 2 == 0 else -2].set_xlabel("Time [s]")
    cbar = fig.colorbar(im, ax=axes.tolist(), shrink=0.8, pad=0.02)
    cbar.set_label("Segment tension [N]")
    fig.suptitle(f"Per-rope tension waterfall — {scenario}  "
                 r"(severed ropes masked to 0 post-fault)", fontsize=9)
    fig.savefig(out); plt.close(fig)


# F05 — Load-sharing imbalance σ_T(t) cross-scenario
def plot_sigma_T_overlay(dfs: Dict[str, pd.DataFrame], out: Path):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    palette = plt.cm.tab10(np.linspace(0, 1, len(dfs)))
    for (name, df), colour in zip(dfs.items(), palette):
        ax.plot(df["time"], sigma_T(df), lw=0.9, color=colour, label=name)
        faults = detect_faults(df, num_drones(df))
        for (_, t_f) in faults:
            ax.axvline(t_f, color=colour, lw=0.4, alpha=0.4, ls=":")
    ax.set_xlabel("Time [s]"); ax.set_ylabel(r"$\sigma_T$ [N]")
    ax.set_title("Load-sharing imbalance — cross-scenario", fontsize=9)
    ax.legend(ncol=3, fontsize=7)
    fig.savefig(out); plt.close(fig)


# F06 — STFT spectrogram of σ_T
def plot_sigma_T_spectrogram(df: pd.DataFrame, out: Path, scenario: str):
    t = df["time"].values
    sig = sigma_T(df)
    fs = 1.0 / np.median(np.diff(t))
    nperseg = int(min(1024, fs * 1.0))
    if nperseg < 32:
        nperseg = 32
    f, tt, Sxx = sp_signal.spectrogram(
        sig - sig.mean(), fs=fs, nperseg=nperseg,
        noverlap=int(nperseg * 0.75), window="hann")
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    im = ax.pcolormesh(
        tt + t[0], f, 10 * np.log10(np.maximum(Sxx, 1e-16)),
        shading="gouraud", cmap="viridis",
        vmin=np.quantile(10 * np.log10(np.maximum(Sxx, 1e-16)), 0.05),
        vmax=np.quantile(10 * np.log10(np.maximum(Sxx, 1e-16)), 0.98))
    ax.set_ylim(0, min(10.0, f.max()))
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Frequency [Hz]")
    faults = detect_faults(df, num_drones(df))
    for (_, t_f) in faults:
        ax.axvline(t_f, color="#ff5555", ls=":", lw=0.9)
    cbar = fig.colorbar(im, ax=ax, pad=0.02)
    cbar.set_label(r"Power $\sigma_T$ [dB/Hz]")
    ax.set_title(f"STFT of $\\sigma_T$ — {scenario}", fontsize=9)
    fig.savefig(out); plt.close(fig)


# F07 — QP active-constraint occupancy stackplot
def plot_active_set(df: pd.DataFrame, out: Path, scenario: str):
    N = num_drones(df)
    keys = ["act_ax_lo", "act_ax_hi", "act_ay_lo", "act_ay_hi",
            "act_az_lo", "act_az_hi"]
    if f"{keys[0]}_0" not in df.columns:
        return
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    # Fraction of drones for which each bound is active at time t
    t = df["time"].values
    fractions = []
    labels = [r"tilt$_x^-$", r"tilt$_x^+$", r"tilt$_y^-$",
              r"tilt$_y^+$", r"thrust$^-$", r"thrust$^+$"]
    for k in keys:
        a = np.stack([df[f"{k}_{i}"].values for i in range(N)], axis=1).mean(axis=1)
        fractions.append(a)
    ax.stackplot(t, fractions, labels=labels,
                 colors=["#D55E00", "#E69F00", "#009E73", "#56B4E9",
                         "#0072B2", "#CC79A7"])
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Active-set fraction")
    ax.set_ylim(0, min(1.2, 1.05 * max(1.0, sum(f.max() for f in fractions))))
    ax.legend(ncol=6, fontsize=6.5, loc="upper right")
    ax.set_title(f"QP active-constraint occupancy — {scenario}", fontsize=9)
    fig.savefig(out); plt.close(fig)


# F08 — Thrust / tilt saturation timeline
def plot_thrust_tilt(df: pd.DataFrame, out: Path, scenario: str):
    N = num_drones(df)
    if f"thrust_cmd_0" not in df.columns:
        return
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=DOUBLE_COL_TALL, sharex=True)
    faults = detect_faults(df, N)
    for i in range(N):
        ax1.plot(df["time"], df[f"thrust_cmd_{i}"], lw=0.8,
                 color=COLORS[i], label=f"Drone {i}")
        ax2.plot(df["time"], np.degrees(df[f"tilt_mag_{i}"]), lw=0.8,
                 color=COLORS[i], label=f"Drone {i}")
    ax1.axhline(150.0, color="#CC0000", ls="--", lw=0.7, label="$F_{\\max}$")
    ax2.axhline(np.degrees(0.6), color="#CC0000", ls="--", lw=0.7,
                label=r"$\theta_{\max}$")
    for _, t_f in faults:
        ax1.axvline(t_f, **FAULT_STYLE); ax2.axvline(t_f, **FAULT_STYLE)
    ax1.set_ylabel(r"$\|F\|$ [N]"); ax2.set_ylabel(r"tilt [$^\circ$]")
    ax2.set_xlabel("Time [s]")
    ax1.set_title(f"Actuator-envelope occupancy — {scenario}", fontsize=9)
    ax1.legend(ncol=min(N + 1, 4), fontsize=6.5, loc="upper right")
    fig.savefig(out); plt.close(fig)


# F09 — Closed-loop pole migration under N_alive sweep
def plot_pole_migration(out: Path):
    """Linearised vertical-channel pole locus vs. N_alive drones.

    Simplified plant: payload mass m_L supported by N_alive ropes of
    effective stiffness k_eff / N_alive (parallel) and damping c_eff /
    N_alive. Each drone's PD gains project onto vertical equilibrium.
    We plot the pair of complex-conjugate poles of the payload-vertical
    subsystem for N_alive ∈ {1..5}.
    """
    m_L = 3.0
    # k per rope (effective, series of 9 beads × 25 kN/m)
    k_rope = 25000.0 / 9.0
    c_rope = 60.0 / 9.0
    fig, ax = plt.subplots(figsize=SINGLE_COL)
    palette = plt.cm.viridis(np.linspace(0.1, 0.9, 5))
    ax.axhline(0, color="#888", lw=0.4)
    ax.axvline(0, color="#888", lw=0.4)
    for idx, Na in enumerate(range(5, 0, -1)):
        # Vertical-channel dynamics: m_L x'' = -N_a k (x - x_ref) - N_a c x'
        # → s^2 + (N_a c / m_L) s + (N_a k / m_L) = 0
        a = 1.0
        b = Na * c_rope / m_L
        c = Na * k_rope / m_L
        disc = b * b - 4 * a * c
        if disc >= 0:
            r = [-b / 2 + np.sqrt(disc) / 2, -b / 2 - np.sqrt(disc) / 2]
            ax.scatter(r, [0, 0], s=36,
                       color=palette[idx], label=f"$N_{{\\mathrm{{alive}}}}={Na}$",
                       edgecolor="black", linewidth=0.5)
        else:
            pr = -b / 2; pi = np.sqrt(-disc) / 2
            ax.scatter([pr, pr], [pi, -pi], s=36,
                       color=palette[idx], label=f"$N_{{\\mathrm{{alive}}}}={Na}$",
                       edgecolor="black", linewidth=0.5)
    ax.set_xlabel(r"$\mathrm{Re}(s)$  [1/s]")
    ax.set_ylabel(r"$\mathrm{Im}(s)$  [1/s]")
    ax.set_title("Vertical-channel pole locus vs. $N_{\\mathrm{alive}}$",
                 fontsize=9)
    ax.legend(fontsize=7, loc="upper left")
    fig.savefig(out); plt.close(fig)


# F10 — Peak rope tension vs inter-fault gap Δt (MC; optional)
def plot_mc_scatter(mc_dir: Path, out: Path):
    if not mc_dir.exists():
        return False
    records = []
    for csv in sorted(mc_dir.glob("scenario_*.csv")):
        try:
            df = pd.read_csv(csv)
            df = df[df["time"] >= BURN_IN_SECONDS].reset_index(drop=True)
            T = tension_matrix(df)
            peak_T_full = float(T.max())
            # Read fault times from sidecar JSON so we can define the
            # "fault window" (just after the 2nd fault, before descent).
            meta_file = csv.with_suffix(".json")
            dt = None
            t_fault_0 = None
            t_fault_1 = None
            if meta_file.exists():
                try:
                    meta = json.load(open(meta_file))
                    dt = float(meta.get("inter_fault_gap", np.nan))
                    t_fault_0 = float(meta.get("t_fault_0", 15.0))
                    t_fault_1 = float(meta.get("t_fault_1", t_fault_0 + dt))
                except Exception:  # noqa: BLE001
                    pass
            if dt is None:
                parts = csv.stem.split("_")
                for tok in parts:
                    if tok.startswith("dt"):
                        try:
                            dt = float(tok[2:])
                        except ValueError:
                            pass
            # Fault-window peak T: from 1 s before first fault to min(5 s
            # after second fault, 3 s before nominal descent start). Keeps
            # the pure fault response amplitude separate from the descent
            # transient.
            t0 = (t_fault_0 or 15.0) - 1.0
            t1 = min((t_fault_1 or 20.0) + 5.0,
                     float(df["time"].max()) - 3.0)
            mask = (df["time"] >= t0) & (df["time"] <= t1)
            peak_T_fault = float(T[mask.values].max()) if mask.any() else np.nan
            peak_err = float(tracking_error(df).max())
            records.append(dict(dt=dt,
                                peak_T_full=peak_T_full,
                                peak_T_fault=peak_T_fault,
                                peak_err=peak_err,
                                file=csv.stem))
        except Exception as exc:  # noqa: BLE001
            print(f"  skipping {csv.name}: {exc}")
    if not records:
        return False
    dfm = pd.DataFrame(records).dropna(subset=["dt"]).sort_values("dt")
    if dfm.empty:
        return False
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=DOUBLE_COL)
    # Fault-window peak (the physically intrinsic fault-response magnitude)
    ax1.scatter(dfm.dt, dfm.peak_T_fault, s=22, c=COLORS[2], alpha=0.9,
                edgecolor="k", linewidth=0.4, label="MC samples")
    ax1.plot(dfm.dt, dfm.peak_T_fault.rolling(3, min_periods=1).max(),
             color="#CC0000", lw=0.7, ls="--", label="running max")
    ax1.set_xlabel(r"Inter-fault gap $\Delta t$ [s]")
    ax1.set_ylabel(r"Fault-window peak $T$ [N]")
    ax1.set_title(r"Fault response (within $[t_{f,1}{-}1, t_{f,2}{+}5]$)",
                  fontsize=8)
    ax1.legend(fontsize=7)
    # Full-run peak (includes descent amplification)
    ax2.scatter(dfm.dt, dfm.peak_T_full, s=22, c=COLORS[0], alpha=0.9,
                edgecolor="k", linewidth=0.4, label="MC samples")
    ax2.plot(dfm.dt, dfm.peak_T_full.rolling(3, min_periods=1).max(),
             color="#CC0000", lw=0.7, ls="--", label="running max")
    ax2.set_xlabel(r"Inter-fault gap $\Delta t$ [s]")
    ax2.set_ylabel(r"Full-run peak $T$ [N]")
    ax2.set_title("Descent-amplified (incl. end-of-trajectory lag)",
                  fontsize=8)
    ax2.legend(fontsize=7)
    fig.suptitle(r"Compound-fault stress envelope: $T_{\max}(\Delta t)$",
                 fontsize=9)
    fig.savefig(out); plt.close(fig)
    return True


# F11 — Ablation/headline metrics heatmap
def plot_metrics_heatmap(dfs: Dict[str, pd.DataFrame], out: Path):
    names = list(dfs.keys())
    metrics = ["RMS $\\|e_p\\|$", "peak $\\|e_p\\|$", "peak $T$",
               "peak $\\|F\\|$", "RMS $\\sigma_T$"]
    M = np.zeros((len(names), len(metrics)))
    for i, name in enumerate(names):
        df = dfs[name]
        err = tracking_error(df)
        T = tension_matrix(df)
        F = np.stack([
            np.sqrt(df[f"fx_{j}"] ** 2 + df[f"fy_{j}"] ** 2 + df[f"fz_{j}"] ** 2).values
            for j in range(num_drones(df))
        ], axis=1)
        M[i, 0] = np.sqrt(np.mean(err ** 2))
        M[i, 1] = err.max()
        M[i, 2] = T.max()
        M[i, 3] = F.max()
        M[i, 4] = np.sqrt(np.mean(sigma_T(df) ** 2))
    # Normalise each column 0..1 for colour (NumPy 2 removed ndarray.ptp).
    Mn = (M - M.min(axis=0)) / (np.ptp(M, axis=0) + 1e-9)
    fig, ax = plt.subplots(figsize=(7.16, 0.4 + 0.35 * len(names)))
    im = ax.imshow(Mn, aspect="auto", cmap="RdYlGn_r", vmin=0, vmax=1)
    ax.set_xticks(range(len(metrics)))
    ax.set_xticklabels(metrics, rotation=25, ha="right", fontsize=7)
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names, fontsize=7)
    for i in range(len(names)):
        for j in range(len(metrics)):
            ax.text(j, i, f"{M[i, j]:.2f}", ha="center", va="center",
                    fontsize=6.5,
                    color="black" if 0.25 < Mn[i, j] < 0.75 else "white")
    ax.set_title("Cross-scenario metrics heatmap (green=best, red=worst)",
                 fontsize=9)
    fig.colorbar(im, ax=ax, pad=0.02).set_label("Normalised (0–1)")
    fig.savefig(out); plt.close(fig)


# F12 — Cross-scenario grouped bars with bootstrap CIs
def plot_grouped_bars(dfs: Dict[str, pd.DataFrame], out: Path):
    names = list(dfs.keys())
    rms_err = []
    peak_T = []
    rms_sigma = []
    ci_rms = []
    for name in names:
        df = dfs[name]
        err = tracking_error(df)
        T = tension_matrix(df)
        rms_err.append(np.sqrt(np.mean(err ** 2)))
        peak_T.append(T.max())
        rms_sigma.append(np.sqrt(np.mean(sigma_T(df) ** 2)))
        ci_rms.append(_bootstrap_ci(err ** 2, n_boot=600))
    ci_lo = [np.sqrt(abs(l)) for (l, h) in ci_rms]
    ci_hi = [np.sqrt(h) for (l, h) in ci_rms]

    fig, axes = plt.subplots(1, 3, figsize=DOUBLE_COL)
    x = np.arange(len(names))
    colour = plt.cm.tab10(np.linspace(0, 1, len(names)))
    for ax, vals, lab in zip(
        axes, [rms_err, peak_T, rms_sigma],
        [r"RMS $\|e_p\|$ [m]", r"peak $T$ [N]", r"RMS $\sigma_T$ [N]"]
    ):
        ax.bar(x, vals, color=colour, edgecolor="black", linewidth=0.4)
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=30, ha="right", fontsize=6.5)
        ax.set_ylabel(lab, fontsize=8)
        for i, v in enumerate(vals):
            ax.text(i, v * 1.01, f"{v:.2f}", ha="center", va="bottom", fontsize=6)
        ax.grid(axis="y", linewidth=0.3, alpha=0.3)
    axes[0].errorbar(x, rms_err, yerr=[np.array(rms_err) - np.array(ci_lo),
                                        np.array(ci_hi) - np.array(rms_err)],
                     fmt="none", ecolor="black", elinewidth=0.6, capsize=2)
    fig.suptitle("Headline metrics — cross-scenario", fontsize=9)
    fig.savefig(out); plt.close(fig)


# S01 — Pickup-phase bead-chain stretch (supplementary)
def plot_pickup_stretch(df: pd.DataFrame, out: Path, scenario: str):
    N = num_drones(df)
    S = num_segments(df)
    if S == 0:
        return
    # Use only drone 0 for clarity
    t = df["time"].values
    m = t <= 3.0
    if not m.any():
        return
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    palette = plt.cm.plasma(np.linspace(0.15, 0.85, S))
    for j in range(S):
        ax.plot(t[m], df[f"seg_T_0_{j}"].values[m], lw=0.9,
                color=palette[j], label=f"seg {j}", alpha=0.9)
    ax.set_xlabel("Time [s]"); ax.set_ylabel(r"$T_{0,j}$ [N]")
    ax.set_title(f"Pickup-phase per-segment tension — {scenario}", fontsize=9)
    ax.legend(ncol=S, fontsize=6, loc="upper right")
    fig.savefig(out); plt.close(fig)


# S02 — Settling-time ECDF per scenario
def plot_settling_ecdf(dfs: Dict[str, pd.DataFrame], out: Path,
                        ep_threshold: float = 0.5):
    fig, ax = plt.subplots(figsize=SINGLE_COL)
    palette = plt.cm.tab10(np.linspace(0, 1, len(dfs)))
    for (name, df), colour in zip(dfs.items(), palette):
        faults = detect_faults(df, num_drones(df))
        if not faults:
            continue
        settlings = []
        for (_, t_f) in faults:
            post = df[df["time"] > t_f].copy()
            err = tracking_error(post)
            rev = err[::-1]
            times_rev = post.time.values[::-1]
            recovered = np.argmax(rev <= ep_threshold)
            if rev[recovered] <= ep_threshold:
                t_settle = times_rev[recovered] - t_f
                settlings.append(t_settle)
        if not settlings:
            continue
        s = np.sort(settlings)
        y = np.arange(1, len(s) + 1) / len(s)
        ax.step(s, y, where="post", color=colour, label=name, lw=1.0)
    ax.set_xlabel(r"Post-fault settling time [s]  ($\|e_p\| \leq 0.5$ m)")
    ax.set_ylabel("Empirical CDF")
    ax.set_title("Settling-time ECDF", fontsize=9)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(fontsize=6.5)
    fig.savefig(out); plt.close(fig)


# DRIVER
def main():
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    root = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)

    dfs = load_scenarios(root)
    if not dfs:
        print(f"No scenarios found in {root}")
        sys.exit(1)

    print(f"[plot_publication] {len(dfs)} scenarios loaded from {root}")

    # Dump per-scenario headline metrics for the paper's result-filler.
    # All metrics use the shared BURN_IN_SECONDS burn-in to stay aligned
    # with the IEEE-published tables (A3 fix).
    records = []
    for name, df in dfs.items():
        df_m = trim_start(df) if "time" in df.columns else df
        err = tracking_error(df_m)
        T = tension_matrix(df_m)
        F = np.stack([
            np.sqrt(df_m[f"fx_{j}"] ** 2 + df_m[f"fy_{j}"] ** 2
                    + df_m[f"fz_{j}"] ** 2).values
            for j in range(num_drones(df_m))
        ], axis=1)
        sT = sigma_T(df_m)
        records.append(dict(
            scenario=name,
            burn_in_s=BURN_IN_SECONDS,
            rms_error=float(np.sqrt(np.mean(err ** 2))),
            peak_error=float(err.max()),
            peak_tension=float(T.max()),
            peak_thrust=float(F.max()),
            rms_sigma_T=float(np.sqrt(np.mean(sT ** 2))),
            qp_solve_us_p99=float(df_m["qp_solve_us_0"].quantile(0.99))
                if "qp_solve_us_0" in df_m.columns else float("nan"),
        ))
    pd.DataFrame(records).to_csv(out / "publication_metrics.csv", index=False)

    # F01 architecture
    plot_architecture_diagram(out / "F01_architecture.pdf")
    plot_architecture_diagram(out / "F01_architecture.png")

    # F09 pole locus
    plot_pole_migration(out / "F09_pole_locus.pdf")
    plot_pole_migration(out / "F09_pole_locus.png")

    # Per-scenario figures
    for name, df in dfs.items():
        plot_3d_trajectory_tube(df, out / f"F02_3d_{name}.pdf", name)
        plot_3d_trajectory_tube(df, out / f"F02_3d_{name}.png", name)
        plot_tension_waterfall(df, out / f"F04_waterfall_{name}.pdf", name)
        plot_tension_waterfall(df, out / f"F04_waterfall_{name}.png", name)
        plot_sigma_T_spectrogram(df, out / f"F06_spectrogram_{name}.pdf", name)
        plot_sigma_T_spectrogram(df, out / f"F06_spectrogram_{name}.png", name)
        plot_active_set(df, out / f"F07_activeset_{name}.pdf", name)
        plot_active_set(df, out / f"F07_activeset_{name}.png", name)
        plot_thrust_tilt(df, out / f"F08_thrust_tilt_{name}.pdf", name)
        plot_thrust_tilt(df, out / f"F08_thrust_tilt_{name}.png", name)
        plot_pickup_stretch(df, out / f"S01_pickup_stretch_{name}.pdf", name)
        plot_pickup_stretch(df, out / f"S01_pickup_stretch_{name}.png", name)

    # Cross-scenario
    plot_tracking_error_overlay(dfs, out / "F03_tracking_error_overlay.pdf")
    plot_tracking_error_overlay(dfs, out / "F03_tracking_error_overlay.png")
    plot_sigma_T_overlay(dfs, out / "F05_sigmaT_overlay.pdf")
    plot_sigma_T_overlay(dfs, out / "F05_sigmaT_overlay.png")
    plot_metrics_heatmap(dfs, out / "F11_metrics_heatmap.pdf")
    plot_metrics_heatmap(dfs, out / "F11_metrics_heatmap.png")
    plot_grouped_bars(dfs, out / "F12_grouped_bars.pdf")
    plot_grouped_bars(dfs, out / "F12_grouped_bars.png")
    plot_settling_ecdf(dfs, out / "S02_settling_ecdf.pdf")
    plot_settling_ecdf(dfs, out / "S02_settling_ecdf.png")

    # Optional MC Phase-D
    mc_dir = root / "monte_carlo"
    if plot_mc_scatter(mc_dir, out / "F10_mc_scatter.pdf"):
        plot_mc_scatter(mc_dir, out / "F10_mc_scatter.png")

    print(f"All figures written to {out}")


if __name__ == "__main__":
    main()
