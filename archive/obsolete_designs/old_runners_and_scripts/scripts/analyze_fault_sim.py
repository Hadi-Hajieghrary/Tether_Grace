#!/usr/bin/env python3
"""
analyze_fault_sim.py — IEEE-style post-processing for Tether_Grace fault injection runs.

Usage:
    python3 analyze_fault_sim.py \
        --log-dir /path/to/logs/timestamp \
        --fault-time 15.0 \
        --fault-quad 0 \
        --output-dir /path/to/analysis \
        [--num-quads 5]
"""

import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ---------------------------------------------------------------------------
# IEEE figure style
# ---------------------------------------------------------------------------
IEEE_COLORS = [
    "#1f77b4",  # blue  (nominal quads / rope 1)
    "#ff7f0e",  # orange
    "#2ca02c",  # green
    "#d62728",  # red
    "#9467bd",  # purple
    "#8c564b",  # brown
    "#e377c2",  # pink
]
FAULT_COLOR = "#d62728"
FAULT_LS    = "--"

def ieee_style():
    plt.rcParams.update({
        "font.family":       "serif",
        "font.serif":        ["Times New Roman", "DejaVu Serif"],
        "font.size":         9,
        "axes.labelsize":    9,
        "axes.titlesize":    9,
        "legend.fontsize":   8,
        "xtick.labelsize":   8,
        "ytick.labelsize":   8,
        "lines.linewidth":   1.2,
        "axes.grid":         True,
        "grid.alpha":        0.35,
        "figure.dpi":        150,
        "savefig.dpi":       300,
        "savefig.bbox":      "tight",
        "savefig.pad_inches": 0.02,
    })

def add_fault_line(ax, fault_time, label=True):
    ax.axvline(fault_time, color=FAULT_COLOR, ls=FAULT_LS, lw=1.0,
               label=f"Cable cut (t={fault_time:.0f} s)" if label else None)


# ---------------------------------------------------------------------------
# Fig 1 — 3D load trajectory with pre- / post-fault colouring
# ---------------------------------------------------------------------------
def fig_3d_trajectory(df_traj, fault_time, out_dir):
    fig = plt.figure(figsize=(3.5, 3.0))
    ax  = fig.add_subplot(111, projection="3d")

    xs, ys, zs = df_traj["load_x"].values, df_traj["load_y"].values, df_traj["load_z"].values
    ts = df_traj["time"].values

    pre  = ts <= fault_time
    post = ts >  fault_time

    ax.plot(xs[pre],  ys[pre],  zs[pre],  color="#1f77b4", lw=1.2, label="Pre-fault")
    ax.plot(xs[post], ys[post], zs[post], color=FAULT_COLOR, lw=1.2, label="Post-fault")
    ax.scatter([xs[pre][-1]], [ys[pre][-1]], [zs[pre][-1]],
               marker="x", s=40, color=FAULT_COLOR, zorder=5)

    ax.set_xlabel("X (m)", labelpad=2)
    ax.set_ylabel("Y (m)", labelpad=2)
    ax.set_zlabel("Z (m)", labelpad=2)
    ax.set_title("Load 3-D Trajectory", pad=4)
    ax.legend(loc="upper left", fontsize=7)
    ax.tick_params(labelsize=7)

    plt.tight_layout()
    path = os.path.join(out_dir, "fig1_load_trajectory.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Fig 2 — Load position (X, Y, Z) vs time, with fault line
# ---------------------------------------------------------------------------
def fig_load_position(df_traj, fault_time, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(3.5, 4.5), sharex=True)
    t = df_traj["time"].values

    for ax, col, label, color in zip(
        axes,
        ["load_x", "load_y", "load_z"],
        ["X (m)", "Y (m)", "Z (m)"],
        ["#1f77b4", "#ff7f0e", "#2ca02c"],
    ):
        ax.plot(t, df_traj[col].values, color=color, lw=1.2)
        add_fault_line(ax, fault_time, label=(col == "load_x"))
        ax.set_ylabel(label)
        ax.grid(axis="y", alpha=0.35)

    axes[0].set_title("Load Position vs Time")
    axes[0].legend(fontsize=7, loc="upper right")
    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()
    path = os.path.join(out_dir, "fig2_load_position_time.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Fig 3 — Rope tensions vs time
# ---------------------------------------------------------------------------
def fig_rope_tensions(df_tens, fault_time, fault_quad, num_quads, out_dir):
    fig, ax = plt.subplots(figsize=(3.5, 2.6))
    t = df_tens["time"].values

    for q in range(num_quads):
        col   = f"rope{q}_mag"
        if col not in df_tens.columns:
            continue
        vals = df_tens[col].values.copy()
        # After the fault, the cut cable's internal bead-chain tensions are
        # simulation artifacts (the gate zeroes the attachment force).
        # Zero them so the figure reflects physical reality.
        if q == fault_quad:
            vals[t > fault_time] = 0.0
        label = f"Rope {q}" + (" (fault)" if q == fault_quad else "")
        ls    = FAULT_LS if q == fault_quad else "-"
        color = FAULT_COLOR if q == fault_quad else IEEE_COLORS[q % len(IEEE_COLORS)]
        ax.plot(t, vals, color=color, ls=ls, lw=1.2, label=label)

    add_fault_line(ax, fault_time)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Tension (N)")
    ax.set_title("Rope Tensions vs Time")
    ax.legend(fontsize=7, ncol=2)

    plt.tight_layout()
    path = os.path.join(out_dir, "fig3_rope_tensions.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Fig 4 — Fault quad retreat (X, Y, Z) vs time
# ---------------------------------------------------------------------------
def fig_fault_quad_retreat(df_traj, fault_time, fault_quad, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(3.5, 4.5), sharex=True)
    t = df_traj["time"].values
    prefix = f"drone{fault_quad}_"

    for ax, axis, color in zip(axes, ["x", "y", "z"], ["#1f77b4", "#ff7f0e", "#2ca02c"]):
        col = prefix + axis
        if col not in df_traj.columns:
            ax.set_visible(False)
            continue
        ax.plot(t, df_traj[col].values, color=color, lw=1.2)
        add_fault_line(ax, fault_time, label=(axis == "x"))
        ax.set_ylabel(f"{axis.upper()} (m)")

    axes[0].set_title(f"Quad {fault_quad} Position (Retreat After Fault)")
    axes[0].legend(fontsize=7, loc="upper right")
    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()
    path = os.path.join(out_dir, "fig4_quad0_retreat.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Fig 5 — Load Z stability: pre-fault RMS vs post-fault |Z - Z_ref| envelope
# ---------------------------------------------------------------------------
def fig_load_z_stability(df_traj, fault_time, out_dir):
    t  = df_traj["time"].values
    z  = df_traj["load_z"].values

    pre  = t <= fault_time
    post = t >  fault_time

    fig, ax = plt.subplots(figsize=(3.5, 2.4))
    ax.plot(t[pre],  z[pre],  color="#1f77b4", lw=1.2, label="Pre-fault")
    ax.plot(t[post], z[post], color=FAULT_COLOR, lw=1.2, label="Post-fault")
    add_fault_line(ax, fault_time)

    if np.any(post):
        # Rolling std of Z (post-fault) as stability envelope
        z_post  = z[post]
        z_mean  = np.mean(z_post[:min(50, len(z_post))])
        ax.axhline(z_mean, color="gray", ls=":", lw=0.8, label=f"Post-mean={z_mean:.2f} m")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Load Z (m)")
    ax.set_title("Load Altitude: Pre- vs Post-Fault")
    ax.legend(fontsize=7)

    plt.tight_layout()
    path = os.path.join(out_dir, "fig5_load_z_stability.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Fig 6 — Remaining quad thrust redistribution after fault
# ---------------------------------------------------------------------------
def fig_thrust_redistribution(df_ctrl, fault_time, fault_quad, num_quads, out_dir):
    if df_ctrl is None:
        return
    t = df_ctrl["time"].values
    fig, ax = plt.subplots(figsize=(3.5, 2.6))

    for q in range(num_quads):
        col = f"drone{q}_f_z"
        if col not in df_ctrl.columns:
            continue
        label = f"Quad {q}" + (" (fault)" if q == fault_quad else "")
        ls    = FAULT_LS if q == fault_quad else "-"
        color = FAULT_COLOR if q == fault_quad else IEEE_COLORS[q % len(IEEE_COLORS)]
        ax.plot(t, df_ctrl[col].values, color=color, ls=ls, lw=1.1, label=label)

    add_fault_line(ax, fault_time)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Thrust Fz (N)")
    ax.set_title("Quad Thrust Redistribution After Fault")
    ax.legend(fontsize=6, ncol=2)

    plt.tight_layout()
    path = os.path.join(out_dir, "fig6_thrust_redistribution.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Reference trajectory reconstruction from config.txt waypoints
# ---------------------------------------------------------------------------
def parse_config(log_dir):
    """Parse key=value config.txt produced by the simulation."""
    cfg = {}
    path = os.path.join(log_dir, "config.txt")
    if not os.path.exists(path):
        return cfg
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" in line:
                k, _, v = line.partition("=")
                cfg[k.strip()] = v.strip()
    return cfg


def parse_waypoints(cfg):
    """Return list of (arrival_time, hold_time, pos_array) from config dict.

    The waypoints in config.txt store drone-frame reference altitudes.
    The load hangs below the drone formation by:
        offset_z = avg_rope_length * rope_stretch_factor + payload_radius + attachment_offset
    where rope_stretch_factor=1.15, attachment_offset=0.1 m.
    We subtract this offset from z so that the reference matches the logged load position.
    """
    avg_rope = float(cfg.get("avg_rope_length", 1.0))
    payload_r = float(cfg.get("payload_radius", 0.15))
    rope_stretch = 1.15   # hardcoded in controller
    attach_off   = 0.1    # quad_attachment_offset.z magnitude
    offset_z = avg_rope * rope_stretch + payload_r + attach_off

    wps = []
    i = 0
    while f"waypoint_{i}_arrival_time" in cfg:
        at = float(cfg[f"waypoint_{i}_arrival_time"])
        ht = float(cfg[f"waypoint_{i}_hold_time"])
        pos_str = cfg[f"waypoint_{i}_position"]  # "(x, y, z)"
        nums = [float(v) for v in pos_str.strip("()").split(",")]
        pos = np.array(nums)
        pos[2] -= offset_z  # convert drone-altitude ref → load-altitude ref
        wps.append((at, ht, pos))
        i += 1
    return wps


def _ref_at(t, waypoints):
    """Evaluate reference position at scalar time t (piecewise-linear + hold)."""
    if not waypoints:
        return np.zeros(3)
    if t <= waypoints[0][0]:
        return waypoints[0][2].copy()
    for i, (at_i, ht_i, pos_i) in enumerate(waypoints):
        hold_end = at_i + ht_i
        # In hold period at waypoint i
        if at_i <= t <= hold_end:
            return pos_i.copy()
        # In transit to waypoint i+1
        if i + 1 < len(waypoints):
            at_next = waypoints[i + 1][0]
            if hold_end < t <= at_next:
                denom = at_next - hold_end
                alpha = (t - hold_end) / denom if denom > 0 else 1.0
                return pos_i + alpha * (waypoints[i + 1][2] - pos_i)
    return waypoints[-1][2].copy()


def compute_reference(t_array, waypoints):
    """Return (N, 3) reference positions for each element of t_array."""
    ref = np.zeros((len(t_array), 3))
    for k, t in enumerate(t_array):
        ref[k] = _ref_at(t, waypoints)
    return ref


# ---------------------------------------------------------------------------
# Fig 7 — Tracking error (requires reference reconstruction)
# ---------------------------------------------------------------------------
def fig_tracking_error(df_traj, ref_pos, fault_time, out_dir):
    t  = df_traj["time"].values
    ex = df_traj["load_x"].values - ref_pos[:, 0]
    ey = df_traj["load_y"].values - ref_pos[:, 1]
    ez = df_traj["load_z"].values - ref_pos[:, 2]
    exy = np.sqrt(ex**2 + ey**2)

    fig, axes = plt.subplots(3, 1, figsize=(3.5, 4.5), sharex=True)

    axes[0].plot(t, ex, color="#1f77b4", lw=1.1, label=r"$e_x$")
    axes[0].plot(t, ey, color="#ff7f0e", lw=1.1, label=r"$e_y$")
    add_fault_line(axes[0], fault_time)
    axes[0].set_ylabel("X/Y Error (m)")
    axes[0].set_title("Load Tracking Error vs Time")
    axes[0].legend(fontsize=7)

    axes[1].plot(t, exy, color="#2ca02c", lw=1.1)
    add_fault_line(axes[1], fault_time, label=False)
    axes[1].set_ylabel("XY Error Norm (m)")

    axes[2].plot(t, ez, color="#9467bd", lw=1.1)
    add_fault_line(axes[2], fault_time, label=False)
    axes[2].set_ylabel("Z Error (m)")
    axes[2].set_xlabel("Time (s)")

    plt.tight_layout()
    path = os.path.join(out_dir, "fig7_tracking_error.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Fig 8 — Load swing (tilt angle from vertical via quaternion)
# ---------------------------------------------------------------------------
def fig_load_swing(df_traj, fault_time, out_dir):
    if "load_qw" not in df_traj.columns:
        print("  Skipping fig8: no quaternion columns in trajectories.csv")
        return
    t  = df_traj["time"].values
    qx = df_traj["load_qx"].values
    qy = df_traj["load_qy"].values
    # Tilt angle: angle between body-z and world-z
    # R_zz = 1 - 2*(qx^2 + qy^2)
    cos_tilt = np.clip(1.0 - 2.0 * (qx**2 + qy**2), -1.0, 1.0)
    tilt_deg = np.degrees(np.arccos(cos_tilt))

    pre  = t <= fault_time
    post = t >  fault_time

    fig, ax = plt.subplots(figsize=(3.5, 2.4))
    ax.plot(t[pre],  tilt_deg[pre],  color="#1f77b4", lw=1.2, label="Pre-fault")
    ax.plot(t[post], tilt_deg[post], color=FAULT_COLOR, lw=1.2, label="Post-fault")
    add_fault_line(ax, fault_time)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Tilt Angle (deg)")
    ax.set_title("Load Swing (Tilt from Vertical)")
    ax.legend(fontsize=7)

    plt.tight_layout()
    path = os.path.join(out_dir, "fig8_load_swing.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Fig 9 — Transient zoom: fault event detail (load X, tracking error, tensions)
# ---------------------------------------------------------------------------
def fig_transient_zoom(df_traj, df_tens, ref_pos, fault_time, fault_quad,
                       num_quads, out_dir):
    t_win_lo = max(df_traj["time"].values[0], fault_time - 5.0)
    t_win_hi = min(df_traj["time"].values[-1], fault_time + 12.0)

    t  = df_traj["time"].values
    mk = (t >= t_win_lo) & (t <= t_win_hi)
    t_m = t[mk]

    fig, axes = plt.subplots(3, 1, figsize=(3.5, 5.0), sharex=True)

    # Panel 1: load X vs reference X
    load_x = df_traj["load_x"].values[mk]
    ref_x  = ref_pos[mk, 0]
    axes[0].plot(t_m, load_x, color="#1f77b4", lw=1.2, label="Load X")
    axes[0].plot(t_m, ref_x,  color="gray",    lw=1.0, ls="--", label="Ref X")
    add_fault_line(axes[0], fault_time)
    axes[0].set_ylabel("X (m)")
    axes[0].set_title(f"Fault Transient (t={fault_time:.0f} s)")
    axes[0].legend(fontsize=7)

    # Panel 2: XY tracking error
    ex  = df_traj["load_x"].values[mk] - ref_pos[mk, 0]
    ey  = df_traj["load_y"].values[mk] - ref_pos[mk, 1]
    exy = np.sqrt(ex**2 + ey**2)
    axes[1].plot(t_m, exy, color="#2ca02c", lw=1.2)
    add_fault_line(axes[1], fault_time, label=False)
    axes[1].set_ylabel("XY Error (m)")

    # Panel 3: tensions in window
    if df_tens is not None:
        tt = df_tens["time"].values
        mk_t = (tt >= t_win_lo) & (tt <= t_win_hi)
        t_t = tt[mk_t]
        for q in range(num_quads):
            col = f"rope{q}_mag"
            if col not in df_tens.columns:
                continue
            vals = df_tens[col].values[mk_t].copy()
            if q == fault_quad:
                vals[t_t > fault_time] = 0.0
            label = f"Rope {q}" + (" (cut)" if q == fault_quad else "")
            ls    = FAULT_LS if q == fault_quad else "-"
            color = FAULT_COLOR if q == fault_quad else IEEE_COLORS[q % len(IEEE_COLORS)]
            axes[2].plot(t_t, vals, color=color, ls=ls, lw=1.1, label=label)
        add_fault_line(axes[2], fault_time, label=False)
        axes[2].set_ylabel("Tension (N)")
        axes[2].legend(fontsize=6, ncol=2)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    path = os.path.join(out_dir, "fig9_transient_zoom.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Fig 10 — Tension force elevation angles (direction analysis)
# ---------------------------------------------------------------------------
def fig_tension_angles(df_tens, fault_time, fault_quad, num_quads, out_dir):
    if df_tens is None:
        return
    if "rope0_fx" not in df_tens.columns:
        print("  Skipping fig10: no tension vector columns (fx/fy/fz)")
        return

    t = df_tens["time"].values
    fig, ax = plt.subplots(figsize=(3.5, 2.6))

    for q in range(num_quads):
        if f"rope{q}_fz" not in df_tens.columns:
            continue
        fx = df_tens[f"rope{q}_fx"].values
        fy = df_tens[f"rope{q}_fy"].values
        fz = df_tens[f"rope{q}_fz"].values
        fxy  = np.sqrt(fx**2 + fy**2)
        elev = np.degrees(np.arctan2(np.abs(fz), fxy))
        elev_plot = elev.copy().astype(float)
        if q == fault_quad:
            elev_plot[t > fault_time] = np.nan
        label = f"Rope {q}" + (" (fault)" if q == fault_quad else "")
        ls    = FAULT_LS if q == fault_quad else "-"
        color = FAULT_COLOR if q == fault_quad else IEEE_COLORS[q % len(IEEE_COLORS)]
        ax.plot(t, elev_plot, color=color, ls=ls, lw=1.1, label=label)

    add_fault_line(ax, fault_time)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Elevation Angle (deg)")
    ax.set_title("Rope Tension Force Direction (Elevation)")
    ax.legend(fontsize=6, ncol=2)

    plt.tight_layout()
    path = os.path.join(out_dir, "fig10_tension_angles.png")
    plt.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Numerical summary
# ---------------------------------------------------------------------------
def print_summary(df_traj, df_tens, fault_time, fault_quad):
    t  = df_traj["time"].values
    z  = df_traj["load_z"].values

    pre  = t <= fault_time
    post = t >  fault_time

    z_pre_rms = np.sqrt(np.mean((z[pre] - np.mean(z[pre]))**2)) if np.any(pre) else float("nan")
    z_post_rms = np.sqrt(np.mean((z[post] - np.mean(z[post][:min(50, np.sum(post))]))**2)) if np.any(post) else float("nan")

    print("\n=== Fault Simulation Summary ===")
    print(f"  Cable cut at t = {fault_time:.1f} s  (quad {fault_quad})")
    print(f"  Load Z:  pre-fault RMS = {z_pre_rms:.4f} m  |  post-fault RMS = {z_post_rms:.4f} m")
    print(f"  Load Z at cut:  {z[pre][-1]:.3f} m")
    if np.any(post):
        print(f"  Load Z at end:  {z[-1]:.3f} m  (delta = {z[-1]-z[pre][-1]:+.3f} m)")

    # Tension drop
    col = f"rope{fault_quad}_mag"
    if df_tens is not None and col in df_tens.columns:
        t_tens = df_tens["time"].values
        tens   = df_tens[col].values
        at_cut = tens[t_tens <= fault_time][-1] if np.any(t_tens <= fault_time) else float("nan")
        after  = tens[t_tens > fault_time]
        post_mean = np.mean(after[:min(50, len(after))]) if len(after) > 0 else float("nan")
        print(f"  Rope {fault_quad} tension at cut: {at_cut:.2f} N  → post-fault mean: {post_mean:.2f} N")
    print("=" * 35)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Analyze fault injection simulation")
    parser.add_argument("--log-dir",    required=True,  help="Path to timestamped log directory")
    parser.add_argument("--fault-time", type=float, default=15.0,  help="Fault time (s)")
    parser.add_argument("--fault-quad", type=int,   default=0,     help="Faulty quadcopter index")
    parser.add_argument("--num-quads",  type=int,   default=5,     help="Number of quadcopters")
    parser.add_argument("--output-dir", default=None, help="Output directory for figures (default: log-dir/../analysis)")
    args = parser.parse_args()

    if not os.path.isdir(args.log_dir):
        print(f"ERROR: log-dir not found: {args.log_dir}", file=sys.stderr)
        sys.exit(1)

    out_dir = args.output_dir or os.path.join(os.path.dirname(args.log_dir), "analysis_t15")
    os.makedirs(out_dir, exist_ok=True)

    print(f"Loading logs from: {args.log_dir}")
    print(f"Output figures to: {out_dir}")

    ieee_style()

    # Load data
    def load_csv(name):
        path = os.path.join(args.log_dir, name)
        if not os.path.exists(path):
            print(f"  Warning: {name} not found; skipping.")
            return None
        df = pd.read_csv(path)
        print(f"  Loaded {name}: {len(df)} rows")
        return df

    df_traj = load_csv("trajectories.csv")
    df_tens = load_csv("tensions.csv")
    df_ctrl = load_csv("control_efforts.csv")

    if df_traj is None:
        print("ERROR: trajectories.csv is required.", file=sys.stderr)
        sys.exit(1)

    # Reconstruct reference trajectory from config.txt waypoints
    cfg       = parse_config(args.log_dir)
    waypoints = parse_waypoints(cfg)
    if waypoints:
        print(f"  Parsed {len(waypoints)} waypoints from config.txt")
        ref_pos = compute_reference(df_traj["time"].values, waypoints)
    else:
        print("  Warning: no waypoints found; tracking error figures will be skipped.")
        ref_pos = None

    print_summary(df_traj, df_tens, args.fault_time, args.fault_quad)

    print("\nGenerating figures...")
    fig_3d_trajectory(df_traj,   args.fault_time, out_dir)
    fig_load_position(df_traj,   args.fault_time, out_dir)
    if df_tens is not None:
        fig_rope_tensions(df_tens, args.fault_time, args.fault_quad, args.num_quads, out_dir)
    fig_fault_quad_retreat(df_traj, args.fault_time, args.fault_quad, out_dir)
    fig_load_z_stability(df_traj,  args.fault_time, out_dir)
    if df_ctrl is not None:
        fig_thrust_redistribution(df_ctrl, args.fault_time, args.fault_quad, args.num_quads, out_dir)

    # Extended analysis (new figures)
    if ref_pos is not None:
        fig_tracking_error(df_traj, ref_pos, args.fault_time, out_dir)
        fig_transient_zoom(df_traj, df_tens, ref_pos, args.fault_time,
                           args.fault_quad, args.num_quads, out_dir)
    fig_load_swing(df_traj, args.fault_time, out_dir)
    if df_tens is not None:
        fig_tension_angles(df_tens, args.fault_time, args.fault_quad, args.num_quads, out_dir)

    print(f"\nDone. {len(os.listdir(out_dir))} files in {out_dir}")


if __name__ == "__main__":
    main()
