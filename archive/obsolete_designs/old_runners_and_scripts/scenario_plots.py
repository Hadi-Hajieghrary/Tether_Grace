#!/usr/bin/env python3
"""
Per-scenario multi-panel plot of the decentralized fault-aware lift sim.

Usage:
    python3 scenario_plots.py <csv_file> [<output_png>]

Produces a 12-panel figure covering:
  - 3D trajectory (payload + drones + reference)
  - Top-down XY
  - Altitude tracking
  - Tracking error (per axis)
  - Rope tensions per drone
  - Commanded thrust per drone
  - N_alive detection signal
  - Target tension adaptation
  - Load-sharing imbalance
  - Control effort
  - Peer-tension sum
  - Drone Z positions
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")  # headless-friendly
import matplotlib.pyplot as plt
from matplotlib import gridspec


def load_scenario(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    return df


def detect_fault_times(df: pd.DataFrame, num_drones: int) -> list[tuple[int, float]]:
    """Detect cable severance from tension signal: the time a tension_i
    drops to ~zero permanently. Returns list of (drone_idx, fault_time)."""
    faults = []
    for i in range(num_drones):
        t = df[f"tension_{i}"].values
        time = df["time"].values
        # Fault = tension drops to near zero and stays there for >0.5s
        near_zero = np.abs(t) < 0.2
        if near_zero.any():
            # Find first index where near_zero is true AND stays true for remainder
            # (allows for brief spikes)
            for j in range(len(t) - 1, 0, -1):
                if not near_zero[j]:
                    # j is the last non-zero — if there's any subsequent data it went to zero
                    if j < len(t) - 1:
                        faults.append((i, float(time[j + 1])))
                    break
    return faults


def num_drones_in_csv(df: pd.DataFrame) -> int:
    n = 0
    while f"quad{n}_x" in df.columns:
        n += 1
    return n


def plot_scenario(df: pd.DataFrame, out_path: Path, title: str = ""):
    N = num_drones_in_csv(df)
    faults = detect_fault_times(df, N)

    t = df["time"].values
    colors = ["#d62728", "#2ca02c", "#1f77b4", "#e6b400"]  # red/green/blue/yellow
    drone_names = [f"Drone {i}" for i in range(N)]

    # Derived signals
    payload_pos = df[["payload_x", "payload_y", "payload_z"]].values
    ref_pos = df[["ref_x", "ref_y", "ref_z"]].values
    track_err = ref_pos - payload_pos
    payload_vel = df[["payload_vx", "payload_vy", "payload_vz"]].values

    # Per-drone thrust magnitude (|f|)
    thrust_mag = np.zeros((len(t), N))
    for i in range(N):
        fx = df[f"fx_{i}"].values
        fy = df[f"fy_{i}"].values
        fz = df[f"fz_{i}"].values
        thrust_mag[:, i] = np.sqrt(fx * fx + fy * fy + fz * fz)

    tensions = np.column_stack([df[f"tension_{i}"].values for i in range(N)])
    n_alive_0 = df["n_alive_0"].values
    target_tension_0 = df["target_tension_0"].values

    # Load-sharing imbalance: std of tensions (excluding faulted drones by time)
    tension_std = np.std(tensions, axis=1)

    # Integrated control effort: cumulative ∫ f_z dt per drone
    fz_per_drone = np.column_stack([df[f"fz_{i}"].values for i in range(N)])
    dt = np.diff(t, prepend=t[0])
    cum_effort = np.cumsum(fz_per_drone * dt[:, None], axis=0)

    # ========= FIGURE =========
    fig = plt.figure(figsize=(18, 22))
    gs = gridspec.GridSpec(6, 3, figure=fig, hspace=0.4, wspace=0.3)

    # -- 3D trajectory --
    ax3d = fig.add_subplot(gs[0, 0], projection="3d")
    ax3d.plot(ref_pos[:, 0], ref_pos[:, 1], ref_pos[:, 2],
              "k--", lw=1.5, label="Reference", alpha=0.6)
    ax3d.plot(payload_pos[:, 0], payload_pos[:, 1], payload_pos[:, 2],
              "r-", lw=2, label="Payload")
    for i in range(N):
        p = df[[f"quad{i}_x", f"quad{i}_y", f"quad{i}_z"]].values
        ax3d.plot(p[:, 0], p[:, 1], p[:, 2], color=colors[i], lw=1, alpha=0.7,
                  label=drone_names[i])
    ax3d.set_xlabel("x [m]"); ax3d.set_ylabel("y [m]"); ax3d.set_zlabel("z [m]")
    ax3d.set_title("3D Trajectory")
    ax3d.legend(loc="upper left", fontsize=7)

    # -- Top-down XY --
    ax_xy = fig.add_subplot(gs[0, 1])
    ax_xy.plot(ref_pos[:, 0], ref_pos[:, 1], "k--", lw=1.5, label="Reference", alpha=0.6)
    ax_xy.plot(payload_pos[:, 0], payload_pos[:, 1], "r-", lw=2, label="Payload")
    for i in range(N):
        p = df[[f"quad{i}_x", f"quad{i}_y"]].values
        ax_xy.plot(p[:, 0], p[:, 1], color=colors[i], lw=1, alpha=0.7,
                   label=drone_names[i])
    ax_xy.set_xlabel("x [m]"); ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Top-Down View (XY)")
    ax_xy.axis("equal"); ax_xy.grid(alpha=0.3); ax_xy.legend(fontsize=7)

    # -- Altitude tracking --
    ax_alt = fig.add_subplot(gs[0, 2])
    ax_alt.plot(t, ref_pos[:, 2], "k--", label="z_ref")
    ax_alt.plot(t, payload_pos[:, 2], "r-", lw=2, label="z_payload")
    for i, ft in faults:
        ax_alt.axvline(ft, color=colors[i], ls=":", alpha=0.7,
                       label=f"Fault drone {i} @ t={ft:.1f}s")
    ax_alt.set_xlabel("t [s]"); ax_alt.set_ylabel("z [m]")
    ax_alt.set_title("Altitude Tracking"); ax_alt.grid(alpha=0.3); ax_alt.legend(fontsize=7)

    # -- Position tracking error --
    ax_err = fig.add_subplot(gs[1, 0])
    ax_err.plot(t, track_err[:, 0], label="e_x", color="#d62728")
    ax_err.plot(t, track_err[:, 1], label="e_y", color="#2ca02c")
    ax_err.plot(t, track_err[:, 2], label="e_z", color="#1f77b4")
    for i, ft in faults: ax_err.axvline(ft, color="gray", ls=":", alpha=0.5)
    ax_err.set_xlabel("t [s]"); ax_err.set_ylabel("error [m]")
    ax_err.set_title("Tracking Error (ref − payload)"); ax_err.grid(alpha=0.3); ax_err.legend()

    # -- Payload velocity --
    ax_vel = fig.add_subplot(gs[1, 1])
    ax_vel.plot(t, payload_vel[:, 0], label="v_x", color="#d62728")
    ax_vel.plot(t, payload_vel[:, 1], label="v_y", color="#2ca02c")
    ax_vel.plot(t, payload_vel[:, 2], label="v_z", color="#1f77b4")
    for i, ft in faults: ax_vel.axvline(ft, color="gray", ls=":", alpha=0.5)
    ax_vel.set_xlabel("t [s]"); ax_vel.set_ylabel("v [m/s]")
    ax_vel.set_title("Payload Velocity"); ax_vel.grid(alpha=0.3); ax_vel.legend()

    # -- Tracking error norm --
    ax_en = fig.add_subplot(gs[1, 2])
    err_norm = np.linalg.norm(track_err, axis=1)
    ax_en.plot(t, err_norm, "b-", lw=1.5)
    for i, ft in faults: ax_en.axvline(ft, color="gray", ls=":", alpha=0.5)
    ax_en.set_xlabel("t [s]"); ax_en.set_ylabel("||error|| [m]")
    ax_en.set_title(f"|Tracking Error|  (RMS={np.sqrt(np.mean(err_norm**2)):.3f} m)")
    ax_en.grid(alpha=0.3)

    # -- Rope tensions --
    ax_ten = fig.add_subplot(gs[2, 0])
    for i in range(N):
        ax_ten.plot(t, tensions[:, i], color=colors[i], label=drone_names[i], lw=1.2)
    for i, ft in faults: ax_ten.axvline(ft, color=colors[i], ls=":", alpha=0.7)
    ax_ten.set_xlabel("t [s]"); ax_ten.set_ylabel("T [N]")
    ax_ten.set_title("Per-Rope Tension"); ax_ten.grid(alpha=0.3); ax_ten.legend()

    # -- Commanded thrust --
    ax_thr = fig.add_subplot(gs[2, 1])
    for i in range(N):
        ax_thr.plot(t, thrust_mag[:, i], color=colors[i], label=drone_names[i], lw=1.2)
    for i, ft in faults: ax_thr.axvline(ft, color=colors[i], ls=":", alpha=0.7)
    ax_thr.set_xlabel("t [s]"); ax_thr.set_ylabel("|F| [N]")
    ax_thr.set_title("Commanded Thrust Magnitude"); ax_thr.grid(alpha=0.3); ax_thr.legend()

    # -- Load-sharing imbalance --
    ax_imb = fig.add_subplot(gs[2, 2])
    ax_imb.plot(t, tension_std, "purple", lw=1.5)
    for i, ft in faults: ax_imb.axvline(ft, color="gray", ls=":", alpha=0.5)
    ax_imb.set_xlabel("t [s]"); ax_imb.set_ylabel("std(T) [N]")
    ax_imb.set_title("Load-Sharing Imbalance  (std of rope tensions)")
    ax_imb.grid(alpha=0.3)

    # -- N_alive detection (per drone) --
    ax_n = fig.add_subplot(gs[3, 0])
    for i in range(N):
        ax_n.plot(t, df[f"n_alive_{i}"].values, color=colors[i],
                  label=drone_names[i], lw=1.2, alpha=0.8)
    for i, ft in faults: ax_n.axvline(ft, color=colors[i], ls=":", alpha=0.7)
    ax_n.set_xlabel("t [s]"); ax_n.set_ylabel("N_alive")
    ax_n.set_title("N_alive Detection (peer count inferred from tensions)")
    ax_n.grid(alpha=0.3); ax_n.legend(); ax_n.set_ylim(-0.2, N + 0.5)

    # -- Target tension --
    ax_tgt = fig.add_subplot(gs[3, 1])
    for i in range(N):
        ax_tgt.plot(t, df[f"target_tension_{i}"].values, color=colors[i],
                    label=drone_names[i], lw=1.2)
    for i, ft in faults: ax_tgt.axvline(ft, color=colors[i], ls=":", alpha=0.7)
    ax_tgt.set_xlabel("t [s]"); ax_tgt.set_ylabel("T_target [N]")
    ax_tgt.set_title("Per-Drone Target Tension  (= m_L·g / N_alive)")
    ax_tgt.grid(alpha=0.3); ax_tgt.legend()

    # -- Drone z positions --
    ax_zq = fig.add_subplot(gs[3, 2])
    for i in range(N):
        ax_zq.plot(t, df[f"quad{i}_z"].values, color=colors[i],
                   label=drone_names[i], lw=1.2)
    ax_zq.plot(t, payload_pos[:, 2], "r--", lw=1.5, label="payload")
    for i, ft in faults: ax_zq.axvline(ft, color=colors[i], ls=":", alpha=0.7)
    ax_zq.set_xlabel("t [s]"); ax_zq.set_ylabel("z [m]")
    ax_zq.set_title("Vertical Positions (drones + payload)")
    ax_zq.grid(alpha=0.3); ax_zq.legend(fontsize=7)

    # -- Cumulative control effort (energy proxy) --
    ax_ce = fig.add_subplot(gs[4, 0])
    for i in range(N):
        ax_ce.plot(t, cum_effort[:, i], color=colors[i], label=drone_names[i], lw=1.2)
    for i, ft in faults: ax_ce.axvline(ft, color=colors[i], ls=":", alpha=0.7)
    ax_ce.set_xlabel("t [s]"); ax_ce.set_ylabel("∫ f_z dt [N·s]")
    ax_ce.set_title("Cumulative Vertical Impulse per Drone (energy proxy)")
    ax_ce.grid(alpha=0.3); ax_ce.legend()

    # -- Torques (tau) per drone --
    ax_tq = fig.add_subplot(gs[4, 1])
    for i in range(N):
        tx = df[f"tau_x_{i}"].values
        ty = df[f"tau_y_{i}"].values
        tz = df[f"tau_z_{i}"].values
        tau_mag = np.sqrt(tx * tx + ty * ty + tz * tz)
        ax_tq.plot(t, tau_mag, color=colors[i], label=drone_names[i], lw=1.0, alpha=0.7)
    for i, ft in faults: ax_tq.axvline(ft, color=colors[i], ls=":", alpha=0.7)
    ax_tq.set_xlabel("t [s]"); ax_tq.set_ylabel("|τ| [N·m]")
    ax_tq.set_title("Torque Magnitude per Drone"); ax_tq.grid(alpha=0.3); ax_tq.legend(fontsize=7)

    # -- Per-drone horizontal position errors --
    ax_hpe = fig.add_subplot(gs[4, 2])
    for i in range(N):
        qx = df[f"quad{i}_x"].values
        qy = df[f"quad{i}_y"].values
        # Each drone's nominal offset from payload: in its formation spoke
        angle = 2.0 * np.pi * i / N
        formation_r = 0.8  # matches C++ formation_radius
        pos_err_x = qx - (ref_pos[:, 0] + formation_r * np.cos(angle))
        pos_err_y = qy - (ref_pos[:, 1] + formation_r * np.sin(angle))
        err_norm_drone = np.sqrt(pos_err_x ** 2 + pos_err_y ** 2)
        ax_hpe.plot(t, err_norm_drone, color=colors[i], label=drone_names[i], lw=1.0)
    for i, ft in faults: ax_hpe.axvline(ft, color=colors[i], ls=":", alpha=0.7)
    ax_hpe.set_xlabel("t [s]"); ax_hpe.set_ylabel("|drone XY err| [m]")
    ax_hpe.set_title("Per-Drone Horizontal Position Error")
    ax_hpe.grid(alpha=0.3); ax_hpe.legend(fontsize=7)

    # -- Summary statistics table --
    ax_stats = fig.add_subplot(gs[5, :])
    ax_stats.axis("off")
    # Compute per-phase metrics
    t_end = t[-1]
    # Phase 1: pre-fault steady-state (use 5-8s window, no faults yet)
    pre_fault_mask = (t >= 5.0) & (t <= 8.0)
    rms_pre = np.sqrt(np.mean(err_norm[pre_fault_mask] ** 2)) if pre_fault_mask.any() else np.nan
    post_all = (t >= t_end - 3.0) & (t <= t_end)
    rms_post = np.sqrt(np.mean(err_norm[post_all] ** 2)) if post_all.any() else np.nan

    stats_text = (
        f"SCENARIO: {title}\n"
        f"  duration: {t_end:.2f} s    samples: {len(t)}    N drones: {N}\n"
        f"  Overall position-error RMS: {np.sqrt(np.mean(err_norm**2)):.3f} m    max: {err_norm.max():.3f} m\n"
        f"  Pre-fault (5-8s) RMS: {rms_pre:.3f} m    Final-3s RMS: {rms_post:.3f} m\n"
        f"  Max rope tension: {tensions.max():.2f} N    Max commanded thrust: {thrust_mag.max():.2f} N\n"
        f"  Total vertical impulse (sum over drones): {cum_effort[-1].sum():.1f} N·s\n"
    )
    if faults:
        stats_text += "  Detected faults: " + ", ".join(
            f"drone {i} @ t={ft:.2f}s" for i, ft in faults) + "\n"
    else:
        stats_text += "  No faults detected.\n"
    ax_stats.text(0.01, 0.95, stats_text, transform=ax_stats.transAxes,
                  fontsize=11, family="monospace", va="top")

    fig.suptitle(
        f"Decentralized Fault-Aware Lift — {title}", fontsize=14, weight="bold")
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return out_path


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    csv_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) >= 3 \
        else csv_path.with_suffix(".png")
    df = load_scenario(csv_path)
    title = csv_path.stem.replace("scenario_", "")
    plot_scenario(df, out_path, title=title)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
