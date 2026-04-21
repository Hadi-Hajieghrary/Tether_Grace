#!/usr/bin/env python3
"""
Generate all IEEE-style individual-plot PNGs for a single scenario CSV.

Produces one PNG per signal, each ready for IEEE Transactions (single- or
double-column figures). Call as:

    python3 plot_scenario.py <csv_file> <out_dir>
"""
from __future__ import annotations
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Make ieee_style importable if called directly
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ieee_style import (
    setup_style, COLORS, REF_STYLE, PAYLOAD_STYLE,
    SINGLE_COL, DOUBLE_COL, annotate_faults, trim_start, detect_faults,
)

setup_style()


# ====================================================================
#  INDIVIDUAL PLOT GENERATORS
# ====================================================================

def plot_3d_trajectory(df, out, faults):
    fig = plt.figure(figsize=SINGLE_COL)
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(df["ref_x"], df["ref_y"], df["ref_z"], **REF_STYLE)
    ax.plot(df["payload_x"], df["payload_y"], df["payload_z"],
            color=PAYLOAD_STYLE["color"], lw=1.3, label="Payload")
    for i in range(4):
        ax.plot(df[f"quad{i}_x"], df[f"quad{i}_y"], df[f"quad{i}_z"],
                color=COLORS[i], lw=0.8, alpha=0.75, label=f"Drone {i}")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.legend(loc="upper left", fontsize=6.5, ncol=1)
    fig.savefig(out)
    plt.close(fig)


def plot_xy_top(df, out, faults):
    fig, ax = plt.subplots(figsize=SINGLE_COL)
    ax.plot(df["ref_x"], df["ref_y"], **REF_STYLE)
    ax.plot(df["payload_x"], df["payload_y"], **PAYLOAD_STYLE)
    for i in range(4):
        ax.plot(df[f"quad{i}_x"], df[f"quad{i}_y"], color=COLORS[i], lw=0.9,
                alpha=0.8, label=f"Drone {i}")
    ax.set_xlabel(r"$x$ [m]"); ax.set_ylabel(r"$y$ [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best", ncol=2)
    fig.savefig(out)
    plt.close(fig)


def plot_altitude_tracking(df, out, faults):
    fig, ax = plt.subplots(figsize=SINGLE_COL)
    ax.plot(df["time"], df["ref_z"], **REF_STYLE)
    ax.plot(df["time"], df["payload_z"], **PAYLOAD_STYLE)
    annotate_faults(ax, faults)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$z$ [m]")
    ax.legend(loc="lower right")
    fig.savefig(out)
    plt.close(fig)


def plot_tracking_error(df, out, faults):
    """Payload tracking error — the CSV reference is the payload target
    trajectory (not the drone formation centre), so this is a direct
    comparison of actual versus commanded payload position.
    """
    err = df[["ref_x", "ref_y", "ref_z"]].values - df[["payload_x", "payload_y", "payload_z"]].values
    err_norm = np.linalg.norm(err, axis=1)

    fig, ax = plt.subplots(figsize=SINGLE_COL)
    ax.plot(df["time"], err_norm, color="#333333", lw=1.1)
    annotate_faults(ax, faults)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$\|e_p\|$ [m]")
    rms = np.sqrt(np.mean(err_norm ** 2))
    ax.text(0.02, 0.93, f"RMS = {rms:.3f} m", transform=ax.transAxes,
            fontsize=7.5, verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor="#cccccc", lw=0.5))
    fig.savefig(out)
    plt.close(fig)


def plot_tensions(df, out, faults):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i in range(4):
        ax.plot(df["time"], df[f"tension_{i}"], color=COLORS[i],
                lw=1.0, label=f"Drone {i}")
    annotate_faults(ax, faults)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$T_i$ [N]")
    ax.legend(ncol=4, loc="upper center")
    fig.savefig(out)
    plt.close(fig)


def plot_thrust_magnitude(df, out, faults):
    fig, ax = plt.subplots(figsize=DOUBLE_COL)
    for i in range(4):
        F = np.sqrt(df[f"fx_{i}"] ** 2 + df[f"fy_{i}"] ** 2 + df[f"fz_{i}"] ** 2)
        ax.plot(df["time"], F, color=COLORS[i], lw=1.0, label=f"Drone {i}")
    annotate_faults(ax, faults)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$\|F_i\|$ [N]")
    ax.legend(ncol=4, loc="upper center")
    fig.savefig(out)
    plt.close(fig)


def plot_swing_speed(df, out, faults):
    """Payload horizontal-swing speed |v_payload_xy| — pendulum kinetic energy
    proxy. Each drone computes this from its own payload observation; the
    four curves should be identical in the absence of measurement noise."""
    fig, ax = plt.subplots(figsize=SINGLE_COL)
    for i in range(4):
        ax.plot(df["time"], df[f"swing_speed_{i}"], color=COLORS[i], lw=0.9,
                alpha=0.85, label=f"Drone {i}")
    annotate_faults(ax, faults)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$\|v_{L,xy}\|$ [m/s]")
    ax.legend(ncol=2, loc="upper left")
    fig.savefig(out)
    plt.close(fig)


def plot_swing_offset(df, out, faults):
    """Per-drone anti-swing slot-offset magnitude — the size of the local
    QP's swing-damping correction on top of the formation slot."""
    fig, ax = plt.subplots(figsize=SINGLE_COL)
    for i in range(4):
        ax.plot(df["time"], df[f"swing_offset_{i}"], color=COLORS[i],
                lw=0.9, alpha=0.85, label=f"Drone {i}")
    annotate_faults(ax, faults)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$\|\Delta p_{\mathrm{slot}}\|$ [m]")
    ax.legend(ncol=2, loc="upper left")
    fig.savefig(out)
    plt.close(fig)


def plot_load_imbalance(df, out, faults):
    T = np.column_stack([df[f"tension_{i}"] for i in range(4)])
    imb = np.std(T, axis=1)
    fig, ax = plt.subplots(figsize=SINGLE_COL)
    ax.plot(df["time"], imb, color="#444444", lw=1.1)
    annotate_faults(ax, faults)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$\sigma_T$ [N]")
    fig.savefig(out)
    plt.close(fig)


def plot_payload_velocity(df, out, faults):
    fig, ax = plt.subplots(figsize=SINGLE_COL)
    ax.plot(df["time"], df["payload_vx"], color="#D55E00", lw=0.9, label=r"$v_x$")
    ax.plot(df["time"], df["payload_vy"], color="#009E73", lw=0.9, label=r"$v_y$")
    ax.plot(df["time"], df["payload_vz"], color="#0072B2", lw=0.9, label=r"$v_z$")
    annotate_faults(ax, faults)
    ax.set_xlabel(r"Time [s]"); ax.set_ylabel(r"$v$ [m/s]")
    ax.legend(ncol=3, loc="upper center")
    fig.savefig(out)
    plt.close(fig)


# ====================================================================
def generate_all(csv_path: Path, out_dir: Path, scenario_id: str):
    out_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(csv_path)
    df = trim_start(df, t_start=0.5)
    faults = detect_faults(df)

    plots = [
        ("fig_3d_trajectory",     plot_3d_trajectory),
        ("fig_xy_top",            plot_xy_top),
        ("fig_altitude_tracking", plot_altitude_tracking),
        ("fig_tracking_error",    plot_tracking_error),
        ("fig_tensions",          plot_tensions),
        ("fig_thrust_magnitude",  plot_thrust_magnitude),
        ("fig_swing_speed",       plot_swing_speed),
        ("fig_swing_offset",      plot_swing_offset),
        ("fig_load_imbalance",    plot_load_imbalance),
        ("fig_payload_velocity",  plot_payload_velocity),
    ]

    print(f"[{scenario_id}] generating {len(plots)} plots → {out_dir}")
    for name, fn in plots:
        fn(df, out_dir / f"{name}.png", faults)
    print(f"[{scenario_id}] done")


def main():
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    csv = Path(sys.argv[1])
    out = Path(sys.argv[2])
    scenario_id = csv.stem.replace("scenario_", "")
    generate_all(csv, out, scenario_id)


if __name__ == "__main__":
    main()
